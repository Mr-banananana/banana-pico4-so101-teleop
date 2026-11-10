"""Feedback-based physical arm. One thread owns all serial I/O.

Construction is read-only. Torque requires measured calibration and explicit
enable or user-configured automatic workspace entry. No EEPROM, homing, ID or
PID writes are performed.
"""
import json
from copy import copy
from pathlib import Path
import threading
from .timing import clock as time
import numpy as np

from .commission import RawCalibration,Commissioning
from .simulation import SimulationArm
from .motion import PHYSICAL_MOTION as MOTION,MOTION_PROFILE_NAME
from .hardware import transact,inspect_motors
from .temperature import ThermalValidator,diagnostic
from .gripper_contact import GripperContactGuard


class SerialBus:
    def __init__(self):
        from scripts.pico_hardware_check import load_sdk,discover_motor_bus,_supply_from_feedback
        found=discover_motor_bus();board=found['port']
        Port,Reader=load_sdk();self.port=Port(board.device)
        try:
            if not self.port.openPort():raise RuntimeError('无法打开控制板')
            self.reader=Reader(self.port);self.packet=self.reader._packet
            report=inspect_motors(self.reader)
            if not report['feedback_complete']:raise ValueError('六路电机反馈不完整')
            registers=[m['registers'] for m in report['motors']]
            if any(r['model']['value_raw']!=777 or r['operating_mode']['value_raw']!=0 for r in registers):
                raise ValueError('当前仅支持已核对的型号字777和单圈位置模式0')
            self.supply_version=_supply_from_feedback(report)
            self.signature=dict(board=found['signature']['board'],models=[r['model']['value_raw'] for r in registers],
                                offsets=[r['homing_offset']['value_raw'] for r in registers])
            self.raw_limits=[[r['min_position']['value_raw'],r['max_position']['value_raw']] for r in registers]
            if self.signature!=found['signature'] or self.raw_limits!=found['raw_limits']:
                raise ValueError('控制板/电机配置在只读识别后发生变化；请重新启动检查')
            self.name=board.device
        except Exception:
            if self.port.is_open:self.port.closePort()
            raise

    def read(self):
        raw=[];voltage=[];temp=[];torque=[]
        supply=getattr(self,'supply_version','12V')
        if supply not in ('12V','7.4V'):raise ValueError('供电版本必须为12V或7.4V')
        low,high=(90,126) if supply=='12V' else (40,74)
        # This cache only requests extra reads; it never authorizes motion or
        # replaces evidence in the independent temporal thermal validator.
        previous=list(getattr(self,'_temperature_last',[None]*6))
        readback={k:[] for k in ('speed_raw','load_raw','current_raw','status_raw','moving_raw','goal_raw','speed_limit_raw','acceleration_raw')}
        readback['bulk_temperature_raw']=[];readback['temperature_confirm_raw']=[]
        readback.update(bulk_voltage_raw=[],voltage_confirm_raw=[],voltage_source=[],
                        supply_version=supply,voltage_range_raw=[low,high])
        for i in range(1,7):
            # One contiguous SRAM read contains both goal/torque and feedback.
            # Preserve every field while avoiding a second USB round trip.
            block,c,e=self.reader.readTxRx(i,40,31)
            if c or e or len(block)!=31:raise RuntimeError(f'ID{i}反馈失败：通信{c} / 设备{e} / 长度{len(block)}')
            state=block[:8];data=block[16:31]
            position=int.from_bytes(bytes(data[:2]),'little')
            if not 0<=position<=4095:raise ValueError(f'ID{i}编码器越界/跨圈')
            measured_voltage=data[6];voltage_pair=None;voltage_source='bulk'
            if not low<=measured_voltage<=high:
                # Only two NEW reads of the voltage register may resolve a
                # bulk discrepancy. A failed or inconsistent pair leaves the
                # original out-of-range value selected for the supply guard.
                values=[];errors=[]
                for _ in range(2):
                    try:
                        value,c,e=self.reader.readTxRx(i,62,1)
                        values.append(value[0] if len(value)==1 else None)
                        errors.append(None if not c and not e and len(value)==1
                                      else f'通信{c} / 设备{e} / 长度{len(value)}')
                    except Exception as exc:
                        values.append(None);errors.append(str(exc))
                accepted=(not any(errors) and all(isinstance(v,(int,np.integer)) and low<=v<=high for v in values)
                          and abs(values[0]-values[1])<=2)
                voltage_pair=dict(values=values,errors=errors,accepted=accepted)
                voltage_source='independent_register_pair' if accepted else 'bulk_unconfirmed'
                if accepted:measured_voltage=max(values)
            readback['bulk_voltage_raw'].append(data[6]);readback['voltage_confirm_raw'].append(voltage_pair)
            readback['voltage_source'].append(voltage_source)
            temperature=data[7];confirmed=None
            if temperature>=55 or previous[i-1] is not None and abs(temperature-previous[i-1])>=ThermalValidator.jump:
                confirmations=[]
                for _ in range(2):
                    value,c,e=self.reader.readTxRx(i,63,1)
                    if c or e:raise RuntimeError(f'ID{i}温度复核失败：通信{c} / 设备{e}')
                    confirmations.append(value[0])
                confirmed=dict(values=confirmations)
                temperature=max(temperature,*confirmations)
                if all(0<=v<60 for v in confirmations) and abs(confirmations[0]-confirmations[1])<=2:
                    previous[i-1]=max(confirmations)
            elif 0<=temperature<55:previous[i-1]=temperature
            readback['bulk_temperature_raw'].append(data[7]);readback['temperature_confirm_raw'].append(confirmed)
            raw.append(position);voltage.append(measured_voltage);temp.append(temperature);torque.append(state[0])
            for key,value in dict(speed_raw=int.from_bytes(bytes(data[2:4]),'little'),
                load_raw=int.from_bytes(bytes(data[4:6]),'little'),current_raw=int.from_bytes(bytes(data[13:15]),'little'),
                status_raw=data[9],moving_raw=data[10],goal_raw=int.from_bytes(bytes(state[2:4]),'little'),
                speed_limit_raw=int.from_bytes(bytes(state[6:8]),'little'),acceleration_raw=state[1]).items():readback[key].append(value)
        self._temperature_last=previous
        return dict(raw=raw,voltage_raw=voltage,temperature=temp,torque=torque,readback=readback)

    def _write(self,i,address,data):
        # Only two SRAM operations, never an arbitrary register-write API.
        if not 1<=i<=6 or (address,len(data)) not in ((41,7),(40,1)):
            raise ValueError('禁止此硬件写入')
        request=[0,0,i,len(data)+3,3,address]+list(data)+[0]
        response,c,e=transact(self.packet,request)
        if c or e or response is None or len(response)!=6 or response[2]!=i or (sum(response[2:])&255)!=255:
            raise RuntimeError(f'ID{i}写入未确认：通信{c} / 设备{e}')

    def goal(self,i,value):
        value=int(value)
        if not 0<=value<=4095:raise ValueError('编码器目标越界')
        # Leave hardware speed headroom above the bounded software trajectory.
        # These are SRAM ceilings, not measured motor gains or PID changes.
        speed=MOTION.raw_motor_speed
        self._write(i,41,[MOTION.raw_motor_acceleration,value&255,value>>8,0,0,speed&255,speed>>8])

    def enable(self,i):self._write(i,40,[1])
    def close(self):self.port.closePort()


class MotorWorker:
    def __init__(self,bus):
        self.bus=bus;self.enabled=False;self.fault=None;self.generation=0
        self.pending=None;self.enable_request=None;self.cancel=False;self.active=None
        self.latch_request=None
        self._queue_revision=0
        self.feedback=None;self.last_sent=None;self.done=threading.Event();self.thread=None
        self._wire_clock=time.perf_counter;self._wire_wait=self.done.wait;self.last_goal_started_at=[None]*6;self.last_goal_values=[None]*6
        self.write_wait_ms=0.
        self._state_lock=threading.Lock()
        self.enabling=False;self.holding=False;self.enabled_guard=None
        self.rejected_target=None;self.rejected_feedback=None;self.cycle_ms=None;self.read_ms=None;self.hold_epoch=0;self.last_error=None
        self.thermal=ThermalValidator();self.thermal_pending=False
        self.thermal_diagnostics=self.thermal.diagnostics;self._thermal_holding=False
        self.thermal_transition=None;self.thermal_recovered_epoch=None
        self.gripper_calibration=None;self.gripper_guard=GripperContactGuard()

    def start(self):
        self.thread=threading.Thread(target=self._run,daemon=True,name='so101-serial')
        self.thread.start()

    def snapshot(self):return self.feedback
    def configure_gripper(self,closed=None,opened=None):
        # Configuration has no I/O. The serial owner applies it on its cycle.
        with self._state_lock:
            self.gripper_calibration=None if closed is None or opened is None else (closed,opened)
    def request_enable(self,guard):
        if self.thermal.pending or self.thermal.fault_axis is not None:
            raise ValueError('温度反馈待确认，等待新鲜测量后接管')
        with self._state_lock:
            self.enable_request=(self.generation,guard);self.holding=False;self._queue_revision+=1
    def submit(self,target,deadline,guard,*,gripper_opening=False):
        with self._state_lock:
            self.pending=(self.generation,np.array(target,int).copy(),deadline,guard,bool(gripper_opening));self.holding=False;self.latch_request=None
            self._queue_revision+=1

    def latch_hold(self,target,guard,*,tolerance_counts=None):
        # Only an already written, verified goal can be latched. No motor write
        # occurs here, and ordinary stop/disable cancels this request.
        with self._state_lock:
            tolerance=MOTION.stationary_tolerance_counts if tolerance_counts is None else tolerance_counts
            if (type(tolerance) is not int or not MOTION.stationary_tolerance_counts<=tolerance<=min(48,MOTION.raw_tracking_limit_counts)):
                raise ValueError('保持反馈容差无效')
            self._queue_revision+=1
            self.latch_request=(self.generation,np.asarray(target,int).copy(),guard,self._queue_revision,tolerance)
    def _cancel_locked(self):
        # A continuous hold latches its first measured position. Re-sampling
        # every neutral XR frame would ratchet the goal along with load sag.
        self.cancel=self.cancel or not self.holding
        self.holding=True
        self.generation+=1;self.pending=None;self.enable_request=None;self.active=None
        self.latch_request=None
    def hold(self):
        with self._state_lock:self._cancel_locked()
    def disable(self):
        with self._state_lock:self._cancel_locked();self.enabled=False
    def begin_calibration(self):
        with self._state_lock:
            if self.enabled:raise ValueError('先退出电机接管，再进行被动标定')
            pending=self.enabling or self.enable_request is not None
            self._cancel_locked()
            if pending:raise ValueError('已取消待执行的电机接管；等待新反馈后再进行标定')

    def _hold_current(self,current,captured_at=None,*,invalidate=False):
        # Explicit controller hold/release already resets its plan. Only an
        # execution-side interruption must invalidate a still-running plan.
        if invalidate:self.hold_epoch+=1
        with self._state_lock:
            if self.pending is None and self.enable_request is None:self.holding=True
        current=current.copy();current[5]=self.gripper_guard.hold(current[5])
        for i,value in enumerate(current,1):
            if captured_at is not None and time.monotonic()-captured_at>.1:raise RuntimeError('保持所需反馈已过期')
            self._goal(i,int(value))
        self.last_sent=current.copy();self.active=None

    def _goal(self,i,value,*,gripper_guard=None):
        if i==6:value=(gripper_guard or self.gripper_guard).outgoing(value)
        self.last_goal_started_at[i-1]=self._wire_clock()
        self.last_goal_values[i-1]=int(value)
        self.bus.goal(i,value)

    def _thermal_hold(self,current,started):
        with self._state_lock:
            static_hold=self.holding and not self.cancel and self.active is None and self.last_sent is not None
            self.pending=None;self.enable_request=None;self.active=None;self.cancel=False
            self.latch_request=None
            if not self._thermal_holding:self.generation+=1
        if not self._thermal_holding:
            previous_epoch=self.hold_epoch
            # Publish the reason before its epoch becomes observable.
            self.thermal_transition=(previous_epoch,previous_epoch+1);self.thermal_recovered_epoch=None
            self.hold_epoch=previous_epoch+1
            # Rewriting an already latched goal to feedback would absorb the
            # load error on every thermal pulse and ratchet the arm downward.
            # A moving or merely queued hold still captures actual feedback.
            if self.enabled and not static_hold:self._hold_current(current,started)
            self._thermal_holding=True

    def _sample(self):
        captured=time.monotonic();began=time.perf_counter();data=self.bus.read()
        self.read_ms=(time.perf_counter()-began)*1000
        fields={k:np.asarray(data.get(k,[]),float) for k in ('raw','voltage_raw','temperature','torque')}
        rules=(('raw','编码器位置',lambda v:(v>=0)&(v<=4095)&(v==np.floor(v))),
               ('voltage_raw','电压原始值',lambda v:(v>=40)&(v<=132)),
               ('torque','扭矩状态',lambda v:np.isin(v,[0,1])))
        for name,label,valid in rules:
            values=fields[name];mask=np.isfinite(values)&valid(values)
            if values.shape!=(6,) or not np.all(mask):
                axis=int(np.flatnonzero(~mask)[0])+1 if values.shape==(6,) else None
                sample={k:[float(v) if np.isfinite(v) else None for v in a.ravel()[:16]] for k,a in fields.items()}
                detail=dict(field=name,axis=axis,values=sample[name],shape=list(values.shape),sample=sample,captured_at=captured,
                    readback=data.get('readback'))
                if self.rejected_feedback is None:self.rejected_feedback=detail
                where='' if axis is None else f'ID{axis}'
                value=sample[name] if axis is None else sample[name][axis-1]
                raise ValueError(f'{where}{label}异常：{value}')
        status=(data.get('readback') or {}).get('status_raw')
        if status is not None:
            status=np.asarray(status,float)
            if status.shape!=(6,) or not np.isfinite(status).all() or np.any(status!=0):
                raise ValueError('电机状态异常：'+str(diagnostic(status)))
        if self.enabled and np.any(fields['torque']!=1):raise ValueError('扭矩使能反馈丢失，接管已撤销')
        data=self.thermal.observe(data,captured,time.monotonic())
        self.thermal_pending=bool((self.thermal.pending or self._thermal_holding)
                                 and self.fault is None and self.thermal.fault_axis is None)
        if self.thermal.fault_axis is not None:
            axis=self.thermal.fault_axis
            if self.rejected_feedback is None:
                sample={k:diagnostic(v) for k,v in fields.items()}
                self.rejected_feedback=dict(field='temperature',axis=axis,values=sample['temperature'],
                    shape=list(fields['temperature'].shape),sample=sample,captured_at=captured,
                    readback=diagnostic(data.get('readback')))
            raise ValueError(f'ID{axis}温度异常：{self.thermal.fault_reason}')
        current=fields['raw'].astype(int)
        if time.monotonic()-captured>.1:raise RuntimeError('六轴反馈采集超过100ms')
        return data,current,captured

    def cycle(self):
        began=time.perf_counter();started=time.monotonic();feedback_ok=False
        self.write_wait_ms=0.
        try:
            data,current,started=self._sample()
            self.feedback=dict(**data,captured_at=started,received_at=time.monotonic())
            feedback_ok=True
            with self._state_lock:calibration=self.gripper_calibration
            self.gripper_guard.configure(*(calibration or (None,None)))
            self.gripper_guard.observe(current[5],started)
            if self.enabled and any(v!=1 for v in data['torque']):raise ValueError('扭矩使能反馈丢失，接管已撤销')
            if self.enabled and (self.enabled_guard is None or not self.enabled_guard()):
                raise ValueError('接管所需标定、供电或反馈条件已改变，接管已撤销')
            if self.thermal.pending:
                self._thermal_hold(current,started)
                return
            if self._thermal_holding:
                # No command queued during quarantine can survive recovery.
                with self._state_lock:
                    self.pending=None;self.enable_request=None;self.active=None;self.cancel=False
                if self.enabled and self.fault is None:self.thermal_recovered_epoch=self.hold_epoch
                self._thermal_holding=False
                self.thermal_pending=False
            with self._state_lock:
                request=self.enable_request;self.enable_request=None;self.enabling=request is not None
            if request:
                generation,guard=request
                if generation!=self.generation or not guard():raise ValueError('接管条件已改变')
                for i,value in enumerate(current,1):
                    if generation!=self.generation or not guard() or time.monotonic()-started>.1:
                        raise ValueError('接管已取消或反馈过期')
                    self._goal(i,int(value))
                    if generation!=self.generation or not guard():raise ValueError('扭矩使能前接管已取消')
                    self.bus.enable(i)
                # ACK alone does not prove torque is enabled. Read it back.
                confirmed,current,started=self._sample()
                if any(v!=1 for v in confirmed['torque']):
                    raise ValueError('六轴扭矩使能未读回确认')
                self.feedback=dict(**confirmed,captured_at=started,received_at=time.monotonic())
                with self._state_lock:
                    if generation!=self.generation or not guard():raise ValueError('接管完成前已取消')
                    self.enabled=True;self.enabled_guard=guard;self.fault=None;self.last_error=None;self.rejected_target=None;self.rejected_feedback=None;self.last_sent=np.asarray(self.last_goal_values,int)
                # This was an explicit enable with six ACKs and torque readback.
                # An unconfirmed thermal spike holds it, never re-enables later.
                self.thermal_pending=self.thermal.pending
                if self.thermal_pending:
                    self._thermal_hold(current,started);return
            with self._state_lock:latch=self.latch_request;self.latch_request=None
            if latch and self.enabled:
                generation,target,guard,revision,tolerance=latch
                matches=(target.shape==(6,) and self.last_sent is not None and np.array_equal(target,self.last_sent)
                         and np.max(abs(target-current))<=tolerance)
                valid=matches and guard()
                with self._state_lock:
                    if valid and generation==self.generation and revision==self._queue_revision and not self.cancel:
                        self.pending=None;self.active=None;self.holding=True
            with self._state_lock:cancel=self.cancel;self.cancel=False
            if cancel:
                if self.last_sent is not None:self._hold_current(current,started)
            with self._state_lock:item=self.pending;self.pending=None
            if item and self.enabled:
                generation,target,deadline,guard,gripper_opening=item
                now=time.monotonic()
                if generation!=self.generation or now>deadline or not guard():
                    self._hold_current(current,started,invalidate=generation==self.generation);return
                # Stage contact release with its command lease. A cancellation
                # before the gripper write must retain the established hold.
                gripper_candidate=copy(self.gripper_guard)
                target[5]=gripper_candidate.command(target[5],current[5],started,opening=gripper_opening)
                lead=abs(target-current)
                step=None if self.last_sent is None else abs(target-self.last_sent)
                if np.max(lead)>MOTION.raw_tracking_limit_counts or step is None:
                    self.rejected_target=dict(target_raw=target.tolist(),feedback_raw=current.tolist(),
                        last_sent_raw=None if self.last_sent is None else self.last_sent.tolist(),
                        max_lead_counts=int(np.max(lead)),max_step_counts=None if step is None else int(np.max(step)),
                        wire_rate_counts_s=MOTION.raw_command_rate)
                    if np.max(lead)>MOTION.raw_tracking_limit_counts:
                        axis=int(np.argmax(lead));raise ValueError(f'目标领先反馈过大：ID{axis+1}偏差{lead[axis]}计数（上限{MOTION.raw_tracking_limit_counts}）')
                    if step is None:raise ValueError('缺少上次电机目标，保持并退出接管')
                for i,value in enumerate(target,1):
                    if (generation!=self.generation or time.monotonic()>deadline or not guard()
                            or time.monotonic()-started>.1):
                        self._hold_current(current,started,invalidate=generation==self.generation);return
                    if int(value)==self.last_goal_values[i-1]:continue
                    previous=self.last_goal_started_at[i-1]
                    if previous is None or self.last_goal_values[i-1] is None:raise ValueError('缺少电机实际写入目标记录')
                    interval=abs(int(value)-self.last_goal_values[i-1])/MOTION.raw_command_rate
                    wait_started=self._wire_clock()
                    while previous is not None and self._wire_clock()-previous+1e-9<interval:
                        if (generation!=self.generation or time.monotonic()>deadline or not guard()
                                or time.monotonic()-started>.1 or self.done.is_set()):
                            self._hold_current(current,started,invalidate=generation==self.generation);return
                        remaining=interval-(self._wire_clock()-previous)
                        self._wire_wait(min(.002,remaining))
                    self.write_wait_ms+=(self._wire_clock()-wait_started)*1000
                    if (generation!=self.generation or time.monotonic()>deadline or not guard()
                            or time.monotonic()-started>.1 or self.done.is_set()):
                        self._hold_current(current,started,invalidate=generation==self.generation);return
                    self._goal(i,int(value),gripper_guard=gripper_candidate)
                    if i==6:self.gripper_guard=gripper_candidate
                self.gripper_guard=gripper_candidate
                self.last_sent=target.copy()
                self.active=(deadline,guard)
            elif self.enabled and self.active:
                deadline,guard=self.active
                if time.monotonic()>deadline or not guard():self._hold_current(current,started,invalidate=True)
            # Contact detection also runs between command frames. One fixed
            # relaxation write replaces the previous closing lead immediately.
            if (self.gripper_guard.contact_raw is not None and self.enabled and self.last_sent is not None
                    and self.last_sent[5]!=self.gripper_guard.contact_raw):
                previous=self.last_goal_started_at[5]
                interval=abs(self.gripper_guard.contact_raw-self.last_goal_values[5])/MOTION.raw_command_rate
                wait_started=self._wire_clock()
                while previous is not None and self._wire_clock()-previous+1e-9<interval:
                    if self.done.is_set() or self.cancel or not self.enabled or time.monotonic()-started>.1:return
                    self._wire_wait(min(.002,interval-(self._wire_clock()-previous)))
                self.write_wait_ms+=(self._wire_clock()-wait_started)*1000
                if self.done.is_set() or self.cancel or not self.enabled or time.monotonic()-started>.1:return
                self._goal(6,self.gripper_guard.contact_raw)
                self.last_sent=self.last_sent.copy();self.last_sent[5]=self.gripper_guard.contact_raw
        except Exception as exc:
            hold_failed=self.last_sent is not None
            if isinstance(exc,ValueError) and feedback_ok and self.last_sent is not None:
                try:self._hold_current(current,started,invalidate=True);hold_failed=False
                except Exception as hold_error:exc=RuntimeError(str(exc)+'；保持未确认：'+str(hold_error))
            with self._state_lock:
                self.enabled=False;self.last_error=str(exc)
                self.thermal_pending=False
                if self.fault is None:self.fault=str(exc)
                self.pending=None;self.enable_request=None;self.active=None
                self.latch_request=None
                self.generation+=1
                if hold_failed:self.cancel=True
            # A failed bus cannot guarantee a final hold. Never auto-resume.
        finally:
            with self._state_lock:self.enabling=False
            self.cycle_ms=(time.perf_counter()-began)*1000

    def _run(self):
        while not self.done.is_set():
            start=time.monotonic();self.cycle();self.done.wait(max(.001,MOTION.motor_period-(time.monotonic()-start)))

    def close(self):
        self.disable()
        # Let the owner thread process cancellation/hold before shutting it
        # down. Never close a serial handle concurrently with its owner.
        until=time.monotonic()+.35
        while self.thread and self.thread.is_alive() and self.cancel and time.monotonic()<until:
            time.sleep(.005)
        self.done.set()
        if self.thread:self.thread.join(timeout=2)
        if self.thread and self.thread.is_alive():raise RuntimeError('串口线程未退出；保持句柄归属，未跨线程关闭')
        self.bus.close()


class PhysicalArm:
    kind='so101_real'
    # Standard CAD zero reaches x≈0.381m. Physical mode has its own envelope;
    # it remains intersected with measured joint limits. Autonomous entry is
    # enabled only by the user's bound control-reference configuration.
    work_min=np.array([.06,-.28,.04]);work_max=np.array([.40,.28,.40])
    def __init__(self,profile_path=None,bus=None,reference_path=None):
        use_default_reference=profile_path is None and bus is None
        from mujoco_pick_place.so101.kinematics import ROOT
        self.geometry=SimulationArm();self.model=self.geometry.model;self.data=self.geometry.data
        self.kinematics=self.geometry.kinematics;self.model_limits=self.geometry.limits.copy();self.limits=self.model_limits.copy()
        self.runtime_dir=ROOT/'artifacts/pico-teleop'
        self.profile_path=Path(profile_path or self.runtime_dir/'imported-calibration.json')
        self.reference_path=Path(reference_path or self.runtime_dir/'control-reference.json')
        self.worker=MotorWorker(bus or SerialBus());self.commission=Commissioning()
        self.confirmed_supply=getattr(self.worker.bus,'supply_version',None)
        self.profile=None;self.calibration_error=None;self.q=np.r_[np.zeros(5),.5];self.feedback_valid=False
        self.preview_signs=np.ones(5);self.reference_view=True
        self.commit_guard=lambda:False;self.last_observed=None;self.ready=False
        if self.profile_path.exists():
            try:
                profile_data=json.loads(self.profile_path.read_text(encoding='utf-8'))
                if profile_data.get('is_virtual') and not getattr(self.worker.bus,'is_virtual',False):raise ValueError('虚拟标定禁止用于真实电机')
                self.profile=RawCalibration(profile_data,self.worker.bus.signature,self.model_limits)
            except Exception as exc:self.calibration_error=str(exc)
        if self.profile:self.limits=self.profile.limits.copy()
        if self.profile:self.worker.configure_gripper(self.profile.closed,self.profile.open)
        self.kinematics.limits=self.limits.copy()
        if self.profile:self.reference_view=False
        self.control_reference=None;self.control_reference_error=None
        if use_default_reference or reference_path is not None:
            from .reference import ControlReference
            import hashlib
            path=self.reference_path
            if path.exists():
                try:
                    if self.profile is None:raise ValueError('遥操作参考需要有效的舵机标定')
                    self.control_reference=ControlReference.load(path,self.worker.bus.signature,
                        hashlib.sha256(self.profile_path.read_bytes()).hexdigest())
                    saved=self.control_reference.data
                    measured=self.profile.decode(saved['robot_raw'],require_bounds=False)
                    if abs(measured[5]-float(saved['gripper_neutral']))>1e-6:
                        raise ValueError('遥操作参考的夹爪开度与原始计数不一致')
                    saved_rotation=self.kinematics.fk(measured[:5])[1]
                    if not np.allclose(saved_rotation,np.asarray(saved['robot_rotation']),atol=1e-6):
                        raise ValueError('遥操作参考朝向与原始关节计数不一致')
                except Exception as exc:
                    self.control_reference=None;self.control_reference_error=str(exc)
        self.coupled_pose_solver=None
        if self.control_reference and self.control_reference.data.get('pose_policy')=='balanced':
            # Import and allocate before serial/control loops begin, not on a
            # deadline-sensitive first hand movement.
            from .mink_pose import CoupledPoseIK
            self.coupled_pose_solver=CoupledPoseIK(self.model,self.kinematics.site)
        self.workspace_entry=None
        if self.control_reference and self.control_reference.data.get('workspace_auto_entry',False):
            from .workspace_entry import WorkspaceEntry
            margin=float(self.control_reference.data.get('workspace_margin_m',.015))
            self.command_work_min=self.work_min+margin;self.command_work_max=self.work_max-margin
            ready_margin=float(self.control_reference.data.get('workspace_ready_margin_m',margin))
            entry_margin=float(self.control_reference.data.get('workspace_entry_margin_m',max(margin+.010,ready_margin+.005)))
            lead=self.control_reference.data.get('workspace_entry_lead_counts',24)
            self.workspace_entry=WorkspaceEntry(self,margin=ready_margin,entry_margin=entry_margin,lead_counts=lead)
        self.worker.start()

    def poll(self):
        snap=self.worker.snapshot();self.feedback_valid=False
        if snap and time.monotonic()-snap['captured_at']<=.1:
            try:
                if snap['captured_at']!=self.last_observed:
                    self.commission.observe(snap['raw']);self.last_observed=snap['captured_at']
                if self.profile:
                    self.q=self.profile.decode(snap['raw'],require_bounds=False);self.feedback_valid=True;self.calibration_error=None
                elif self.commission.zero is not None and not self.reference_view:
                    from .commission import TICK
                    self.q=np.r_[(np.asarray(snap['raw'][:5])-self.commission.zero)*self.preview_signs*TICK,.5]
            except ValueError as exc:self.calibration_error=str(exc);self.worker.disable()
        if self.reference_view:self.feedback_valid=False
        feedback_in_range=False
        if self.profile and self.feedback_valid and snap:
            try:self.profile.validate_feedback(snap['raw']);feedback_in_range=True
            except ValueError:pass
        self.ready=bool(self.profile and self.feedback_valid and self.worker.enabled and not self.worker.fault
                        and snap and all(v==1 for v in snap['torque'])
                        and feedback_in_range)
        # Before calibration this is explicitly a zero-pose reference, not an
        # observed physical pose. UI/HUD expose that distinction.
        display=np.r_[np.zeros(5),.5] if self.reference_view else self.q
        self.geometry.data.qpos[:5]=display[:5];self.geometry._set_gripper(display[5])
        import mujoco
        mujoco.mj_forward(self.model,self.data)

    def read(self):return np.r_[np.zeros(5),.5] if self.reference_view else self.q.copy()
    def command_reference(self,q):
        return self.profile.project_command(q) if self.profile else np.asarray(q,float).copy()
    def points(self):return self.geometry.points()
    def hold(self):self.worker.hold()

    @property
    def gripper_target(self):
        if self.profile is None or self.worker.last_sent is None:return None
        return float(np.clip((self.worker.last_sent[5]-self.profile.closed)/(self.profile.open-self.profile.closed),0,1))

    @property
    def gripper_contact(self):return self.worker.gripper_guard.diagnostics

    @property
    def held_reference(self):
        worker=self.worker
        with worker._state_lock:
            if (self.profile is None or not worker.holding or worker.cancel or worker.pending is not None
                    or worker.active is not None or worker.last_sent is None):return None
            raw=worker.last_sent.copy()
        return self.profile.decode(raw,require_bounds=False)

    @property
    def gripper_rate(self):
        return MOTION.gripper_speed_rad_s/(abs(self.profile.open-self.profile.closed)*self.profile.tick) if self.profile else .05

    @property
    def motion_epoch(self):return self.worker.hold_epoch

    @property
    def thermal_pending(self):return self.worker.thermal_pending

    @property
    def thermal_transition(self):return self.worker.thermal_transition

    @property
    def thermal_recovered_epoch(self):return self.worker.thermal_recovered_epoch

    @property
    def tracking_lead(self):
        # Planner budget stays below the independent worker guard. The measured
        # elbow static error reaches 14 counts; leave room for it and braking,
        # then separate margins for quantization and in-flight writes.
        if not self.profile:return None
        return np.r_[np.full(5,MOTION.tracking_lead_counts*self.profile.tick),MOTION.tracking_lead_counts/abs(self.profile.open-self.profile.closed)]

    def command(self,q,*,expected_motion_epoch=None,gripper_opening=False):
        if not self.ready:raise ValueError('真机未就绪/反馈无效')
        raw=self.profile.encode(q)
        epoch=self.worker.hold_epoch if expected_motion_epoch is None else expected_motion_epoch
        profile=self.profile;checked=[None,False]
        target_q=profile.decode(raw)[:5]
        if self.workspace_entry is not None:
            from .motion_errors import MotionConstraintError
            p=self.kinematics.fk(target_q)[0]
            if np.any(p<self.work_min) or np.any(p>self.work_max):
                raise MotionConstraintError('电机目标受到工作区边界限制')
        def guard():
            if self.worker.hold_epoch!=epoch or self.profile is not profile or not self.commit_guard():return False
            if self.workspace_entry is None:return True
            snap=self.worker.snapshot()
            if not snap or time.monotonic()-snap['captured_at']>.1:return False
            key=tuple(snap['raw'])
            if checked[0]!=key:
                actual=profile.decode(snap['raw'],require_bounds=False)[:5]
                kin=self.workspace_entry._check_kinematics
                points=[kin.fk(actual+t*(target_q-actual))[0] for t in (0.,.25,.5,.75,1.)]
                checked[:]=[key,all(np.all(p>=self.work_min) and np.all(p<=self.work_max) for p in points)]
            return checked[1]
        self.worker.submit(raw,time.monotonic()+.08,guard,gripper_opening=gripper_opening)

    def hardware_status(self):
        snap=self.worker.snapshot()
        outside=[];contact=[]
        if self.profile and self.feedback_valid:
            from mujoco_pick_place.so101.calibration import JOINT_NAMES
            tolerance=self.profile.feedback_margin_counts*self.profile.tick
            outside=[name for i,name in enumerate(JOINT_NAMES[:5]) if not self.limits[i,0]-tolerance<=self.q[i]<=self.limits[i,1]+tolerance]
            contact=[name for i,name in enumerate(JOINT_NAMES[:5]) if not self.limits[i,0]<=self.q[i]<=self.limits[i,1]]
        return dict(port=getattr(self.worker.bus,'name','test'),enabled=self.worker.enabled,ready=self.ready,
                    is_virtual=bool(getattr(self.worker.bus,'is_virtual',False)),
                    calibrated=self.profile is not None,feedback_valid=self.feedback_valid,
                    gripper_contact=self.gripper_contact,gripper_target=self.gripper_target,
                    confirmed_supply=self.confirmed_supply,
                    feedback_age_ms=None if not snap else round((time.monotonic()-snap['captured_at'])*1000,1),
                    raw=None if not snap else snap['raw'],voltage_raw=None if not snap else snap['voltage_raw'],
                    temperature=None if not snap else snap['temperature'],torque=None if not snap else snap['torque'],
                    validated_temperature=None if not snap else snap.get('validated_temperature'),
                    temperature_age_ms=None if not snap else [None if t is None else round((time.monotonic()-t)*1000,1)
                        for t in snap.get('temperature_validated_at',[None]*6)],
                    fault=self.worker.fault,calibration_error=self.calibration_error,
                    outside_joints=outside,limit_contact_joints=contact,
                    feedback_margin_counts=None if not self.profile else self.profile.feedback_margin_counts,
                    limits_deg=np.degrees(self.limits).tolist(),
                    workspace_min=self.work_min.tolist(),workspace_max=self.work_max.tolist(),
                    command_workspace_min=getattr(self,'command_work_min',self.work_min).tolist(),
                    command_workspace_max=getattr(self,'command_work_max',self.work_max).tolist(),
                    workspace_preparation=None if self.workspace_entry is None else self.workspace_entry.status(),
                    calibration_source=None if not self.profile else self.profile.data.get('source_file','手动标定'),
                    pose_source='zero_reference' if self.reference_view else 'feedback' if self.feedback_valid else 'calibration_preview',
                    transport=dict(cycle_ms=self.worker.cycle_ms,read_ms=self.worker.read_ms,
                        write_wait_ms=self.worker.write_wait_ms,
                        last_error=self.worker.last_error,
                        hold_epoch=self.worker.hold_epoch,
                        thermal_pending=self.worker.thermal_pending,
                        temperature_confirmation=self.worker.thermal_diagnostics,
                        last_goal_raw=None if self.worker.last_sent is None else self.worker.last_sent.tolist(),
                        rejected_target=self.worker.rejected_target,rejected_feedback=self.worker.rejected_feedback),
                    motion_profile=dict(vars(MOTION)),motion_profile_name=MOTION_PROFILE_NAME,
                    readback=None if not snap else snap.get('readback'),
                    commission=self.commission.status(),signature=self.worker.bus.signature)

    def _invalidate_control_reference(self):
        if self.control_reference is not None:
            self.control_reference.deactivate()
            self.control_reference_error='舵机标定已改变，旧遥操作参考已禁用；请重新记录机械臂参考'

    def _import_lerobot_calibration(self,message,snap):
        from .commission import import_lerobot
        from .reference import ControlReference
        from mujoco_pick_place.so101.calibration import JOINT_NAMES
        import hashlib
        payload=message.get('calibration')
        if not isinstance(payload,dict):raise ValueError('请选择有效的 LeRobot JSON 标定文件')
        rows=payload.get('calibration',payload)
        if not isinstance(rows,dict) or set(rows)!=set(JOINT_NAMES):
            raise ValueError('标定文件必须包含六个标准 SO101 关节')
        data=import_lerobot(rows,self.worker.bus.signature,self.worker.bus.raw_limits,self.worker.bus.supply_version)
        data['source_file']='LeRobot 标定文件（网页导入）'
        profile=RawCalibration(data,self.worker.bus.signature,self.model_limits)
        profile.validate_feedback(snap['raw'])
        q=profile.decode(snap['raw'],require_bounds=False)
        rotation=self.kinematics.fk(q[:5])[1]
        self.runtime_dir.mkdir(parents=True,exist_ok=True)
        temporary=self.profile_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        temporary.replace(self.profile_path)
        digest=hashlib.sha256(self.profile_path.read_bytes()).hexdigest()
        reference_data=dict(schema=1,signature=self.worker.bus.signature,calibration_sha256=digest,
            robot_raw=[int(v) for v in snap['raw']],robot_q_rad=q[:5].tolist(),robot_rotation=rotation.tolist(),
            gripper_neutral=float(q[5]),hand_reference=None,orientation_mapping='pointing_roll',
            wrist_offset_m=[0,0,0],pose_policy='split_roll',pose_orientation_length_m=.2,
            workspace_auto_entry=True,workspace_margin_m=.015,workspace_ready_margin_m=.04,
            workspace_entry_margin_m=.06,workspace_entry_lead_counts=80,tool_roll_adjustment_deg=90,
            gripper_mode='toggle',captured_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
        reference=ControlReference(reference_data,self.reference_path)
        ref_tmp=self.reference_path.with_suffix(self.reference_path.suffix+'.tmp')
        ref_tmp.write_text(json.dumps(reference_data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        ref_tmp.replace(self.reference_path)
        self.profile=profile;self.worker.configure_gripper(profile.closed,profile.open)
        self.confirmed_supply=self.worker.bus.supply_version
        self.limits=profile.limits.copy();self.kinematics.limits=self.limits.copy()
        self.control_reference=reference;self.control_reference_error=None
        self.reference_view=False;self.feedback_valid=True;self.q=q.copy();self.calibration_error=None
        self.commission.recording=False

    def control(self,action,message):
        self.poll()
        if action=='hardware_disable':self.worker.disable();return
        if action=='hardware_enable':
            if self.worker.thermal.pending or self.worker.thermal.fault_axis is not None:
                raise ValueError('温度反馈待确认，等待新鲜测量后接管')
            if not self.profile or not self.feedback_valid:raise ValueError('先完成标定，等待新鲜有效反馈')
            if message.get('workcell_verified') is not True:raise ValueError('请确认当前固定、空载、清空的工位')
            profile=self.profile;supply=profile.data['supply_version'];low,high=(90,126) if supply=='12V' else (40,74)
            self.worker.bus.supply_version=supply
            def validate_enable():
                s=self.worker.snapshot()
                if self.profile is not profile:raise ValueError('标定已改变，请重新检查后接管')
                if not s or time.monotonic()-s['captured_at']>.1:raise ValueError('电机反馈已过期，等待新鲜反馈后接管')
                if not low<=min(s['voltage_raw'])<=max(s['voltage_raw'])<=high:
                    axis=next(i+1 for i,v in enumerate(s['voltage_raw']) if not low<=v<=high)
                    value=s['voltage_raw'][axis-1]
                    if self.worker.rejected_feedback is None:
                        self.worker.rejected_feedback=dict(field='voltage_raw',axis=axis,
                            values=diagnostic(s['voltage_raw']),shape=[6],supply_version=supply,
                            expected_range_raw=[low,high],captured_at=s['captured_at'],
                            sample=diagnostic({k:s.get(k) for k in ('raw','voltage_raw','temperature','torque')}),
                            readback=diagnostic(s.get('readback')))
                    raise ValueError(f'ID{axis}实测电压原始值{value}与确认的{supply}供电版本不符（范围{low}..{high}）')
                try:profile.validate_feedback(s['raw'])
                except ValueError as exc:raise ValueError('当前关节或夹爪反馈超出允许范围：'+str(exc)) from exc
            def guard():
                validate_enable();return True
            validate_enable()
            self.worker.request_enable(guard);return
        if action=='cal_import':
            self.worker.begin_calibration()
            snap=self.worker.snapshot()
            if not snap or time.monotonic()-snap['captured_at']>.1:raise ValueError('等待六路新鲜反馈')
            if any(snap['torque']):raise ValueError('导入前请先退出接管并确认电机已卸力；导入不会启用电机')
            self._import_lerobot_calibration(message,snap)
            return
        self.worker.begin_calibration()
        snap=self.worker.snapshot()
        if not snap or time.monotonic()-snap['captured_at']>.1:raise ValueError('等待六路新鲜反馈')
        if any(snap['torque']):raise ValueError('被动标定需要各轴已卸力并得到支撑；程序不自动卸力')
        self.ready=False
        if action=='cal_signs':
            signs=np.asarray(message.get('signs'),float)
            if signs.shape!=(5,) or not np.isin(signs,[-1,1]).all():raise ValueError('方向须为五个 +1/-1')
            self.preview_signs=signs;self.reference_view=False
        elif action=='cal_reference':self.reference_view=True;self.q=np.r_[np.zeros(5),.5]
        elif action=='cal_zero':
            self.commission.capture_zero();self.reference_view=False;self.profile=None;self.feedback_valid=False
            self.worker.configure_gripper()
            self._invalidate_control_reference()
            self.limits=self.model_limits.copy();self.kinematics.limits=self.limits.copy();self.calibration_error=None
        elif action=='cal_range':self.commission.start_range()
        elif action=='cal_closed':self.commission.capture_gripper('closed')
        elif action=='cal_open':self.commission.capture_gripper('open')
        elif action=='cal_save':
            data=self.commission.build(self.worker.bus.signature,message.get('signs'),message.get('pose_verified'),
                    message.get('directions_verified'),message.get('workcell_verified'),message.get('supply_version'))
            profile=RawCalibration(data,self.worker.bus.signature,self.model_limits)
            profile.decode(snap['raw'])
            temporary=self.profile_path.with_suffix('.tmp');temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8');temporary.replace(self.profile_path)
            self._invalidate_control_reference()
            self.profile=profile;self.limits=profile.limits.copy();self.kinematics.limits=self.limits.copy();self.commission.recording=False;self.calibration_error=None;self.reference_view=False
            self.worker.configure_gripper(profile.closed,profile.open)
        else:raise ValueError('未知真机操作')

    def close(self):self.worker.close()
