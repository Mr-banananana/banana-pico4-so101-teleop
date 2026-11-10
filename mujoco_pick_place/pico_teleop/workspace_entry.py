"""Feedback-confirmed autonomous entry, separate from the XR motion lease."""
import numpy as np
from .timing import clock as time
from .motion import PHYSICAL_MOTION as MOTION
from .workspace_geometry import nearest_entry_plan


class WorkspaceEntry:
    def __init__(self,arm,margin=.015,entry_margin=.025,lead_counts=24):
        self.arm=arm;self.margin=margin;self.entry_margin=entry_margin
        self.lead_counts=min(lead_counts,MOTION.raw_tracking_limit_counts-4)
        self.hard_lead_counts=self.lead_counts+4
        self.settle_counts=(min(self.lead_counts,max(MOTION.stationary_tolerance_counts,32))
                            if lead_counts>24 else MOTION.stationary_tolerance_counts)
        self.planned_margin=None
        self.low=arm.work_min+margin;self.high=arm.work_max-margin
        self.state='waiting';self.message='等待反馈，准备自动进入工作区'
        self.running=False;self.cancelled=False;self.plan=None;self._token=None;self._last_goal_m=None
        self._enable_at=-float('inf');self._failed_raw=None;self._stable=[];self._owns_enable=False
        self._feedback_progress=0.;self._feedback_progress_at=None;self._feedback_raw=None;self._timeout_s=60.
        self.block_diagnostics=None
        self._check_kinematics=type(arm.kinematics)(arm.model,site=arm.kinematics.site)

    def status(self):
        return dict(state=self.state,running=self.running,message=self.message,
                    target_m=self._last_goal_m if self.plan is None else self.plan['goal_m'],
                    margin_m=self.margin,entry_margin_m=self.entry_margin,
                    planned_margin_m=self.planned_margin,
                    feedback_progress_m=self._feedback_progress,
                    timeout_s=self._timeout_s,
                    block_diagnostics=self.block_diagnostics,
                    lead_counts=self.lead_counts,settle_counts=self.settle_counts)

    def cancel(self,reason,latch=False):
        self.running=False;self._token=None;self.plan=None;self._stable=[]
        self.cancelled=self.cancelled or latch;self.state='cancelled' if self.cancelled else 'waiting'
        self.message=reason;self.arm.worker.hold();self._owns_enable=False

    def resume(self):
        self.cancelled=False;self._failed_raw=None;self.state='waiting'

    def _inside(self,p):return bool(np.all(p>=self.low) and np.all(p<=self.high))

    def _violation(self,p):return np.maximum(self.arm.work_min-p,0)+np.maximum(p-self.arm.work_max,0)

    def _healthy(self):
        a=self.arm;w=a.worker;s=w.snapshot()
        if getattr(a,'control_reference_error',None):return False
        if a.profile is None or not s or time.monotonic()-s['captured_at']>.1:return False
        if a.confirmed_supply!=a.profile.data['supply_version']:return False
        low,high=(90,126) if a.confirmed_supply=='12V' else (40,74)
        if min(s['voltage_raw'])<low or max(s['voltage_raw'])>high:return False
        try:a.profile.validate_feedback(s['raw'])
        except ValueError:return False
        return True

    def _block(self,reason):
        snap=self.arm.worker.snapshot()
        if snap:
            readback=snap.get('readback') or {}
            self.block_diagnostics=dict(reason=reason,captured_at=snap.get('captured_at'),
                raw=snap.get('raw'),goal_raw=readback.get('goal_raw'),torque=snap.get('torque'),
                voltage_raw=snap.get('voltage_raw'),temperature=snap.get('temperature'),
                current_raw=readback.get('current_raw'),load_raw=readback.get('load_raw'),
                moving_raw=readback.get('moving_raw'),speed_raw=readback.get('speed_raw'),
                status_raw=readback.get('status_raw'))
        if self.running or self._owns_enable:self.arm.worker.hold()
        self._owns_enable=False
        self.running=False;self._token=None;self.state='blocked';self.message=reason
        snap=self.arm.worker.snapshot();self._failed_raw=None if snap is None else tuple(snap['raw'])
        return False

    def _point(self,distance):
        i=min(int(np.searchsorted(self._arc,distance,side='right'))-1,len(self._path)-2)
        i=max(0,i);ratio=(distance-self._arc[i])/max(self._arc[i+1]-self._arc[i],1e-12)
        return self._path[i]+np.clip(ratio,0,1)*(self._path[i+1]-self._path[i])

    def _guard(self,token,raw):
        a=self.arm;w=a.worker
        if (token is not self._token or not self.running or self.cancelled or w.fault
                or not w.enabled or w.thermal_pending or w.hold_epoch!=self._epoch
                or a.profile is not self._profile or not self._healthy()):return False
        current=np.asarray(w.snapshot()['raw'])
        if np.max(abs(raw-current))>self.hard_lead_counts or abs(current[5]-self._gripper_raw)>4:return False
        measured=self._profile.decode(current,require_bounds=False)
        p=self._check_kinematics.fk(measured[:5])[0]
        return bool(np.all(self._violation(p)<=self._initial_violation+.004))

    def tick(self,now,dt,*,allow=True):
        a=self.arm;w=a.worker
        if self.cancelled:return False
        if not allow:
            if self.running or self._owns_enable:self.cancel('自动进入已暂停')
            return False
        if w.fault:return self._block('自动进入已停止：'+str(w.fault))
        if not np.isfinite(dt) or not 0<dt<=.15:
            if self.running or self._owns_enable:self.cancel('控制周期延迟，等待重新规划')
            return False
        if not self._healthy():
            if self.running or self._owns_enable:self.cancel('反馈或供电条件变化，暂停自动进入')
            self.message='等待标定、匹配供电和新鲜反馈';return False
        if w.thermal_pending:
            self.running=False;self._token=None;self.plan=None
            self.state='waiting';self.message='温度待核验，自动进入保持中';return True
        if not w.enabled:
            self.state='enabling';self.message='自动接管当前姿态，准备进入工作区'
            if now-self._enable_at>.25:
                a.control('hardware_enable',{'workcell_verified':True});self._enable_at=now
                self._owns_enable=True
            return True
        self._owns_enable=False
        current=a.read();p=a.kinematics.fk(current[:5])[0]
        if not self.running:
            if self._inside(p):self.state='ready';self.message='已在工作区内，可授权手柄';return False
            raw_tuple=tuple(w.snapshot()['raw'])
            if self.state=='blocked' and self._failed_raw is not None and np.max(abs(np.asarray(raw_tuple)-self._failed_raw))<=32:return False
            if self.state!='stabilizing':
                if w.active is not None or w.pending is not None:w.hold()
                self.state='stabilizing';self.message='先保持稳定，再自动进入工作区'
                self._before_plan=[];self._before_epoch=w.hold_epoch;self._before_at=now
                return True
            if w.hold_epoch!=self._before_epoch:return self._block('准备期间执行端已中断')
            stamp=w.snapshot()['captured_at']
            if not self._before_plan or stamp!=self._before_plan[-1][0]:self._before_plan.append((stamp,np.asarray(raw_tuple)))
            self._before_plan=[v for v in self._before_plan if stamp-v[0]<=.2]
            stable=(len(self._before_plan)>=4 and stamp-self._before_plan[0][0]>=.08
                    and np.max(np.ptp([v[1] for v in self._before_plan],axis=0))<=2)
            if not stable:
                if now-self._before_at>1.:return self._block('当前反馈尚未稳定，已保持而不执行准备动作')
                return True
            try:
                # Quantized feedback may sit a few counts beyond a soft joint
                # endpoint; project only the planning seed, never the readback.
                seed=a.command_reference(current)[:5]
                minimum=self.margin+.005
                margins=([self.entry_margin,(self.entry_margin+minimum)/2,minimum]
                         if self.entry_margin-self.margin>.015 else [self.entry_margin])
                last_error=None
                for candidate in margins:
                    try:
                        plan=nearest_entry_plan(a.kinematics,seed,a.limits,a.work_min,a.work_max,margin=candidate)
                        self.plan=plan;self.planned_margin=candidate;break
                    except ValueError as exc:last_error=exc
                else:raise last_error
            except ValueError as exc:return self._block('无法验证进入路径：'+str(exc))
            if w.hold_epoch!=self._before_epoch:return self._block('计算路径期间执行端已中断')
            self._last_goal_m=self.plan['goal_m']
            self._path=np.asarray(self.plan['q_path']);lengths=np.max(abs(np.diff(self._path,axis=0)),axis=1)
            self._arc=np.r_[0.,np.cumsum(lengths)];self._total=float(self._arc[-1])
            if self._total<=1e-9:return self._block('目标没有可执行的进入行程')
            derivative=np.max(np.linalg.norm(np.diff(np.asarray(self.plan['tcp_path']),axis=0),axis=1)/np.maximum(lengths,1e-12))
            self._speed=min(1.0,.9*MOTION.raw_command_rate*a.profile.tick,.12/max(derivative,1e-9))
            self._acc=min(6.,MOTION.joint_acceleration);self._distance=0.;self._velocity=0.
            self._profile=a.profile;self._epoch=w.hold_epoch;self._token=object();self.running=True
            self._gripper_raw=int(w.snapshot()['raw'][5]);self._grip=float(current[5])
            self._initial_violation=self._violation(p);self._started=now;self._stable=[];self._last_sample=None
            self._feedback_progress=0.;self._feedback_progress_at=now
            self._feedback_raw=np.asarray(w.snapshot()['raw'],int).copy()
            self.state='moving';self.message='正在自动进入最近可达工作区，请等待'
        if w.hold_epoch!=self._epoch:return self._block('执行端中断了进入轨迹，请检查反馈')
        if now-self._started>self._timeout_s:return self._block('自动进入超过60秒仍未完成，已保持；检查电机跟随和工作区可达性')
        if self.state=='latching':
            if w.holding and w.active is None and w.pending is None and w.latch_request is None:
                self.running=False;self._token=None;self.state='ready';self.message='已自动进入工作区，可授权手柄';return False
            return True
        reference=self._point(self._distance)
        # Measure progress from encoder feedback, not from targets submitted by
        # software. Stop with a useful cause if the arm cannot follow at all.
        segments=np.diff(self._path,axis=0)
        relative=current[None,:5]-self._path[:-1]
        fraction=np.clip(np.einsum('ij,ij->i',relative,segments)/
                          np.maximum(np.einsum('ij,ij->i',segments,segments),1e-12),0.,1.)
        projected=self._path[:-1]+fraction[:,None]*segments
        distances=np.einsum('ij,ij->i',projected-current[None,:5],projected-current[None,:5])
        nearest=int(np.argmin(distances))
        measured_progress=float(self._arc[nearest]+fraction[nearest]*(self._arc[nearest+1]-self._arc[nearest]))
        progress_advanced=measured_progress>self._feedback_progress+.0001
        if progress_advanced:
            self._feedback_progress=measured_progress;self._feedback_progress_at=now
        feedback_raw=np.asarray(w.snapshot()['raw'],int)
        encoder_changed=self._feedback_raw is not None and np.any(feedback_raw!=self._feedback_raw)
        if encoder_changed:
            self._feedback_raw=feedback_raw.copy();self._feedback_progress_at=now
        if (self._distance>.01 and self._total-self._feedback_progress>.01
                and self._feedback_progress_at is not None and now-self._feedback_progress_at>5.):
            return self._block('自动进入目标已下发，但编码器连续5秒无变化；停止前电机快照已保存')
        room=max(0.,self.lead_counts*a.profile.tick-float(np.max(abs(reference-current[:5]))))
        brake=lambda r:np.sqrt((self._acc*dt)**2+2*self._acc*max(0.,r))-self._acc*dt
        cap=min(self._speed,brake(self._total-self._distance),brake(room))
        self._velocity=min(cap,self._velocity+self._acc*dt)
        self._distance=min(self._total,self._distance+self._velocity*dt)
        command=np.r_[self._point(self._distance),self._grip];raw=self._profile.encode(command)
        target_p=a.kinematics.fk(self._profile.decode(raw)[:5])[0]
        if np.any(self._violation(target_p)>self._initial_violation+.0015):return self._block('量化目标超出预检进入路径')
        if np.max(abs(raw-np.asarray(w.snapshot()['raw'])))>self.hard_lead_counts:return self._block('进入目标领先反馈过多')
        token=self._token;guard=lambda:self._guard(token,raw)
        if self._distance>=self._total-1e-6 and self._inside(p):
            stamp=w.snapshot()['captured_at']
            if stamp!=self._last_sample:
                self._stable.append((stamp,np.asarray(w.snapshot()['raw']).copy()));self._last_sample=stamp
            self._stable=[v for v in self._stable if stamp-v[0]<=.25]
            if (len(self._stable)>=4 and stamp-self._stable[0][0]>=.16
                    and np.max(np.ptp([v[1] for v in self._stable],axis=0))<=2
                    and np.max(abs(raw-np.asarray(w.snapshot()['raw'])))<=self.settle_counts
                    and w.last_sent is not None and np.array_equal(w.last_sent,raw)):
                w.latch_hold(raw,guard,tolerance_counts=self.settle_counts);self.state='latching';return True
        w.submit(raw,time.monotonic()+.08,guard)
        return True
