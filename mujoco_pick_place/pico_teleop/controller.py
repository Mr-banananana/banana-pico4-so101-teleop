"""Simulation-only teleoperation state machine and bounded pose retargeting."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

from .protocol import ClockSync, parse_frame
from .motion import PHYSICAL_MOTION as MOTION,bounded_prediction,conditioned_trigger
from .intent import HandIntent
from .orientation import AbsoluteOrientation,XR_TO_ROBOT
from .motion_errors import MotionConstraintError
from .grasp import ToggleGrasp


WORK_MIN = np.array([.06, -.28, .025])
WORK_MAX = np.array([.36, .28, .40])


class TeleopController:
    def __init__(self, backend, *, allow_emulated=False):
        self.backend = backend
        self.motor_response=getattr(backend,'kind',None)=='servo_response_simulation'
        self.physical=getattr(backend,'kind',None)=='so101_real'
        self.physical_label='虚拟舵机' if getattr(getattr(getattr(backend,'worker',None),'bus',None),'is_virtual',False) else '真机'
        self.require_passthrough=self.physical and self.physical_label=='真机'
        self.work_min=np.asarray(getattr(backend,'work_min',WORK_MIN),float)
        self.work_max=np.asarray(getattr(backend,'work_max',WORK_MAX),float)
        self.allow_emulated=bool(allow_emulated) and getattr(backend,'kind',None) in ('kinematic_simulation','servo_response_simulation')
        self.session_id = None
        self.clock = ClockSync()
        self.latest = None
        self.last_seq = 0
        self.received_at = 0.
        self.armed = False
        self.stopped = False
        self.engaged = False
        self.mode = "pose" if self.physical and MOTION.orientation_tracking and callable(getattr(backend.kinematics,'ik_pose_step',None)) else "position"
        self.reason = "等待 PICO 连接"
        self.last_command = self._planning_reference(backend.read())
        self._motion_epoch=getattr(backend,'motion_epoch',None)
        self.requested_position = None
        self.operator_requested_position = None
        self.requested_rotation = None
        self.orientation = AbsoluteOrientation()
        self.control_reference=getattr(backend,'control_reference',None)
        reference_data=self.control_reference.data if self.control_reference else {}
        self.gripper_mode=reference_data.get('gripper_mode',getattr(backend,'gripper_mode','analog'))
        self.grasp=ToggleGrasp(reference_data.get('gripper_neutral',float(self.last_command[5])))
        self._idle_reference=None;self._idle_waiting=False
        self.pose_policy=reference_data.get('pose_policy','position_priority')
        self.roll=None;self.requested_roll=None
        self.pose_orientation_length=float(reference_data.get('pose_orientation_length_m',.2))
        self.coupled_pose_solver=getattr(backend,'coupled_pose_solver',None)
        if self.physical and self.pose_policy=='balanced' and self.coupled_pose_solver is None:
            from .mink_pose import CoupledPoseIK
            self.coupled_pose_solver=CoupledPoseIK(backend.kinematics.model,backend.kinematics.site)
        self._anchor = None
        self._release_pending = False
        self._arm_button = False
        self._arm_released = False
        self._unlock_held_since = None
        self.wall_clock = None
        self._physical_velocity=np.zeros(5)
        self.prediction_offset=np.zeros(3)
        self.tracking_limited=False
        self._tracking_wait=None
        self._geometry_wait=None
        self.limit_limited=False
        self.limit_buffer_events=0
        self._thermal_wait=False
        self.control_point=None
        self.constraint_pause=False;self.constraint_pause_events=0;self.last_motion_pause=None
        self._workspace_wait=False;self.workspace_limited=False

    def attach(self, session_id):
        self.revoke("新连接：先松开握持键和扳机，再授权")
        if self.control_reference:self.control_reference.deactivate()
        self.session_id = session_id
        self.clock = ClockSync()
        self.latest = None
        self.last_seq = 0
        self._arm_button = False
        self._arm_released = False

    def detach(self, session_id):
        if session_id == self.session_id:
            self.revoke("PICO 已断开，重新连接后需要重新授权")
            self.session_id = None
            self.latest = None

    def revoke(self, reason):
        entry=getattr(self.backend,'workspace_entry',None)
        preparing=(entry is not None and not getattr(entry,'cancelled',False)
                   and (entry.running or entry.state in ('enabling','stabilizing','latching')))
        # XR authorization and the explicitly configured startup preparation
        # are separate leases. stop() cancels preparation before reaching here;
        # a page refresh alone must not interrupt it on every invalid XR frame.
        independent_preparation=preparing and self.physical and not self.backend.hardware_status().get('fault')
        if (self.motor_response or self.physical) and not independent_preparation:self.backend.hold()
        self.armed = False
        self.grasp.reset(float(self.backend.read()[5]))
        self._idle_reference=None;self._idle_waiting=False
        self.engaged = False
        self._anchor = None
        self._release_pending = False
        self._arm_released = False
        self.reason = reason
        self._unlock_held_since = None
        self._physical_velocity[:]=0
        self.prediction_offset[:]=0
        self.tracking_limited=False
        self._tracking_wait=None
        self._geometry_wait=None
        self.limit_limited=False
        self.requested_rotation=None
        self.operator_requested_position=None
        self.orientation.reset()
        if self.roll is not None:self.roll.reset()
        self.requested_roll=None
        self._thermal_wait=False
        self.control_point=None
        self.constraint_pause=False
        self._workspace_wait=False;self.workspace_limited=False

    def stop(self, reason="软件停止已锁存"):
        entry=getattr(self.backend,'workspace_entry',None)
        if entry is not None:entry.cancel(reason,latch=True)
        self.revoke(reason)
        self.stopped = True

    def reset_stop(self):
        self.stopped = False
        entry=getattr(self.backend,'workspace_entry',None)
        if entry is not None:entry.resume()
        self.revoke("停止已解除；松开右 A、握持键与扳机，再短按 A 授权")

    def fresh(self, now):
        return (self.latest is not None and self.session_id is not None
                and self.clock.valid_time(self.latest.client_time_ms, now)
                and now-self.received_at <= .2)

    def _planning_reference(self,current):
        project=getattr(self.backend,'command_reference',None)
        return project(current) if self.physical and callable(project) else np.asarray(current,float).copy()

    def _command_box(self):
        return (np.maximum(self.work_min,getattr(self.backend,'command_work_min',self.work_min)),
                np.minimum(self.work_max,getattr(self.backend,'command_work_max',self.work_max)))

    def _absorb_joint_edge(self,hand,position):
        # Saturate positional intent only. Absolute orientation and the saved
        # trigger map retain their reference, and joint velocity is not reset.
        a=self._anchor;a['hand']=np.asarray(hand).copy();a['position']=np.asarray(position).copy()
        a['hand_velocity'][:]=0;a['hand_acceleration'][:]=0
        if a.get('intent') is not None:a['intent'].suppress_prediction()
        self.requested_position=np.asarray(position).copy();self.prediction_offset[:]=0
        self.limit_buffer_events+=1

    def arm(self, now):
        f = self.latest
        if self.physical:
            error=getattr(self.backend,'control_reference_error',None)
            if error:raise ValueError(error)
            if self.mode=='pose' and self.control_reference and self.control_reference.pending:
                raise ValueError('将手柄竖直、松开握持键和扳机，短按 A 只记录参考；之后再授权')
            if not self.backend.ready:raise ValueError('真机未就绪：先完成标定、接管电机并等待有效反馈')
            entry=getattr(self.backend,'workspace_entry',None)
            if entry is not None and entry.state!='ready':
                if entry.state=='blocked' and not self.stopped:entry.resume()
                raise ValueError('正在自动进入工作区，完成后即可授权手柄')
            p,_=self.backend.kinematics.fk(self.backend.read()[:5])
            if np.any(p<self.work_min) or np.any(p>self.work_max):
                if entry is not None:
                    self.revoke('正在自动进入工作区');entry.resume()
                    raise ValueError('正在自动将末端移入工作区，请稍候后授权')
                raise ValueError('实际末端不在真机工作区；需要先进入工作区')
        if self.stopped or not self.fresh(now) or not f.valid or not f.visible:
            raise ValueError("需要新鲜有效输入，且停止锁存已解除")
        if self.require_passthrough and f.mode!='immersive-ar':
            raise ValueError('真实机械臂需要透视遥操作输入；请退出 VR 仿真预览并进入 AR 透视')
        if f.grip > .1 or f.trigger > .05 or f.stop:
            raise ValueError("请先完全松开 grip 和 trigger")
        if self.motor_response and np.max(abs(self.backend.servos.velocity))>.05:
            raise ValueError('舵机仍在减速，请保持松开，稍后再短按 A')
        self.armed = True
        self._motion_epoch=getattr(self.backend,'motion_epoch',None)
        self.engaged = False
        self.tracking_limited=False;self._tracking_wait=None
        self._anchor = None
        self.last_command=self._planning_reference(self.backend.read())
        self.grasp.reset(self._gripper_reference(self.last_command[5]))
        self._idle_reference=None;self._idle_waiting=False
        self.reason = "已授权；按住右 grip 跟随，右 B 停止"

    def _refresh_control_reference(self):
        reference=getattr(self.backend,'control_reference',None) if self.physical else None
        if reference is None or reference is self.control_reference:return
        self.control_reference=reference;data=reference.data
        self.pose_policy=data.get('pose_policy',self.pose_policy)
        self.pose_orientation_length=float(data.get('pose_orientation_length_m',self.pose_orientation_length))
        self.gripper_mode=data.get('gripper_mode',self.gripper_mode)
        self.grasp=ToggleGrasp(float(data.get('gripper_neutral',self.last_command[5])))

    def capture_upright(self,now):
        f=self.latest
        error=getattr(self.backend,'control_reference_error',None)
        if error:raise ValueError(error)
        self._refresh_control_reference()
        if (not self.physical or self.control_reference is None or self.armed
                or not self.fresh(now) or not f.valid or not f.visible or f.emulated
                or f.mode!='immersive-ar' or f.grip>.1 or f.trigger>.05 or f.stop):
            raise ValueError('记录竖直参考需要未授权、透视追踪有效，并松开握持键、扳机和 B')
        self.control_reference.capture_hand(f.orientation,f.reference_space,f.space_epoch)
        self.orientation.set_alignment(self.control_reference.alignment())
        if self.pose_policy=='split_roll':self._configure_roll()
        self.reason='竖直参考已保存；本次未授予手柄控制，也未执行电机使能。接管后松开 A，再短按 A 授权'

    def set_mode(self, mode):
        if self.physical and mode=='pose' and not callable(getattr(self.backend.kinematics,'ik_pose_step',None)):
            raise ValueError('当前后端缺少受约束姿态 IK')
        if mode not in ("position", "pose") or self.armed:
            raise ValueError("只能在未授权时切换 position / pose")
        self.mode = mode
        self.requested_rotation=None
        self.orientation.reset()

    def ingest(self, payload, now):
        try:
            f = parse_frame(payload, self.session_id, self.last_seq)
            # A valid stop button is fail-safe even when the position stream
            # is estimated or late. These conditions must never grant motion,
            # but must not prevent latching an operator's stop request.
            if f.stop:self.stop()
            if f.emulated and not self.allow_emulated:
                raise ValueError("当前后端禁止估算位姿，仅明确启用的运动学仿真可使用")
            if self.latest and f.client_time_ms < self.latest.client_time_ms:
                raise ValueError("source time moved backwards")
            if not self.clock.valid_time(f.client_time_ms, now):
                raise ValueError("源时间未同步、过期或超前")
        except (ValueError, TypeError) as exc:
            self.revoke(f"输入被拒绝：{exc}")
            raise ValueError(str(exc)) from exc
        previous = self.latest
        if previous and f.emulated!=previous.emulated:
            self.revoke("追踪质量已改变，请松开 A 后重新授权，避免位姿跳变")
        if now-self.received_at > .2:
            self._unlock_held_since = None
        if previous and (f.space_epoch != previous.space_epoch
                         or f.reference_space != previous.reference_space):
            self.revoke("头显坐标原点已重置：跟随已暂停；松开握持键和扳机，再短按 A 重新授权")
            if self.control_reference:self.control_reference.deactivate()
        if previous and previous.grip >= .35 and f.grip < .35:
            # Preserve an edge even if a re-grip arrives before the next tick.
            self._release_pending = True
            if self.motor_response or self.physical:self.backend.hold()
        self.latest, self.last_seq, self.received_at = f, f.seq, now
        if self.require_passthrough and f.mode!='immersive-ar':
            self.revoke('真实机械臂需要透视输入；VR 仅供仿真预览，跟随已暂停')
        if f.stop:
            self.stop()
        elif not f.valid or not f.visible:
            self.revoke("追踪或应用可见性无效，需重新授权")
        if f.valid and f.visible and not f.arm:
            self._arm_released = True
        arm_edge = f.arm and not self._arm_button and self._arm_released
        self._arm_button = f.arm
        if self.stopped:
            # Recover only from a deliberate continuous, neutral A hold.
            # This clears the latch only; releasing and pressing A again is
            # still required before authorization can be granted.
            neutral=f.grip<=.1 and f.trigger<=.05 and not f.stop
            if f.valid and f.visible and f.arm and self._arm_released and neutral:
                if self._unlock_held_since is None:self._unlock_held_since=now
                held=now-self._unlock_held_since
                if held>=2.:self.reset_stop()
                else:self.reason=f"停止已锁存；保持右 A {max(0.,2.-held):.1f} 秒以解除（握持键、扳机与 B 保持松开）"
            else:
                self._unlock_held_since=None
            return
        if arm_edge and f.valid and f.visible and not self.armed:
            try:
                self._refresh_control_reference()
                if self.mode=='pose' and self.control_reference and self.control_reference.pending:
                    self.capture_upright(now)
                else:self.arm(now)
            except ValueError as exc:
                self.reason = str(exc)
        if self.armed and not self.stopped and f.valid and f.visible and self.gripper_mode=='toggle':
            self.grasp.update(f.trigger)

    def _engage(self, frame, q):
        self._idle_reference=None;self._idle_waiting=False
        self.workspace_limited=False
        self._geometry_wait=None
        self.limit_limited=False
        self.limit_buffer_events=0
        p, rotation = self.backend.kinematics.fk(q[:5])
        target_R=None
        if self._split_roll():
            if self.roll is None:self._configure_roll()
            self.requested_roll=self.roll.engage(frame.orientation,frame.seq,frame.precision)
        elif self.physical and (self.mode=='pose' or np.any(self._wrist_offset())):
            target_R=self.orientation.engage(frame.orientation,frame.seq,frame.precision)
        hand=self._control_point(frame.position)
        trigger=conditioned_trigger(frame.trigger) if self.physical else frame.trigger
        self._anchor = dict(hand=hand.copy(), position=p.copy(),
                            operator_hand=hand.copy(),operator_position=p.copy(),
                            intent=HandIntent(hand,MOTION,frame.seq,frame.precision) if self.physical and MOTION.stable_input else None,
                            filtered_hand=hand.copy(),accepted_hand=hand.copy(),
                            hand_velocity=np.zeros(3),hand_acceleration=np.zeros(3),
                            **({} if self.physical else dict(hand_R=Rotation.from_quat(frame.orientation).as_matrix(),tool_R=rotation.copy())),
                            precision=frame.precision,
                            gripper=float(q[5]), trigger=trigger,accepted_trigger=trigger)
        if self._split_roll():
            self._anchor['level_rotation']=self._horizontal_tool_rotation(rotation)
            self._anchor['level_joint']=float(q[4])
        self.engaged = True
        self.operator_requested_position=p.copy()
        self.prediction_offset[:]=0
        self.requested_rotation=(None if self._split_roll() else target_R
                                 if self.physical and self.mode=='pose' else rotation.copy() if self.mode=='pose' else None)
        self.tracking_limited=False;self._tracking_wait=None
        self._motion_epoch=getattr(self.backend,'motion_epoch',None)
        if self.physical:self._physical_velocity[:]=0
        self.reason = self.physical_label+"跟随中" if self.physical else "跟随中（仿真）"

    def _wrist_offset(self):
        if self._split_roll():return np.zeros(3)
        return self.control_reference.wrist_offset if self.physical and self.control_reference else np.zeros(3)

    def _split_roll(self):return self.physical and self.mode=='pose' and self.pose_policy=='split_roll'

    @staticmethod
    def _horizontal_tool_rotation(rotation):
        """Keep the calibrated tool-forward axis level while preserving its yaw and bank."""
        value=np.asarray(rotation,float)
        world_up=np.array([0.,0.,1.])
        forward=value[:,0]-world_up*float(value[:,0]@world_up)
        norm=float(np.linalg.norm(forward))
        if norm<1e-5:
            forward=np.array([1.,0.,0.])
        else:forward/=norm
        tool_up=value[:,2]-forward*float(value[:,2]@forward)
        norm=float(np.linalg.norm(tool_up))
        if norm<1e-5:
            tool_up=world_up-forward*float(world_up@forward)
            norm=float(np.linalg.norm(tool_up))
        tool_up/=norm
        tool_left=np.cross(tool_up,forward)
        tool_left/=np.linalg.norm(tool_left)
        tool_up=np.cross(forward,tool_left)
        return np.column_stack((forward,tool_left,tool_up))

    def _configure_roll(self):
        from .roll import AbsoluteRoll
        d=self.control_reference.data if self.control_reference else {}
        hand=(d.get('hand_reference') or {}).get('quaternion',[0,0,0,1])
        if self.control_reference and getattr(self.backend,'profile',None) is not None and 'robot_raw' in d:
            neutral=float(self.backend.profile.decode(d['robot_raw'],require_bounds=False)[4])
        else:neutral=float(d.get('robot_q_rad',self.backend.read()[:5])[4])
        self.roll=AbsoluteRoll(hand,neutral,tool_roll_adjustment_deg=float(d.get('tool_roll_adjustment_deg',0.)))

    def _control_point(self,position):
        offset=self._wrist_offset()
        self.control_point=np.asarray(position).copy()
        if np.any(offset):self.control_point+=self.orientation.hand_rotation@offset
        return self.control_point.copy()

    def _thermal_epoch_change(self):
        epoch=getattr(self.backend,'motion_epoch',None)
        return (getattr(self.backend,'thermal_transition',None)==(self._motion_epoch,epoch)
                and not self.backend.hardware_status().get('fault')
                and (self._thermal_wait or getattr(self.backend,'thermal_pending',False)
                     or getattr(self.backend,'thermal_recovered_epoch',None)==epoch))

    def _discard_thermal_plan(self,current):
        self._thermal_wait=True;self.engaged=False;self._anchor=None
        self.orientation.reset();self._physical_velocity[:]=0;self.prediction_offset[:]=0
        self.requested_rotation=None;self.requested_position=None;self.operator_requested_position=None
        self.last_command=self._planning_reference(current)
        self.tracking_limited=False;self.limit_limited=False;self._tracking_wait=None
        self.reason='温度单点待核验，保持中；有效反馈恢复后从当前手柄继续'

    def _recover_motion(self,current,reason,now):
        # Only typed planning outcomes enter here. Never clear a worker fault,
        # enable torque, replay an old target, or ratchet a held motor goal.
        commit_now=self.wall_clock() if self.wall_clock is not None else now
        if not self.armed or self.stopped:return
        if not self.fresh(commit_now) or commit_now-now>.15:
            self.revoke('规划恢复时输入或计算已过期，已撤权');return
        if not self.backend.ready or self.backend.hardware_status().get('fault'):
            self.revoke('规划期间真机反馈或接管条件已改变，已撤权');return
        if self._motion_epoch is not None and self.backend.motion_epoch!=self._motion_epoch:
            if self._thermal_epoch_change():self._discard_thermal_plan(current)
            else:self.revoke('电机执行端已暂停旧目标；松开握持键和扳机，再短按 A 重新授权')
            return
        if not self.constraint_pause:self.backend.hold()
        self.constraint_pause=True;self.constraint_pause_events+=1
        self.last_motion_pause=dict(reason=str(reason),sequence=self.last_seq,at=now)
        # Keep the clutch map and accepted input filters. A fresh unchanged
        # hand pose is still the desired pose; re-clutching here would silently
        # throw away part of the operator's displacement after every hiccup.
        if self._anchor is not None:
            self._anchor['hand_velocity'][:]=0;self._anchor['hand_acceleration'][:]=0
            if self._anchor.get('intent') is not None:self._anchor['intent'].suppress_prediction()
        self._physical_velocity[:]=0;self.prediction_offset[:]=0
        self.tracking_limited=False;self._tracking_wait=None;self._geometry_wait=None
        self.last_command=self._planning_reference(current)
        self.reason='目标暂受限，已保持授权；正在自动重新求解：'+str(reason)

    def _pace_physical(self,command,reference,current,now,braking_limited=False):
        lead=getattr(self.backend,'tracking_lead',None)
        self.tracking_limited=False
        if lead is None:return command
        delta=command-reference
        room=np.where(delta>=0,lead-(reference-current),lead+(reference-current))
        ratios=np.minimum(1.,np.maximum(0.,room)/np.maximum(abs(delta),1e-12))
        alpha=float(np.min(ratios[:5]))
        # Gripper pacing must not abruptly brake the five-joint TCP trajectory.
        command=reference+np.r_[np.full(5,alpha),ratios[5]]*delta
        lagging=bool(np.max(abs(command[:5]-current[:5])/lead[:5])>MOTION.stationary_tolerance_counts/MOTION.tracking_lead_counts+1e-8)
        self.tracking_limited=bool(braking_limited or min(alpha,ratios[5])<1.-1e-8)
        # A rate/geometry constraint is not evidence of a stalled motor.
        # Only a material command-feedback error starts the motor watchdog.
        if not lagging:
            self._tracking_wait=None
            return command
        # Delayed feedback paces the trajectory, rather than allowing the
        # planner to run into the worker's hard tracking-error protection.
        # Persistent lack of progress still latches a stop; no automatic retry.
        direction_now=(command[:5]-current[:5])/lead[:5]
        direction_now/=max(np.linalg.norm(direction_now),1e-12)
        if self._tracking_wait is None:
            direction=direction_now
            self._tracking_wait=(now,current.copy(),direction)
        since,baseline,direction=self._tracking_wait
        if np.dot(direction_now,direction)<.5:
            # Compare progress in the current target direction. A turn by
            # itself does not reset the no-progress timer on a blocked arm.
            baseline=current.copy();direction=direction_now
            self._tracking_wait=(since,baseline,direction)
        if np.dot((current[:5]-baseline[:5])/lead[:5],direction)>=2/MOTION.tracking_lead_counts:
            self._tracking_wait=(now,current.copy(),direction_now)
        elif now-since>=1.5:
            self.stop('电机反馈持续未跟上目标，停止已锁存；检查机械阻挡和反馈后再授权')
            return None
        return command

    def _gripper_reference(self,fallback):
        target=getattr(self.backend,'gripper_target',None)
        return float(fallback if target is None else target)

    def _idle_gripper(self,current,now,dt):
        """Move only ID6 against a single acknowledged five-axis hold goal."""
        if self._idle_reference is None:
            if not self._idle_waiting:
                self.backend.hold();self._idle_waiting=True
            if hasattr(self.backend,'held_reference'):
                reference=self.backend.held_reference
                if reference is None:return
            else:reference=self._planning_reference(current)
            self._idle_reference=np.asarray(reference,float).copy()
        command=self._idle_reference.copy()
        base=self._gripper_reference(self.last_command[5])
        command[5]=base+np.clip(self.grasp.target-base,-self.backend.gripper_rate*dt,self.backend.gripper_rate*dt)
        lead=getattr(self.backend,'tracking_lead',None)
        if lead is not None:command[5]=np.clip(command[5],current[5]-lead[5],current[5]+lead[5])
        commit_now=self.wall_clock() if self.wall_clock else now
        if not self.fresh(commit_now):self.revoke('夹爪输入已过期');return
        try:
            if self._motion_epoch is not None:
                self.backend.command(command,expected_motion_epoch=self._motion_epoch,
                                     gripper_opening=self.grasp.active and not self.grasp.closing)
            else:self.backend.command(command,gripper_opening=self.grasp.active and not self.grasp.closing)
            self.last_command=command.copy()
            self.reason=('夹爪保持当前开度；扣一下扳机抓取' if not self.grasp.active else
                         '夹爪抓取保持；再扣扳机张开' if self.grasp.closing else '夹爪张开；扣一下扳机抓取')
        except (ValueError,RuntimeError) as exc:self.revoke('夹爪保持暂停：'+str(exc))

    def _physical_velocity_bounds(self,reference,current,joint_limit,dt,now):
        """Bounds enter the Cartesian solve, rather than distorting its result."""
        acc=MOTION.joint_acceleration
        brake=lambda room:np.sqrt((acc*dt)**2+2*acc*np.maximum(room,0.))-acc*dt
        positive=np.minimum(joint_limit,brake(self.backend.limits[:,1]-1e-5-reference[:5]))
        negative=np.minimum(joint_limit,brake(reference[:5]-self.backend.limits[:,0]-1e-5))
        lead=getattr(self.backend,'tracking_lead',None)
        if lead is not None:
            budget=MOTION.tracking_brake_counts/MOTION.tracking_lead_counts*lead[:5]
            error=reference[:5]-current[:5]
            # Reserve the entire stop against the last observed position:
            # v*dt + v**2/(2*a) <= room. Historical feedback velocity cannot
            # supply future room: the 25 Hz worker may return the same sample
            # to successive 30 Hz ticks, or the motor may stop advancing.
            positive=np.minimum(positive,brake(budget-error))
            negative=np.minimum(negative,brake(budget+error))
        # The braking budget is a soft anticipatory limit. Preserve
        # acceleration if encoder quantization moves this soft bound suddenly;
        # the configured next-command and independent worker bounds never relax.
        hard_lower=np.maximum(-joint_limit,self._physical_velocity-acc*dt)
        hard_upper=np.minimum(joint_limit,self._physical_velocity+acc*dt)
        hard_lower=np.maximum(hard_lower,(self.backend.limits[:,0]+1e-5-reference[:5])/dt)
        hard_upper=np.minimum(hard_upper,(self.backend.limits[:,1]-1e-5-reference[:5])/dt)
        if lead is not None:
            hard_lower=np.maximum(hard_lower,(current[:5]-lead[:5]-reference[:5])/dt)
            hard_upper=np.minimum(hard_upper,(current[:5]+lead[:5]-reference[:5])/dt)
        if np.any(hard_lower>hard_upper+1e-9):raise MotionConstraintError('轨迹加速度与当前关节/反馈安全范围冲突')
        lower=np.maximum(-negative,hard_lower)
        upper=np.minimum(positive,hard_upper)
        conflict=lower>upper
        safe=np.clip(np.clip(self._physical_velocity,-negative,positive),hard_lower,hard_upper)
        lower=np.where(conflict,safe,lower);upper=np.where(conflict,safe,upper)
        return lower,upper,negative,positive

    def _fit_bounded_tcp(self,p_ref,step,seed,lower,upper,dt,tcp_limit):
        constrained=getattr(self.backend.kinematics,'ik_tcp_step',None)
        if constrained is not None:
            return constrained(p_ref+step,seed,lower*dt,upper*dt,tcp_limit*dt)
        solve=lambda alpha:self.backend.kinematics.ik_position_step(
            p_ref+alpha*step,seed,lower_step=lower*dt,upper_step=upper*dt)
        fits=lambda q:np.linalg.norm(self.backend.kinematics.fk(q)[0]-p_ref)<=tcp_limit*dt+1e-9
        # Uniform velocity scaling can cross acceleration lower bounds. Reduce
        # Cartesian progress by re-solving in the original joint bounds instead.
        best=solve(0.)
        if not fits(best):raise MotionConstraintError('当前制动轨迹暂时无法满足末端限速')
        lo,hi=0.,1.
        for _ in range(8):
            mid=(lo+hi)/2;candidate=solve(mid)
            if fits(candidate):best=candidate;lo=mid
            else:hi=mid
        return best

    def tick(self, now, dt):
        self._refresh_control_reference()
        current = self.backend.read()
        thermal_pending=bool(self.physical and getattr(self.backend,'thermal_pending',False)
                             and not self.backend.hardware_status().get('fault'))
        if self.physical and not self.backend.ready and self.armed:
            self.revoke('真机反馈或接管无效，已撤权')
        if not math.isfinite(dt) or not 0 < dt <= .15:
            self.revoke("控制周期超时，未补赶；需重新授权")
        if self.armed and not self.fresh(now):
            self.revoke("输入超过 200 ms 未更新，需重新授权")
        if (self.physical and self.armed and self._motion_epoch is not None
                and self.backend.motion_epoch!=self._motion_epoch):
            if self._thermal_epoch_change():self._discard_thermal_plan(current)
            else:self.revoke('电机执行端已暂停旧目标；松开握持键和扳机，再短按 A 重新授权')
        entry=getattr(self.backend,'workspace_entry',None)
        outside_workspace=False
        if self.physical and entry is not None:
            actual_p=self.backend.kinematics.fk(current[:5])[0]
            outside_workspace=bool(np.any(actual_p<self.work_min) or np.any(actual_p>self.work_max))
        if (self.physical and entry is not None and not self.stopped
                and (not self.armed or entry.running or outside_workspace or self._workspace_wait)
                and math.isfinite(dt) and 0<dt<=.15):
            was_workspace_wait=self._workspace_wait
            if entry.tick(now,dt):
                self._workspace_wait=True;self._physical_velocity[:]=0;self.prediction_offset[:]=0
                self.last_command=self._planning_reference(self.backend.read())
                self.reason=entry.message
                return self.status(now)
            if self._workspace_wait:
                self._workspace_wait=False;self._physical_velocity[:]=0
                self.last_command=self._planning_reference(self.backend.read())
                if self._anchor is not None and self._anchor.get('intent') is not None:self._anchor['intent'].suppress_prediction()
                if entry.state=='ready':
                    self.reason='工作区准备完成，继续跟随当前手柄' if self.armed else '工作区准备完成；松开按键后可用 A 记录参考并授权'
            if self.armed and entry.state=='blocked' and (outside_workspace or was_workspace_wait):
                if self._thermal_epoch_change():self._discard_thermal_plan(current);entry.resume()
                else:self.revoke(entry.message)
        if self.stopped or not self.armed or not self.fresh(now):
            self.last_command = self._planning_reference(current)
            return self.status(now)
        f = self.latest
        if thermal_pending:
            # Worker holds once at entry; do not ratchet its goal with fresh
            # sagging feedback. Throw away all pre-quarantine Cartesian debt.
            self._discard_thermal_plan(current)
            return self.status(now)
        if self._thermal_wait:
            self._thermal_wait=False;self._motion_epoch=getattr(self.backend,'motion_epoch',None)
            self.last_command=self._planning_reference(current)
        if self._release_pending:
            self.workspace_limited=False
            self.limit_limited=False
            self.requested_rotation=None
            self.orientation.reset()
            self.prediction_offset[:]=0
            self.tracking_limited=False;self._tracking_wait=None
            self.engaged = False
            self._anchor = None
            self._release_pending = False
            self._idle_reference=None;self._idle_waiting=True
            self.last_command = self._planning_reference(current)
            self.reason = "握持已释放，保持；再次握持重置位置起点，并追到手柄当前绝对朝向"
            return self.status(now)
        if f.grip < .35:
            self.workspace_limited=False
            self.limit_limited=False
            self.requested_rotation=None
            self.orientation.reset()
            self.prediction_offset[:]=0
            self.tracking_limited=False;self._tracking_wait=None
            self.engaged = False
            self._anchor = None
            if self.physical and self.gripper_mode=='toggle':
                self._physical_velocity[:]=0
                self._idle_gripper(current,now,dt)
            else:
                if self.motor_response or self.physical:self.backend.hold()
                self.last_command = self._planning_reference(current)
            return self.status(now)
        if f.grip < .65 and not self.engaged:
            return self.status(now)
        if not self.engaged or self._anchor["precision"] != f.precision:
            if self.motor_response and not self.engaged and np.max(abs(self.backend.servos.velocity))>.05:
                self.reason='等待舵机减速稳定后重新接管';return self.status(now)
            self._engage(f, current)
            self.last_command = self._planning_reference(current)
            return self.status(now)
        a = self._anchor
        target_R=None
        if self._split_roll():
            self.requested_roll=self.roll.update(f.orientation,dt,f.seq)
        elif self.physical and (self.mode=='pose' or np.any(self._wrist_offset())):
            target_R=self.orientation.update(f.orientation,dt,f.seq)
        hand=self._control_point(f.position)
        if self.physical and a.get('intent') is not None:
            hand=a['intent'].update(hand,dt,f.seq)
            a['hand_velocity']=a['intent'].prediction_velocity
            a['hand_acceleration']=a['intent'].acceleration.copy()
        elif self.physical:
            previous_hand=a['accepted_hand'].copy()
            alpha=1.-math.exp(-dt/MOTION.hand_filter_tau)
            a['filtered_hand']+=alpha*(hand-a['filtered_hand'])
            delta=a['filtered_hand']-a['accepted_hand'];distance=np.linalg.norm(delta)
            deadband=.0015 if f.precision else MOTION.hand_deadband
            if distance>deadband:a['accepted_hand']+=delta*(1.-deadband/distance)
            beta=1.-math.exp(-dt/MOTION.prediction_velocity_tau)
            previous_velocity=a['hand_velocity'].copy()
            a['hand_velocity']+=beta*((a['accepted_hand']-previous_hand)/dt-a['hand_velocity'])
            a['hand_acceleration']+=beta*((a['hand_velocity']-previous_velocity)/dt-a['hand_acceleration'])
            hand=a['accepted_hand']
        tcp_limit=(.01 if f.precision else MOTION.tcp_speed) if self.physical else .12 if self.motor_response and not f.precision else .03
        joint_limit=(self.backend.servos.profile.velocity_limits[:5]
                     if self.motor_response and not f.precision else np.full(5,.3))
        if self.physical:
            joint_limit=np.full(5,.08 if f.precision else MOTION.joint_speed)
            if self.mode=='pose' and not f.precision:
                joint_limit[4]=max(MOTION.joint_speed,MOTION.wrist_roll_speed_rad_s)
        grip_limit=1. if self.motor_response and not f.precision else .6
        if self.physical:grip_limit=self.backend.gripper_rate
        lead_limit=MOTION.tcp_lookahead if self.physical else .04 if self.motor_response else .025
        scale = .2 if f.precision else MOTION.hand_scale if self.physical else .5
        self.operator_requested_position=a['operator_position']+scale*XR_TO_ROBOT@(hand-a['operator_hand'])
        requested = a["position"] + scale * XR_TO_ROBOT @ (hand-a["hand"])
        command_min,command_max=self._command_box()
        clipped = np.clip(requested, command_min, command_max)
        self.workspace_limited=bool(np.any(clipped<=command_min+.002) or np.any(clipped>=command_max-.002))
        # Absorb workspace overflow, but never absorb a destination merely
        # because motor feedback has not reached it yet. Otherwise a quick
        # 10 cm hand move is silently rebased to about 2.5 cm forever.
        if not np.allclose(requested, clipped, atol=1e-9):
            a["hand"], a["position"] = hand.copy(), clipped.copy()
        self.requested_position = clipped.copy()
        self.prediction_offset=bounded_prediction(scale*XR_TO_ROBOT@a['hand_velocity'],
            MOTION.prediction_horizon,MOTION.prediction_max_distance,
            acceleration=scale*XR_TO_ROBOT@a['hand_acceleration'] if MOTION.prediction_braking else None) if self.physical and not f.precision else np.zeros(3)
        clipped=np.clip(clipped+self.prediction_offset,command_min,command_max)
        p_now, _ = self.backend.kinematics.fk(current[:5])
        violation=lambda p:np.maximum(self.work_min-p,0)+np.maximum(p-self.work_max,0)
        outside=violation(p_now)
        recovery=self.motor_response and np.any(outside>0)
        if recovery:tcp_limit=min(tcp_limit,.03);joint_limit=np.minimum(joint_limit,.3)
        lead = clipped-p_now
        if np.linalg.norm(lead) > lead_limit:
            clipped = p_now + lead*lead_limit/np.linalg.norm(lead)
        # Plan continuously across encoder quantization and servo lag. Restarting
        # each tiny step at feedback can repeatedly send the same 1-2 counts.
        # The physical worker still enforces its raw lead/step bounds, expiry,
        # fresh feedback and cancellation before every motor write.
        reference=self.last_command.copy() if self.motor_response or self.physical else current.copy()
        p_ref,R_ref=self.backend.kinematics.fk(reference[:5])
        step = clipped-p_ref
        planning_ratio=MOTION.tcp_planning_ratio if self.physical else 1.
        step *= min(1., tcp_limit*planning_ratio*dt/max(np.linalg.norm(step), 1e-12))
        if self.mode == "pose" and not self._split_roll():
            if self.physical:
                # Full absolute target: joint velocity/acceleration and raw
                # feedback bounds pace motion. Do not shrink the error before
                # soft IK, which otherwise weakens orientation convergence.
                pass # Already conditioned once, shared with wrist compensation.
            else:
                relative = Rotation.from_quat(f.orientation).as_matrix() @ a["hand_R"].T
                target_R = XR_TO_ROBOT @ relative @ XR_TO_ROBOT.T @ a["tool_R"]
            self.requested_rotation=target_R.copy()
        try:
            braking_limited=False;rebase_at_limit=False;self.limit_limited=False
            local_ik=getattr(self.backend.kinematics,'ik_position_step',None)
            pose_ik=getattr(self.backend.kinematics,'ik_pose_step',None)
            bounded_ik=self.physical and ((self.mode=='position' or self._split_roll()) and local_ik is not None or self.mode=='pose' and pose_ik is not None)
            if bounded_ik:
                lower,upper,negative,positive=self._physical_velocity_bounds(reference,current,joint_limit,dt,now)
                if self._split_roll():
                    # Pin wrist roll to its independent absolute target. The
                    # other joints track position and softly keep tool-forward level.
                    roll_goal=float(np.clip(self.requested_roll,self.backend.limits[4,0]+1e-5,self.backend.limits[4,1]-1e-5))
                    error=roll_goal-reference[4];acc=MOTION.joint_acceleration
                    brake=np.sqrt((acc*dt)**2+2*acc*abs(error))-acc*dt
                    velocity=np.clip(np.sign(error)*brake,lower[4],upper[4])
                    lower[4]=upper[4]=velocity
                    target_R=a['level_rotation']@Rotation.from_rotvec(
                        [-(roll_goal-a['level_joint']),0.,0.]).as_matrix()
                    self.requested_rotation=target_R.copy()
                    candidate=pose_ik(p_ref+step,target_R,reference[:5],lower*dt,upper*dt,tcp_limit*dt,
                                      orientation_weight=.35,motion_dt=dt,
                                      joint_acceleration=MOTION.joint_acceleration)
                elif self.mode=='pose' and self.pose_policy=='balanced':
                    candidate=self.coupled_pose_solver.step(clipped,target_R,reference[:5],lower*dt,upper*dt,
                        tcp_limit*dt,self.backend.limits,dt,orientation_length=self.pose_orientation_length,
                        workspace=np.array([np.minimum(command_min,p_ref),np.maximum(command_max,p_ref)]),
                        workspace_acceleration=MOTION.joint_acceleration if getattr(self.backend,'workspace_entry',None) is not None else None)
                else:
                    candidate=(pose_ik(p_ref+step,target_R,reference[:5],lower*dt,upper*dt,tcp_limit*dt,
                                       orientation_weight=MOTION.orientation_weight,motion_dt=dt,
                                       joint_acceleration=MOTION.joint_acceleration) if self.mode=='pose' else
                               local_ik(p_ref+step,reference[:5],lower_step=lower*dt,upper_step=upper*dt))
                velocity=(candidate-reference[:5])/dt
                braking_limited=bool(np.any(((velocity>positive-1e-4)&(positive<joint_limit-1e-4))|
                    ((velocity<-negative+1e-4)&(negative<joint_limit-1e-4))))
                joints=candidate
                nearest=float(np.min(np.minimum(reference[:5]-self.backend.limits[:,0],self.backend.limits[:,1]-reference[:5])))
                remaining=float(np.linalg.norm(self.requested_position-p_ref))
                step_length=float(np.linalg.norm(step));direction=step/max(step_length,1e-12)
                progress=float(np.dot(self.backend.kinematics.fk(candidate)[0]-p_ref,direction))
                if remaining>.003 and step_length>1e-6 and progress<min(.00015,.1*step_length):
                    self.limit_limited=True
                    wait=self._geometry_wait
                    if wait is None or np.dot(direction,wait[2])<.5 or np.linalg.norm(p_ref-wait[1])>.0005:
                        self._geometry_wait=(now,p_ref.copy(),direction.copy())
                    rebase_at_limit=nearest<.0035 or now-self._geometry_wait[0]>=.12
                else:self._geometry_wait=None
            else:candidate = self.backend.kinematics.ik(p_ref+step, target_R, seed_q=reference[:5])
            if not bounded_ik:joints = reference[:5]+np.clip(candidate-reference[:5], -joint_limit*dt, joint_limit*dt)
            if self.physical and not bounded_ik:
                desired=(joints-reference[:5])/dt
                lead=getattr(self.backend,'tracking_lead',None)
                if lead is not None:
                    # Leave an independent margin before the worker's hard
                    # limit; the soft budget includes measured static error.
                    room=np.maximum(0.,MOTION.tracking_brake_counts/MOTION.tracking_lead_counts*lead[:5]-np.sign(desired)*(reference[:5]-current[:5]))
                    # The pose/fallback path uses the same full-stop reserve
                    # as bounded Cartesian IK; do not credit future feedback.
                    acc=MOTION.joint_acceleration
                    speed=np.sqrt((acc*dt)**2+2*acc*room)-acc*dt
                    paced=np.clip(desired,-speed,speed)
                    braking_limited=bool(np.any(abs(paced-desired)>1e-8))
                    desired=paced
                velocity=np.clip(desired,self._physical_velocity-MOTION.joint_acceleration*dt,self._physical_velocity+MOTION.joint_acceleration*dt)
                joints=reference[:5]+velocity*dt
            joints = np.clip(joints, self.backend.limits[:,0]+1e-5, self.backend.limits[:,1]-1e-5)
            # Bounded IK must keep its acceleration box during TCP limiting.
            if bounded_ik and np.linalg.norm(self.backend.kinematics.fk(joints)[0]-p_ref)>tcp_limit*dt+1e-9:
                joints=self._fit_bounded_tcp(p_ref,step,reference[:5],lower,upper,dt,tcp_limit)
            for _ in range(0 if bounded_ik else 3):
                p_final, _ = self.backend.kinematics.fk(joints)
                distance = np.linalg.norm(p_final-p_ref)
                if distance <= tcp_limit*dt+1e-9:
                    break
                joints = reference[:5]+(joints-reference[:5])*(tcp_limit*dt/distance)*.98
            trigger=f.trigger
            if self.physical:
                a['accepted_trigger']=conditioned_trigger(trigger,a['accepted_trigger'])
                trigger=a['accepted_trigger']
            opening = (self.control_reference.gripper_target(trigger) if self.physical and self.control_reference
                       else np.clip(a["gripper"]+a["trigger"]-trigger, 0, 1))
            if self.physical and self.gripper_mode=='toggle':opening=self.grasp.target
            if self.physical:reference[5]=self._gripper_reference(reference[5])
            gripper = reference[5]+np.clip(opening-reference[5], -grip_limit*dt, grip_limit*dt)
            command = np.r_[joints, gripper]
            if self.physical:
                command=self._pace_physical(command,reference,current,now,braking_limited)
                if command is None:return self.status(now)
                joints=command[:5]
            if bounded_ik:
                final_velocity=(joints-reference[:5])/dt
                if np.any(final_velocity<lower-1e-8) or np.any(final_velocity>upper+1e-8):
                    raise ValueError('IK 输出超出已求解的关节步长或加速度约束')
            remaining=outside.copy()
            for alpha in (.25, .5, .75, 1.):
                p, _ = self.backend.kinematics.fk(current[:5]+alpha*(joints-current[:5]))
                if recovery:
                    next_violation=violation(p)
                    if np.any(next_violation>remaining+1e-9):raise ValueError('边缘恢复必须持续向工作区内移动')
                    remaining=next_violation
                elif np.any(p < self.work_min) or np.any(p > self.work_max):
                    raise MotionConstraintError("本步 TCP 到达工作区边界")
            if recovery and remaining.sum()>=outside.sum()-1e-8:raise ValueError('边缘恢复没有向内进展')
            if not np.isfinite(command).all():
                raise ValueError("non-finite command")
            # IK may have blocked the event loop. Use actual commit time, not
            # the timestamp captured when this tick started.
            commit_now = self.wall_clock() if self.wall_clock is not None else now
            if not self.fresh(commit_now) or commit_now-now > .15:
                self.revoke("计算后输入或控制周期已过期，丢弃本步；需重新授权")
                return self.status(commit_now)
            if not self.armed or self.stopped:
                return self.status(commit_now)
            if self.physical and self._motion_epoch is not None:
                if self.backend.motion_epoch!=self._motion_epoch:
                    if self._thermal_epoch_change():self._discard_thermal_plan(self.backend.read())
                    else:self.revoke('电机执行端已暂停旧目标；松开握持键和扳机，再短按 A 重新授权')
                    return self.status(commit_now)
                self.backend.command(command,expected_motion_epoch=self._motion_epoch,
                    gripper_opening=self.gripper_mode=='toggle' and self.grasp.active and not self.grasp.closing)
            else:self.backend.command(command)
            if self.physical:self._physical_velocity=(joints-reference[:5])/dt
            self.last_command = command.copy()
            self.constraint_pause=False
            if rebase_at_limit:self._absorb_joint_edge(hand,self.backend.kinematics.fk(joints)[0])
            self.reason = self.physical_label+"跟随中" if self.physical else "跟随中（仿真）"
            if self.physical and self.tracking_limited:
                lead=getattr(self.backend,'tracking_lead',None)
                lagging=lead is not None and np.max(abs(command-current)/lead)>MOTION.stationary_tolerance_counts/MOTION.tracking_lead_counts
                self.reason='跟随受限：电机正在追赶，目标推进已减速' if lagging else '轨迹限速中，电机反馈正常'
            if self.limit_limited:self.reason='到达关节边界：已缓冲向外位移，反向移动即可继续，无需重新授权'
        except MotionConstraintError as exc:
            if self.physical:self._recover_motion(self.backend.read(),exc,now)
            else:self.revoke(str(exc))
        except (ValueError, RuntimeError) as exc:
            self.tracking_limited=False;self._tracking_wait=None
            self.reason = f"本步保持：{exc}"
            # Absorb rejected input; reversing recovers without accumulated debt.
            if self.motor_response or self.physical:
                if self.physical:self.revoke(f'轨迹约束暂停：{exc}；松开握持键检查方向后重新授权')
                else:
                    self.backend.hold();self.last_command=self._planning_reference(current)
                    self.engaged=False;self._anchor=None
            else:self._engage(f, current)
            if not self.physical:self.reason = f"本步保持：{exc}"
        return self.status(now)

    def status(self, now):
        q = self.backend.read()
        p, rotation = self.backend.kinematics.fk(q[:5])
        command_p=self.backend.kinematics.fk(self.last_command[:5])[0] if self.motor_response or self.physical else p
        f = self.latest
        age = self.clock.age_upper(f.client_time_ms, now) if f else math.inf
        state = ("STOPPED" if self.stopped else "FOLLOWING" if self.armed and self.engaged
                 else "ARMED" if self.armed else "READY" if self.fresh(now) else "WAITING")
        return dict(type="status", backend=getattr(self.backend,'kind','kinematic_simulation'), state=state,
                    hardware=self.backend.hardware_status() if self.physical else None,
                    workspace_preparation=None if getattr(self.backend,'workspace_entry',None) is None else self.backend.workspace_entry.status(),
                    motor=self.backend.motor_status() if self.motor_response else None,
                    allow_emulated_simulation=self.allow_emulated,
                    tracking_limited=self.tracking_limited,
                    workspace_limited=self.workspace_limited,
                    thermal_pause=self._thermal_wait,
                    constraint_pause=self.constraint_pause,constraint_pause_events=self.constraint_pause_events,
                    last_motion_pause=self.last_motion_pause,pose_policy=self.pose_policy,
                    requested_roll_rad=self.requested_roll,
                    pose_solver=None if self.coupled_pose_solver is None else self.coupled_pose_solver.last_step,
                    control_point_m=None if self.control_point is None else self.control_point.tolist(),
                    unlock_hold_s=round(min(2.,max(0.,now-self._unlock_held_since)),2)
                        if self.stopped and self._unlock_held_since is not None and self.fresh(now) else 0.,
                    reason=self.reason, mode=self.mode, session_id=self.session_id,
                    gripper_mode=self.gripper_mode,grasp_closing=self.grasp.closing,
                    grasp_target=self.grasp.target,
                    source_seq=self.last_seq, age_ms=round(max(0, age)*1000, 1) if math.isfinite(age) else None,
                    input=None if f is None else dict(position=None if f.position is None else f.position.tolist(),
                        orientation=None if f.orientation is None else f.orientation.tolist(),
                        grip=f.grip, trigger=f.trigger, precision=f.precision,arm=f.arm,stop=f.stop,valid=f.valid,emulated=f.emulated,
                        visible=f.visible, xr_mode=f.mode),
                    q_rad=q[:5].tolist(), q_deg=np.degrees(q[:5]).tolist(), gripper=float(q[5]),
                    command=self.last_command.tolist(),command_tcp_m=command_p.tolist(), tcp_m=p.tolist(),tcp_rotation=rotation.tolist(),
                    control_reference=None if self.control_reference is None else self.control_reference.status(),
                    control_reference_error=getattr(self.backend,'control_reference_error',None),
                    requested_m=None if self.requested_position is None else self.requested_position.tolist(),
                    joint_limit_limited=self.limit_limited,limit_buffer_events=self.limit_buffer_events,
                    operator_requested_m=None if self.operator_requested_position is None else self.operator_requested_position.tolist(),
                    prediction_m=self.prediction_offset.tolist(),
                    requested_orientation=None if self.requested_rotation is None else Rotation.from_matrix(self.requested_rotation).as_quat().tolist(),
                    orientation_error_deg=None if self.requested_rotation is None else float(np.degrees(np.linalg.norm(Rotation.from_matrix(self.requested_rotation@rotation.T).as_rotvec()))),
                    rotation_input=(self.roll.status() if self.roll is not None else None) if self._split_roll() else
                        self.orientation.status() if self.physical and self.mode=='pose' else None,
                    input_filter=None if self._anchor is None or self._anchor.get('intent') is None else
                        dict(self._anchor['intent'].status(),trigger=self._anchor['accepted_trigger']),
                    points=self.backend.points())
