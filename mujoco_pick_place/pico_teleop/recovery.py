"""Explicit local, slow inward recovery; never authorizes headset following."""
import json
from pathlib import Path
from .timing import clock as time
import numpy as np


def build_plan(arm,raw):
    raw=np.asarray(raw,int);q=arm.profile.decode(raw,require_bounds=False)
    lo,hi=arm.limits[:,0],arm.limits[:,1]
    outside=np.maximum(lo-q[:5],0)+np.maximum(q[:5]-hi,0)
    if not (q[2]>hi[2] and outside[2]<=np.radians(1.) and np.count_nonzero(outside)==1):
        raise ValueError('恢复仅适用于肘关节上端超界不超过1度的当前情况')
    other=[0,1,3,4]
    if np.min(np.minimum(q[:5][other]-lo[other],hi[other]-q[:5][other]))<np.radians(3):
        raise ValueError('其他关节距离限位不足3度')
    target=raw.copy();target[2]-=120
    # Normal calibration validation applies to the destination.
    arm.profile.decode(target)
    points=[];previous=None
    for offset in range(121):
        r=raw.copy();r[2]-=offset
        state=arm.profile.decode(r,require_bounds=False);p=arm.kinematics.fk(state[:5])[0]
        if np.any(p<arm.work_min+.005) or np.any(p>arm.work_max-.005):raise ValueError('恢复路径靠近工作区边界')
        if previous is not None and p[2]<previous[2]-1e-8:raise ValueError('恢复路径没有持续抬高末端')
        points.append(p.tolist());previous=p
    if points[-1][2]-points[0][2]<.005:raise ValueError('恢复抬高幅度不足')
    return dict(initial_raw=raw.tolist(),target_raw=target.tolist(),tcp_path=points,
                speed_counts_s=40,acceleration_counts_s2=100,lead_counts=28)


def build_lift_plan(arm,raw):
    """Narrow table-adjacent corridor, never the normal XR workspace."""
    raw=np.asarray(raw,int);q=arm.profile.decode(raw)
    origin=arm.kinematics.fk(q[:5])[0]
    if (not -.02<=origin[2]<arm.work_min[2] or q[2]<np.radians(80)
            or np.any(origin[:2]<arm.work_min[:2]+.005) or np.any(origin[:2]>arm.work_max[:2]-.005)):
        raise ValueError('准备抬升仅适用于肘部接近上端、末端接近桌面的当前姿态')
    if np.min(np.minimum(q[[0,1,3,4]]-arm.limits[[0,1,3,4],0],
                         arm.limits[[0,1,3,4],1]-q[[0,1,3,4]]))<np.radians(3):
        raise ValueError('其他关节缺少准备动作余量')
    points=[];previous=None;max_derivative=0.
    for offset in range(501):
        candidate=raw.copy();candidate[2]-=offset
        state=arm.profile.decode(candidate);p=arm.kinematics.fk(state[:5])[0]
        if (np.any(p[:2]<arm.work_min[:2]+.005) or np.any(p[:2]>arm.work_max[:2]-.005)
                or p[2]>arm.work_max[2]-.005 or previous is not None and p[2]<previous[2]-1e-10):
            raise ValueError('准备路径不是持续向上且向工作区内部移动')
        if previous is not None:max_derivative=max(max_derivative,float(np.linalg.norm(p-previous)))
        points.append(p.tolist());previous=p
        if p[2]>=.10:break
    else:raise ValueError('有限肘部行程内无法到达准备高度')
    if 80*max_derivative>.04:raise ValueError('准备路径末端速度超限')
    return dict(initial_raw=raw.tolist(),target_raw=candidate.tolist(),tcp_path=points,
                speed_counts_s=80,acceleration_counts_s2=250,lead_counts=28)


def lift_command_is_safe(arm,command,plan):
    command=np.asarray(command);initial=np.asarray(plan['initial_raw']);target=np.asarray(plan['target_raw'])
    if command.shape!=(6,) or not np.isfinite(command).all() or np.any(command!=np.rint(command)):return False
    if not np.array_equal(command[[0,1,3,4,5]],initial[[0,1,3,4,5]]):return False
    if not target[2]<=command[2]<=initial[2]:return False
    # Every integer encoder goal was FK checked before enabling. This lookup
    # stays thread-safe when used by the serial worker immediately at commit.
    p=np.asarray(plan['tcp_path'][int(initial[2]-command[2])])
    return bool(np.all(p[:2]>=arm.work_min[:2]+.005) and np.all(p[:2]<=arm.work_max[:2]-.005)
                and p[2]>=plan['tcp_path'][0][2]-1e-9 and p[2]<=arm.work_max[2]-.005)


def takeover_reference(initial,preloaded,feedback):
    initial=np.asarray(initial);preloaded=np.asarray(preloaded);feedback=np.asarray(feedback)
    if (preloaded.shape!=(6,) or np.max(abs(preloaded-initial))>1 or np.max(abs(feedback-initial))>2
            or preloaded[2]>initial[2]):raise ValueError('接管期间姿态改变，取消恢复以避免向外回摆')
    return preloaded.astype(float).copy()


def recovery_step(reference,current,velocity,target,dt,*,speed_limit=40.,acceleration=100.):
    if (not np.isfinite([velocity,dt,speed_limit,acceleration]).all() or not 0<dt<=.06
            or not 0<speed_limit<=100 or not 0<acceleration<=350):
        raise ValueError('恢复周期或速度无效')
    a=acceleration
    brake=lambda room:np.sqrt((a*dt)**2+2*a*max(0.,room))-a*dt
    cap=min(speed_limit,brake(reference[2]-target[2]),brake(24-(current[2]-reference[2])))
    lower=max(0.,velocity-a*dt)
    if cap<lower-1e-8:raise RuntimeError('恢复制动空间不足，取消轨迹')
    speed=min(cap,velocity+a*dt)
    result=reference.copy();result[2]-=speed*dt
    command=np.rint(result).astype(int)
    if not command_is_safe(command,current):
        raise RuntimeError('恢复反馈改变，取消轨迹以避免回摆或领先超限')
    return result,(reference[2]-result[2])/dt


def command_is_safe(command,current):
    command=np.asarray(command);current=np.asarray(current)
    return bool(command.shape==(6,) and current.shape==(6,) and np.isfinite(current).all()
                and command[2]<=current[2]+1 and np.max(abs(command-current))<=28)


def run(output,*,lift=False,expected_raw=None):
    from .real import PhysicalArm
    from .motion import MOTION_PROFILE_NAME
    if MOTION_PROFILE_NAME!='measured':raise ValueError('低速恢复禁止使用高速配置')
    arm=PhysicalArm();report=dict(ok=False,scope='table preparation lift; no XR input' if lift else 'slow elbow inward recovery; no XR input');allowed=[False];phase=['takeover']
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:
            arm.poll()
            if arm.feedback_valid:break
            time.sleep(.01)
        h=arm.hardware_status()
        if not arm.feedback_valid or arm.worker.fault:raise ValueError('恢复初始反馈无效')
        if lift and (expected_raw is None or np.asarray(expected_raw).shape!=(6,)
                     or not np.isfinite(expected_raw).all() or np.any(expected_raw!=np.rint(expected_raw))
                     or np.max(abs(np.asarray(h['raw'])-expected_raw))>1):
            raise ValueError('当前姿态与已预检姿态不同，未使能电机')
        plan=(build_lift_plan if lift else build_plan)(arm,h['raw']);report['plan']=plan
        initial=np.asarray(plan['initial_raw']);target=np.asarray(plan['target_raw']);samples=[];report['samples']=samples
        def guard():
            s=arm.worker.snapshot()
            if not allowed[0] or not s or time.monotonic()-s['captured_at']>.1:return False
            if min(s['voltage_raw'])<90 or max(s['voltage_raw'])>126 or max(s['temperature'])>=55:return False
            r=np.asarray(s['raw']);other=[0,1,3,4,5]
            if phase[0]=='takeover' and (np.max(abs(r-initial))>1 or r[2]>initial[2]):return False
            return bool(np.max(abs(r[other]-initial[other]))<=4 and target[2]-3<=r[2]<=initial[2]+2)
        allowed[0]=True;arm.worker.request_enable(guard)
        end=time.monotonic()+2
        while time.monotonic()<end:
            arm.poll()
            if arm.worker.fault:raise RuntimeError(arm.worker.fault)
            if arm.worker.enabled:break
            time.sleep(.01)
        if not arm.worker.enabled:raise RuntimeError('低速恢复接管未完成')
        reference=takeover_reference(initial,arm.worker.last_sent,arm.worker.snapshot()['raw'])
        if lift:
            plan=build_lift_plan(arm,np.rint(reference).astype(int));report['plan']=plan
            initial=np.asarray(plan['initial_raw']);target=np.asarray(plan['target_raw'])
        phase[0]='motion'
        epoch=arm.motion_epoch;velocity=0.;start=previous=time.monotonic()
        baseline=initial[2];progress_at=start
        timeout=12 if lift else 8
        while time.monotonic()-start<timeout:
            now=time.monotonic();elapsed=now-previous;previous=now
            if lift and elapsed>.06:raise RuntimeError('准备周期超时，取消而不补赶')
            dt=max(.001,min(elapsed,.06));arm.poll();s=arm.worker.snapshot()
            if lift and time.monotonic()-now>.06:raise RuntimeError('准备计算超时，丢弃本步而不补赶')
            if arm.worker.fault or not arm.worker.enabled or arm.motion_epoch!=epoch or not guard():raise RuntimeError('恢复条件改变，已取消')
            current=np.asarray(s['raw'])
            reference,velocity=recovery_step(reference,current,velocity,target,dt,
                speed_limit=plan['speed_counts_s'],acceleration=plan['acceleration_counts_s2'])
            command=np.rint(reference).astype(int)
            q=arm.profile.decode(command,require_bounds=False);p=arm.kinematics.fk(q[:5])[0]
            if lift:
                if not lift_command_is_safe(arm,command,plan):raise RuntimeError('准备目标超出已预检单轴路径')
            elif np.any(p<arm.work_min+.005) or np.any(p>arm.work_max-.005):raise RuntimeError('恢复目标超出预检工作区')
            deadline=now+(.06 if lift else .07)
            if lift and time.monotonic()>deadline:raise RuntimeError('准备计算超时，丢弃本步而不补赶')
            def commit_guard(command=command.copy(),deadline=deadline):
                latest=arm.worker.snapshot()
                return (allowed[0] and arm.motion_epoch==epoch and guard() and latest is not None
                        and time.monotonic()<=deadline
                        and command_is_safe(command,latest['raw'])
                        and (not lift or lift_command_is_safe(arm,command,plan)))
            arm.worker.submit(command,deadline,commit_guard)
            samples.append(dict(t=now-start,goal=command.tolist(),raw=current.tolist(),tcp=p.tolist()))
            if baseline-current[2]>=2:baseline=current[2];progress_at=now
            if now-progress_at>1.5 and reference[2]>target[2]+1:raise RuntimeError('恢复期间电机无进展')
            recent=[r for r in samples if samples[-1]['t']-r['t']<=.4]
            if (abs(reference[2]-target[2])<.6 and len(recent)>8
                    and recent[-1]['t']-recent[0]['t']>=.35
                    and np.ptp([r['raw'][2] for r in recent])<=2 and abs(current[2]-target[2])<=16):
                arm.profile.decode(current);report['ok']=True;break
            time.sleep(max(.001,.04-(time.monotonic()-now)))
        if not report['ok']:raise RuntimeError('准备/恢复未在限定时间内完成')
    except Exception as exc:report['error']=str(exc)
    finally:
        arm.worker.disable()
        allowed[0]=False
        try:
            time.sleep(.16);arm.poll();report['final']=arm.hardware_status()
            final=report['final'];held=arm.worker.last_sent
            if (not final['feedback_valid'] or final['fault'] or not isinstance(final['torque'],(list,tuple))
                    or any(v!=1 for v in final['torque']) or held is None
                    or np.max(abs(np.asarray(final['raw'])-held))>4):
                report['ok']=False;report['error']=report.get('error','恢复结束保持未确认')
            if lift and report['ok']:
                try:
                    r=np.asarray(final['raw']);state=arm.profile.decode(r)
                    p=arm.kinematics.fk(state[:5])[0];report['final_tcp_m']=p.tolist()
                    if (np.max(abs(r[[0,1,3,4,5]]-initial[[0,1,3,4,5]]))>4
                            or not target[2]-3<=r[2]<=target[2]+16
                            or np.any(p<arm.work_min+.005) or np.any(p>arm.work_max-.005)):
                        raise ValueError('反馈与批准的结束姿态或工作区不符')
                except ValueError as exc:
                    report['ok']=False;report['error']='准备结束姿态验证失败：'+str(exc)
        finally:
            arm.close();Path(output).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report
