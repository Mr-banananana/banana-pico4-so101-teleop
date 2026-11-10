"""Kinematics in the pinned manufacturer's joint convention."""
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares,minimize
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
MODEL_PATH=ROOT/'assets/so101/so101_new_calib.xml'


def validate_path(points, admissible):
    return all(bool(admissible(np.asarray(p))) for p in points)


class Kinematics:
    def __init__(self,model=None,site='gripperframe'):
        self.model=model or mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data=mujoco.MjData(self.model)
        self.site=self.model.site(site).id
        self.limits=self.model.jnt_range[:5].copy()

    def fk(self,q):
        q=np.asarray(q,float)
        if q.shape!=(5,) or not np.isfinite(q).all():
            raise ValueError('five finite radians required')
        self.data.qpos[:5]=q
        mujoco.mj_kinematics(self.model,self.data)
        return self.data.site_xpos[self.site].copy(),self.data.site_xmat[self.site].reshape(3,3).copy()

    def ik_position_step(self,position,seed_q,tolerance=.002,*,lower_step=None,upper_step=None):
        """Local, damped position IK for the continuous physical trajectory.

        Three Cartesian constraints leave five joints underdetermined. A weak
        seed penalty removes that null-space ambiguity; the analytic MuJoCo
        Jacobian avoids numerical-difference iterations in the control loop.
        General pose IK below retains its existing API and behavior.
        """
        p=np.asarray(position,float);seed=np.asarray(seed_q,float)
        if p.shape!=(3,) or seed.shape!=(5,) or not np.isfinite(p).all() or not np.isfinite(seed).all() or np.linalg.norm(p)>.6:
            raise ValueError('local IK requires a finite position and five finite radians')
        low=np.maximum(self.limits[:,0]+1e-5,seed-.08)
        high=np.minimum(self.limits[:,1]-1e-5,seed+.08)
        if np.any(low>=high) or np.any(seed<low) or np.any(seed>high):raise ValueError('local IK seed outside joint limits')
        if lower_step is not None or upper_step is not None:
            lower_step=np.asarray(lower_step,float);upper_step=np.asarray(upper_step,float)
            if lower_step.shape!=(5,) or upper_step.shape!=(5,) or not np.isfinite(lower_step).all() or not np.isfinite(upper_step).all() or np.any(lower_step>upper_step):
                raise ValueError('local IK requires ordered finite step bounds')
            low=np.maximum(low,seed+lower_step);high=np.minimum(high,seed+upper_step)
            if np.any(low>high+1e-12):raise ValueError('local IK step bounds outside joint limits')
        # A fixed coordinate is removed from the optimizer, whose bounds must
        # be strictly ordered. It still participates in the task residual.
        free=high-low>1e-12;fixed=(low+high)*.5
        def expand(x):
            q=fixed.copy();q[free]=x;return q
        d=mujoco.MjData(self.model);jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp)
        d.qpos[:5]=seed;mujoco.mj_kinematics(self.model,d)
        direction=p-d.site_xpos[self.site];length=np.linalg.norm(direction)
        # Prefer reducing progress along the requested line over taking a
        # sideways shortcut when one joint is acceleration/feedback limited.
        weight=np.eye(3)
        if lower_step is not None and length>1e-9:
            unit=direction/length;weight+=9.*(np.eye(3)-np.outer(unit,unit))
        def objective(x):
            q=expand(x)
            d.qpos[:5]=q;mujoco.mj_kinematics(self.model,d)
            return np.r_[weight@(d.site_xpos[self.site]-p)*10.,(q-seed)*.02]
        def jacobian(x):
            q=expand(x)
            d.qpos[:5]=q;mujoco.mj_kinematics(self.model,d);mujoco.mj_comPos(self.model,d)
            mujoco.mj_jacSite(self.model,d,jp,jr,self.site)
            return np.r_[weight@jp[:,:5]*10.,np.eye(5)*.02][:,free]
        if np.any(free):
            fit=least_squares(objective,np.clip(seed,low,high)[free],bounds=(low[free],high[free]),jac=jacobian,
                max_nfev=16,ftol=1e-7,xtol=1e-7,gtol=1e-8)
            x=fit.x;nfev=int(fit.nfev)
        else:x=np.empty(0);nfev=0
        result=expand(x);objective(x);error=float(np.linalg.norm(d.site_xpos[self.site]-p))
        self.last_step=dict(nfev=nfev,residual_m=error,joint_step_rad=float(np.linalg.norm(result-seed)))
        # With trajectory bounds a temporary Cartesian miss can be mandatory
        # while braking. It is not an unreachable unconstrained IK solution.
        # The controller validates final speed, workspace and progress.
        if not np.isfinite(result).all() or (lower_step is None and error>tolerance):raise ValueError('local IK position residual exceeds tolerance')
        return result

    def ik_tcp_step(self,position,seed_q,lower_step,upper_step,tcp_radius):
        """One constrained solve: joint step box and exact nonlinear TCP ball."""
        seed=np.asarray(seed_q,float);target=np.asarray(position,float)
        lo=np.asarray(lower_step,float);hi=np.asarray(upper_step,float)
        if (seed.shape!=(5,) or target.shape!=(3,) or lo.shape!=(5,) or hi.shape!=(5,)
                or not np.isfinite(np.r_[seed,target,lo,hi,tcp_radius]).all() or tcp_radius<=0 or np.any(lo>hi)):
            raise ValueError('invalid constrained TCP step')
        lo=np.maximum(lo,self.limits[:,0]+1e-5-seed);hi=np.minimum(hi,self.limits[:,1]-1e-5-seed)
        if np.any(lo>hi):raise ValueError('constrained TCP step outside joint limits')
        origin=self.fk(seed)[0];d=mujoco.MjData(self.model)
        jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp)
        free=hi-lo>1e-12;fixed=(lo+hi)/2
        # Normalize small joint increments so solver tolerances remain useful.
        scale=max(float(np.max(abs(np.r_[lo,hi]))),1e-4)
        def evaluate(x):
            delta=fixed.copy();delta[free]=x*scale;d.qpos[:5]=seed+delta
            mujoco.mj_kinematics(self.model,d);mujoco.mj_comPos(self.model,d)
            mujoco.mj_jacSite(self.model,d,jp,jr,self.site)
            return delta,d.site_xpos[self.site].copy(),jp[:,:5][:,free]*scale
        length=max(np.linalg.norm(target-origin),tcp_radius)
        def objective(x):
            delta,p,j=evaluate(x);e=(p-target)/length
            return float(e@e+1e-5*np.sum((delta/scale)**2))
        def gradient(x):
            delta,p,j=evaluate(x)
            return 2*j.T@(p-target)/length**2+2e-5*delta[free]/scale
        def ball(x):
            _,p,_=evaluate(x);return 1-float(np.sum((p-origin)**2))/tcp_radius**2
        def ball_jac(x):
            _,p,j=evaluate(x);return -2*j.T@(p-origin)/tcp_radius**2
        x=np.clip(np.zeros(5),lo,hi)[free]/scale
        if np.any(free):
            result=minimize(objective,x,jac=gradient,bounds=list(zip(lo[free]/scale,hi[free]/scale)),
                constraints=[dict(type='ineq',fun=ball,jac=ball_jac)],method='SLSQP',
                options=dict(maxiter=24,ftol=1e-9))
            x=result.x;iterations=int(result.nit)
        else:iterations=0
        delta,p,_=evaluate(x);q=seed+delta
        # Accept only independently verified feasible results, including when
        # SLSQP stops early. A status flag cannot substitute for these checks.
        if (not np.isfinite(q).all() or np.any(delta<lo-1e-9) or np.any(delta>hi+1e-9)
                or np.linalg.norm(p-origin)>tcp_radius+1e-9):
            raise ValueError('当前制动轨迹无法满足末端和关节约束，已暂停轨迹')
        self.last_step=dict(nfev=iterations,residual_m=float(np.linalg.norm(p-target)),
                            joint_step_rad=float(np.linalg.norm(delta)),solver='joint_tcp_slsqp')
        return q

    def ik_pose_step(self,position,rotation,seed_q,lower_step,upper_step,tcp_radius,orientation_weight=.1,
                     *,motion_dt=None,joint_acceleration=None):
        """Position-priority soft orientation inside the same joint/TCP bounds.

        A verified position solution defines a feasible error tube. Orientation
        may improve inside it but cannot sacrifice position to chase missing DOF.
        """
        target=np.asarray(position,float);R=np.asarray(rotation,float);seed=np.asarray(seed_q,float)
        lo=np.asarray(lower_step,float);hi=np.asarray(upper_step,float)
        if (target.shape!=(3,) or R.shape!=(3,3) or seed.shape!=(5,) or lo.shape!=(5,) or hi.shape!=(5,)
                or not np.isfinite(np.r_[target,R.ravel(),seed,lo,hi,tcp_radius,orientation_weight]).all()
                or tcp_radius<=0 or orientation_weight<=0 or np.any(lo>hi)
                or not np.allclose(R.T@R,np.eye(3),atol=1e-6) or abs(np.linalg.det(R)-1)>1e-6):
            raise ValueError('invalid constrained pose target')
        stopping=motion_dt is not None or joint_acceleration is not None
        if stopping and (motion_dt is None or joint_acceleration is None
                or not np.isfinite([motion_dt,joint_acceleration]).all()
                or not 0<motion_dt<=.15 or joint_acceleration<=0):
            raise ValueError('invalid orientation braking horizon')
        lo=np.maximum(lo,self.limits[:,0]+1e-5-seed);hi=np.minimum(hi,self.limits[:,1]-1e-5-seed)
        if np.any(lo>hi):raise ValueError('pose step outside joint limits')
        origin=self.fk(seed)[0]
        try:base=self.ik_position_step(target,seed,lower_step=lo,upper_step=hi)
        except ValueError:base=self.ik_tcp_step(target,seed,lo,hi,tcp_radius)
        if (np.asarray(base).shape!=(5,) or not np.isfinite(base).all()
                or np.any(base-seed<lo-1e-9) or np.any(base-seed>hi+1e-9)
                or np.linalg.norm(self.fk(base)[0]-origin)>tcp_radius+1e-9):
            base=self.ik_tcp_step(target,seed,lo,hi,tcp_radius)
        base_position=self.fk(base)[0];tube=max(.001,float(np.linalg.norm(base_position-target))+1e-5)
        d=mujoco.MjData(self.model);jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp)
        stop_data=mujoco.MjData(self.model) if stopping else None
        free=hi-lo>1e-12;fixed=(lo+hi)/2;scale=max(float(np.max(abs(np.r_[lo,hi]))),1e-4)
        def evaluate(x):
            delta=fixed.copy();delta[free]=x*scale;d.qpos[:5]=seed+delta
            mujoco.mj_kinematics(self.model,d);mujoco.mj_comPos(self.model,d)
            mujoco.mj_jacSite(self.model,d,jp,jr,self.site)
            actual=d.site_xmat[self.site].reshape(3,3).copy()
            angular=jr[:,:5][:,free]*scale
            if stopping:
                # Aim the stopping orientation at the absolute target. With a
                # distant target the joint box saturates; near it, predicted
                # braking distance causes deceleration before overshoot.
                stop_data.qpos[:5]=seed+delta+delta*abs(delta)/(2*joint_acceleration*motion_dt**2)
                mujoco.mj_kinematics(self.model,stop_data);mujoco.mj_comPos(self.model,stop_data)
                stop_jp=np.zeros_like(jp);stop_jr=np.zeros_like(jr)
                mujoco.mj_jacSite(self.model,stop_data,stop_jp,stop_jr,self.site)
                actual=stop_data.site_xmat[self.site].reshape(3,3).copy()
                angular=stop_jr[:,:5][:,free]*scale*(1+abs(delta[free])/(joint_acceleration*motion_dt**2))
            error=Rotation.from_matrix(R@actual.T).as_rotvec()
            return delta,d.site_xpos[self.site].copy(),error,jp[:,:5][:,free]*scale,angular
        def objective(x):
            delta,p,e,_,_=evaluate(x)
            return float(np.sum((p-target)**2)/.005**2+orientation_weight*(e@e)+1e-6*np.sum((delta/scale)**2))
        def gradient(x):
            delta,p,e,j,w=evaluate(x)
            return 2*j.T@(p-target)/.005**2-2*orientation_weight*w.T@e+2e-6*delta[free]/scale
        def constraints(x):
            _,p,_,_,_=evaluate(x)
            return np.array([1-np.sum((p-origin)**2)/tcp_radius**2,1-np.sum((p-target)**2)/tube**2])
        def constraints_jac(x):
            _,p,_,j,_=evaluate(x)
            return np.array([-2*j.T@(p-origin)/tcp_radius**2,-2*j.T@(p-target)/tube**2])
        x=(base-seed)[free]/scale;iterations=0;fallback=False;q=base.copy()
        if np.any(free):
            try:
                result=minimize(objective,x,jac=gradient,bounds=list(zip(lo[free]/scale,hi[free]/scale)),
                    constraints=[dict(type='ineq',fun=constraints, jac=constraints_jac)],method='SLSQP',
                    options=dict(maxiter=24,ftol=1e-10))
                iterations=int(result.nit)
                if result.x.shape!=x.shape or not np.isfinite(result.x).all():raise ValueError('invalid pose solver output')
                delta,p,_,_,_=evaluate(result.x)
                if (np.any(delta<lo-1e-9) or np.any(delta>hi+1e-9)
                        or np.linalg.norm(p-origin)>tcp_radius+1e-9 or np.linalg.norm(p-target)>tube+1e-9):
                    raise ValueError('infeasible optional orientation result')
                q=seed+delta
            except (ValueError,RuntimeError,FloatingPointError):fallback=True
        actual_p,actual_R=self.fk(q)
        self.last_step=dict(solver='pose_joint_tcp_slsqp',nfev=iterations,
            residual_m=float(np.linalg.norm(actual_p-target)),position_tube_m=tube,
            orientation_error_rad=float(np.linalg.norm(Rotation.from_matrix(R@actual_R.T).as_rotvec())),
            orientation_fallback=fallback,joint_step_rad=float(np.linalg.norm(q-seed)))
        return q

    def ik(self,position,rotation=None,seed_q=None,tolerance=.002):
        p=np.asarray(position,float)
        if p.shape!=(3,) or not np.isfinite(p).all() or np.linalg.norm(p)>.6:
            raise ValueError('IK target outside workcell')
        d=mujoco.MjData(self.model)
        seed=np.zeros(5) if seed_q is None else np.asarray(seed_q,float)
        low,high=self.limits[:,0]+1e-5,self.limits[:,1]-1e-5
        if seed.shape!=(5,) or not np.isfinite(seed).all():
            raise ValueError('IK seed invalid')
        R=None if rotation is None else np.asarray(rotation,float)
        def objective(q):
            d.qpos[:5]=q;mujoco.mj_kinematics(self.model,d)
            residual=(d.site_xpos[self.site]-p)*10
            if R is not None:
                actual=d.site_xmat[self.site].reshape(3,3)
                residual=np.r_[residual,(actual-R).flatten()*.12]
            return residual
        result=least_squares(objective,np.clip(seed,low,high),bounds=(low,high),
            max_nfev=100,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        objective(result.x)
        if np.linalg.norm(d.site_xpos[self.site]-p)>tolerance:
            raise ValueError('IK position residual exceeds tolerance')
        return result.x
