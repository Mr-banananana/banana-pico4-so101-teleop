"""Coupled full-pose differential IK using upstream Mink and DAQP.

All five arm joints solve one SE(3) task. Gripper and scene-object DOFs are
frozen. Linearized limits are followed by independent nonlinear FK checks.
"""
import itertools
import time
import numpy as np
import mujoco
import mink
from .motion_errors import MotionConstraintError


class _StepLimit(mink.Limit):
    def __init__(self,model,site,lower,upper,radius,workspace,workspace_acceleration=None):
        self.model=model;self.site=site;self.lower=lower;self.upper=upper
        self.radius=radius;self.workspace=workspace
        self.workspace_acceleration=workspace_acceleration

    def compute_qp_inequalities(self,configuration,dt):
        nv=self.model.nv;eye=np.eye(nv)
        lo=np.r_[self.lower,np.zeros(nv-5)];hi=np.r_[self.upper,np.zeros(nv-5)]
        jp=np.zeros((3,nv));jr=np.zeros_like(jp)
        mujoco.mj_jacSite(self.model,configuration.data,jp,jr,self.site)
        directions=np.array([v for v in itertools.product([-1.,0.,1.],repeat=3) if any(v)])
        directions/=np.linalg.norm(directions,axis=1)[:,None]
        # The 26-direction polytope fits inside the linear TCP ball. Leave a
        # margin for curvature, then verify the actual nonlinear displacement.
        matrices=[eye,-eye,directions@jp]
        bounds=[hi,-lo,np.full(len(directions),self.radius*.88)]
        if self.workspace is not None:
            p=configuration.data.site_xpos[self.site]
            positive=self.workspace[1]-p;negative=p-self.workspace[0]
            if self.workspace_acceleration is not None:
                # Anticipate stopping before the face, instead of waiting for
                # a position inequality to conflict with the acceleration box.
                acceleration=np.maximum(.05,.5*np.sum(abs(jp[:,:5]),axis=1)*self.workspace_acceleration)
                brake=lambda room:np.sqrt((acceleration*dt)**2+2*acceleration*np.maximum(room,0))-acceleration*dt
                positive=np.minimum(positive,brake(positive)*dt)
                negative=np.minimum(negative,brake(negative)*dt)
            matrices.extend([jp,-jp]);bounds.extend([positive,negative])
        return mink.Constraint(np.vstack(matrices),np.concatenate(bounds))


class CoupledPoseIK:
    def __init__(self,model,site):
        self.model=model;self.site=site;self.configuration=mink.Configuration(model)
        self.task=mink.FrameTask(site,'site',position_cost=1.,orientation_cost=.2,
                                 gain=.18,lm_damping=.001)
        self.last_step=None

    def step(self,position,rotation,seed,lower,upper,tcp_radius,joint_limits,dt,*,
             orientation_length=.2,workspace=None,workspace_acceleration=None):
        started=time.perf_counter()
        p=np.asarray(position,float);r=np.asarray(rotation,float);q=np.asarray(seed,float)
        lo=np.asarray(lower,float);hi=np.asarray(upper,float);limits=np.asarray(joint_limits,float)
        if (p.shape!=(3,) or r.shape!=(3,3) or q.shape!=(5,) or lo.shape!=(5,) or hi.shape!=(5,)
                or limits.shape!=(5,2) or not np.isfinite(np.r_[p,r.ravel(),q,lo,hi,limits.ravel(),tcp_radius,dt,orientation_length]).all()
                or not 0<dt<=.15 or tcp_radius<=0 or not 0<orientation_length<=1
                or np.any(lo>hi) or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6):
            raise ValueError('invalid coupled pose input')
        if workspace is not None:
            workspace=np.asarray(workspace,float)
            if workspace.shape!=(2,3) or not np.isfinite(workspace).all() or np.any(workspace[0]>=workspace[1]):
                raise ValueError('invalid pose workspace')
        if workspace_acceleration is not None and (not np.isfinite(workspace_acceleration) or workspace_acceleration<=0):
            raise ValueError('invalid workspace braking acceleration')
        lo=np.maximum(lo,limits[:,0]+1e-5-q);hi=np.minimum(hi,limits[:,1]-1e-5-q)
        if np.any(lo>hi+1e-12):raise MotionConstraintError('关节边界暂时限制本步位姿')
        qpos=self.model.qpos0.copy();qpos[:5]=q;self.configuration.update(qpos)
        origin=self.configuration.data.site_xpos[self.site].copy()
        self.task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(r),p))
        self.task.set_orientation_cost(orientation_length)
        self.task.gain=1-np.exp(-dt/.10)
        candidate=None;radius=tcp_radius
        for attempt in range(3):
            constraint=_StepLimit(self.model,self.site,lo,hi,radius,workspace,workspace_acceleration)
            try:
                velocity=mink.solve_ik(self.configuration,[self.task],dt,'daqp',damping=1e-7,
                                       limits=[constraint],safety_break=True)
            except mink.NoSolutionFound as exc:
                raise MotionConstraintError('位姿与本步运动约束暂时冲突') from exc
            delta=velocity[:5]*dt;candidate=q+delta
            if not np.isfinite(velocity).all() or np.max(abs(velocity[5:]),initial=0)>1e-7:
                raise ValueError('coupled IK changed a frozen scene degree of freedom')
            if np.any(delta<lo-1e-8) or np.any(delta>hi+1e-8):raise ValueError('coupled IK exceeded joint step bounds')
            trial=qpos.copy();trial[:5]=candidate
            self.configuration.update(trial)
            actual_p=self.configuration.data.site_xpos[self.site].copy()
            actual_r=self.configuration.data.site_xmat[self.site].reshape(3,3).copy()
            valid=np.linalg.norm(actual_p-origin)<=tcp_radius+1e-9
            if workspace is not None:valid=valid and np.all(actual_p>=workspace[0]-1e-9) and np.all(actual_p<=workspace[1]+1e-9)
            if valid:break
            radius*=.75;self.configuration.update(qpos)
        else:raise MotionConstraintError('非线性位姿复验要求保持本步')
        self.last_step=dict(solver='mink_daqp_coupled',orientation_length_m=orientation_length,
            residual_m=float(np.linalg.norm(actual_p-p)),
            orientation_error_rad=float(np.linalg.norm(mink.SO3.from_matrix(r@actual_r.T).log())),
            solve_ms=(time.perf_counter()-started)*1000,nonlinear_retries=attempt)
        return candidate
