"""Deterministic MuJoCo kinematic preview; no physics claims or motor I/O."""
import numpy as np
import mujoco

from mujoco_pick_place.so101.kinematics import Kinematics, ROOT
from .servo import PositionServoBank,ServoProfile


class SimulationArm:
    kind = "kinematic_simulation"

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/workcell_so101.xml"))
        self.data = mujoco.MjData(self.model)
        self.kinematics = Kinematics(self.model, site="grasp_center")
        self.limits = self.kinematics.limits.copy()
        seed = np.array([-.25, -.65, .95, .95, .05])
        self.data.qpos[:5] = self.kinematics.ik([.20, 0., .18], seed_q=seed)
        self.data.qpos[6:9] = [.22, -.06, .013]
        self.data.qpos[9:13] = [1, 0, 0, 0]
        self._gripper_range = self.model.jnt_range[5].copy()
        self._set_gripper(.4)
        mujoco.mj_forward(self.model, self.data)

    def _set_gripper(self, opening):
        low, high = self._gripper_range
        self.data.qpos[5] = low + opening * (high-low)

    def read(self):
        low, high = self._gripper_range
        return np.r_[self.data.qpos[:5], np.clip((self.data.qpos[5]-low)/(high-low), 0, 1)]

    def command(self, q):
        q = np.asarray(q, float)
        if q.shape != (6,) or not np.isfinite(q).all() or not 0 <= q[5] <= 1:
            raise ValueError("invalid simulation command")
        if np.any(q[:5] < self.limits[:,0]) or np.any(q[:5] > self.limits[:,1]):
            raise ValueError("simulation joint limit")
        self.data.qpos[:5] = q[:5]
        self._set_gripper(q[5])
        self.data.ctrl[:] = self.data.qpos[:6]
        mujoco.mj_forward(self.model, self.data)

    def points(self):
        names = ("base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper")
        result = [[0., 0., 0.]]
        # Use joint-attached body origins; model names differ across CAD revisions.
        for joint in range(5):
            result.append(self.data.xpos[self.model.jnt_bodyid[joint]].tolist())
        result.append(self.kinematics.fk(self.read()[:5])[0].tolist())
        return result


class ServoSimulationArm(SimulationArm):
    """Same FK geometry, now driven by delayed simulated servo feedback."""
    kind='servo_response_simulation'

    def __init__(self,profile=None):
        super().__init__()
        initial=self.data.qpos[:6].copy()
        self.servos=PositionServoBank(initial,self.model.jnt_range[:6],profile or ServoProfile.standard_reference())
        self._sync_feedback()

    def _sync_feedback(self):
        self.data.qpos[:6]=self.servos.feedback()
        self.data.ctrl[:]=self.servos.target
        mujoco.mj_forward(self.model,self.data)

    def read(self):
        # Parent constructor calls read before the response bank exists.
        if not hasattr(self,'servos'):return super().read()
        q=self.servos.feedback();low,high=self._gripper_range
        return np.r_[q[:5],np.clip((q[5]-low)/(high-low),0,1)]

    def command(self,q):
        q=np.asarray(q,float)
        if q.shape!=(6,) or not np.isfinite(q).all() or not 0<=q[5]<=1:raise ValueError('invalid servo arm target')
        if np.any(q[:5]<self.limits[:,0]) or np.any(q[:5]>self.limits[:,1]):raise ValueError('servo arm joint limit')
        low,high=self._gripper_range
        self.servos.command(np.r_[q[:5],low+q[5]*(high-low)])

    def hold(self):self.servos.hold()

    def advance(self,dt):self.servos.advance(dt);self._sync_feedback()

    def motor_status(self):return self.servos.status()
