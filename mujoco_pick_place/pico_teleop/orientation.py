"""Absolute controller-to-tool orientation, independent of Cartesian clutching."""
import numpy as np
from .intent import RotationIntent


XR_TO_ROBOT = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
# Correct the reported jaw/handle quarter-turn around the TOOL'S forward +X.
# Post-multiplication preserves forward and does not rotate room/base axes.
# This is fixed mechanical alignment, never a captured grip/tool zero.
TOOL_ROLL_OFFSET_DEG = 90
TOOL_ROLL_CORRECTION = np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])
TOOL_TO_GRIP = XR_TO_ROBOT.T @ TOOL_ROLL_CORRECTION


class AbsoluteOrientation:
    def __init__(self):
        self.intent = None
        self.alignment=TOOL_TO_GRIP.copy();self.calibrated=False

    def set_alignment(self,matrix):
        value=np.asarray(matrix,float)
        if (value.shape!=(3,3) or not np.isfinite(value).all()
                or not np.allclose(value.T@value,np.eye(3),atol=1e-6) or abs(np.linalg.det(value)-1)>1e-6):
            raise ValueError('invalid tool orientation alignment')
        self.alignment=value.copy();self.calibrated=True;self.reset()

    def engage(self, quaternion, sequence, precision=False):
        # Restart conditioning at the CURRENT hand angle, including released
        # rotation. No previous tool orientation participates in this mapping.
        self.intent = RotationIntent(quaternion, sequence, precision, fast=True)
        return self._target(self.intent.accepted)

    def _target(self,rotation):
        return XR_TO_ROBOT @ rotation.as_matrix() @ self.alignment

    def update(self, quaternion, dt, sequence):
        if self.intent is None:
            raise ValueError('absolute orientation requires an active grip')
        return self._target(self.intent.update(quaternion, dt, sequence))

    def reset(self):
        self.intent = None

    @property
    def hand_rotation(self):
        if self.intent is None:raise ValueError('hand orientation is not conditioned')
        return self.intent.accepted.as_matrix()

    def status(self):
        return None if self.intent is None else dict(self.intent.status(), mapping='absolute',
            alignment_source='saved_upright' if self.calibrated else 'fixed_tool_axes',
            tool_roll_offset_deg=None if self.calibrated else TOOL_ROLL_OFFSET_DEG)
