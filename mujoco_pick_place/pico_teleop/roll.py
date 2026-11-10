"""Saved-reference absolute controller bank for the wrist_roll joint alone."""
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .intent import RotationIntent


class AbsoluteRoll:
    """Map bank around hand local -Z to an unwrapped joint angle in radians.

    Calibration belongs to the saved hand/robot reference, never the clutch.
    The SO101 positive wrist_roll axis opposes grasp_center forward +X, so
    positive physical tool roll requires a negative joint increment by default.
    Limits and velocity/acceleration constraints belong to the joint controller.
    A vertical saved reference has undefined bank: it uses nominal room bank
    zero, reports reference_singular, and holds neutral while pointing vertical.
    This fallback is fixed at construction and is never captured on regrip.
    The room-up bank frame has two hemispheres. Its sign is transported across
    a vertical crossing so pitch over either pole cannot create a half-turn.
    """

    # Bank is ill-conditioned within approximately 2.9 degrees of room vertical.
    _MIN_HORIZONTAL = .05

    def __init__(self, hand_reference_quaternion, neutral_joint_rad,
                 tool_roll_adjustment_deg=0., joint_sign=-1.):
        neutral=float(neutral_joint_rad)
        adjustment=float(tool_roll_adjustment_deg)
        sign=float(joint_sign)
        if not np.isfinite([neutral, adjustment, sign]).all() or sign not in (-1., 1.):
            raise ValueError('invalid absolute roll calibration')
        reference=self._bank(hand_reference_quaternion)
        self.reference_singular=reference is None
        self.reference_bank_rad=0. if reference is None else reference
        self.neutral_joint_rad=neutral
        self.tool_roll_adjustment_deg=adjustment
        self.joint_sign=sign
        self._zero_joint=neutral+sign*math.radians(adjustment)
        self._target_joint=self._zero_joint
        self._hand_bank=self.reference_bank_rad
        self._singular=self.reference_singular
        reference_rotation=self._rotation(hand_reference_quaternion)
        self._last_forward=None if self.reference_singular else -reference_rotation[:, 2]
        self._last_right=None if self.reference_singular else np.cross(self._last_forward, [0., 1., 0.])
        if self._last_right is not None:self._last_right/=np.linalg.norm(self._last_right)
        self._hemisphere=1
        self.intent=None

    @staticmethod
    def _rotation(quaternion):
        value=np.asarray(quaternion, float)
        if value.shape!=(4,) or not np.isfinite(value).all() or np.linalg.norm(value)<1e-12:
            raise ValueError('absolute roll requires a finite nonzero quaternion')
        return Rotation.from_quat(value).as_matrix()

    @classmethod
    def _bank(cls, quaternion):
        rotation=cls._rotation(quaternion)
        forward=-rotation[:, 2]
        right=np.cross(forward, [0., 1., 0.])
        horizontal=float(np.linalg.norm(right))
        if horizontal<=cls._MIN_HORIZONTAL:
            return None
        right/=horizontal
        normal_up=np.cross(right, forward)
        local_up=rotation[:, 1]
        return math.atan2(float(local_up@right), float(local_up@normal_up))

    @classmethod
    def _crosses_vertical(cls, previous, current):
        """Whether the shortest pointing arc passes the singular vertical cone."""
        if previous[1]*current[1]<=0:
            return False
        cosine=float(np.clip(previous@current, -1., 1.))
        angle=math.acos(cosine)
        if angle<1e-8:return False
        tangent=(current-cosine*previous)/math.sin(angle)
        pole=math.copysign(1., previous[1])
        closest_angle=math.atan2(pole*tangent[1], pole*previous[1])
        if not 0<closest_angle<angle:return False
        height=previous[1]*math.cos(closest_angle)+tangent[1]*math.sin(closest_angle)
        return abs(height)>=math.sqrt(1.-cls._MIN_HORIZONTAL**2)

    def _measure_bank(self, quaternion):
        rotation=self._rotation(quaternion)
        forward=-rotation[:, 2]
        right=np.cross(forward, [0., 1., 0.])
        horizontal=float(np.linalg.norm(right))
        if horizontal<=self._MIN_HORIZONTAL:return None
        right*=self._hemisphere/horizontal
        if (self._last_forward is not None and right@self._last_right<0
                and self._crosses_vertical(self._last_forward, forward)):
            self._hemisphere*=-1
            right=-right
        self._last_forward=forward.copy()
        self._last_right=right.copy()
        normal_up=np.cross(right, forward)
        local_up=rotation[:, 1]
        return math.atan2(float(local_up@right), float(local_up@normal_up))

    @staticmethod
    def _bank_quaternion(bank):
        return Rotation.from_rotvec([0., 0., -bank]).as_quat()

    def _target(self, bank):
        candidate=self._zero_joint+self.joint_sign*(bank-self.reference_bank_rad)
        # Select the physically equivalent turn nearest the previous requested
        # angle. Keeping this state across reset prevents +/-pi regrip reversals.
        difference=candidate-self._target_joint
        self._target_joint+=math.atan2(math.sin(difference), math.cos(difference))
        self._hand_bank=bank
        return self._target_joint

    def engage(self, quaternion, sequence, precision=False):
        bank=self._measure_bank(quaternion)
        self._singular=bank is None
        self.intent=RotationIntent(self._bank_quaternion(self._hand_bank if bank is None else bank),
                                   sequence, precision, fast=True)
        return self._target_joint if bank is None else self._target(bank)

    def update(self, quaternion, dt, sequence):
        if self.intent is None:
            raise ValueError('absolute roll requires an active grip')
        if not math.isfinite(dt) or not 0<dt<=.15 or sequence<self.intent.last_sequence:
            raise ValueError('invalid absolute roll sample')
        bank=self._measure_bank(quaternion)
        self._singular=bank is None
        if bank is None:
            self.intent.last_sequence=sequence
            return self._target_joint
        # Filter only measured bank: filtering the full hand rotation first can
        # create spurious bank while yaw and pitch change together.
        accepted=self.intent.update(self._bank_quaternion(bank), dt, sequence).as_matrix()
        return self._target(math.atan2(float(accepted[0, 1]), float(accepted[1, 1])))

    def reset(self):
        """Release conditioning while retaining calibration and the last turn."""
        self.intent=None

    def status(self):
        diagnostics={} if self.intent is None else self.intent.status()
        return dict(diagnostics, active=self.intent is not None, singular=self._singular,
                    reference_singular=self.reference_singular,
                    bank_hemisphere=self._hemisphere,
                    mapping='absolute_roll', alignment_source='saved_hand_robot_reference',
                    reference_bank_rad=self.reference_bank_rad, hand_bank_rad=self._hand_bank,
                    neutral_joint_rad=self.neutral_joint_rad, target_joint_rad=self._target_joint,
                    joint_sign=self.joint_sign, tool_roll_adjustment_deg=self.tool_roll_adjustment_deg)
