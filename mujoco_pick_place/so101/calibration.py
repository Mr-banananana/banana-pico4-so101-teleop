"""Explicit SDK degree/range conversion; defaults are simulation-only."""
from dataclasses import dataclass
import numpy as np

JOINT_NAMES = ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')


def target_array(value):
    x = np.asarray(value, dtype=np.float64)
    if x.shape != (6,) or not np.isfinite(x).all() or not 0 <= x[-1] <= 1:
        raise ValueError('expected five finite radians and gripper in [0,1]')
    if np.any(np.abs(x[:5]) > np.pi):
        raise ValueError('radian target outside conservative joint range')
    return x


@dataclass
class Calibration:
    signs: object = (1,1,1,1,1)
    offsets_deg: object = (0,0,0,0,0)
    gripper_closed: float = 0.
    gripper_open: float = 100.
    measured: bool = False

    def __post_init__(self):
        self.signs = np.asarray(self.signs, float)
        self.offsets_deg = np.asarray(self.offsets_deg, float)
        if self.signs.shape != (5,) or self.offsets_deg.shape != (5,) or not np.isfinite(self.offsets_deg).all():
            raise ValueError('five finite joint mappings required')
        if not np.isin(self.signs, [-1,1]).all():
            raise ValueError('joint signs must be +/-1')
        if not np.isfinite([self.gripper_closed,self.gripper_open]).all() or abs(self.gripper_open-self.gripper_closed)<1:
            raise ValueError('distinct finite gripper endpoints required')

    def require_measured(self):
        if self.measured is not True:
            raise ValueError('real hardware requires measured calibration')

    def to_sdk(self, target):
        x = target_array(target)
        return np.r_[self.offsets_deg + self.signs*np.degrees(x[:5]),
                     self.gripper_closed+x[-1]*(self.gripper_open-self.gripper_closed)]

    def from_sdk(self, values):
        v=np.asarray(values,float)
        if v.shape!=(6,) or not np.isfinite(v).all():
            raise ValueError('expected six finite SDK values')
        return target_array(np.r_[np.radians((v[:5]-self.offsets_deg)/self.signs),
            (v[-1]-self.gripper_closed)/(self.gripper_open-self.gripper_closed)])


def shape_target(requested, current, last_sent, max_step=.03, gripper_step=.05):
    desired, actual, previous = map(target_array, (requested,current,last_sent))
    if max_step<=0 or gripper_step<=0:
        raise ValueError('positive step limits required')
    limits=np.r_[np.repeat(max_step,5),gripper_step]
    if np.any(np.abs(actual-previous)>2*limits+1e-5):
        raise ValueError('feedback jumped outside the command envelope; re-anchor explicitly')
    # Do not accumulate a far-away commanded target during physical stalls.
    shaped=np.clip(desired,actual-limits,actual+limits)
    return np.clip(shaped,previous-limits,previous+limits).astype(np.float32)
