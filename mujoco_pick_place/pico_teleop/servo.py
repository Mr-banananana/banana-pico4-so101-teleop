"""Uncalibrated position-servo response, not a torque/contact physics model."""
from collections import deque
from dataclasses import dataclass
import math
import json
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class ServoProfile:
    name: str = 'STS3215-C001 标准从臂参考（未标定）'
    gear_ratio: int = 345
    encoder_steps: int = 4096
    reference_voltage: float = 6.
    no_load_seconds_60_deg: float = .238
    calibrated: bool = False
    # Engineering commissioning assumptions, NOT manufacturer loaded ratings.
    velocity_rad_s: tuple = (1.5,1.,1.2,1.5,1.8,1.5)
    acceleration_rad_s2: tuple = (5.,3.,4.,5.,6.,5.)
    position_gain: float = 10.
    command_delay_s: float = .02
    feedback_delay_s: float = .02

    @classmethod
    def standard_reference(cls):
        return cls.from_file(Path(__file__).resolve().parents[2]/'configs/pico-sts3215-reference.json')

    @classmethod
    def from_file(cls,path):
        data=json.loads(Path(path).read_text(encoding='utf-8'))
        profile=cls(name=data['name'],**data['datasheet_reference'],**data['response_assumptions'])
        profile.validate();return profile

    @property
    def no_load_rad_s(self):return math.radians(60)/self.no_load_seconds_60_deg

    @property
    def encoder_tick(self):return 2*math.pi/self.encoder_steps

    @property
    def velocity_limits(self):return np.asarray(self.velocity_rad_s,float)

    @property
    def acceleration_limits(self):return np.asarray(self.acceleration_rad_s2,float)

    def validate(self):
        if (type(self.encoder_steps) is not int or self.encoder_steps<=0 or type(self.gear_ratio) is not int
                or self.gear_ratio<=0 or not math.isfinite(self.no_load_seconds_60_deg) or self.no_load_seconds_60_deg<=0
                or not math.isfinite(self.reference_voltage) or self.reference_voltage<=0):
            raise ValueError('invalid servo datasheet reference')
        v,a=self.velocity_limits,self.acceleration_limits
        if (v.shape!=(6,) or a.shape!=(6,) or not np.isfinite(v).all() or not np.isfinite(a).all()
                or np.any(v<=0) or np.any(a<=0) or np.any(v>self.no_load_rad_s)):
            raise ValueError('invalid servo operating envelope')
        if not (math.isfinite(self.position_gain) and self.position_gain>0
                and 0<=self.command_delay_s<=.1 and 0<=self.feedback_delay_s<=.1):
            raise ValueError('invalid servo response assumptions')


class PositionServoBank:
    def __init__(self,initial,limits,profile):
        profile.validate();self.profile=profile
        self.limits=np.asarray(limits,float).copy();self.actual=np.asarray(initial,float).copy()
        if (self.limits.shape!=(6,2) or self.actual.shape!=(6,) or not np.isfinite(self.limits).all()
                or not np.isfinite(self.actual).all() or np.any(self.limits[:,0]>=self.limits[:,1])
                or np.any(self.actual<self.limits[:,0]) or np.any(self.actual>self.limits[:,1])):
            raise ValueError('invalid servo initialization')
        self.velocity=np.zeros(6);self.target=self.actual.copy();self.time=0.;self.holding=True
        self.pending=deque(maxlen=256);self.history=deque([(0.,self.actual.copy())],maxlen=1000)
        self._vmax=profile.velocity_limits;self._amax=profile.acceleration_limits

    def command(self,target):
        target=np.asarray(target,float)
        if target.shape!=(6,) or not np.isfinite(target).all():raise ValueError('six finite servo targets required')
        target=np.clip(np.round(target/self.profile.encoder_tick)*self.profile.encoder_tick,self.limits[:,0],self.limits[:,1])
        self.pending.append((self.time+self.profile.command_delay_s,target.copy()))
        self.holding=False

    def hold(self):
        self.pending.clear();self.target=self.actual.copy();self.holding=True

    def advance(self,dt):
        if not math.isfinite(dt) or dt<=0:raise ValueError('positive finite servo dt required')
        if dt>.15:self.hold();dt=.15
        count=max(1,math.ceil(dt/.002));h=dt/count
        for _ in range(count):
            self.time+=h
            while self.pending and self.pending[0][0]<=self.time+1e-12:
                _,self.target=self.pending.popleft()
            error=self.target-self.actual
            if self.holding:wanted=np.zeros(6)
            else:
                speed=np.minimum(self._vmax,np.minimum(self.profile.position_gain*abs(error),np.sqrt(2*self._amax*abs(error))))
                wanted=np.sign(error)*speed
                wanted=np.where(abs(error)<=self.profile.encoder_tick*.5,0.,wanted)
            new_v=self.velocity+np.clip(wanted-self.velocity,-self._amax*h,self._amax*h)
            self.actual+=.5*(self.velocity+new_v)*h
            clipped=np.clip(self.actual,self.limits[:,0],self.limits[:,1])
            self.velocity=np.where(clipped!=self.actual,0.,new_v);self.actual=clipped
            if self.holding:self.target=self.actual.copy()
            self.history.append((self.time,self.actual.copy()))
        cutoff=self.time-self.profile.feedback_delay_s
        while len(self.history)>1 and self.history[1][0]<=cutoff+1e-12:self.history.popleft()

    def feedback(self):
        values=np.round(self.history[0][1]/self.profile.encoder_tick)*self.profile.encoder_tick
        return np.clip(values,self.limits[:,0],self.limits[:,1])

    def status(self):
        return dict(name=self.profile.name,calibrated=self.profile.calibrated,holding=self.holding,
            velocity_rad_s=self.velocity.tolist(),target_rad=self.target.tolist(),actual_rad=self.actual.tolist(),
            feedback_rad=self.feedback().tolist(),velocity_limit_rad_s=self._vmax.tolist(),
            acceleration_limit_rad_s2=self._amax.tolist(),command_delay_s=self.profile.command_delay_s,
            feedback_delay_s=self.profile.feedback_delay_s,encoder_steps=self.profile.encoder_steps,
            reference_voltage=self.profile.reference_voltage,actual_supply_voltage=None,
            no_load_envelope_rad_s=self.profile.no_load_rad_s,
            parameter_status='datasheet_envelope_plus_uncalibrated_response',
            unmodeled=['重力与负载扭矩','电压变化','摩擦与齿隙','电流温升与过载保护'])
