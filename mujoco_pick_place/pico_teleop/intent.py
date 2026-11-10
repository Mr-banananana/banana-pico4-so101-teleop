"""Causal XR intent filter: quiet hold and confirmation of tracking pulses."""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation


class HandIntent:
    def __init__(self,position,motion,sequence,precision=False):
        self.motion=motion;self.last_sequence=sequence;self.elapsed=0.
        self.accepted=np.asarray(position,float).copy();self.filtered=self.accepted.copy()
        self.trusted=self.accepted.copy();self.rest_reference=self.accepted.copy()
        self.rest=True;self.radius=.0015 if precision else .003
        self.deadband=.0015 if precision else motion.hand_deadband
        self.velocity=np.zeros(3);self.acceleration=np.zeros(3)
        self.pending=[];self.jump_direction=None;self.rejected_pulses=0;self.confidence=0
        self.history=deque()

    def _pose(self,raw,dt,new):
        if not new:return not self.pending
        if self.pending:
            if np.linalg.norm(raw-self.trusted)<=.003:
                self.pending=[];self.jump_direction=None
            else:
                self.pending.append(raw.copy());self.pending=self.pending[-4:]
                if len(self.pending)<4:return False
                points=np.asarray(self.pending);steps=np.diff(points,axis=0)
                stable=np.linalg.norm(np.ptp(points,axis=0))<=.003
                projected=steps@self.jump_direction
                continuing=np.all(projected>=.001)&np.all(projected>=.5*np.linalg.norm(steps,axis=1))
                if not (stable or continuing):return False
                self.pending=[];self.jump_direction=None
        elif np.linalg.norm(raw-self.trusted)>max(.012,.6*dt):
            jump=raw-self.trusted;self.jump_direction=jump/np.linalg.norm(jump)
            self.pending=[raw.copy()];self.rejected_pulses+=1
            return False
        self.trusted=raw.copy()
        return True

    def _quiet(self):
        self.velocity[:]=0;self.acceleration[:]=0;self.confidence=0

    def update(self,position,dt,sequence):
        raw=np.asarray(position,float)
        if raw.shape!=(3,) or not np.isfinite(raw).all() or not math.isfinite(dt) or not 0<dt<=.15:
            raise ValueError('invalid hand intent sample')
        if sequence<self.last_sequence:raise ValueError('old hand intent sequence')
        new=sequence!=self.last_sequence;self.last_sequence=sequence;self.elapsed+=dt
        if not self._pose(raw,dt,new):
            self._quiet();self.filtered=self.accepted.copy();self.history.clear()
            return self.accepted.copy()
        raw=self.trusted
        if self.rest:
            if np.linalg.norm(raw-self.rest_reference)<=self.radius:
                self._quiet();return self.accepted.copy()
            self.rest=False;self.history.clear()
        previous=self.accepted.copy();previous_velocity=self.velocity.copy()
        alpha=1-math.exp(-dt/self.motion.hand_filter_tau)
        self.filtered+=alpha*(raw-self.filtered)
        delta=self.filtered-self.accepted;length=np.linalg.norm(delta)
        if length>self.deadband:self.accepted+=delta*(1-self.deadband/length)
        beta=1-math.exp(-dt/self.motion.prediction_velocity_tau)
        self.velocity+=beta*((self.accepted-previous)/dt-self.velocity)
        self.acceleration+=beta*((self.velocity-previous_velocity)/dt-self.acceleration)
        if np.linalg.norm(self.velocity)<=.02:self.confidence=0
        elif new:self.confidence=self.confidence+1 if np.dot(self.velocity,previous_velocity)>0 else 1
        if new:self.history.append((self.elapsed,raw.copy()))
        while self.history and self.elapsed-self.history[0][0]>.2:self.history.popleft()
        if len(self.history)>=4 and self.history[-1][0]-self.history[0][0]>=.15:
            times=np.asarray([p[0] for p in self.history]);times-=times.mean()
            points=np.asarray([p[1] for p in self.history]);slope=times@points/max(1e-12,times@times)
            if np.linalg.norm(np.ptp(points,axis=0))<=.003 and np.linalg.norm(slope)<=.005:
                self.rest=True;self.rest_reference=np.median(points,axis=0)
                self.filtered=self.accepted.copy();self._quiet()
        return self.accepted.copy()

    @property
    def prediction_velocity(self):
        return self.velocity.copy() if self.confidence>=3 and not self.rest and not self.pending else np.zeros(3)

    def suppress_prediction(self):
        self._quiet();self.history.clear()

    def status(self):
        return dict(rest=self.rest,pending_samples=len(self.pending),tracking_pulses=self.rejected_pulses,
                    prediction_ready=self.confidence>=3 and not self.rest and not self.pending,
                    accepted_position=self.accepted.tolist())


class RotationIntent:
    """SO(3) filtering without Euler wrapping or quaternion sign jumps."""
    def __init__(self,quaternion,sequence,precision=False,*,fast=False):
        self.accepted=Rotation.from_quat(quaternion);self.filtered=self.accepted
        self.trusted=self.accepted;self.last_sequence=sequence;self.pending=[]
        self.radius=np.radians((.35 if precision else .75) if fast else (.75 if precision else 1.5))
        self.deadband=np.radians((.1 if precision else .2) if fast else (.5 if precision else 1.))
        self.filter_tau=.008 if fast else .02
        self.rejected_pulses=0

    @staticmethod
    def distance(a,b):return float(np.linalg.norm((a*b.inv()).as_rotvec()))

    def update(self,quaternion,dt,sequence):
        if not math.isfinite(dt) or not 0<dt<=.15 or sequence<self.last_sequence:
            raise ValueError('invalid rotation intent sample')
        raw=Rotation.from_quat(quaternion);new=sequence!=self.last_sequence;self.last_sequence=sequence
        if self.pending:
            if not new:return self.accepted
            if self.distance(raw,self.trusted)<=np.radians(3):self.pending=[]
            else:
                self.pending.append(raw);self.pending=self.pending[-3:]
                if len(self.pending)<3 or any(self.distance(r,raw)>np.radians(3) for r in self.pending):return self.accepted
                self.pending=[]
        elif new and self.distance(raw,self.trusted)>max(np.radians(15),8*dt):
            self.pending=[raw];self.rejected_pulses+=1;return self.accepted
        if new:self.trusted=raw
        raw=self.trusted
        if self.distance(raw,self.accepted)<=self.radius:
            self.filtered=self.accepted;return self.accepted
        delta=(raw*self.filtered.inv()).as_rotvec()
        self.filtered=Rotation.from_rotvec(delta*(1-math.exp(-dt/self.filter_tau)))*self.filtered
        delta=(self.filtered*self.accepted.inv()).as_rotvec();length=np.linalg.norm(delta)
        if length>self.deadband:self.accepted=Rotation.from_rotvec(delta*(1-self.deadband/length))*self.accepted
        return self.accepted

    def status(self):
        return dict(pending_samples=len(self.pending),tracking_pulses=self.rejected_pulses,
                    accepted_quaternion=self.accepted.as_quat().tolist())
