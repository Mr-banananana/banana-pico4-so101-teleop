"""Causal XR intent filter: quiet hold and confirmation of tracking pulses."""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation


class HandIntent:
    def __init__(self,position,motion,sequence,precision=False,*,sample_time_ms=None):
        self.motion=motion;self.last_sequence=sequence;self.elapsed=0.
        self.last_sample_time_ms = sample_time_ms
        self.last_sample_elapsed = 0.
        self.accepted=np.asarray(position,float).copy();self.filtered=self.accepted.copy()
        self.trusted=self.accepted.copy();self.rest_reference=self.accepted.copy()
        self.precision = precision
        self.rest=True;self.radius=.00075 if precision else .003
        self.deadband=.00035 if precision else motion.hand_deadband
        self.rest_span = .0008 if precision else .003
        self.rest_speed = .0015 if precision else .005
        self.wake_offset = None
        self.velocity=np.zeros(3);self.acceleration=np.zeros(3)
        self.pending = deque(maxlen=4)
        self.rejected_pulses = 0
        self.confidence = 0
        self.history = deque()

    def _pose(self, raw, sample_dt):
        if self.pending:
            if np.linalg.norm(raw - self.trusted) > .003:
                self.pending.append((raw.copy(), sample_dt))
                if len(self.pending) < self.pending.maxlen:
                    return False
                points = np.array([point for point, _ in self.pending])
                intervals = np.array([interval for _, interval in self.pending])[1:]
                lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
                stable = np.linalg.norm(np.ptp(points, axis=0)) <= .003
                # Confirm from recent local steps because the first jump
                # is untrusted and may be followed by a real turn.
                continuing = (
                    np.all(lengths <= np.maximum(.003, 1.6 * intervals))
                    and np.linalg.norm(points[-1] - points[0]) >= .5 * lengths.sum()
                )
                if not (stable or continuing):
                    return False
            self.pending.clear()
        elif np.linalg.norm(raw - self.trusted) > max(.012, .6 * sample_dt):
            self.pending.append((raw.copy(), sample_dt))
            self.rejected_pulses += 1
            return False
        self.trusted = raw.copy()
        return True

    def _quiet(self):
        self.velocity[:]=0;self.acceleration[:]=0;self.confidence=0

    def update(self,position,dt,sequence,*,sample_time_ms=None):
        raw=np.asarray(position,float)
        if raw.shape!=(3,) or not np.isfinite(raw).all() or not math.isfinite(dt) or not 0<dt<=.15:
            raise ValueError('invalid hand intent sample')
        if sequence<self.last_sequence:raise ValueError('old hand intent sequence')
        new=sequence!=self.last_sequence;self.last_sequence=sequence;self.elapsed+=dt
        if new:
            sample_dt = self.elapsed - self.last_sample_elapsed
            if sample_time_ms is not None and self.last_sample_time_ms is not None:
                measured_dt = (sample_time_ms - self.last_sample_time_ms) / 1000.
                if measured_dt > 0:
                    sample_dt = measured_dt
            self.last_sample_elapsed = self.elapsed
            self.last_sample_time_ms = sample_time_ms
            measured_step = raw - self.trusted
            trusted = self._pose(raw, min(sample_dt, .05))
            if trusted:
                # Filtered velocity can keep pointing forward for a control
                # tick after the operator turns back. Drop that feedforward.
                distance = np.linalg.norm(measured_step)
                if distance > self.deadband and np.dot(measured_step, self.velocity) < 0:
                    self._quiet()
        else:
            trusted = not self.pending
            if self.last_sample_time_ms is None:
                self.last_sample_time_ms = sample_time_ms
        if not trusted:
            self.wake_offset = None
            self._quiet();self.filtered=self.accepted.copy();self.history.clear()
            return self.accepted.copy()
        raw=self.trusted
        if self.rest:
            offset = raw - self.rest_reference
            distance = np.linalg.norm(offset)
            if distance <= self.radius:
                self.wake_offset = None
                self._quiet();return self.accepted.copy()
            if self.precision:
                if not new:
                    return self.accepted.copy()
                previous = self.wake_offset
                self.wake_offset = offset
                if previous is None or np.dot(previous, offset) < .5 * np.linalg.norm(previous) * distance:
                    return self.accepted.copy()
                self.wake_offset = None
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
            if np.linalg.norm(np.ptp(points,axis=0))<=self.rest_span and np.linalg.norm(slope)<=self.rest_speed:
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
