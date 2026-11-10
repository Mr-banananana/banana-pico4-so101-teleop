"""Serial-owner position-contact guard; this is not calibrated force control.

No register writes or current/load-to-force assumptions are made here. A small
position lead limits persistent closure error, and a feedback plateau latches a
single contact goal instead of repeatedly following drift into the object.
"""
import math


class GripperContactGuard:
    max_closing_lead_counts=24
    plateau_counts=2
    plateau_seconds=.100
    minimum_samples=3
    closing_error_counts=8
    contact_lead_counts=4
    opening_release_counts=8

    def __init__(self):
        self.calibration=None;self.requested_raw=None;self.target_raw=None
        self.contact_raw=None;self._samples=[];self._last_capture=None

    def configure(self,closed=None,opened=None):
        calibration=None if closed is None or opened is None else (float(closed),float(opened))
        if calibration is not None:
            if (not all(math.isfinite(v) and 0<=v<=4095 for v in calibration)
                    or abs(calibration[1]-calibration[0])<100):
                raise ValueError('夹爪接触保护需要有效的闭合和打开标定')
        if calibration==self.calibration:return
        self.calibration=calibration;self.requested_raw=None;self.target_raw=None
        self.contact_raw=None;self._samples=[];self._last_capture=None

    @property
    def direction(self):return 1 if self.calibration[1]>self.calibration[0] else -1

    def _clip(self,raw):
        low,high=sorted(self.calibration)
        return int(min(math.floor(high),max(math.ceil(low),round(raw))))

    def _closing(self,actual):
        return (self.calibration is not None and self.requested_raw is not None
                and (actual-self.requested_raw)*self.direction>=self.closing_error_counts)

    def observe(self,actual,captured_at):
        """Consume each fresh feedback capture once, returning a new latch flag."""
        if self._last_capture is not None and captured_at<=self._last_capture:return False
        self._last_capture=captured_at
        if self.contact_raw is not None or not self._closing(actual):
            self._samples=[];return False
        samples=self._samples+[(float(captured_at),int(actual))]
        # Keep the longest trailing window whose total excursion is <=2 counts.
        while samples and max(v for _,v in samples)-min(v for _,v in samples)>self.plateau_counts:
            samples.pop(0)
        self._samples=samples
        if (len(samples)<self.minimum_samples
                or captured_at-samples[0][0]+1e-9<self.plateau_seconds):return False
        self.contact_raw=self._clip(actual-self.direction*self.contact_lead_counts)
        self.target_raw=self.contact_raw;self._samples=[]
        return True

    def command(self,requested,actual,captured_at,*,opening=False):
        """Bound a validated command; only a deliberate opening clears contact."""
        if self.calibration is None:return int(requested)
        requested=self._clip(requested)
        if self.contact_raw is not None:
            if not opening and (requested-self.contact_raw)*self.direction<=self.opening_release_counts:
                self.requested_raw=requested;self.target_raw=self.contact_raw
                return self.contact_raw
            self.contact_raw=None;self._samples=[]
        was_closing=self._closing(actual)
        self.requested_raw=requested
        if self._closing(actual):
            if not was_closing:
                self._samples=[(float(captured_at),int(actual))]
                self._last_capture=captured_at
            target=actual-self.direction*min((actual-requested)*self.direction,self.max_closing_lead_counts)
        else:
            self._samples=[];target=requested
        self.target_raw=self._clip(target)
        return self.target_raw

    def hold(self,actual):
        # Interrupting the arm cancels an unfinished closure, but an established
        # contact goal remains fixed. This never changes the observed position.
        self._samples=[];self.requested_raw=None
        if self.calibration is None:return int(actual)
        self.target_raw=self.contact_raw if self.contact_raw is not None else int(actual)
        return self.target_raw

    def outgoing(self,raw):
        return int(self.contact_raw if self.contact_raw is not None else raw)

    @property
    def diagnostics(self):
        target=None
        if self.calibration is not None and self.target_raw is not None:
            closed,opened=self.calibration;target=float((self.target_raw-closed)/(opened-closed))
        return dict(state='contact_hold' if self.contact_raw is not None else 'closing' if self._samples else 'inactive',
            contact=self.contact_raw is not None,target_raw=self.target_raw,requested_raw=self.requested_raw,
            target=target,position_control_only=True,force_calibrated=False,
            max_closing_lead_counts=self.max_closing_lead_counts,contact_lead_counts=self.contact_lead_counts,
            plateau_samples=len(self._samples))
