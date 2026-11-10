"""Per-axis thermal evidence, independent of motion and serial I/O.

A bulk anomaly suspends motion while NEW measurements establish its value.
We never substitute a saved temperature for a measurement. Register consensus
can resolve a safe bulk discrepancy; heat always needs temporal confirmation.
"""
import math
import numpy as np


def diagnostic(value):
    """Keep raw evidence JSON-safe, including corrupt/nonfinite samples."""
    if isinstance(value,dict):return {k:diagnostic(v) for k,v in value.items()}
    if isinstance(value,(list,tuple,np.ndarray)):return [diagnostic(v) for v in value]
    if isinstance(value,np.generic):return diagnostic(value.item())
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


class ThermalValidator:
    alarm=60
    jump=8
    expiry=.5
    confirmation_span=.12
    recovery_span=.06

    def __init__(self):
        self.axes=[dict(value=None,validated_at=None,pending_since=None,
                        candidate=None,samples=0,first_at=None,last_at=None) for _ in range(6)]
        self.pending=False;self.fault_axis=None;self.fault_reason=None
        self.diagnostics=dict(count=0,last=None,history=[])

    @staticmethod
    def usable(value):
        return bool(np.isfinite(value) and 0<=value<=150 and value==int(value))

    def observe(self,data,captured,received):
        raw=np.asarray(data.get('temperature',[]),float)
        values=raw.copy() if raw.shape==(6,) else np.full(6,np.nan)
        readback=data.get('readback') or {};pairs=readback.get('temperature_confirm_raw',[])
        sources=['bulk']*6;paired=set()
        if isinstance(pairs,(list,tuple)) and len(pairs)==6 and received-captured<=.1:
            for i,proof in enumerate(pairs):
                pair=np.asarray(proof.get('values',[]) if isinstance(proof,dict) else [],float)
                if pair.shape==(2,) and all(self.usable(v) for v in pair) and abs(pair[0]-pair[1])<=2:
                    values[i]=max(pair);sources[i]='independent_register_pair';paired.add(i)
        was_pending=self.pending;before=[a['value'] for a in self.axes]
        anomalous=[];self.fault_axis=None;self.fault_reason=None
        for i,(axis,value) in enumerate(zip(self.axes,values)):
            valid=self.usable(value)
            baseline=axis['value']
            sudden=valid and baseline is not None and abs(value-baseline)>=self.jump
            suspect=not valid or value>=self.alarm or (sudden and i not in paired)
            if axis['pending_since'] is None and not suspect:
                axis.update(value=float(value),validated_at=captured)
                continue
            anomalous.append(i+1)
            if axis['pending_since'] is None:axis['pending_since']=captured
            if received-axis['pending_since']+1e-9>=self.expiry:
                if self.fault_axis is None:
                    self.fault_axis=i+1;self.fault_reason='温度反馈待确认超过500ms'
                continue
            if valid and value<self.alarm and i in paired:
                axis.update(value=float(value),validated_at=captured,pending_since=None,
                            candidate=None,samples=0,first_at=None,last_at=None)
                continue
            kind='hot' if valid and value>=self.alarm else 'safe' if valid else 'invalid'
            candidate=axis['candidate']
            consistent=(kind!='invalid' and candidate is not None and candidate[0]==kind
                        and abs(candidate[1]-value)<=2)
            if not consistent:
                axis.update(candidate=(kind,float(value)) if valid else None,
                            samples=1 if valid else 0,first_at=captured,last_at=captured)
            elif captured>axis['last_at']:
                axis['samples']+=1;axis['last_at']=captured
            span=captured-axis['first_at']
            near_baseline=valid and baseline is not None and abs(value-baseline)<self.jump
            count=2 if near_baseline else 3
            duration=self.recovery_span if near_baseline else self.confirmation_span
            if kind=='hot' and axis['samples']>=3 and span+1e-9>=self.confirmation_span:
                if self.fault_axis is None:self.fault_axis=i+1;self.fault_reason='持续高温已由新鲜测量确认'
            elif kind=='safe' and axis['samples']>=count and span+1e-9>=duration:
                axis.update(value=float(value),validated_at=captured,pending_since=None,
                            candidate=None,samples=0,first_at=None,last_at=None)
            if (axis['pending_since'] is not None and received-axis['pending_since']+1e-9>=self.expiry
                    and self.fault_axis is None):
                self.fault_axis=i+1;self.fault_reason='温度反馈待确认超过500ms'
        self.pending=any(a['pending_since'] is not None for a in self.axes)
        discrepancy=any(i in paired and (not np.isfinite(raw[i]) or values[i]!=raw[i])
                        for i in range(6)) if raw.shape==(6,) else False
        if anomalous or discrepancy:
            event=dict(original=diagnostic(data),baseline=before,
                       axes=anomalous or [i+1 for i in paired],started_at=captured,
                       recovered=not self.pending,source='independent_register_pair' if discrepancy else 'temporal_samples',
                       pending=self.pending,states=diagnostic(self.axes))
            if self.fault_reason:event['reason']=self.fault_reason
            self.diagnostics['count']+=1;self.diagnostics['last']=event
            self.diagnostics['history'].append(event)
            self.diagnostics['history']=self.diagnostics['history'][-16:]
        elif was_pending and not self.pending and self.diagnostics['last']:
            self.diagnostics['last'].update(recovered=True,pending=False)
        self.diagnostics['pending']=self.pending
        self.diagnostics['axes']=diagnostic(self.axes)
        return diagnostic(dict(data,temperature=diagnostic(values),
                    validated_temperature=[a['value'] for a in self.axes],
                    temperature_validated_at=[a['validated_at'] for a in self.axes],
                    readback=dict(readback,temperature_selected_source=sources)))
