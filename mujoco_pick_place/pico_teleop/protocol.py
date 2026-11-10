"""Strict snapshots and bounded clock-offset estimation for XR input."""
from dataclasses import dataclass
import math
import secrets

import numpy as np


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: finite number required")
    try:value=float(value)
    except OverflowError:raise ValueError(f"{name}: finite number required") from None
    if not math.isfinite(value):raise ValueError(f"{name}: finite number required")
    return value


def integer(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name}: non-negative integer required")
    return value


def boolean(value, name):
    if type(value) is not bool:
        raise ValueError(f"{name}: boolean required")
    return value


def vector(value, count, name):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{name}: {count} values required")
    return np.array([number(v, name) for v in value], dtype=float)


@dataclass(frozen=True)
class InputFrame:
    seq: int
    client_time_ms: float
    space_epoch: int
    reference_space: str
    mode: str
    visible: bool
    valid: bool
    position: np.ndarray | None
    orientation: np.ndarray | None
    grip: float
    trigger: float
    precision: bool
    stop: bool
    arm: bool
    emulated: bool


def parse_frame(payload, session_id, last_seq):
    if not isinstance(payload, dict) or payload.get("type") != "frame":
        raise ValueError("frame object required")
    if payload.get("session_id") != session_id:
        raise ValueError("wrong session")
    seq = integer(payload.get("seq"), "sequence")
    if seq <= last_seq:
        raise ValueError("sequence must increase")
    epoch = integer(payload.get("space_epoch"), "space_epoch")
    reference = payload.get("reference_space")
    mode = payload.get("mode")
    if reference not in ("local", "local-floor", "bounded-floor"):
        raise ValueError("unsupported reference space")
    if mode not in ("immersive-ar", "immersive-vr", "synthetic"):
        raise ValueError("unsupported XR mode")
    visible = boolean(payload.get("visible"), "visible")
    right = payload.get("right")
    if not isinstance(right, dict):
        raise ValueError("right controller required")
    valid = boolean(right.get("valid"), "valid")
    position = vector(right.get("position"), 3, "position") if valid else None
    orientation = vector(right.get("orientation"), 4, "orientation") if valid else None
    if valid:
        norm = np.linalg.norm(orientation)
        if not math.isfinite(norm) or norm < 1e-8 or np.linalg.norm(position) > 100:
            raise ValueError("invalid pose")
        orientation = orientation / norm
    grip, trigger = (number(right.get(k), k) for k in ("grip", "trigger"))
    if not (0 <= grip <= 1 and 0 <= trigger <= 1):
        raise ValueError("grip/trigger outside [0,1]")
    return InputFrame(seq, number(payload.get("client_time_ms"), "client time"), epoch,
                      reference, mode, visible, valid, position, orientation, grip, trigger,
                      boolean(right.get("precision"), "precision"),
                      boolean(right.get("stop"), "stop"), boolean(right.get("arm"), "arm"),
                      boolean(right.get("emulated",False), "emulated"))


class ClockSync:
    """Client minus server offset interval; all wire times in milliseconds."""
    def __init__(self):
        self.pending = {}
        self.bounds = None
        self.updated_at = None

    def challenge(self, now):
        self.pending = {k:v for k,v in self.pending.items() if now-v < .25}
        key = secrets.token_hex(8)
        self.pending[key] = now
        return dict(type="clock_ping", id=key)

    def accept(self, message, now):
        key = message.get("id")
        sent = self.pending.pop(key, None)
        if sent is None or not 0 <= now-sent <= .25:
            raise ValueError("unknown or expired clock challenge")
        c1 = number(message.get("client_recv_ms"), "client receive time")
        c2 = number(message.get("client_send_ms"), "client send time")
        low, high = c2-now*1000, c1-sent*1000
        if c2 < c1 or low > high + .001:
            raise ValueError("impossible clock reply")
        self.bounds = (low, high)
        self.updated_at = now

    def age_upper(self, client_ms, now):
        if self.bounds is None or now-self.updated_at > 5:
            return math.inf
        return (now*1000-client_ms+self.bounds[1])/1000

    def valid_time(self, client_ms, now):
        age = self.age_upper(client_ms, now)
        if not math.isfinite(age):
            return False
        # Also reject a source timestamp implausibly ahead of the server.
        earliest_age = (now*1000-client_ms+self.bounds[0])/1000
        return age <= .2 and earliest_age >= -.05
