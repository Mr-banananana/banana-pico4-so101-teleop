"""Typed planner outcomes; hardware and input failures are not recoverable here."""


class MotionConstraintError(ValueError):
    """A valid target currently has no feasible bounded motion step."""
