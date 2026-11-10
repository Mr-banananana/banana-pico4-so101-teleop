"""Deliberate trigger edges latch intent, independently of the pose clutch."""
import math


class ToggleGrasp:
    def __init__(self,opening):
        self.opening=float(opening)
        if not math.isfinite(self.opening) or not 0<=self.opening<=1:
            raise ValueError('invalid gripper opening')
        self.reset(self.opening)

    def reset(self,current):
        self.target=float(current)
        self.closing=False
        self.active=False
        self._released=True

    def update(self,trigger):
        if trigger<=.2:self._released=True
        elif trigger>=.65 and self._released:
            self._released=False
            self.closing=not self.closing
            self.active=True
            self.target=0. if self.closing else self.opening
        return self.target
