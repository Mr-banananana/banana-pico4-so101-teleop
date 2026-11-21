import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from mujoco_pick_place.pico_teleop.commission import RawCalibration
from mujoco_pick_place.pico_teleop.simulation import SimulationArm
from mujoco_pick_place.pico_teleop.workspace_entry import WorkspaceEntry


class EntryMotor:
    """Model a measured offset under load using only synthetic calibration."""

    def __init__(self, raw, lag):
        self.initial = raw.copy()
        self.raw = raw.copy()
        self.lag = np.array(lag)
        self.last_sent = raw.copy()
        self.stamp = 0.
        self.enabled = True
        self.fault = None
        self.thermal_pending = False
        self.hold_epoch = 0
        self.active = self.pending = self.latch_request = None
        self.holding = True

    def snapshot(self):
        return dict(raw=self.raw.tolist(), voltage_raw=[120]*6, captured_at=self.stamp)

    def hold(self):
        self.active = None
        self.holding = True
        self.last_sent = self.raw.copy()
        self.initial = self.raw.copy()

    def submit(self, raw, deadline, guard):
        if not guard():
            raise AssertionError("Entry command failed the execution guard")
        self.last_sent = raw.copy()
        self.holding = False
        self.active = (deadline, guard)
        difference = raw - self.initial
        self.raw = self.initial + np.sign(difference) * np.maximum(abs(difference) - self.lag, 0)

    def latch_hold(self, raw, guard, *, tolerance_counts):
        if guard() and np.max(abs(raw - self.raw)) <= tolerance_counts:
            self.active = None
            self.holding = True


class WorkspaceEntryTests(unittest.TestCase):
    def run_entry(self, lag):
        model = SimulationArm()
        signature = {}
        profile = RawCalibration(dict(schema=1, signature=signature,
            supply_version="12V", pose_verified=True, directions_verified=True,
            workcell_verified=True, zero_counts=[2048]*5, signs=[1]*5,
            raw_bounds=[[600, 3500]]*5, gripper_closed=1900, gripper_open=3000),
            signature, model.limits)
        initial = profile.encode(np.array([.972, .594, .290, 1.037, .186, .8]))
        worker = EntryMotor(initial, lag)
        arm = SimpleNamespace(kinematics=model.kinematics, model=model.model,
            work_min=np.array([.06, -.28, -.06]), work_max=np.array([.47, .28, .40]),
            worker=worker, confirmed_supply="12V", profile=profile, limits=profile.limits,
            command_reference=profile.project_command,
            read=lambda: profile.decode(worker.raw))
        entry = WorkspaceEntry(arm, margin=.015, entry_margin=.020, lead_counts=80)
        self.assertFalse(entry._inside(arm.kinematics.fk(arm.read()[:5])[0]))
        with patch("mujoco_pick_place.pico_teleop.workspace_entry.time") as clock:
            clock.monotonic.side_effect = lambda: worker.stamp
            for tick in range(750):
                worker.stamp = tick * .02
                entry.tick(worker.stamp, .02)
                if entry.state in ("ready", "blocked"):
                    break
        return entry, arm

    def test_partial_motion_at_shallow_goal_replans_inward_and_reaches_ready(self):
        entry, arm = self.run_entry([0, 32, 0, 7, 0, 0])
        self.assertEqual(entry.state, "ready", entry.message)
        self.assertGreater(entry.planned_margin, entry.entry_margin)
        self.assertTrue(entry._inside(arm.kinematics.fk(arm.read()[:5])[0]))
        self.assertLess(arm.worker.stamp, 5.)

    def test_normal_motion_keeps_the_nearest_goal(self):
        entry, arm = self.run_entry([0]*6)
        self.assertEqual(entry.state, "ready", entry.message)
        self.assertEqual(entry.planned_margin, entry.entry_margin)
        self.assertTrue(arm.worker.holding)

    def test_no_encoder_progress_still_blocks_without_deeper_replan(self):
        entry, arm = self.run_entry([4095]*6)
        self.assertEqual(entry.state, "blocked", entry.message)
        self.assertEqual(entry.planned_margin, entry.entry_margin)
        self.assertTrue(arm.worker.holding)
