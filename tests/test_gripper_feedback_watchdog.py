import unittest
from types import SimpleNamespace

import numpy as np

from mujoco_pick_place.pico_teleop.controller import TeleopController


class GripperFeedbackWatchdogTests(unittest.TestCase):
    def test_gripper_only_closure_does_not_start_arm_stall_watchdog(self):
        controller = TeleopController.__new__(TeleopController)
        controller.backend = SimpleNamespace(tracking_lead=np.ones(6))
        controller.tracking_limited = False
        controller._tracking_wait = None
        stops = []
        controller.stop = stops.append

        reference = np.zeros(6)
        current = np.array([-0.75, 0, 0, 0, 0, 0])
        command = np.array([0, 0, 0, 0, 0, -0.1])

        controller._pace_physical(command, reference, current, 0.0)
        controller._pace_physical(command, reference, current, 2.0)

        self.assertEqual(stops, [])
        self.assertIsNone(controller._tracking_wait)

    def test_active_arm_target_still_stops_after_persistent_stall(self):
        controller = TeleopController.__new__(TeleopController)
        controller.backend = SimpleNamespace(tracking_lead=np.ones(6))
        controller.tracking_limited = False
        controller._tracking_wait = None
        stops = []
        controller.stop = stops.append

        reference = np.zeros(6)
        current = np.zeros(6)
        command = np.array([0.75, 0, 0, 0, 0, 0])

        controller._pace_physical(command, reference, current, 0.0)
        controller._pace_physical(command, reference, current, 1.6)

        self.assertEqual(len(stops), 1)
        self.assertIn("电机反馈持续未跟上目标", stops[0])


if __name__ == "__main__":
    unittest.main()
