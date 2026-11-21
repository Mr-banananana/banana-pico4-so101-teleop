import unittest

from mujoco_pick_place.pico_teleop.gripper_contact import GripperContactGuard


class GripperContactGuardTests(unittest.TestCase):
    def test_detected_contact_gets_eight_count_closing_preload(self):
        guard = GripperContactGuard()
        guard.configure(closed=0, opened=1000)
        guard.command(requested=500, actual=600, captured_at=0.0)

        guard.observe(actual=570, captured_at=0.05)
        guard.observe(actual=570, captured_at=0.10)
        latched = guard.observe(actual=570, captured_at=0.151)

        self.assertTrue(latched)
        self.assertEqual(guard.contact_raw, 562)
        self.assertTrue(guard.diagnostics["position_control_only"])
        self.assertFalse(guard.diagnostics["force_calibrated"])


if __name__ == "__main__":
    unittest.main()
