import unittest

import numpy as np

from mujoco_pick_place.so101.kinematics import Kinematics


class PitchIkRollIsolationTests(unittest.TestCase):
    def setUp(self):
        self.kinematics = Kinematics()
        self.seed = np.array([0.1035689886, -0.1365576294, 0.6567041054,
                              0.9098727441, -1.4584047947])
        self.position, rotation = self.kinematics.fk(self.seed)
        forward = rotation[:, 0]
        self.tool_pitch = float(np.arctan2(-forward[2], np.linalg.norm(forward[:2])))

    def test_lateral_position_step_does_not_change_wrist_roll(self):
        kinematics = self.kinematics

        candidate = kinematics.ik_pitch_step(
            self.position + np.array([0.0, 0.02, 0.0]),
            self.tool_pitch,
            self.seed,
            np.full(5, -0.03),
            np.full(5, 0.03),
            0.03,
        )

        self.assertAlmostEqual(candidate[4], self.seed[4], delta=1e-8)

    def test_pitch_command_still_uses_other_joints_and_holds_wrist_roll(self):
        candidate = self.kinematics.ik_pitch_step(
            self.position,
            self.tool_pitch + 0.05,
            self.seed,
            np.full(5, -0.03),
            np.full(5, 0.03),
            0.03,
        )
        actual_position, rotation = self.kinematics.fk(candidate)
        forward = rotation[:, 0]
        actual_pitch = float(np.arctan2(-forward[2], np.linalg.norm(forward[:2])))

        self.assertAlmostEqual(candidate[4], self.seed[4], delta=1e-8)
        self.assertLess(abs(actual_pitch - (self.tool_pitch + 0.05)), 0.05)
        self.assertLess(np.linalg.norm(actual_position - self.position), 0.002)

    def test_braking_bounds_allow_only_the_minimum_required_roll_step(self):
        lower = np.array([-0.03, -0.03, -0.03, -0.03, 0.005])
        candidate = self.kinematics.ik_pitch_step(
            self.position,
            self.tool_pitch,
            self.seed,
            lower,
            np.full(5, 0.03),
            0.03,
        )

        self.assertAlmostEqual(candidate[4] - self.seed[4], 0.005, delta=1e-8)


if __name__ == "__main__":
    unittest.main()
