import unittest

import numpy as np

from mujoco_pick_place.pico_teleop.intent import HandIntent
from mujoco_pick_place.pico_teleop.motion import profile_for


class HandIntentTests(unittest.TestCase):
    def setUp(self):
        self.motion = profile_for("agile_fast")
        self.origin = np.zeros(3)

    def test_fast_step_followed_by_turn_reacquires_during_motion(self):
        intent = HandIntent(self.origin, self.motion, 0)
        accepted = [intent.update([.025, i * .005, 0.], .02, i + 1) for i in range(6)]
        np.testing.assert_array_equal(accepted[0], self.origin)
        self.assertGreater(accepted[3][1], .010)
        self.assertGreater(accepted[5][1], accepted[3][1])

    def test_skipped_frames_use_elapsed_pose_sample_time(self):
        intent = HandIntent(self.origin, self.motion, 1)
        intent.update(self.origin, .020, 1, sample_time_ms=1000.)
        position = intent.update([.019, 0., 0.], .020, 4, sample_time_ms=1040.)
        self.assertGreater(position[0], .010)
        self.assertEqual(intent.status()["pending_samples"], 0)

    def test_isolated_jump_never_moves_robot_target(self):
        intent = HandIntent(self.origin, self.motion, 0)
        for sequence, position in enumerate(([.09, 0., 0.], self.origin, self.origin), 1):
            accepted = intent.update(position, .02, sequence)
            np.testing.assert_array_equal(accepted, self.origin)

    def test_prediction_drops_old_direction_on_first_reverse_sample(self):
        intent = HandIntent(self.origin, self.motion, 0)
        for sequence in range(1, 8):
            intent.update([sequence * .01, 0., 0.], .02, sequence)
        self.assertGreater(intent.prediction_velocity[0], .1)

        intent.update([.067, 0., 0.], .02, 8)

        np.testing.assert_array_equal(intent.prediction_velocity, self.origin)

    def test_precision_follows_a_one_millimeter_step_after_confirmation(self):
        intent = HandIntent(self.origin, self.motion, 0, precision=True)
        sample = [.001, 0., 0.]
        np.testing.assert_array_equal(intent.update(sample, .02, 1), self.origin)
        np.testing.assert_array_equal(intent.update(sample, .02, 1), self.origin)
        self.assertGreater(intent.update(sample, .02, 2)[0], .00045)

    def test_precision_rejects_stationary_jitter_and_one_sample_wake(self):
        intent = HandIntent(self.origin, self.motion, 0, precision=True)
        samples = [[.001, 0., 0.]] + [[(-1)**i * .00055, 0., 0.] for i in range(30)]
        for sequence, sample in enumerate(samples, 1):
            np.testing.assert_array_equal(intent.update(sample, .02, sequence), self.origin)

    def test_precision_keeps_tracking_slow_motion_beyond_rest_window(self):
        intent = HandIntent(self.origin, self.motion, 0, precision=True)
        for sequence in range(1, 100):
            position = intent.update([sequence * .00008, 0., 0.], .02, sequence)
        self.assertGreater(position[0], .007)
