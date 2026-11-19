import threading
import unittest
from types import SimpleNamespace

import numpy as np

from mujoco_pick_place.pico_teleop.controller import TeleopController


class SimulationBackend:
    kind = "kinematic_simulation"

    def read(self):
        return np.zeros(6)

    def hold(self):
        pass


class RecoveryWorker:
    def __init__(self):
        self.enabled = False
        self.fault = "test feedback fault"
        self.enabling = False
        self.enable_request = None
        self._state_lock = threading.RLock()
        self.bus = SimpleNamespace(is_virtual=False)


class PhysicalBackend:
    kind = "so101_real"

    def __init__(self, *, enable_error=None):
        self.worker = RecoveryWorker()
        self.kinematics = SimpleNamespace()
        self.profile = SimpleNamespace(data={"workcell_verified": True})
        self.work_min = np.array([0.0, 0.0, 0.0])
        self.work_max = np.ones(3)
        self.ready = False
        self.enable_error = enable_error
        self.enable_calls = 0

    def read(self):
        return np.zeros(6)

    def hold(self):
        pass

    def hardware_status(self):
        return {"fault": self.worker.fault}

    def control(self, action, message):
        self.enable_calls += 1
        if self.enable_error:
            raise ValueError(self.enable_error)
        self.worker.enabling = True
        self.worker.enable_request = object()


def frame(seq, *, arm=False, valid=True, visible=True, stop=False, mode="immersive-vr"):
    right = {
        "valid": valid,
        "grip": 0.0,
        "trigger": 0.0,
        "precision": False,
        "stop": stop,
        "arm": arm,
        "emulated": False,
    }
    if valid:
        right.update(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
    return {
        "type": "frame",
        "session_id": "test-session",
        "seq": seq,
        "client_time_ms": 10_000.0 + seq,
        "space_epoch": 0,
        "reference_space": "local-floor",
        "mode": mode,
        "visible": visible,
        "right": right,
    }


def connected_controller(backend, now=10.0):
    controller = TeleopController(backend)
    controller.attach("test-session")
    controller.clock.bounds = (0.0, 0.0)
    controller.clock.updated_at = now
    controller.stop("test stop")
    return controller


class StopRecoveryTests(unittest.TestCase):
    def test_short_a_press_resets_software_stop_without_release_prerequisite(self):
        controller = connected_controller(SimulationBackend())

        controller.ingest(frame(1, arm=True), 10.001)

        self.assertFalse(controller.stopped)
        self.assertFalse(controller.armed)
        self.assertIn("停止已解除", controller.reason)

    def test_invalid_or_hidden_input_cannot_reset_stop(self):
        controller = connected_controller(SimulationBackend())

        controller.ingest(frame(1, arm=True, valid=False, visible=False), 10.001)

        self.assertTrue(controller.stopped)

    def test_failed_recovery_does_not_retry_until_a_is_released(self):
        backend = PhysicalBackend(enable_error="fresh feedback unavailable")
        controller = connected_controller(backend)

        controller.ingest(frame(1, arm=True, mode="immersive-ar"), 10.001)
        controller.ingest(frame(2, arm=True, mode="immersive-ar"), 10.002)
        controller.ingest(frame(3, arm=True, mode="immersive-ar"), 10.003)

        self.assertEqual(backend.enable_calls, 1)
        self.assertTrue(controller.stopped)
        self.assertEqual(controller._fault_recovery_state, "failed")

        controller.ingest(frame(4, arm=False, mode="immersive-ar"), 10.004)
        controller.ingest(frame(5, arm=True, mode="immersive-ar"), 10.005)
        self.assertEqual(backend.enable_calls, 2)

    def test_async_hardware_recovery_finishes_after_a_is_released(self):
        backend = PhysicalBackend()
        controller = connected_controller(backend)

        controller.ingest(frame(1, arm=True, mode="immersive-ar"), 10.001)
        self.assertTrue(controller.stopped)
        self.assertEqual(controller._fault_recovery_state, "pending")
        controller.ingest(frame(2, arm=False, mode="immersive-ar"), 10.002)

        backend.worker.enabled = True
        backend.worker.fault = None
        backend.worker.enabling = False
        backend.worker.enable_request = None
        backend.ready = True
        controller._advance_stop_recovery()

        self.assertFalse(controller.stopped)
        self.assertEqual(controller._fault_recovery_state, "idle")

    def test_pending_recovery_does_not_queue_duplicate_enable_requests(self):
        backend = PhysicalBackend()
        controller = connected_controller(backend)

        for seq in (1, 2, 3):
            controller.ingest(frame(seq, arm=True, mode="immersive-ar"), 10.0 + seq / 1000)

        self.assertEqual(backend.enable_calls, 1)
        self.assertTrue(controller.stopped)
        self.assertEqual(controller._fault_recovery_state, "pending")

    def test_expired_clock_sample_cannot_reset_or_enable(self):
        backend = PhysicalBackend()
        controller = connected_controller(backend)
        controller.clock.updated_at = 0.0

        with self.assertRaisesRegex(ValueError, "源时间"):
            controller.ingest(frame(1, arm=True, mode="immersive-ar"), 10.001)

        self.assertTrue(controller.stopped)
        self.assertEqual(backend.enable_calls, 0)

    def test_vr_input_never_enables_real_hardware_during_recovery(self):
        backend = PhysicalBackend()
        controller = connected_controller(backend)

        controller.ingest(frame(1, arm=True, mode="immersive-vr"), 10.001)

        self.assertEqual(backend.enable_calls, 0)
        self.assertTrue(controller.stopped)
        self.assertEqual(controller._fault_recovery_state, "failed")


if __name__ == "__main__":
    unittest.main()
