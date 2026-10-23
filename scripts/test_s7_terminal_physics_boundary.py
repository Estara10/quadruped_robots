"""Regression checks for a physical fall stop between two policy decisions."""
import copy
import unittest

from s1_analyze_run import motion_safety, terminal_safety_end_time


class TerminalPhysicsBoundaryTest(unittest.TestCase):
    def setUp(self):
        self.event = {"start_physics_step": 5956, "start_sim_time": 11.912,
                      "end_physics_step": 6106, "end_sim_time": 12.212,
                      "confirmed": True, "closed": False,
                      "confirmed_physics_step": 6106, "confirmed_sim_time": 12.212}
        self.snapshot = {"status": "LIVE", "version": 5, "capture_id": "capture-a",
                         "sequence": 12214, "monotonic_ns": 200,
                         "physics_step": 6106, "sim_time": 12.212,
                         "physics_coverage_complete": True,
                         "collision_history_overflow": False, "fall_history_overflow": False,
                         "event_latches_complete": True, "fall_history": [self.event]}
        self.context = {"capture_id": "capture-a",
                        "result_contract": {"classification": "FALL_TERMINATION"},
                        "lifecycle": {"fall_observed": [copy.deepcopy(self.event)],
                                      "post_terminal_strategy_stop": {"publish_start_monotonic_ns": 300}}}
        self.payload = {"collision_snapshot": self.snapshot}

    def test_confirmed_fall_uses_terminal_detection_sample(self):
        end, info = terminal_safety_end_time(self.context, self.payload, 12.202)
        self.assertEqual(end, 12.212)
        self.assertTrue(info["applied"])
        start = {**self.snapshot, "sequence": 11802, "physics_step": 5900,
                 "sim_time": 11.8, "fall_history": []}
        result = motion_safety([{"collision_snapshot": start}, self.payload], 11.8, end,
                               start_boundary_snapshot=start, end_boundary_snapshot=self.snapshot)
        self.assertIs(result["fall"], True)

    def test_arrival_does_not_admit_later_fall(self):
        self.context["result_contract"]["classification"] = "ARRIVED"
        self.assertEqual(terminal_safety_end_time(self.context, self.payload, 12.202)[0], 12.202)

    def test_missing_supervisor_event_does_not_extend(self):
        self.context["lifecycle"]["fall_observed"] = []
        self.assertFalse(terminal_safety_end_time(self.context, self.payload, 12.202)[1]["applied"])

    def test_different_capture_does_not_extend(self):
        self.snapshot["capture_id"] = "other-capture"
        self.assertFalse(terminal_safety_end_time(self.context, self.payload, 12.202)[1]["applied"])

    def test_cleanup_sample_does_not_extend(self):
        self.snapshot["monotonic_ns"] = 301
        self.assertFalse(terminal_safety_end_time(self.context, self.payload, 12.202)[1]["applied"])

    def test_different_confirmation_does_not_extend(self):
        self.context["lifecycle"]["fall_observed"][0]["confirmed_physics_step"] = 6107
        self.assertFalse(terminal_safety_end_time(self.context, self.payload, 12.202)[1]["applied"])


if __name__ == "__main__":
    unittest.main()
