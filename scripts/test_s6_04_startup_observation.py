#!/usr/bin/env python3
"""Focused offline checks for the one-off S6-04 startup observation gate."""
import unittest
from types import SimpleNamespace

from s1_run_diagnostic import (
    CollisionStatus,
    SIM_CLOCK_STALE,
    advance_startup_observation,
    new_startup_observation_state,
    note_startup_policy_frame,
    parse_startup_trace_lines,
    select_strategy_terminal,
    startup_low_height_only_fall,
    startup_normal_posture,
)


def fall_snapshot(**updates):
    value = {
        "status": CollisionStatus.LIVE.value,
        "version": 5,
        "physics_coverage_complete": True,
        "unknown_contacts": 0,
        "invalid_posture_steps": 0,
        "fall_condition_flags": 1,
        "fall_candidate": True,
        "fall_confirmed": True,
        "base_height_m": 0.10,
        "base_roll_rad": 0.02,
        "base_pitch_rad": -0.03,
        "fall_history": [{"confirmed": True, "closed": False, "start_sim_time": 3.0}],
    }
    value.update(updates)
    return value


def monitor_fixture():
    anchor = {"sim_time_s": 10.0, "sequence": 100, "physics_step": 1000,
              "monotonic_ns": 1_000_000_000, "unknown_contact_steps": 0,
              "invalid_posture_steps": 0, "fall_history_count": 0}
    return new_startup_observation_state(anchor, 4.0, capture_id="run-capture",
                                         model_fingerprint="model", scenario_id="random_04",
                                         session_id=77)


def physics_payload(sequence, step, sim_time, **updates):
    value = {
        "version": 5, "status": "LIVE", "authoritative": True,
        "physics_coverage_complete": True, "event_latches_complete": True,
        "capture_id": "run-capture", "runtime_model_fingerprint": "model",
        "scenario_id": "random_04", "sequence": sequence, "physics_step": step,
        "sim_time": sim_time, "monotonic_ns": sequence * 10_000_000,
        "unknown_contacts": 0, "unknown_contact_steps": 0,
        "invalid_posture_steps": 0, "fall_condition_flags": 0,
        "fall_candidate": False, "fall_confirmed": False,
        "base_height_m": 0.30, "base_roll_rad": 0.0, "base_pitch_rad": 0.0,
        "robot_obstacle_contacts": 0, "nonfoot_obstacle_contacts": 0,
        "collision_history": [], "collision_history_overflow": False,
        "fall_history": [], "fall_history_count": 0, "fall_history_overflow": False,
        "event_latches_complete": True, "authoritative": True,
    }
    value.update(updates)
    if "fall_history_count" not in updates:
        value["fall_history_count"] = len(value["fall_history"])
    return value


class StartupObservationTests(unittest.TestCase):
    def test_only_fresh_low_height_episode_is_delay_eligible(self):
        allowed, _ = startup_low_height_only_fall(fall_snapshot(), 2.9)
        self.assertTrue(allowed)
        for update in (
            {"fall_condition_flags": 3},
            {"base_roll_rad": 1.1},
            {"fall_history": [{"confirmed": True, "closed": False, "start_sim_time": 2.8}]},
            {"status": CollisionStatus.STALE.value},
            {"version": 4},
            {"physics_coverage_complete": False},
            {"unknown_contacts": 1},
            {"invalid_posture_steps": 1},
        ):
            allowed, _ = startup_low_height_only_fall(fall_snapshot(**update), 2.9)
            self.assertFalse(allowed, update)

    def test_normal_posture_requires_valid_live_state_and_closed_episode(self):
        normal, _ = startup_normal_posture({
            "status": CollisionStatus.LIVE.value, "version": 5,
            "physics_coverage_complete": True, "unknown_contacts": 0,
            "invalid_posture_steps": 0, "base_height_m": 0.23,
            "base_roll_rad": 0.0, "base_pitch_rad": 0.0,
            "fall_candidate": False, "fall_confirmed": False,
        })
        self.assertTrue(normal)
        normal, _ = startup_normal_posture({
            "status": CollisionStatus.LIVE.value, "version": 5,
            "physics_coverage_complete": True, "unknown_contacts": 0,
            "invalid_posture_steps": 0, "base_height_m": 0.23,
            "base_roll_rad": 0.0, "base_pitch_rad": 0.0,
            "fall_candidate": True, "fall_confirmed": False,
        })
        self.assertFalse(normal)

    def test_hard_stop_priority_is_unchanged(self):
        self.assertEqual(select_strategy_terminal(True, False, True, False, False),
                         "SYSTEM_SAFETY_ABORT")
        self.assertEqual(select_strategy_terminal(False, True, True, False, False),
                         "COLLISION_TERMINATION")

    def test_controller_trace_parser_requires_s6_schema(self):
        text = '[INFO] [S6-04-TRACE] {"schema":"s6-04-controller-startup-v1","policy_cycle":0}\n'
        self.assertEqual(parse_startup_trace_lines(text)[0]["policy_cycle"], 0)
        with self.assertRaises(ValueError):
            parse_startup_trace_lines('[INFO] [S6-04-TRACE] {"schema":"other"}\n')

    def test_policy_stale_clock_and_gaps_are_quality_only(self):
        state = monitor_fixture()
        stale = SimpleNamespace(rl_step=0, sim_clock_valid=False, sim_clock_status=SIM_CLOCK_STALE,
                                sequence=10, session_id=77)
        self.assertEqual(note_startup_policy_frame(state, stale)["status"], "OK")
        self.assertEqual(state["policy_clock_invalid_frames"], 1)
        self.assertEqual(state["policy_stale_clock_frames"], 1)
        later = SimpleNamespace(rl_step=3, sim_clock_valid=True, sequence=13, session_id=77)
        self.assertEqual(note_startup_policy_frame(state, later)["status"], "OK")
        self.assertEqual(state["policy_gaps"][0]["missing_policy_cycles"], 2)
        decision = advance_startup_observation(
            state, "LIVE", physics_payload(101, 1001, 10.01), 2_000_000_000)
        self.assertEqual(decision["status"], "CONTINUE")
        cycle_zero_missing_state = monitor_fixture()
        first_seen = SimpleNamespace(rl_step=2, sim_clock_valid=True,
                                     sequence=20, session_id=77)
        self.assertEqual(note_startup_policy_frame(cycle_zero_missing_state, first_seen)["status"], "OK")
        self.assertEqual(cycle_zero_missing_state["policy_gaps"][0]["missing_policy_cycles"], 2)

    def test_physics_missing_stale_invalid_reset_and_source_change_stop(self):
        for status, payload in (("MISSING", None), ("STALE", None), ("INVALID", None),
                                ("LIVE", physics_payload(101, 1001, 10.01,
                                                         capture_id="other-run")),
                                ("LIVE", physics_payload(101, 1001, 10.01,
                                                         runtime_model_fingerprint="other-model")),
                                ("LIVE", physics_payload(101, 1001, 10.01,
                                                         scenario_id="other-scene")),
                                ("LIVE", physics_payload(101, 1001, 9.99)),
                                ("LIVE", physics_payload(99, 999, 9.99)),
                                ("LIVE", physics_payload(101, 1001, 10.01,
                                                         unknown_contact_steps=1)),
                                ("LIVE", physics_payload(101, 1001, 10.01,
                                                         invalid_posture_steps=1)),
                                ("LIVE", physics_payload(101, 1001, 10.01,
                                                         fall_history_overflow=True)),
                                ("LIVE", physics_payload(101, 1001, 10.01,
                                                         physics_coverage_complete=False))):
            decision = advance_startup_observation(monitor_fixture(), status, payload,
                                                   2_000_000_000)
            self.assertEqual(decision["status"], "STOP", (status, payload))
            self.assertEqual(decision["terminal"], "STARTUP_PHYSICS_SOURCE_FAILURE")

    def test_four_second_deadline_uses_physics_time(self):
        state = monitor_fixture()
        decision = advance_startup_observation(
            state, "LIVE", physics_payload(101, 1001, 14.0), 2_000_000_000)
        self.assertEqual(decision["status"], "STOP")
        self.assertEqual(decision["terminal"], "STARTUP_OBSERVATION_LIMIT")

    def test_recovery_does_not_end_observation_and_later_fall_stops(self):
        state = monitor_fixture()
        low = physics_payload(101, 1001, 10.10, base_height_m=0.18,
                              fall_candidate=True,
                              fall_history=[{"start_sim_time": 10.1, "end_sim_time": 10.1,
                                             "start_physics_step": 1001, "end_physics_step": 1001,
                                             "closed": False,
                                             "confirmed": False}])
        self.assertEqual(advance_startup_observation(state, "LIVE", low, 2_000_000_000)["status"], "CONTINUE")
        confirmed_low = physics_payload(102, 1002, 10.20, base_height_m=0.18,
                                        fall_condition_flags=1, fall_candidate=True,
                                        fall_confirmed=True,
                                        fall_history=[{"start_sim_time": 10.1, "end_sim_time": 10.2,
                                                       "start_physics_step": 1001, "end_physics_step": 1002,
                                                       "closed": False,
                                                       "confirmed": True}])
        self.assertEqual(advance_startup_observation(state, "LIVE", confirmed_low, 2_010_000_000)["status"],
                         "CONTINUE")
        self.assertTrue(state["delay_low_height_fall"])
        normal1 = physics_payload(103, 1003, 10.30,
                                  fall_history=[{"start_sim_time": 10.1, "end_sim_time": 10.25,
                                                 "start_physics_step": 1001, "end_physics_step": 1002,
                                                 "closed": True,
                                                 "confirmed": True}])
        self.assertEqual(advance_startup_observation(state, "LIVE", normal1, 2_020_000_000)["status"], "CONTINUE")
        policy0 = SimpleNamespace(rl_step=0, sim_clock_valid=True, sequence=10, session_id=77)
        policy3 = SimpleNamespace(rl_step=3, sim_clock_valid=False,
                                  sim_clock_status=SIM_CLOCK_STALE, sequence=13, session_id=77)
        self.assertEqual(note_startup_policy_frame(state, policy0)["status"], "OK")
        self.assertEqual(note_startup_policy_frame(state, policy3)["status"], "OK")
        # The reader skips physical steps, but the publisher's sticky complete
        # coverage and full cumulative history remain intact.
        normal2 = physics_payload(104, 1040, 10.61,
                                  fall_history=[{"start_sim_time": 10.1, "end_sim_time": 10.25,
                                                 "start_physics_step": 1001, "end_physics_step": 1002,
                                                 "closed": True,
                                                 "confirmed": True}])
        self.assertEqual(advance_startup_observation(state, "LIVE", normal2, 2_030_000_000)["status"],
                         "RECOVERED_CONTINUE")
        self.assertAlmostEqual(state["recovery_sim_time_s"], 10.61)
        new_fall = physics_payload(105, 1041, 10.70, base_height_m=0.18,
                                   fall_condition_flags=1, fall_candidate=True, fall_confirmed=True,
                                   fall_history=[{"start_sim_time": 10.6, "end_sim_time": 10.7,
                                                  "start_physics_step": 1041, "end_physics_step": 1041,
                                                  "closed": False,
                                                  "confirmed": True}])
        decision = advance_startup_observation(state, "LIVE", new_fall, 2_040_000_000)
        self.assertEqual(decision["terminal"], "FALL_TERMINATION")

    def test_new_closed_anomaly_restarts_dwell_from_accumulated_history(self):
        state = monitor_fixture()
        first = physics_payload(101, 1001, 10.10, base_height_m=0.18, fall_candidate=True,
            fall_history=[{"start_sim_time": 10.1, "end_sim_time": 10.1,
                           "start_physics_step": 1001, "end_physics_step": 1001,
                           "closed": False, "confirmed": False}])
        advance_startup_observation(state, "LIVE", first, 2_000_000_000)
        closed_first = physics_payload(102, 1002, 10.30, fall_history=[
            {"start_sim_time": 10.1, "end_sim_time": 10.25, "start_physics_step": 1001,
             "end_physics_step": 1002, "closed": True, "confirmed": True}])
        advance_startup_observation(state, "LIVE", closed_first, 2_010_000_000)
        second = physics_payload(103, 1010, 10.50, base_height_m=0.18, fall_candidate=True,
            fall_history=[
                {"start_sim_time": 10.1, "end_sim_time": 10.25, "start_physics_step": 1001,
                 "end_physics_step": 1002, "closed": True, "confirmed": True},
                {"start_sim_time": 10.4, "end_sim_time": 10.48, "start_physics_step": 1008,
                 "end_physics_step": 1009, "closed": True, "confirmed": False},
            ])
        advance_startup_observation(state, "LIVE", second, 2_020_000_000)
        after_anomaly = physics_payload(104, 1050, 10.77, fall_history=[
            {"start_sim_time": 10.1, "end_sim_time": 10.25, "start_physics_step": 1001,
             "end_physics_step": 1002, "closed": True, "confirmed": True},
            {"start_sim_time": 10.4, "end_sim_time": 10.48, "start_physics_step": 1008,
             "end_physics_step": 1009, "closed": True, "confirmed": False},
        ])
        self.assertNotEqual(advance_startup_observation(state, "LIVE", after_anomaly,
                                                        2_030_000_000)["status"], "RECOVERED_CONTINUE")
        later = physics_payload(105, 1080, 10.79, fall_history=after_anomaly["fall_history"])
        self.assertEqual(advance_startup_observation(state, "LIVE", later,
                                                     2_040_000_000)["status"], "RECOVERED_CONTINUE")

    def test_incomplete_accumulated_recovery_evidence_is_unknown(self):
        state = monitor_fixture()
        low = physics_payload(101, 1001, 10.1, base_height_m=0.18,
            fall_history=[{"start_sim_time": 10.1, "end_sim_time": 10.1,
                           "start_physics_step": 1001, "end_physics_step": 1001,
                           "closed": False, "confirmed": False}])
        advance_startup_observation(state, "LIVE", low, 2_000_000_000)
        closed = physics_payload(102, 1002, 10.3, fall_history=[
            {"start_sim_time": 10.1, "end_sim_time": 10.25, "start_physics_step": 1001,
             "end_physics_step": 1002, "closed": True, "confirmed": False}])
        advance_startup_observation(state, "LIVE", closed, 2_010_000_000)
        broken = physics_payload(103, 1010, 10.7, fall_history=[
            {"start_sim_time": 10.1, "end_sim_time": 10.25, "start_physics_step": 1001,
             "end_physics_step": 1002, "closed": True, "confirmed": False}], fall_history_count=2)
        decision = advance_startup_observation(state, "LIVE", broken, 2_020_000_000)
        self.assertEqual(decision["terminal"], "STARTUP_RECOVERY_EVIDENCE_INCOMPLETE")
        self.assertEqual(state["recovery_evidence_status"], "INCOMPLETE")

    def test_physics_safety_stop_precedes_low_height_delay(self):
        state = monitor_fixture()
        collision = physics_payload(101, 1001, 10.1, base_height_m=0.10,
                                    robot_obstacle_contacts=1)
        decision = advance_startup_observation(state, "LIVE", collision, 2_000_000_000)
        self.assertEqual(decision["terminal"], "COLLISION_TERMINATION")
        self.assertFalse(state["delay_low_height_fall"])

    def test_default_terminal_logic_remains_the_regular_priority(self):
        self.assertEqual(select_strategy_terminal(False, False, True, False, False),
                         "FALL_TERMINATION")


if __name__ == "__main__":
    unittest.main()
