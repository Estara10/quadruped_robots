#!/usr/bin/env python3
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from s1_run_diagnostic import parse_startup_gain_trace_lines


def gain_row(event, **extra):
    base = {
        "schema": "s6-05-gain-v1", "event": event, "session_id": 123, "steady_ns": 456,
        "joint_names": [f"joint_{i}" for i in range(12)],
        "entry_kp": [80.0] * 12, "entry_kd": [3.0] * 12,
        "target_kp": [30.0] * 12, "target_kd": [0.65] * 12,
    }
    if event != "entry":
        base.update({"written_kp": [80.0] * 12, "written_kd": [3.0] * 12,
                     "update": 0, "elapsed_s": 0.0, "period_s": 0.005, "alpha": 0.0})
    base.update(extra)
    return base


class StartupGainTraceTests(unittest.TestCase):
    def test_reads_entry_and_actual_interface_write(self):
        lines = "\n".join("[INFO] [S6-05-GAIN] " + json.dumps(row)
                           for row in (gain_row("entry"), gain_row("command_write")))
        rows = parse_startup_gain_trace_lines(lines)
        self.assertEqual([row["event"] for row in rows], ["entry", "command_write"])
        self.assertEqual(rows[1]["written_kp"], [80.0] * 12)

    def test_rejects_wrong_schema_or_joint_width(self):
        bad_schema = gain_row("entry", schema="unexpected")
        with self.assertRaises(ValueError):
            parse_startup_gain_trace_lines("[S6-05-GAIN] " + json.dumps(bad_schema))
        bad_width = gain_row("command_write", written_kd=[0.1] * 11)
        with self.assertRaises(ValueError):
            parse_startup_gain_trace_lines("[S6-05-GAIN] " + json.dumps(bad_width))

    def test_empty_log_is_explicitly_empty(self):
        self.assertEqual(parse_startup_gain_trace_lines("ordinary ROS output"), [])


if __name__ == "__main__":
    unittest.main()
