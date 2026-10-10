#!/usr/bin/env python3
"""Offline tests for the opt-in P1-10 common-start gate and anchor."""

from __future__ import annotations

import copy
import hashlib
import json
import mmap
import os
import shutil
import tempfile
import unittest
from argparse import Namespace
from pathlib import Path
from unittest import mock

import sys

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import p1_08_baseline_capture as capture  # noqa: E402
import p1_10_common_start as common  # noqa: E402
import p1_10_stage_a_common_start as manifest_tool  # noqa: E402
from run_record import RunRecordRecorder, load_record, summarize_record  # noqa: E402
from test_run_record import fixture, pack_frame  # noqa: E402


MANIFEST = REPO / "docs/evidence/P1-10/stage_a_common_start_execution_manifest_20260907.json"
BASELINE = REPO / "docs/evidence/P1-08/P1-08_baseline_manifest.json"


def _pad(value: str, size: int) -> bytes:
    return value.encode("ascii") + b"\0" * (size - len(value))


def _make_gate(name: str, capture_id: str, qpos_sha256: str) -> Path:
    path = common._shm_path(name)
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        os.ftruncate(fd, common.GATE_SIZE)
        values = [
            common.MAGIC, common.VERSION, 2,
            common.STATE_WAITING_FOR_RELEASE, common.FLAG_INITIAL_READY,
            _pad(capture_id, 64), _pad(qpos_sha256, 64),
            1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0,
        ]
        mm = mmap.mmap(fd, common.GATE_SIZE, access=mmap.ACCESS_WRITE)
        mm[:] = common.GATE_STRUCT.pack(*values)
        mm.flush()
        mm.close()
    finally:
        os.close(fd)
    return path


class CommonStartTests(unittest.TestCase):
    def test_gate_release_is_one_shot_and_capture_bound(self):
        capture_id = "p1-10-capture-" + "a" * 32
        qpos = "b" * 64
        name = "/p1_10_common_start_test_" + "a" * 16
        path = _make_gate(name, capture_id, qpos)
        self.addCleanup(lambda: path.exists() and path.unlink())
        with common.GateClient(name) as client:
            ready = client.wait_ready(0.1)
            self.assertEqual(ready["state_name"], "WAITING_FOR_RELEASE")
            with self.assertRaises(common.CommonStartError):
                client.release("p1-10-capture-" + "c" * 32)
            released = client.release(capture_id)
            self.assertEqual(released["state_name"], "RELEASED")
            with self.assertRaises(common.CommonStartError):
                client.release(capture_id)

    def test_manifest_recomputes_and_does_not_use_p1_08_identity(self):
        result = manifest_tool.validate_manifest(MANIFEST, expected_executable=str(manifest_tool.EXECUTABLE))
        original = result["manifest"]["identity_input"]
        self.assertEqual(original["execution_kind"], manifest_tool.EXECUTION_KIND)
        self.assertNotEqual(original["baseline_parent"]["accepted_executable_sha256"],
                            original["artifacts"][0]["sha256"])
        self.assertNotIn("/home/", json.dumps(original, sort_keys=True))
        self.assertNotIn("p1-10-capture-" + "a" * 32,
                          json.dumps(original, sort_keys=True))
        with self.assertRaises(manifest_tool.CommonStartManifestError):
            manifest_tool.validate_manifest(BASELINE)

    def test_manifest_required_contract_mutations_reject(self):
        original = json.loads(MANIFEST.read_text(encoding="utf-8"))
        mutations = (
            lambda value: value["identity_input"]["common_start_contract"].__setitem__("version", 2),
            lambda value: value["identity_input"]["anchor_contract"].__setitem__("version", 2),
            lambda value: value["identity_input"]["fixed_binding"].__setitem__("window_s", 24.0),
            lambda value: value["identity_input"]["scene"].__setitem__("root_xml_sha256", "0" * 64),
            lambda value: value["identity_input"]["artifacts"][0].__setitem__("sha256", "0" * 64),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                with tempfile.TemporaryDirectory(dir=manifest_tool.EVIDENCE_DIR) as td:
                    value = copy.deepcopy(original)
                    mutate(value)
                    value["identity_sha256"] = hashlib.sha256(
                        manifest_tool.canonical_bytes(value["identity_input"])).hexdigest()
                    path = Path(td) / "manifest.json"
                    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
                    with self.assertRaises(manifest_tool.CommonStartManifestError):
                        manifest_tool.validate_manifest(path, expected_executable=str(manifest_tool.EXECUTABLE))

    def test_common_preflight_binds_context_and_environment_without_launch(self):
        args = Namespace(
            out_dir=str(REPO / "docs/evidence/P1-10/nonexistent-common-start-test"),
            window_s=25.0, scene="scene_flat.xml", mujoco_bin=str(manifest_tool.EXECUTABLE),
            manifest=str(BASELINE), stage_a_execution_manifest=None,
            stage_a_common_start_execution_manifest=str(MANIFEST),
            stage_b_execution_manifest=None, scenario="flat_goal_forward",
            root_seed=20260902, variant="stabilized", initial_state_source="scene_default",
        )
        with mock.patch.object(capture, "inspect_residual_processes", return_value=("none", {})), \
             mock.patch.object(capture, "run_cmd", return_value=(0, "")), \
             mock.patch.object(capture, "check_ldd", return_value=(True, {"ok": True})), \
             mock.patch.object(capture, "clean_task_shms", return_value=(True, {})):
            ok, evidence, env = capture.preflight(
                args, Path(args.out_dir), "p1-10-capture-" + "d" * 32)
        self.assertTrue(ok, evidence)
        context = evidence["scenario_context"]
        self.assertTrue(context["common_start_contract"]["enabled"])
        self.assertEqual(env["ABS_P1_10_COMMON_START"], "1")
        self.assertEqual(env["ABS_P1_10_INITIAL_QPOS_SHA256"], context["initial_state"]["qpos_sha256"])
        self.assertEqual(evidence["checks"]["manifest"]["stage_a_common_start"], True)

    def test_common_start_record_requires_and_persists_anchor(self):
        anchor = {
            "schema": common.SCHEMA, "version": common.VERSION,
            "capture_id": "p1-10-capture-" + "e" * 32,
            "initial_qpos_sha256": "f" * 64, "gate_state": "RELEASED",
            "release_monotonic_ns": 90, "rl_session_id": 7, "rl_step": 0,
            "rl_frame_sequence": 0, "rl_monotonic_ns": 95,
            "first_runtime_frame_sequence": 2, "first_runtime_frame_rl_step": 1,
            "first_runtime_frame_monotonic_ns": 100, "first_physics_step": 1,
            "first_physics_monotonic_ns": 99, "first_sim_time": 0.002,
            "flags": (common.FLAG_INITIAL_READY | common.FLAG_RELEASE_RECORDED |
                      common.FLAG_RL_ENTER_RECORDED | common.FLAG_FIRST_PHYSICS_RECORDED |
                      common.FLAG_FIRST_RUNTIME_FRAME_RECORDED),
        }
        with tempfile.TemporaryDirectory() as td:
            path = str(Path(td) / "runtime_record.jsonl")
            recorder = RunRecordRecorder(
                path, capture_id=anchor["capture_id"], common_start_required=True,
                expected_initial_qpos_sha256=anchor["initial_qpos_sha256"])
            recorder.start()
            recorder.set_start_anchor(anchor)
            recorder.record_snapshot(pack_frame(fixture(sequence=2, monotonic_ns=100)), now_ns=101)
            recorder.finalize({"exit_code": 0, "forced_termination": False,
                               "shutdown_complete": True, "shutdown_request_source": "SIGINT"})
            data = load_record(path)
            self.assertEqual(data.terminal["start_anchor"], anchor)
            self.assertEqual(summarize_record(path)["record_validity"], "VALID")

    def test_missing_anchor_fails_closed_and_legacy_record_stays_readable(self):
        capture_id = "p1-10-capture-" + "1" * 32
        with tempfile.TemporaryDirectory() as td:
            gated = str(Path(td) / "gated.jsonl")
            recorder = RunRecordRecorder(
                gated, capture_id=capture_id, common_start_required=True,
                expected_initial_qpos_sha256="2" * 64)
            recorder.start()
            recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
            recorder.finalize({"exit_code": 0, "forced_termination": False,
                               "shutdown_complete": True, "shutdown_request_source": "SIGINT"})
            summary = summarize_record(gated)
            self.assertEqual(summary["record_validity"], "INVALID")
            self.assertTrue(any("common_start_anchor" in reason for reason in summary["record_validity_reasons"]))

            legacy = str(Path(td) / "legacy.jsonl")
            old = RunRecordRecorder(legacy)
            old.start()
            old.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
            old.finalize({"exit_code": 0, "forced_termination": False,
                          "shutdown_complete": True, "shutdown_request_source": "SIGINT"})
            self.assertEqual(summarize_record(legacy)["record_validity"], "VALID")
            self.assertNotIn("common_start_required", load_record(legacy).meta)


if __name__ == "__main__":
    unittest.main(verbosity=2)
