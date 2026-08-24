#!/usr/bin/env python3
"""Validate P1-01 hashes, model I/O, observation slices and order mappings.

Run with ``conda run -n abs python scripts/validate_p1_01_contract.py``.
The validator is read-only. It validates the recorded conditional mapping but
does not convert missing artifact provenance into proof.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from typing import Any, Dict, Iterable, List

import torch
import yaml


ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "artifacts/manifest.yaml"
CONTRACT_PATH = ROOT / "artifacts/p1_01_contract.json"
ISAAC_EVIDENCE_PATH = ROOT / "docs/evidence/P1-01/isaac_gym_asset_order.json"


class Results:
    def __init__(self) -> None:
        self.passes: List[str] = []
        self.failures: List[str] = []
        self.known_gaps: List[str] = []

    def check(self, condition: bool, label: str, detail: str = "") -> None:
        message = f"{label}{': ' + detail if detail else ''}"
        (self.passes if condition else self.failures).append(message)

    def gap(self, message: str) -> None:
        self.known_gaps.append(message)

    def report(self, regression_only: bool) -> int:
        for item in self.passes:
            print(f"PASS  {item}")
        for item in self.known_gaps:
            print(f"KNOWN {item}")
        for item in self.failures:
            print(f"FAIL  {item}")
        print(
            f"SUMMARY pass={len(self.passes)} known={len(self.known_gaps)} "
            f"fail={len(self.failures)}"
        )
        if self.failures:
            print("P1-01 CONTRACT REGRESSION: FAIL")
            return 1
        print("P1-01 CONTRACT REGRESSION: PASS")
        if self.known_gaps:
            print("P1-01 ACCEPTANCE: BLOCKED (known gaps remain)")
            return 0 if regression_only else 2
        print("P1-01 ACCEPTANCE: validator has no known gaps; Reviewer decision still required")
        return 0


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def reorder(values: List[Any], indices: Iterable[int]) -> List[Any]:
    return [values[index] for index in indices]


def normalized_source(path: Path) -> str:
    return re.sub(r"\s+", "", path.read_text(encoding="utf-8"))


def check_model(
    results: Results,
    name: str,
    manifest: Dict[str, Any],
    fixture: Dict[str, Any],
) -> None:
    path = ROOT / manifest["path"]
    results.check(path.is_file(), f"{name} exists", str(path.relative_to(ROOT)))
    if not path.is_file():
        return

    results.check(path.stat().st_size == manifest["size_bytes"], f"{name} size")
    results.check(sha256_file(path) == manifest["sha256"], f"{name} SHA-256")
    results.check(manifest["input_dim"] == fixture["input_dim"], f"{name} input manifest")
    results.check(manifest["output_dim"] == fixture["output_dim"], f"{name} output manifest")
    with zipfile.ZipFile(path) as archive:
        archive_roots = {member.split("/", 1)[0] for member in archive.namelist()}
    results.check(manifest["archive_root"] in archive_roots, f"{name} TorchScript archive root")

    model = torch.jit.load(str(path), map_location="cpu").eval()
    weight_shapes = [
        list(tensor.shape)
        for key, tensor in model.state_dict().items()
        if key.endswith(".weight")
    ]
    architecture = [weight_shapes[0][1]] + [shape[0] for shape in weight_shapes]
    results.check(architecture == fixture["architecture"], f"{name} architecture")

    input_dim = fixture["input_dim"]
    golden_input = torch.arange(1, input_dim + 1, dtype=torch.float32).reshape(1, -1) / 100.0
    with torch.no_grad():
        output = model(golden_input)
    expected = torch.tensor(fixture["golden_output"], dtype=torch.float32).reshape(1, -1)
    results.check(list(output.shape) == [1, fixture["output_dim"]], f"{name} forward shape")
    results.check(bool(torch.isfinite(output).all()), f"{name} finite golden output")
    results.check(
        bool(torch.allclose(output, expected, rtol=1e-6, atol=1e-7)),
        f"{name} golden output",
    )

    wrong_shape_rejected = False
    try:
        model(torch.zeros(1, input_dim + 1))
    except (RuntimeError, ValueError):
        wrong_shape_rejected = True
    results.check(wrong_shape_rejected, f"{name} wrong-shape rejection")

    for label, value in (("NaN", math.nan), ("Inf", math.inf)):
        fault = torch.zeros(1, input_dim)
        fault[0, 0] = value
        accepted_fault = False
        fault_output_finite = False
        try:
            fault_output = model(fault)
            accepted_fault = True
            fault_output_finite = bool(torch.isfinite(fault_output).all())
        except (RuntimeError, ValueError):
            pass
        if accepted_fault:
            results.gap(
                f"{name} accepts a {label} input at model boundary "
                f"(output_finite={fault_output_finite})"
            )
        else:
            results.check(True, f"{name} {label} rejection")

    install_relative = Path("quadruped_ros2_control_humble/install/go2_description/share/go2_description")
    source_relative = Path(manifest["path"])
    config_index = source_relative.parts.index("config")
    installed = ROOT / install_relative.joinpath(*source_relative.parts[config_index:])
    if installed.exists():
        results.check(sha256_file(installed.resolve()) == manifest["sha256"], f"{name} installed binding")
    else:
        results.gap(f"{name} installed artifact path is absent")


def check_observation_layouts(results: Results, contract: Dict[str, Any]) -> None:
    for name, spec in contract["observations"].items():
        cursor = 0
        names = []
        for field in spec["fields"]:
            results.check(field["start"] == cursor, f"{name} {field['name']} contiguous start")
            results.check(field["end"] > field["start"], f"{name} {field['name']} positive width")
            cursor = field["end"]
            names.append(field["name"])
        results.check(cursor == spec["total"], f"{name} total dimension")

        asymmetric = []
        for field_index, field in enumerate(spec["fields"], start=1):
            asymmetric.extend(
                field_index * 1000 + offset
                for offset in range(field["end"] - field["start"])
            )
        results.check(len(asymmetric) == spec["total"], f"{name} asymmetric fixture length")
        for field in spec["fields"]:
            results.check(
                len(asymmetric[field["start"] : field["end"]]) == field["end"] - field["start"],
                f"{name} {field['name']} asymmetric slice",
            )

    abs_yaml = yaml.safe_load(
        (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml").read_text()
    )
    rec_yaml = yaml.safe_load(
        (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/config.yaml").read_text()
    )
    agile_names = [field["name"] for field in contract["observations"]["agile"]["fields"]]
    recovery_names = [field["name"] for field in contract["observations"]["recovery"]["fields"]]
    results.check(abs_yaml["num_observations"] == 61, "Agile YAML dimension")
    results.check(abs_yaml["observations"] == agile_names, "Agile YAML field order")
    results.check(rec_yaml["num_observations"] == 49, "Recovery YAML dimension")
    recovery_yaml_names = ["commands" if name == "safe_twist" else name for name in recovery_names]
    results.check(rec_yaml["observations"] == recovery_yaml_names, "Recovery YAML field order")


def check_orders(results: Results, contract: Dict[str, Any]) -> None:
    orders = contract["orders"]
    policy = orders["isaac_gym_training_dofs"]["names"]
    controller = orders["ros2_controller_dofs"]["names"]
    ctrl_to_policy = orders["controller_to_candidate_policy_indices"]
    policy_to_ctrl = orders["candidate_policy_to_controller_indices"]
    motor_indices = orders["unitree_motor_indices_by_controller_index"]

    results.check(len(policy) == len(set(policy)) == 12, "training DOF names unique")
    results.check(len(controller) == len(set(controller)) == 12, "controller DOF names unique")
    results.check(reorder(controller, ctrl_to_policy) == policy, "controller-to-policy name remap")
    results.check(reorder(policy, policy_to_ctrl) == controller, "policy-to-controller name remap")

    labels = [101, 102, 103, 201, 202, 203, 301, 302, 303, 401, 402, 403]
    controller_labels = reorder(labels, policy_to_ctrl)
    results.check(reorder(controller_labels, ctrl_to_policy) == labels, "asymmetric DOF round trip")
    results.check(sorted(motor_indices) == list(range(12)), "motor map bijective")
    final_motor_values = [None] * 12
    for controller_index, motor_index in enumerate(motor_indices):
        final_motor_values[motor_index] = controller_labels[controller_index]
    results.check(None not in final_motor_values, "all policy actions reach one motor slot")

    policy_feet = orders["isaac_gym_training_feet"]["names"]
    controller_feet = orders["mujoco_controller_feet"]["names"]
    contact_indices = orders["controller_to_candidate_policy_contact_indices"]
    results.check(reorder(controller_feet, contact_indices) == policy_feet, "contact name remap")
    contact_labels = [11, 22, 33, 44]
    candidate_policy_contacts = reorder(contact_labels, contact_indices)
    results.check(candidate_policy_contacts == [22, 11, 44, 33], "asymmetric contact fixture")

    state_rl = normalized_source(
        ROOT / "quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp"
    )
    state_rec = normalized_source(
        ROOT / "quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp"
    )
    dof_literal = "{3,4,5,0,1,2,9,10,11,6,7,8}"
    contact_literal = "{1,0,3,2}"
    results.check(state_rl.count(dof_literal) >= 2, "StateRL forward/inverse DOF remaps present")
    results.check(contact_literal in state_rl, "StateRL contact remap present")
    results.check(state_rec.count(dof_literal) >= 2, "StateRLRec forward/inverse DOF remaps present")
    results.check(contact_literal in state_rec, "StateRLRec contact remap present")

    robot_yaml = yaml.safe_load(
        (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml").read_text()
    )
    configured = robot_yaml["rl_quadruped_controller"]["ros__parameters"]["joints"]
    results.check(configured == controller, "ROS2 configured joint order")

    header = normalized_source(
        ROOT / "quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/include/hardware_unitree_mujoco/HardwareUnitree.h"
    )
    results.check("motor_index_map_{0,1,2,3,4,5,6,7,8,9,10,11}" in header, "hardware identity motor map")

    mjcf = ET.parse(ROOT / "unitree_mujoco/unitree_robots/go2/go2.xml")
    actuator = mjcf.getroot().find("actuator")
    actuator_joints = [element.attrib["joint"] for element in actuator if "joint" in element.attrib]
    results.check(actuator_joints[:12] == controller, "MuJoCo actuator joint order")


def check_isaac_evidence(results: Results, contract: Dict[str, Any]) -> None:
    evidence = json.loads(ISAAC_EVIDENCE_PATH.read_text(encoding="utf-8"))
    urdf = ROOT / evidence["asset"]["path"]
    results.check(sha256_file(urdf) == evidence["asset"]["sha256"], "Isaac Gym evidence URDF hash")
    results.check(
        evidence["dof_names"] == contract["orders"]["isaac_gym_training_dofs"]["names"],
        "Isaac Gym evidence DOF order",
    )
    results.check(
        evidence["feet_names"] == contract["orders"]["isaac_gym_training_feet"]["names"],
        "Isaac Gym evidence feet order",
    )
    results.check(len(evidence["rigid_body_names"]) == 17, "Isaac Gym rigid-body count")
    results.check(evidence["termination_contact_names"] == [], "empty base termination match recorded")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--regression-only",
        action="store_true",
        help="Return zero when recorded regression checks pass even if Acceptance gaps remain.",
    )
    args = parser.parse_args()
    results = Results()
    manifest = yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))

    results.check(manifest["schema_version"] == 2, "artifact manifest schema")
    results.check(contract["schema_version"] == 1, "P1-01 contract schema")
    for name in ("agile_policy", "ra_value", "recovery_policy"):
        check_model(results, name, manifest["artifacts"][name], contract["models"][name])

    recovery_candidates = {
        "ABS/training/legged_gym/resources/policy/recover_v4_twist_original.pt":
            "ce9aab3205045948d22880f5f7039d8919ba0a3d99ba9eb8c6cfa2877fd7e0a4",
        "ABS/training/legged_gym/resources/policy/recover_v3_twist.pt":
            "c0e7414940760e90bc0dfea6add2165ede3b1b37426d3ab0192541b85fc215ce",
        "ABS/training/legged_gym/resources/policy/05_27_19-34-13_model_6000.pt":
            "798afbf7a3273e06878dd0074908e0a07a0cbd40165f6a3219fa73b8f5cfb01f",
    }
    deployed_recovery_hash = manifest["artifacts"]["recovery_policy"]["sha256"]
    for relative, expected_hash in recovery_candidates.items():
        candidate_hash = sha256_file(ROOT / relative)
        results.check(candidate_hash == expected_hash, f"Recovery candidate hash {Path(relative).name}")
        results.check(candidate_hash != deployed_recovery_hash, f"Recovery candidate differs {Path(relative).name}")

    check_observation_layouts(results, contract)
    check_orders(results, contract)
    check_isaac_evidence(results, contract)

    if contract["orders"]["deployed_policy_dofs"]["status"] == "UNKNOWN":
        results.gap("deployed Agile/Recovery policy joint order remains UNKNOWN")
    if contract["orders"]["real_go2_foot_force_slot_order"]["status"] == "UNKNOWN":
        results.gap("real Go2 foot-force slot order remains UNKNOWN")
    for name in ("agile_policy", "ra_value", "recovery_policy"):
        status = manifest["artifacts"][name]["provenance"]["status"]
        if "unknown" in status:
            results.gap(f"{name} provenance is {status}")

    return results.report(args.regression_only)


if __name__ == "__main__":
    sys.exit(main())
