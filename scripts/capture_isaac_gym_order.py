#!/usr/bin/env python3
"""Capture authoritative Go2 asset order from the Isaac Gym runtime API.

Run under the ``abs`` Conda environment. The probe creates a CPU PhysX sim,
loads only the Go2 asset with the training asset options, prints JSON, and
destroys the sim. It does not create a training runner or step an environment.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Dict

from isaacgym import gymapi


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "ABS/training/legged_gym/resources/robots/go2/urdf/go2.urdf"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def capture() -> Dict[str, Any]:
    gym = gymapi.acquire_gym()
    sim_params = gymapi.SimParams()
    sim_params.use_gpu_pipeline = False
    sim_params.physx.use_gpu = False
    sim = gym.create_sim(0, -1, gymapi.SIM_PHYSX, sim_params)
    if sim is None:
        raise RuntimeError("Isaac Gym create_sim() failed")

    try:
        options = gymapi.AssetOptions()
        options.default_dof_drive_mode = 3
        options.collapse_fixed_joints = True
        options.replace_cylinder_with_capsule = True
        options.flip_visual_attachments = True
        options.fix_base_link = False
        options.density = 0.001
        options.angular_damping = 0.0
        options.linear_damping = 0.0
        options.max_angular_velocity = 1000.0
        options.max_linear_velocity = 1000.0
        options.armature = 0.0
        options.thickness = 0.01
        options.disable_gravity = False

        asset = gym.load_asset(sim, str(URDF.parent), URDF.name, options)
        if asset is None:
            raise RuntimeError(f"Isaac Gym load_asset() failed for {URDF}")

        dof_names = list(gym.get_asset_dof_names(asset))
        body_names = list(gym.get_asset_rigid_body_names(asset))
        feet_names = [name for name in body_names if "foot" in name]
        return {
            "asset_path": str(URDF.relative_to(ROOT)),
            "asset_sha256": sha256_file(URDF),
            "dof_names": dof_names,
            "rigid_body_names": body_names,
            "feet_names": feet_names,
            "feet_asset_indices": [
                gym.find_asset_rigid_body_index(asset, name) for name in feet_names
            ],
            "termination_contact_names": [name for name in body_names if "base" in name],
        }
    finally:
        gym.destroy_sim(sim)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--expect",
        type=Path,
        help="Compare captured order with a P1-01 evidence JSON file.",
    )
    args = parser.parse_args()

    result = capture()
    print(json.dumps(result, indent=2))
    if args.expect:
        expected = json.loads(args.expect.read_text(encoding="utf-8"))
        expected_subset = {
            "asset_path": expected["asset"]["path"],
            "asset_sha256": expected["asset"]["sha256"],
            "dof_names": expected["dof_names"],
            "rigid_body_names": expected["rigid_body_names"],
            "feet_names": expected["feet_names"],
            "feet_asset_indices": expected["feet_asset_indices"],
            "termination_contact_names": expected["termination_contact_names"],
        }
        if result != expected_subset:
            print("P1-01 Isaac Gym order: FAIL")
            return 1
        print("P1-01 Isaac Gym order: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
