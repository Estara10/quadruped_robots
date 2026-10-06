#!/usr/bin/env python3
"""P1-01 — offline, non-mutating asymmetric probe of the ACTUAL deployed artifacts.

Read-only: loads the deployed TorchScripts (Agile 61->12, Recovery 49->12,
RA 19->1) with torch.jit.load, inspects embedded metadata, and empirically
determines the deployed policy's internal action/observation coupling from the
artifact itself — not from filenames, source convention, tensor-shape matching,
or synthetic fixtures. No MuJoCo/ROS2/benchmark/robot.

Methods (all on the real artifact):
  A. one-hot input scan: for every input slot i, obs=e_i (+1.0), record output.
     The slots where the output mirrors the input identify the previous-action
     observation block and reveal the output action order relative to it.
  B. central-difference numerical Jacobian over all 61x12 couplings.
  C. embedded metadata dump (named attributes + TorchScript code string).
"""
from __future__ import annotations
import json, sys, torch
from pathlib import Path

ROOT = Path("/home/lidio/quadruped_robots")
DEPLOYED = {
    "agile": (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/policy.pt", 61, 12),
    "recovery": (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/policy.pt", 49, 12),
    "ra": (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/ra_value.pt", 19, 1),
}
OUT = Path("/tmp/p1_01_probe/probe_results.json")

def metadata_dump(model) -> dict:
    meta = {}
    try:
        meta["code"] = model.code if hasattr(model, "code") else None
    except Exception as exc:
        meta["code_error"] = str(exc)
    named = {}
    try:
        for k, v in model.named_attributes():
            named[k] = str(v)
    except Exception as exc:
        named["_error"] = str(exc)
    meta["named_attributes"] = named
    return meta

def onehot_scan(model, obs_dim, act_dim, values=(1.0, -1.0)) -> dict:
    results = {}
    for v in values:
        rows = []
        for i in range(obs_dim):
            x = torch.zeros(1, obs_dim)
            x[0, i] = v
            with torch.no_grad():
                y = model(x)[0]
            rows.append([float(t) for t in y])
        results[str(v)] = rows
    return results

def jacobian(model, obs_dim, act_dim, eps=0.05) -> list:
    x0 = torch.zeros(1, obs_dim)
    with torch.no_grad():
        y0 = model(x0)[0]
    J = []
    for i in range(obs_dim):
        xp = x0.clone(); xp[0, i] = eps
        xm = x0.clone(); xm[0, i] = -eps
        with torch.no_grad():
            yp = model(xp)[0]; ym = model(xm)[0]
        J.append([float((yp[j] - ym[j]) / (2 * eps)) for j in range(act_dim)])
    return J

def main() -> int:
    results = {"torch_version": torch.__version__, "artifacts": {}}
    for name, (path, obs_dim, act_dim) in DEPLOYED.items():
        if not path.exists():
            results["artifacts"][name] = {"error": f"missing {path}"}
            continue
        entry = {"path": str(path), "size_bytes": path.stat().st_size, "declared_dims": [obs_dim, act_dim]}
        try:
            model = torch.jit.load(str(path), map_location="cpu")
            model.eval()
        except Exception as exc:
            entry["load_error"] = str(exc)
            results["artifacts"][name] = entry
            continue
        # verify the manifest dimensions actually run on the deployed artifact
        try:
            x = torch.zeros(1, obs_dim)
            with torch.no_grad():
                y = model(x)
            actual = list(y.shape)
        except Exception as exc:
            entry["dims_mismatch"] = f"{obs_dim}->{act_dim} probe failed: {exc}"
            results["artifacts"][name] = entry
            continue
        entry["actual_output_shape"] = actual
        entry["obs_dim"], entry["act_dim"] = obs_dim, act_dim
        entry["metadata"] = metadata_dump(model)
        entry["onehot_scan"] = onehot_scan(model, obs_dim, act_dim)
        entry["jacobian"] = jacobian(model, obs_dim, act_dim)
        # deterministic repeatability: same one-hot twice
        x = torch.zeros(1, obs_dim); x[0, 0] = 1.0
        with torch.no_grad():
            r1 = model(x)[0]; r2 = model(x)[0]
        entry["deterministic"] = all(float(a) == float(b) for a, b in zip(r1, r2))
        results["artifacts"][name] = entry
    OUT.write_text(json.dumps(results, indent=2, sort_keys=True), encoding="utf-8")
    print(f"probe complete -> {OUT}")
    for name, e in results["artifacts"].items():
        if "obs_dim" in e:
            print(f"  {name}: {e['obs_dim']}->{e['act_dim']} deterministic={e.get('deterministic')}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
