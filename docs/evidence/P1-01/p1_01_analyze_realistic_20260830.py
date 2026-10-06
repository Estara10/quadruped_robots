#!/usr/bin/env python3
"""Realistic-baseline local Jacobian on the ACTUAL deployed artifacts.

Nominal observation built per the candidate 61-D layout; central-difference
sensitivity of each output to each dof_pos / prev_action / contact slot at the
operating point. Only the real artifact is used.
"""
import json, torch
from pathlib import Path

ROOT = Path("/home/lidio/quadruped_robots")
ART = {
    "agile": (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/policy.pt", 61, 12),
    "recovery": (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/policy.pt", 49, 12),
}
# Candidate layout blocks (documented in POLICY_IO_CONTRACT)
BLOCKS = {
    61: {"contact": (0, 4), "ang_vel": (4, 7), "gravity": (7, 10), "goal": (10, 13),
         "timer": (13, 14), "dof_pos": (14, 26), "dof_vel": (26, 38), "prev_action": (38, 50), "rays": (50, 61)},
    49: {"contact": (0, 4), "ang_vel": (4, 7), "gravity": (7, 10), "twist": (10, 13),
         "dof_pos": (13, 25), "dof_vel": (25, 37), "prev_action": (37, 49)},
}

def nominal(dim):
    blocks = BLOCKS[dim]
    x = torch.zeros(1, dim)
    x[0, blocks["contact"][0]:blocks["contact"][1]] = 1.0      # all feet contact (standing)
    x[0, blocks["gravity"][0]:blocks["gravity"][1]] = torch.tensor([0.0, 0.0, -1.0])  # level body
    if "timer" in blocks:
        x[0, blocks["timer"][0]:blocks["timer"][1]] = 1.0
    if "rays" in blocks:
        x[0, blocks["rays"][0]:blocks["rays"][1]] = 2.585       # log2(6m), all clear
    return x

out = {}
for name, (path, dim, act) in ART.items():
    model = torch.jit.load(str(path), map_location="cpu"); model.eval()
    x0 = nominal(dim)
    eps = 0.05
    blocks = BLOCKS[dim]
    J = []
    with torch.no_grad():
        y0 = model(x0)[0]
    for i in range(dim):
        xp = x0.clone(); xp[0, i] += eps
        xm = x0.clone(); xm[0, i] -= eps
        with torch.no_grad():
            yp = model(xp)[0]; ym = model(xm)[0]
        J.append([float((yp[j] - ym[j]) / (2 * eps)) for j in range(act)])
    rec = {"baseline_output": [round(float(v), 4) for v in y0]}
    for block, (a, b) in blocks.items():
        rows = {}
        for i in range(a, b):
            rows[i] = [round(J[i][j], 4) for j in range(act)]
        rec[block] = {"slots": f"{a}:{b}", "rows": rows}
    # per output, the dof_pos slot with max |J|
    if "dof_pos" in blocks:
        a, b = blocks["dof_pos"]
        for j in range(act):
            top = sorted(range(a, b), key=lambda i: -abs(J[i][j]))[:2]
            rec[f"out{j}_dofpos_top"] = [(i, round(J[i][j], 3)) for i in top]
    out[name] = rec

Path("/tmp/p1_01_probe/realistic_jacobian.json").write_text(json.dumps(out, indent=2, sort_keys=True))
print("saved /tmp/p1_01_probe/realistic_jacobian.json")
for name, rec in out.items():
    print(f"\n=== {name} baseline_output={rec['baseline_output']}")
    for j in range(12):
        if f"out{j}_dofpos_top" in rec:
            print(f"  out{j} dof_pos top: {rec[f'out{j}_dofpos_top']}")
