#!/usr/bin/env python3
"""Robustness check: mean-|coupling| over multiple realistic operating points.

For each output j, find the prev_action / dof_pos slot with the largest mean
absolute local sensitivity across N randomized-but-realistic observations. A
persistent diagonal (37+i -> output i) would be robust order evidence; a
scattered pattern is not.
"""
import torch
from pathlib import Path

ROOT = Path("/home/lidio/quadruped_robots")
ART = {
    "recovery": (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/policy.pt", 49, 12, (37, 49), (13, 25)),
    "agile": (ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/policy.pt", 61, 12, (38, 50), (14, 26)),
}
N = 40
EPS = 0.05

for name, (path, dim, act, pa, dp) in ART.items():
    model = torch.jit.load(str(path), map_location="cpu"); model.eval()
    torch.manual_seed(0)
    # realistic baseline: contacts=1, gravity=[0,0,-1], others small random
    acc_pa = torch.zeros(act, pa[1] - pa[0])
    acc_dp = torch.zeros(act, dp[1] - dp[0])
    for _ in range(N):
        x = torch.zeros(1, dim)
        if dim == 61:
            x[0, 0:4] = 1.0; x[0, 7:10] = torch.tensor([0.0, 0.0, -1.0]); x[0, 50:61] = 2.585
        else:
            x[0, 0:4] = 1.0; x[0, 7:10] = torch.tensor([0.0, 0.0, -1.0])
        x += torch.randn(1, dim) * 0.05
        for block, acc, (a, b) in ((pa, acc_pa, pa), (dp, acc_dp, dp)):
            for i in range(a, b):
                xp = x.clone(); xp[0, i] += EPS
                xm = x.clone(); xm[0, i] -= EPS
                with torch.no_grad():
                    yp = model(xp)[0]; ym = model(xm)[0]
                d = (yp - ym).abs() / (2 * EPS)
                acc[:, i - a] += d
    acc_pa /= N; acc_dp /= N
    print(f"\n=== {name}: mean-|coupling| argmax per output ===")
    print("  prev_action block:")
    for j in range(act):
        top = torch.argsort(acc_pa[j], descending=True)[:2]
        print(f"    out{j}: slot {pa[0]+int(top[0])}({acc_pa[j,top[0]]:.3f}) slot {pa[0]+int(top[1])}({acc_pa[j,top[1]]:.3f})")
    print("  dof_pos block:")
    for j in range(act):
        top = torch.argsort(acc_dp[j], descending=True)[:2]
        print(f"    out{j}: slot {dp[0]+int(top[0])}({acc_dp[j,top[0]]:.3f}) slot {dp[0]+int(top[1])}({acc_dp[j,top[1]]:.3f})")
    # diagonal persistence: does the diagonal argmax == output index?
    diag_pa = sum(1 for j in range(act) if int(torch.argmax(acc_pa[j])) == j)
    diag_dp = sum(1 for j in range(act) if int(torch.argmax(acc_dp[j])) == j)
    print(f"  diagonal persistence: prev_action {diag_pa}/{act}, dof_pos {diag_dp}/{act}")
