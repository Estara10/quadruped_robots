#!/usr/bin/env python3
"""One-off per-leg fixed-foot constraint comparison for the two Go2 walks."""
import csv
import json
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
LEGS = ("FR", "FL", "RR", "RL")
AXES = ("x", "y", "z")
RUNS = {
    "first": {
        "constraints": OUT / "first_leg_constraints.csv",
        "replay": ROOT / "logs/real_go2/20261105_estimator_replay/first.csv",
        "raw": ROOT / "logs/real_go2/20261105_104408_native_slow_walk/estimator.csv",
        "pre": (35.0, 45.0), "motion": (78.0, 90.0), "post": (100.0, 110.0),
    },
    "repeat": {
        "constraints": OUT / "repeat_leg_constraints.csv",
        "replay": ROOT / "logs/real_go2/20261105_estimator_replay/repeat.csv",
        "raw": ROOT / "logs/real_go2/20261105_104924_native_slow_walk_repeat/estimator.csv",
        "pre": (10.0, 20.0), "motion": (25.0, 40.0), "post": (45.0, 55.0),
    },
}


def load(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def f(row, key):
    return float(row[key])


def avg_std_min_max(array):
    return {"mean": np.mean(array, axis=0).tolist(), "std": np.std(array, axis=0, ddof=1).tolist(),
            "min": np.min(array, axis=0).tolist(), "max": np.max(array, axis=0).tolist(), "n": len(array)}


def summarize_constraints(samples, scheme, frame):
    result = {}
    for leg in LEGS:
        v = np.array([[f(sample[leg], f"base_v_{frame}_{scheme}_{axis}") for axis in AXES]
                      for sample in samples])
        result[leg] = avg_std_min_max(v)
    return result


def disagreement(samples, scheme):
    candidates = np.array([[[f(sample[leg], f"base_v_body_{scheme}_{axis}") for axis in AXES]
                            for leg in LEGS] for sample in samples])
    axis_spread = np.std(candidates, axis=1, ddof=0)
    pairwise = {}
    all_distances = []
    for i, first in enumerate(LEGS):
        for j in range(i + 1, len(LEGS)):
            second = LEGS[j]
            d = np.linalg.norm(candidates[:, i, :] - candidates[:, j, :], axis=1)
            pairwise[f"{first}-{second}"] = {
                "mean_norm_m_s": float(np.mean(d)), "median_norm_m_s": float(np.median(d)),
                "p95_norm_m_s": float(np.percentile(d, 95))}
            all_distances.append(d)
    all_distances = np.concatenate(all_distances)
    return {"axis_spread_per_time_mean_m_s": np.mean(axis_spread, axis=0).tolist(),
            "axis_spread_per_time_p95_m_s": np.percentile(axis_spread, 95, axis=0).tolist(),
            "pairwise_norm_mean_m_s": float(np.mean(all_distances)),
            "pairwise_norm_median_m_s": float(np.median(all_distances)),
            "pairwise_norm_p95_m_s": float(np.percentile(all_distances, 95)),
            "pairwise": pairwise}


def corr(x, y):
    if np.std(x) == 0 or np.std(y) == 0:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def main():
    out = {}
    for run, spec in RUNS.items():
        rows = load(spec["constraints"])
        by_time = defaultdict(dict)
        for row in rows:
            by_time[round(f(row, "update_time_s"), 6)][row["leg"]] = row
        times = np.array(sorted(by_time))
        samples = [by_time[t] for t in times]
        replay_rows = load(spec["replay"])
        replay_by_time = {round(f(row, "update_time_s"), 6): row for row in replay_rows}
        raw_rows = load(spec["raw"])
        raw_by_time = {round(f(row, "update_time_s"), 6): row for row in raw_rows}

        run_result = {"windows_s": {key: spec[key] for key in ("pre", "motion", "post")},
                      "time_basis": "observer steady_clock update_time_s; independent 200 Hz rows; no interpolation",
                      "windows": {}}
        for window in ("pre", "motion", "post"):
            lo, hi = spec[window]
            indices = np.flatnonzero((times >= lo) & (times <= hi))
            window_samples = [samples[i] for i in indices]
            window_times = times[indices]
            result = {"n_updates": len(indices), "n_leg_rows": len(indices) * 4,
                      "per_leg_noomega_body_m_s": summarize_constraints(window_samples, "noomega", "body"),
                      "per_leg_withomega_body_m_s": summarize_constraints(window_samples, "omega", "body"),
                      "disagreement_noomega": disagreement(window_samples, "noomega"),
                      "disagreement_withomega": disagreement(window_samples, "omega")}
            force_summary = {}
            joint_summary = {}
            for leg in LEGS:
                forces = np.array([f(sample[leg], "force_raw") for sample in window_samples])
                force_summary[leg] = {"min_raw": float(forces.min()), "median_raw": float(np.median(forces)),
                                      "max_raw": float(forces.max()), "rows_ge_1": int(np.count_nonzero(forces >= 1.0)),
                                      "rows": len(forces)}
                joint_norm = np.array([np.linalg.norm([f(sample[leg], name) for name in
                    ("hip_dq_rad_s", "thigh_dq_rad_s", "calf_dq_rad_s")]) for sample in window_samples])
                joint_summary[leg] = {"dq_norm_mean_rad_s": float(joint_norm.mean()),
                                      "dq_norm_p95_rad_s": float(np.percentile(joint_norm, 95))}
            result["force_raw_and_threshold"] = force_summary
            result["joint_speed_norm"] = joint_summary

            if window == "motion":
                c_body_vx, a_body_vx = [], []
                for t in window_times:
                    replay_row = replay_by_time[round(float(t), 6)]
                    c_body_vx.append(f(replay_row, "C_body_vx"))
                    a_body_vx.append(f(replay_row, "A_body_vx"))
                spread_noomega = np.array([np.std([f(sample[leg], "base_v_body_noomega_x") for leg in LEGS])
                                           for sample in window_samples])
                spread_omega = np.array([np.std([f(sample[leg], "base_v_body_omega_x") for leg in LEGS])
                                         for sample in window_samples])
                dt = np.diff(window_times)
                c_accel = np.abs(np.diff(c_body_vx) / dt)
                result["temporal_association_pearson"] = {
                    "x_leg_spread_noomega_vs_abs_C_vx_change_per_s": corr(spread_noomega[:-1], c_accel),
                    "x_leg_spread_withomega_vs_abs_C_vx_change_per_s": corr(spread_omega[:-1], c_accel),
                    "x_leg_spread_noomega_vs_abs_C_vx": corr(spread_noomega, np.abs(c_body_vx)),
                    "x_leg_spread_withomega_vs_abs_C_vx": corr(spread_omega, np.abs(c_body_vx)),
                    "interpretation": "descriptive contemporaneous association only; Estimator output is not used to form leg candidates"}
            run_result["windows"][window] = result

        motion_lo, motion_hi = spec["motion"]
        ids = np.flatnonzero((times >= motion_lo) & (times <= motion_hi))
        t = times[ids]
        motion_samples = [samples[i] for i in ids]
        replay_motion = [replay_by_time[round(float(time), 6)] for time in t]
        fig, axes = plt.subplots(7, 1, figsize=(13, 17), sharex=True, constrained_layout=True)
        for leg in LEGS:
            axes[0].plot(t, [f(sample[leg], "base_v_body_omega_x") for sample in motion_samples],
                         lw=0.75, label=f"{leg} fixed-foot candidate")
            axes[1].plot(t, [f(sample[leg], "base_v_body_noomega_x") for sample in motion_samples],
                         lw=0.75, label=leg)
            axes[2].plot(t, [f(sample[leg], "base_v_body_omega_y") for sample in motion_samples],
                         lw=0.75, label=f"{leg} fixed-foot candidate")
            axes[3].plot(t, [f(sample[leg], "base_v_body_noomega_y") for sample in motion_samples],
                         lw=0.75, label=leg)
            axes[4].plot(t, [f(sample[leg], "force_raw") for sample in motion_samples],
                         lw=0.75, label=leg)
            axes[5].plot(t, [np.linalg.norm([f(sample[leg], name) for name in
                ("hip_dq_rad_s", "thigh_dq_rad_s", "calf_dq_rad_s")]) for sample in motion_samples],
                         lw=0.75, label=leg)
        axes[0].plot(t, [f(row, "A_body_vx") for row in replay_motion], "k--", lw=1.0, label="Estimator A vx")
        axes[0].plot(t, [f(row, "C_body_vx") for row in replay_motion], color="magenta", ls="--", lw=1.0,
                     label="Estimator C vx")
        axes[2].plot(t, [f(row, "A_body_vy") for row in replay_motion], "k--", lw=1.0, label="Estimator A vy")
        axes[2].plot(t, [f(row, "C_body_vy") for row in replay_motion], color="magenta", ls="--", lw=1.0,
                     label="Estimator C vy")
        axes[4].axhline(1.0, color="k", ls="--", lw=0.8, label="contact threshold 1")
        axes[6].plot(t, [np.std([f(sample[leg], "base_v_body_noomega_x") for leg in LEGS])
                         for sample in motion_samples], label="between-leg vx spread, no omega")
        axes[6].plot(t, [np.std([f(sample[leg], "base_v_body_omega_x") for leg in LEGS])
                         for sample in motion_samples], label="between-leg vx spread, with omega")
        axes[0].set_ylabel("body vx (m/s), with omega")
        axes[1].set_ylabel("body vx (m/s), no omega")
        axes[2].set_ylabel("body vy (m/s), with omega")
        axes[3].set_ylabel("body vy (m/s), no omega")
        axes[4].set_ylabel("raw foot force")
        axes[5].set_ylabel("joint dq norm (rad/s)")
        axes[6].set_ylabel("cross-leg vx SD (m/s)")
        axes[6].set_xlabel("observer steady time (s)")
        axes[0].set_title(f"{run}: fixed exploratory window {motion_lo:g}–{motion_hi:g}s; raw 200Hz rows")
        for axis in axes[:4]: axis.legend(ncol=3, fontsize=8)
        axes[4].legend(ncol=5, fontsize=8); axes[5].legend(ncol=4, fontsize=8); axes[6].legend(fontsize=8)
        for axis in axes:
            axis.grid(alpha=0.22)
        fig.savefig(OUT / f"{run}_leg_constraints.png", dpi=170)
        plt.close(fig)
        out[run] = run_result

    (OUT / "metrics.json").write_text(json.dumps(out, indent=2) + "\n")
    for run in out:
        print(run, "window metrics written", OUT / "metrics.json")


if __name__ == "__main__":
    main()
