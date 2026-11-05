#!/usr/bin/env python3
"""One-off A/B/C summaries and plots for the two 2026-11-05 Go2 replays."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
RUNS = {
    "first": {
        "raw": ROOT / "logs/real_go2/20261105_104408_native_slow_walk/estimator.csv",
        "replay": OUT / "first.csv",
        "heading": ROOT / "logs/real_go2/20261105_104408_native_slow_walk/heading_projection.json",
        "pre": (35.0, 45.0), "motion": (78.0, 90.0), "post": (100.0, 110.0),
    },
    "repeat": {
        "raw": ROOT / "logs/real_go2/20261105_104924_native_slow_walk_repeat/estimator.csv",
        "replay": OUT / "repeat.csv",
        "heading": ROOT / "logs/real_go2/20261105_104924_native_slow_walk_repeat/heading_projection.json",
        "pre": (10.0, 20.0), "motion": (25.0, 40.0), "post": (45.0, 55.0),
    },
}
MODES = {"A": "A", "B": "B", "C": "C"}
COMPONENTS = ("x", "y", "z")


def read_rows(path):
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


def vector(rows, prefix, frame="body"):
    return np.array([[float(row[f"{prefix}_{frame}_v{axis}"]) for axis in COMPONENTS]
                     for row in rows])


def stats(array):
    return {"mean": array.mean(axis=0).tolist(), "std": array.std(axis=0, ddof=1).tolist(),
            "min": array.min(axis=0).tolist(), "max": array.max(axis=0).tolist(), "n": len(array)}


def integrate(rows, prefix, heading):
    t = np.array([float(row["update_time_s"]) for row in rows])
    v = vector(rows, prefix, "world")
    delta = np.sum((v[1:] + v[:-1]) * (np.diff(t)[:, None] / 2.0), axis=0)
    forward = np.array(heading, dtype=float)
    forward /= np.linalg.norm(forward)
    left = np.array([-forward[1], forward[0]])
    return {"world_delta_xyz_m": delta.tolist(), "horizontal_net_m": float(np.linalg.norm(delta[:2])),
            "forward_net_m": float(delta[:2] @ forward), "left_net_m": float(delta[:2] @ left),
            "integrated_interval_s": float(t[-1] - t[0]), "n": len(rows)}


def main():
    all_results = {}
    for name, spec in RUNS.items():
        replay = read_rows(spec["replay"])
        complete = [r for r in replay if r["original_row_complete"] == "1"]
        raw = read_rows(spec["raw"])
        raw_by_t = {r["update_time_s"]: r for r in raw if len(r) == 89}
        world_diffs, body_diffs = [], []
        for row in complete:
            source = raw_by_t[row["update_time_s"]]
            world_diffs.append([float(row[f"A_world_v{a}"]) - float(source[f"estimator_world_v{a}_m_s"])
                                for a in COMPONENTS])
            body_diffs.append([float(row[f"A_body_v{a}"]) - float(source[f"estimator_body_v{a}_m_s"])
                               for a in COMPONENTS])
        heading_data = json.loads(spec["heading"].read_text())
        heading = heading_data["initial_forward_unit_xy"]
        metrics = {"source_replay_check": {
            "compared_rows": len(complete),
            "world_max_abs_m_s": np.max(np.abs(world_diffs), axis=0).tolist(),
            "world_rms_m_s": np.sqrt(np.mean(np.square(world_diffs), axis=0)).tolist(),
            "body_max_abs_m_s": np.max(np.abs(body_diffs), axis=0).tolist(),
        }}
        time = np.array([float(r["update_time_s"]) for r in replay])
        for phase in ("pre", "motion", "post"):
            lo, hi = spec[phase]
            phase_rows = [r for r, t in zip(replay, time) if lo <= t <= hi]
            metrics[phase] = {}
            for mode in MODES:
                body = vector(phase_rows, mode, "body")
                metrics[phase][mode] = {"body_velocity": stats(body)}
                if phase == "motion":
                    metrics[phase][mode]["world_integral"] = integrate(phase_rows, mode, heading)
            if phase == "motion":
                metrics[phase]["C_fixed_foot_residuals"] = {}
                for leg in ("FR", "FL", "RR", "RL"):
                    residual = np.array([[float(row[f"{leg}_fixed_foot_residual_world_{axis}_m_s"])
                                          for axis in COMPONENTS] for row in phase_rows])
                    norms = np.linalg.norm(residual, axis=1)
                    forces = np.array([float(raw_by_t[row["update_time_s"]][f"{leg}_foot_force_raw"])
                                       for row in phase_rows])
                    metrics[phase]["C_fixed_foot_residuals"][leg] = {
                        "norm_mean_m_s": float(norms.mean()), "norm_median_m_s": float(np.median(norms)),
                        "norm_p95_m_s": float(np.percentile(norms, 95)),
                        "force_min_raw": float(forces.min()), "force_median_raw": float(np.median(forces)),
                        "rows_force_ge_1": int(np.count_nonzero(forces >= 1.0)), "n": len(norms)}
        all_results[name] = {"windows_s": {k: spec[k] for k in ("pre", "motion", "post")},
                             "heading_forward_xy": heading, "metrics": metrics}

        fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
        motion_lo, motion_hi = spec["motion"]
        rows = [r for r, t in zip(replay, time) if motion_lo <= t <= motion_hi]
        t = np.array([float(r["update_time_s"]) for r in rows])
        for mode in MODES:
            axes[0].plot(t, vector(rows, mode, "body")[:, 0], lw=0.9, label=f"{mode}: body vx")
            disp = np.zeros((len(rows), 2))
            world = vector(rows, mode, "world")
            if len(rows) > 1:
                disp[1:] = np.cumsum((world[1:, :2] + world[:-1, :2]) * (np.diff(t)[:, None] / 2.0), axis=0)
            axes[1].plot(t, disp @ np.array(heading), lw=1.0, label=f"{mode}: forward projection")
        axes[0].set_ylabel("body velocity x (m/s)")
        axes[1].set_ylabel("integrated forward projection (m)")
        axes[1].set_xlabel("observer steady time (s)")
        axes[0].grid(alpha=0.25); axes[1].grid(alpha=0.25)
        axes[0].legend(ncol=3); axes[1].legend(ncol=3)
        fig.suptitle(f"{name}: fixed exploratory window {motion_lo:g}–{motion_hi:g}s")
        fig.savefig(OUT / f"{name}_comparison.png", dpi=170)
        plt.close(fig)

    (OUT / "metrics.json").write_text(json.dumps(all_results, indent=2) + "\n")
    for name, result in all_results.items():
        print(name, json.dumps(result["metrics"], separators=(",", ":")))


if __name__ == "__main__":
    main()
