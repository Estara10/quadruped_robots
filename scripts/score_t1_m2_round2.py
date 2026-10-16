#!/usr/bin/env python3
"""Score the frozen M2 144-run matrix; incomplete/UNKNOWN rows stay unscored."""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WEIGHTS = {"arrival": 0.40, "no_terminal_collision": 0.30, "efficiency": 0.20, "stability": 0.10}
TIME_CAP = 20.0
F_REF = 257.8
BOOTSTRAP_SEED = 2026101603
BOOTSTRAP_N = 10000
LEVELS = [-0.06, -0.07, -0.08, -0.09, -0.10, -0.11, -0.12, -0.13]
MAPS = [f"m2dev{i:02d}.xml" for i in range(1, 7)]


def number(value):
    try:
        out = float(value)
        return out if math.isfinite(out) else None
    except (TypeError, ValueError):
        return None


def truth(value):
    if value is True or value == "True": return True
    if value is False or value == "False": return False
    return None


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def score_row(row: dict) -> dict:
    result = dict(row)
    result.update({"score_status": "UNKNOWN", "arrival_score_0_1": None,
        "no_terminal_collision_score_0_1": None, "efficiency_score_0_1": None,
        "stability_score_0_1": None, "total_score_100": None, "score_missing_reasons": ""})
    reasons = []
    if row.get("status") != "VALID_TASK_RESULT" or row.get("record_validity") != "VALID":
        reasons.append("not_valid_task_record")
    collision = truth(row.get("motion_collision_result"))
    if collision is None: reasons.append("task_window_collision_UNKNOWN")
    elapsed = number(row.get("elapsed_task_duration_s"))
    if elapsed is None: reasons.append("task_duration_missing_or_invalid")
    reverse_count = number(row.get("short_reverse_switch_count"))
    valid_duration = number(row.get("valid_motion_duration_s"))
    if row.get("short_reverse_switch_coverage") != "complete": reasons.append("short_reverse_frequency_incomplete")
    if reverse_count is None or valid_duration is None or valid_duration <= 0:
        reasons.append("short_reverse_frequency_unavailable")
    frequency = number(row.get("short_reverse_per_valid_min"))
    if frequency is None and reverse_count is not None and valid_duration and valid_duration > 0 and row.get("short_reverse_switch_coverage") == "complete":
        frequency = reverse_count / valid_duration * 60.0
    if frequency is None: reasons.append("short_reverse_frequency_UNKNOWN")
    if reasons:
        result["score_missing_reasons"] = ";".join(dict.fromkeys(reasons))
        return result
    arrived = row.get("terminal_result") == "ARRIVED"
    arrival = 1.0 if arrived else 0.0
    safe = 1.0 if collision is False else 0.0
    efficiency = max(0.0, 1.0 - elapsed / TIME_CAP) if arrived else 0.0
    stability = 1.0 / (1.0 + frequency / F_REF)
    total = 100.0 * (WEIGHTS["arrival"]*arrival + WEIGHTS["no_terminal_collision"]*safe +
                     WEIGHTS["efficiency"]*efficiency + WEIGHTS["stability"]*stability)
    result.update({"score_status": "COMPLETE", "arrival_score_0_1": arrival,
        "no_terminal_collision_score_0_1": safe, "efficiency_score_0_1": efficiency,
        "stability_score_0_1": stability, "total_score_100": total,
        "short_reverse_per_valid_min_scored": frequency,
        "arrival_points": 100*WEIGHTS["arrival"]*arrival,
        "no_terminal_collision_points": 100*WEIGHTS["no_terminal_collision"]*safe,
        "efficiency_points": 100*WEIGHTS["efficiency"]*efficiency,
        "stability_points": 100*WEIGHTS["stability"]*stability,
        "score_missing_reasons": ""})
    return result


def aggregate(source: Path, out: Path) -> dict:
    with source.open(newline="", encoding="utf-8-sig") as stream:
        raw = list(csv.DictReader(stream))
    if len(raw) != 144:
        raise ValueError(f"expected 144 plan rows, got {len(raw)}")
    scored = [score_row(row) for row in raw]
    write_csv(out / "run_scores_144.csv", scored)
    by_cell = defaultdict(list)
    map_summary = []
    for map_name in MAPS:
        for x in LEVELS:
            rows = [r for r in scored if r.get("map") == map_name and math.isclose(float(r["exit_threshold_X"]), x, abs_tol=1e-8)]
            complete = [r for r in rows if r["score_status"] == "COMPLETE"]
            by_cell[(map_name, x)] = complete
            term = Counter(r.get("terminal_result") or "NOT_RUN" for r in rows)
            map_summary.append({"map": map_name, "scene_id": rows[0].get("scene_id") if rows else "",
                "density": rows[0].get("density") if rows else "", "route_structure": rows[0].get("route_structure") if rows else "",
                "exit_threshold_X": f"{x:.2f}", "planned_n": 3,
                "attempt_n": sum(int(number(r.get("attempts")) or 0) for r in rows),
                "valid_task_n": sum(r.get("status") == "VALID_TASK_RESULT" and r.get("record_validity") == "VALID" for r in rows),
                "complete_score_n": len(complete), "unknown_score_n": 3-len(complete),
                "arrived_n": term["ARRIVED"], "collision_terminal_n": term["COLLISION_TERMINATION"],
                "fall_terminal_n": term["FALL_TERMINATION"], "sim_timeout_n": term["DIAGNOSTIC_SIM_TIME_LIMIT"],
                "safety_abort_n": term["SYSTEM_SAFETY_ABORT"], "not_run_n": term["NOT_RUN"],
                "task_collision_true_n": sum(truth(r.get("motion_collision_result")) is True for r in rows),
                "task_collision_false_n": sum(truth(r.get("motion_collision_result")) is False for r in rows),
                "task_collision_unknown_n": sum(truth(r.get("motion_collision_result")) is None for r in rows),
                "fall_true_n": sum(truth(r.get("motion_fall_result")) is True for r in rows),
                "fall_unknown_n": sum(truth(r.get("motion_fall_result")) is None for r in rows),
                "mean_total_score_complete_only": statistics.mean(float(r["total_score_100"]) for r in complete) if complete else None,
                "mean_arrived_time_s": statistics.mean(number(r.get("elapsed_task_duration_s")) for r in rows if r.get("terminal_result") == "ARRIVED" and number(r.get("elapsed_task_duration_s")) is not None) if any(r.get("terminal_result") == "ARRIVED" and number(r.get("elapsed_task_duration_s")) is not None for r in rows) else None})
    write_csv(out / "map_threshold_summary_48.csv", map_summary)

    # Map means are the independent clusters. Resample shared map indices for all arms.
    rng = random.Random(BOOTSTRAP_SEED)
    draws = [[rng.randrange(len(MAPS)) for _ in MAPS] for _ in range(BOOTSTRAP_N)]
    threshold_summary, bootstrap_rows = [], []
    for x in LEVELS:
        all_rows = [r for r in scored if math.isclose(float(r["exit_threshold_X"]), x, abs_tol=1e-8)]
        valid = [r for r in all_rows if r.get("status") == "VALID_TASK_RESULT" and r.get("record_validity") == "VALID"]
        complete_maps = [m for m in MAPS if len(by_cell[(m, x)]) == 3]
        map_score_means = {m: statistics.mean(float(r["total_score_100"]) for r in by_cell[(m, x)]) for m in complete_maps}
        map_arrival = {}
        map_collision = {}
        for m in MAPS:
            rr = [r for r in valid if r.get("map") == m]
            map_arrival[m] = sum(r.get("terminal_result") == "ARRIVED" for r in rr)/len(rr) if rr else None
            known = [truth(r.get("motion_collision_result")) for r in rr]
            known = [v for v in known if v is not None]
            map_collision[m] = sum(known)/len(known) if known else None
        score_boot, arrival_boot, collision_boot = [], [], []
        for draw in draws:
            s = [map_score_means[MAPS[i]] for i in draw if MAPS[i] in map_score_means]
            a = [map_arrival[MAPS[i]] for i in draw if map_arrival[MAPS[i]] is not None]
            c = [map_collision[MAPS[i]] for i in draw if map_collision[MAPS[i]] is not None]
            if s: score_boot.append(statistics.mean(s))
            if a: arrival_boot.append(statistics.mean(a))
            if c: collision_boot.append(statistics.mean(c))
        def bounds(vals):
            if not vals: return (None, None)
            vals = sorted(vals)
            return vals[int(.025*(len(vals)-1))], vals[int(.975*(len(vals)-1))]
        slo, shi = bounds(score_boot); alo, ahi = bounds(arrival_boot); clo, chi = bounds(collision_boot)
        cell_complete = sum(len(by_cell[(m, x)]) == 3 for m in MAPS)
        threshold_summary.append({"exit_threshold_X": f"{x:.2f}", "planned_n": 18,
            "attempt_n": sum(int(number(r.get("attempts")) or 0) for r in all_rows),
            "valid_task_n": len(valid), "complete_score_n": sum(r["score_status"] == "COMPLETE" for r in all_rows),
            "unknown_score_n": sum(r["score_status"] != "COMPLETE" for r in all_rows),
            "complete_map_cells_n": cell_complete, "maps_with_any_complete_run_score_n": sum(bool(by_cell[(m, x)]) for m in MAPS),
            "maps_with_score_n": len(complete_maps),
            "arrived_n": sum(r.get("terminal_result") == "ARRIVED" for r in valid),
            "collision_terminal_n": sum(r.get("terminal_result") == "COLLISION_TERMINATION" for r in valid),
            "fall_terminal_n": sum(r.get("terminal_result") == "FALL_TERMINATION" for r in valid),
            "sim_timeout_n": sum(r.get("terminal_result") == "DIAGNOSTIC_SIM_TIME_LIMIT" for r in valid),
            "task_collision_true_n": sum(truth(r.get("motion_collision_result")) is True for r in valid),
            "task_collision_false_n": sum(truth(r.get("motion_collision_result")) is False for r in valid),
            "task_collision_unknown_n": sum(truth(r.get("motion_collision_result")) is None for r in valid),
            "mean_arrival_time_s_arrived_only": statistics.mean(number(r.get("elapsed_task_duration_s")) for r in valid if r.get("terminal_result") == "ARRIVED" and number(r.get("elapsed_task_duration_s")) is not None) if any(r.get("terminal_result") == "ARRIVED" and number(r.get("elapsed_task_duration_s")) is not None for r in valid) else None,
            "mean_score_equal_map_weight_complete_cells_only": statistics.mean(map_score_means.values()) if map_score_means else None,
            "score_ci95_low": slo, "score_ci95_high": shi,
            "arrival_fraction_valid_results": sum(r.get("terminal_result") == "ARRIVED" for r in valid)/len(valid) if valid else None,
            "arrival_fraction_ci95_low": alo, "arrival_fraction_ci95_high": ahi,
            "collision_fraction_known_only": sum(truth(r.get("motion_collision_result")) is True for r in valid)/(sum(truth(r.get("motion_collision_result")) is not None for r in valid)) if any(truth(r.get("motion_collision_result")) is not None for r in valid) else None,
            "collision_fraction_ci95_low": clo, "collision_fraction_ci95_high": chi,
            "bootstrap_cluster_unit": "map", "bootstrap_n": BOOTSTRAP_N, "bootstrap_seed": BOOTSTRAP_SEED,
            "bootstrap_scope_note": "Purpose-built 6 development maps; missing map summaries omitted and reported, intervals descriptive only."})
        bootstrap_rows.extend([
            {"exit_threshold_X": f"{x:.2f}", "metric": "score_complete_map_mean", "estimate": statistics.mean(map_score_means.values()) if map_score_means else None, "ci95_low": slo, "ci95_high": shi, "map_clusters": len(complete_maps), "draws": len(score_boot)},
            {"exit_threshold_X": f"{x:.2f}", "metric": "arrival_fraction_valid", "estimate": statistics.mean(v for v in map_arrival.values() if v is not None) if any(v is not None for v in map_arrival.values()) else None, "ci95_low": alo, "ci95_high": ahi, "map_clusters": sum(v is not None for v in map_arrival.values()), "draws": len(arrival_boot)},
            {"exit_threshold_X": f"{x:.2f}", "metric": "collision_fraction_known", "estimate": statistics.mean(v for v in map_collision.values() if v is not None) if any(v is not None for v in map_collision.values()) else None, "ci95_low": clo, "ci95_high": chi, "map_clusters": sum(v is not None for v in map_collision.values()), "draws": len(collision_boot)},
        ])
    write_csv(out / "threshold_summary_8.csv", threshold_summary)
    write_csv(out / "map_cluster_bootstrap_ci.csv", bootstrap_rows)
    score_complete = [r for r in scored if r["score_status"] == "COMPLETE"]
    summary = {"planned": 144, "attempts": sum(int(number(r.get("attempts")) or 0) for r in scored),
        "valid_task_records": sum(r.get("status") == "VALID_TASK_RESULT" and r.get("record_validity") == "VALID" for r in scored),
        "complete_scores": len(score_complete), "unknown_scores": 144-len(score_complete),
        "terminal_counts_valid": dict(Counter(r.get("terminal_result") for r in scored if r.get("status") == "VALID_TASK_RESULT" and r.get("record_validity") == "VALID")),
        "terminal_counts_all_rows": dict(Counter(r.get("terminal_result") or "NOT_RUN" for r in scored)),
        "score_weights": WEIGHTS, "efficiency_time_cap_s": TIME_CAP, "stability_reference_per_min": F_REF,
        "bootstrap_seed": BOOTSTRAP_SEED, "bootstrap_replicates": BOOTSTRAP_N,
        "note": "No A reference in M2; no paired B-A score. Incomplete/UNKNOWN rows are unscored, never zero-filled."}
    (out / "scoring_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    labels = [f"{x:.2f}" for x in LEVELS]
    terms = ("ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION", "DIAGNOSTIC_SIM_TIME_LIMIT", "SYSTEM_SAFETY_ABORT")
    fig, ax = plt.subplots(figsize=(10, 5)); bottom = [0]*len(LEVELS)
    for term in terms:
        values = [sum(r.get("terminal_result") == term and r.get("status") == "VALID_TASK_RESULT" and r.get("record_validity") == "VALID" and math.isclose(float(r["exit_threshold_X"]), x, abs_tol=1e-8) for r in scored) for x in LEVELS]
        ax.bar(labels, values, bottom=bottom, label=term); bottom = [a+b for a,b in zip(bottom,values)]
    ax.set(title="M2 B threshold runs: task terminal composition", xlabel="Exit threshold X", ylabel="valid task runs"); ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(out/"terminal_composition.png", dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5)); matrix = [[math.nan]*len(LEVELS) for _ in MAPS]
    for i, m in enumerate(MAPS):
        for j, x in enumerate(LEVELS):
            cell = by_cell[(m,x)]
            if cell: matrix[i][j] = statistics.mean(float(r["total_score_100"]) for r in cell)
    image = ax.imshow(matrix, aspect="auto", cmap="viridis", vmin=0, vmax=100)
    ax.set_xticks(range(len(labels)), labels); ax.set_yticks(range(len(MAPS)), MAPS)
    ax.set(title="Complete per-map mean score; cells with UNKNOWN are blank", xlabel="Exit threshold X")
    fig.colorbar(image, ax=ax, label="score / 100"); fig.tight_layout(); fig.savefig(out/"map_threshold_score_heatmap.png", dpi=150); plt.close(fig)

    component_fields = [("arrival_score_0_1", "Arrival"), ("no_terminal_collision_score_0_1", "No terminal collision"), ("efficiency_score_0_1", "Efficiency"), ("stability_score_0_1", "Stability")]
    fig, ax = plt.subplots(figsize=(10, 5))
    for field, label in component_fields:
        vals = []
        for x in LEVELS:
            v = [float(r[field]) for r in scored if r["score_status"] == "COMPLETE" and math.isclose(float(r["exit_threshold_X"]), x, abs_tol=1e-8)]
            vals.append(statistics.mean(v) if v else math.nan)
        ax.plot(labels, vals, marker="o", label=label)
    ax.set(title="Weighted-score components (complete scores only)", xlabel="Exit threshold X", ylabel="mean 0–1 component"); ax.grid(alpha=.25); ax.legend(); fig.tight_layout(); fig.savefig(out/"score_components.png", dpi=150); plt.close(fig)

    for field, filename, title, only_arrived in (("elapsed_task_duration_s", "arrival_time.png", "Arrival time (ARRIVED only)", True),
                                                  ("short_reverse_per_valid_min", "short_reverse_frequency.png", "Short reversals per valid task minute", False),
                                                  ("recovery_share_valid_motion_time", "recovery_share.png", "Recovery share of valid motion time", False)):
        means=[]; counts=[]
        for x in LEVELS:
            vals=[number(r.get(field)) for r in scored if r.get("status")=="VALID_TASK_RESULT" and r.get("record_validity")=="VALID" and math.isclose(float(r["exit_threshold_X"]),x,abs_tol=1e-8) and (not only_arrived or r.get("terminal_result")=="ARRIVED")]
            vals=[v for v in vals if v is not None]; counts.append(len(vals)); means.append(statistics.mean(vals) if vals else math.nan)
        fig, ax = plt.subplots(figsize=(9,4)); ax.plot(labels,means,"o-"); ax.set(title=f"{title}; n={counts}",xlabel="Exit threshold X",ylabel=field); ax.grid(alpha=.25); fig.tight_layout(); fig.savefig(out/filename,dpi=150); plt.close(fig)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = aggregate(args.source.resolve(), args.output_dir.resolve())
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
