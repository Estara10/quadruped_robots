#!/usr/bin/env python3
"""Aggregate ABS simulation evaluation summaries.

Reads summary.json files produced by scripts/run_abs_eval.py and writes an
aggregate CSV plus a Markdown report for quick experiment review.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

ROOT = Path.home() / "quadruped_robots"
DEFAULT_INPUT = ROOT / "logs" / "abs_eval"
MUJOCO_SCENE_DIR = ROOT / "unitree_mujoco" / "unitree_robots" / "go2"
DEFAULT_SPAWN_CLEARANCE_RADIUS = 0.75
DEFAULT_CORRIDOR_CLEAR_X = 1.5
DEFAULT_CORRIDOR_HALF_WIDTH = 0.45
STRESS_TEST_SCENES = {"scene_test3.xml"}

CSV_FIELDS = [
    "scene", "run_index", "ablation_mode", "result", "success", "time_to_goal_s",
    "duration_limit_s", "min_goal_error_m", "final_goal_error_m",
    "min_ray_distance_m", "recovery_entries", "recovery_exits",
    "recovery_ratio", "ra_min", "ra_max", "ra_mean", "dds_timeout_count",
    "rec_nan_count", "fall", "fall_reason", "collision", "collision_reason",
    "collision_count_max", "collision_event_total", "collision_robot_geom", "collision_obstacle_geom",
    "collision_available", "collision_proxy", "collision_proxy_reason", "eval_samples",
    "heading_abs_mean_rad",
    "heading_abs_max_rad", "mean_body_speed_mps", "mean_abs_lateral_velocity_mps",
    "qpos_available", "ray2d_available", "scene_clearance_status",
    "scene_min_spawn_clearance_m", "scene_spawn_violation", "scene_corridor_warning",
    "stuck_recovery_loop", "scene_group", "runtime_log", "summary_json",
]


def is_success(summary: Dict[str, Any]) -> bool:
    return summary.get("result") == "SUCCESS"


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        if not math.isfinite(value):
            return "-"
        return f"{value:.{digits}f}"
    return str(value)


def mean(values: Iterable[Optional[float]]) -> Optional[float]:
    clean = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    return sum(clean) / len(clean) if clean else None


def median(values: Iterable[Optional[float]]) -> Optional[float]:
    clean = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    return statistics.median(clean) if clean else None


def _parse_vec(text: str, expected: int) -> Optional[List[float]]:
    try:
        values = [float(v) for v in text.split()]
    except ValueError:
        return None
    if len(values) < expected:
        return None
    return values


def _footprint_clearance(geom_type: str, pos: List[float], size: List[float]) -> Optional[float]:
    x, y = pos[0], pos[1]
    if geom_type == "box":
        sx, sy = size[0], size[1]
        return math.hypot(max(abs(x) - sx, 0.0), max(abs(y) - sy, 0.0))
    if geom_type in {"cylinder", "sphere", "capsule", "ellipsoid"}:
        return max(0.0, math.hypot(x, y) - size[0])
    return None


def _intersects_initial_corridor(geom_type: str, pos: List[float], size: List[float]) -> bool:
    x, y = pos[0], pos[1]
    if geom_type == "box":
        front = x - size[0]
        rear = x + size[0]
        lateral_clearance = abs(y) - size[1]
    elif geom_type in {"cylinder", "sphere", "capsule", "ellipsoid"}:
        radius = size[0]
        front = x - radius
        rear = x + radius
        lateral_clearance = abs(y) - radius
    else:
        return False
    return rear >= 0.0 and front <= DEFAULT_CORRIDOR_CLEAR_X and lateral_clearance <= DEFAULT_CORRIDOR_HALF_WIDTH


def analyze_scene_file(scene: str) -> Dict[str, Any]:
    path = MUJOCO_SCENE_DIR / scene
    if not path.exists():
        return {}
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        return {}

    min_clearance: Optional[float] = None
    spawn_violation = False
    corridor_warning = False
    obstacle_types = {"box", "cylinder", "sphere", "capsule", "ellipsoid"}
    for geom in root.findall(".//geom"):
        geom_type = geom.attrib.get("type", "")
        if geom_type not in obstacle_types:
            continue
        pos = _parse_vec(geom.attrib.get("pos", ""), 3)
        size = _parse_vec(geom.attrib.get("size", ""), 1)
        if pos is None or size is None or (geom_type == "box" and len(size) < 2):
            continue
        clearance = _footprint_clearance(geom_type, pos, size)
        if clearance is not None:
            min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
            if clearance < DEFAULT_SPAWN_CLEARANCE_RADIUS:
                spawn_violation = True
        if _intersects_initial_corridor(geom_type, pos, size):
            corridor_warning = True

    status = "FAIL_SPAWN_CLEARANCE" if spawn_violation else ("WARN_CORRIDOR" if corridor_warning else "OK")
    return {
        "scene_clearance_status": status,
        "scene_min_spawn_clearance_m": min_clearance,
        "scene_spawn_violation": spawn_violation,
        "scene_corridor_warning": corridor_warning,
    }


def is_stuck_recovery_loop(summary: Dict[str, Any]) -> bool:
    if is_success(summary):
        return False
    if summary.get("fall") is True or summary.get("collision") is True or summary.get("collision_proxy") is True:
        return False
    if int(summary.get("dds_timeout_count") or 0) > 0:
        return False

    recovery_ratio = summary.get("recovery_ratio")
    recovery_entries = int(summary.get("recovery_entries") or 0)
    min_goal_error = summary.get("min_goal_error_m")
    final_goal_error = summary.get("final_goal_error_m")
    if recovery_ratio is None or min_goal_error is None or final_goal_error is None:
        return False

    return (
        float(recovery_ratio) >= 0.45
        and recovery_entries >= 20
        and float(min_goal_error) >= 3.0
        and float(final_goal_error) >= 3.0
    )


def classify_summary(summary: Dict[str, Any]) -> Dict[str, Any]:
    summary = dict(summary)
    scene = str(summary.get("scene", ""))
    summary["scene_group"] = "stress" if scene in STRESS_TEST_SCENES else "baseline"

    if summary.get("result") == "TIMEOUT" and is_stuck_recovery_loop(summary):
        summary["result"] = "STUCK_RECOVERY_LOOP"
        summary["stuck_recovery_loop"] = True
    else:
        summary["stuck_recovery_loop"] = bool(summary.get("stuck_recovery_loop", False))

    return summary


def enrich_missing_scene_clearance(summary: Dict[str, Any]) -> Dict[str, Any]:
    if summary.get("scene_clearance_status"):
        return summary
    scene = summary.get("scene")
    if not scene:
        return summary
    info = analyze_scene_file(str(scene))
    if info:
        summary = dict(summary)
        summary.update(info)
        summary.setdefault(
            "invalid_spawn_failure",
            summary.get("result") == "FALL"
            and summary.get("monitor_samples") is not None
            and int(summary.get("monitor_samples") or 999999) <= 3
            and bool(summary.get("scene_spawn_violation")),
        )
    return summary


def load_summaries(input_path: Path) -> List[Dict[str, Any]]:
    if input_path.is_file():
        candidates = [input_path]
    else:
        candidates = sorted(input_path.rglob("summary.json"))
        candidates = [p for p in candidates if p.name == "summary.json"]

    summaries: List[Dict[str, Any]] = []
    for path in candidates:
        try:
            data = json.loads(path.read_text())
        except json.JSONDecodeError as exc:
            print(f"[skip] invalid JSON {path}: {exc}")
            continue
        data.setdefault("summary_json", str(path))
        summaries.append(classify_summary(enrich_missing_scene_clearance(data)))
    return summaries


def write_csv(summaries: List[Dict[str, Any]], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(summaries)


def scene_stats(summaries: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for item in summaries:
        grouped[str(item.get("scene", "unknown"))].append(item)

    stats: Dict[str, Dict[str, Any]] = {}
    for scene, rows in grouped.items():
        n = len(rows)
        successes = sum(1 for r in rows if is_success(r))
        stats[scene] = {
            "runs": n,
            "successes": successes,
            "success_rate": successes / n if n else 0.0,
            "results": Counter(str(r.get("result", "UNKNOWN")) for r in rows),
            "mean_time_to_goal_s": mean(r.get("time_to_goal_s") for r in rows),
            "median_time_to_goal_s": median(r.get("time_to_goal_s") for r in rows),
            "mean_final_goal_error_m": mean(r.get("final_goal_error_m") for r in rows),
            "min_ray_distance_m": min(
                [float(r["min_ray_distance_m"]) for r in rows if r.get("min_ray_distance_m") is not None],
                default=None,
            ),
            "mean_recovery_entries": mean(r.get("recovery_entries") for r in rows),
            "mean_recovery_ratio": mean(r.get("recovery_ratio") for r in rows),
            "mean_ra_max": mean(r.get("ra_max") for r in rows),
            "mean_heading_abs_rad": mean(r.get("heading_abs_mean_rad") for r in rows),
            "max_heading_abs_rad": max(
                [float(r["heading_abs_max_rad"]) for r in rows if r.get("heading_abs_max_rad") is not None],
                default=None,
            ),
            "mean_body_speed_mps": mean(r.get("mean_body_speed_mps") for r in rows),
            "mean_abs_lateral_velocity_mps": mean(r.get("mean_abs_lateral_velocity_mps") for r in rows),
            "dds_timeouts": sum(int(r.get("dds_timeout_count") or 0) for r in rows),
            "falls": sum(1 for r in rows if r.get("fall") is True),
            "collisions": sum(1 for r in rows if r.get("collision") is True or r.get("result") == "COLLISION"),
            "collision_proxies": sum(1 for r in rows if r.get("collision_proxy") is True),
            "stuck_recovery_loops": sum(1 for r in rows if r.get("stuck_recovery_loop") is True),
            "invalid_spawn_failures": sum(1 for r in rows if r.get("invalid_spawn_failure") is True),
            "scene_clearance_statuses": Counter(str(r.get("scene_clearance_status", "UNKNOWN")) for r in rows),
        }
    return stats


def ablation_stats(summaries: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for item in summaries:
        grouped[str(item.get("ablation_mode", "unknown"))].append(item)

    stats: Dict[str, Dict[str, Any]] = {}
    for mode, rows in grouped.items():
        n = len(rows)
        successes = sum(1 for r in rows if is_success(r))
        stats[mode] = {
            "runs": n,
            "successes": successes,
            "success_rate": successes / n if n else 0.0,
            "results": Counter(str(r.get("result", "UNKNOWN")) for r in rows),
            "mean_time_to_goal_s": mean(r.get("time_to_goal_s") for r in rows),
            "mean_recovery_ratio": mean(r.get("recovery_ratio") for r in rows),
            "mean_recovery_entries": mean(r.get("recovery_entries") for r in rows),
            "min_ray_distance_m": min(
                [float(r["min_ray_distance_m"]) for r in rows if r.get("min_ray_distance_m") is not None],
                default=None,
            ),
            "falls": sum(1 for r in rows if r.get("fall") is True),
            "collisions": sum(1 for r in rows if r.get("collision") is True or r.get("result") == "COLLISION"),
            "collision_proxies": sum(1 for r in rows if r.get("collision_proxy") is True),
            "stuck_recovery_loops": sum(1 for r in rows if r.get("stuck_recovery_loop") is True),
        }
    return stats


def write_group_overview(lines: List[str], title: str, summaries: List[Dict[str, Any]]) -> None:
    total = len(summaries)
    success_count = sum(1 for r in summaries if is_success(r))
    results = Counter(str(r.get("result", "UNKNOWN")) for r in summaries)
    recovery_ratios = [r.get("recovery_ratio") for r in summaries]

    lines.append(f"## {title}")
    lines.append("")
    lines.append(f"- Episodes: **{total}**")
    lines.append(f"- Successes: **{success_count}/{total}** ({(success_count / total * 100.0 if total else 0.0):.1f}%)")
    lines.append(f"- Result distribution: `{dict(results)}`")
    lines.append(f"- Mean recovery ratio: **{fmt(mean(recovery_ratios))}**")
    lines.append(f"- Total DDS timeout logs: **{sum(int(r.get('dds_timeout_count') or 0) for r in summaries)}**")
    lines.append(f"- Fall episodes: **{sum(1 for r in summaries if r.get('fall') is True)}**")
    lines.append(f"- True collision episodes: **{sum(1 for r in summaries if r.get('collision') is True or r.get('result') == 'COLLISION')}**")
    lines.append(f"- Stuck recovery-loop episodes: **{sum(1 for r in summaries if r.get('stuck_recovery_loop') is True)}**")
    lines.append(f"- Invalid spawn/early-fall episodes: **{sum(1 for r in summaries if r.get('invalid_spawn_failure') is True)}**")
    lines.append(f"- Collision-proxy episodes: **{sum(1 for r in summaries if r.get('collision_proxy') is True)}**")
    lines.append("")


def write_markdown(summaries: List[Dict[str, Any]], md_path: Path) -> None:
    stats = scene_stats(summaries)
    abl_stats = ablation_stats(summaries)
    baseline = [r for r in summaries if r.get("scene_group") == "baseline"]
    stress = [r for r in summaries if r.get("scene_group") == "stress"]

    lines: List[str] = []
    lines.append("# ABS Simulation Evaluation Report")
    lines.append("")
    lines.append(f"Generated: {datetime.now().isoformat(timespec='seconds')}")
    lines.append("")

    write_group_overview(lines, "Overall", summaries)
    write_group_overview(lines, "Standard Baseline", baseline)
    write_group_overview(lines, "Pressure Tests", stress)

    lines.append("## Ablation summary")
    lines.append("")
    lines.append("| Mode | Runs | Success | Results | Mean T_goal(s) | Min ray(m) | Mean rec entries | Mean rec ratio | Fall | Collision | Stuck loop | Collision proxy |")
    lines.append("|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for mode in sorted(abl_stats):
        s = abl_stats[mode]
        lines.append(
            f"| {mode} | {s['runs']} | {s['successes']}/{s['runs']} ({s['success_rate'] * 100.0:.1f}%) | "
            f"`{dict(s['results'])}` | {fmt(s['mean_time_to_goal_s'])} | {fmt(s['min_ray_distance_m'])} | "
            f"{fmt(s['mean_recovery_entries'])} | {fmt(s['mean_recovery_ratio'])} | {s['falls']} | "
            f"{s['collisions']} | {s['stuck_recovery_loops']} | {s['collision_proxies']} |"
        )
    lines.append("")

    lines.append("## Per-scene summary")
    lines.append("")
    lines.append("| Scene | Group | Runs | Success | Results | Clearance | Mean T_goal(s) | Mean final err(m) | Min ray(m) | Mean rec entries | Mean rec ratio | Mean heading abs(rad) | Max heading abs(rad) | Mean speed(m/s) | Mean abs vy(m/s) | DDS | Fall | Collision | Stuck loop | Invalid spawn | Collision proxy |")
    lines.append("|---|---|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for scene in sorted(stats):
        s = stats[scene]
        lines.append(
            "| "
            f"{scene} | {'stress' if scene in STRESS_TEST_SCENES else 'baseline'} | "
            f"{s['runs']} | {s['successes']}/{s['runs']} ({s['success_rate'] * 100.0:.1f}%) | "
            f"`{dict(s['results'])}` | `{dict(s['scene_clearance_statuses'])}` | "
            f"{fmt(s['mean_time_to_goal_s'])} | {fmt(s['mean_final_goal_error_m'])} | "
            f"{fmt(s['min_ray_distance_m'])} | {fmt(s['mean_recovery_entries'])} | "
            f"{fmt(s['mean_recovery_ratio'])} | {fmt(s['mean_heading_abs_rad'])} | "
            f"{fmt(s['max_heading_abs_rad'])} | {fmt(s['mean_body_speed_mps'])} | "
            f"{fmt(s['mean_abs_lateral_velocity_mps'])} | {s['dds_timeouts']} | {s['falls']} | "
            f"{s['collisions']} | {s['stuck_recovery_loops']} | {s['invalid_spawn_failures']} | {s['collision_proxies']} |"
        )
    lines.append("")

    failures = [r for r in summaries if not is_success(r)]
    lines.append("## Failure / non-success episodes")
    lines.append("")
    if not failures:
        lines.append("No non-success episodes recorded.")
    else:
        lines.append("| Scene | Run | Result | Final err(m) | Min ray(m) | Recovery entries | Reason | Log |")
        lines.append("|---|---:|---|---:|---:|---:|---|---|")
        for r in failures:
            reason = r.get("collision_reason") or r.get("fall_reason") or r.get("collision_proxy_reason") or r.get("skip_reason") or "-"
            if r.get("stuck_recovery_loop") is True:
                reason = "high recovery ratio with poor goal progress"
            if r.get("invalid_spawn_failure") is True:
                reason = f"invalid spawn / early fall: {reason}"
            lines.append(
                f"| {r.get('scene')} | {r.get('run_index')} | {r.get('result')} | "
                f"{fmt(r.get('final_goal_error_m'))} | {fmt(r.get('min_ray_distance_m'))} | "
                f"{r.get('recovery_entries')} | {reason} | `{r.get('runtime_log')}` |"
            )
    lines.append("")

    lines.append("## Interpretation notes")
    lines.append("")
    lines.append("- `COLLISION` is true MuJoCo robot-vs-obstacle contact: robot collision geom group 3 touching static box/cylinder/sphere/capsule/ellipsoid obstacle geoms.")
    lines.append("- `COLLISION_PROXY` is only a ray-distance near-collision risk; it is kept as a diagnostic, not a replacement for true contact counting.")
    lines.append("- `recovery_ratio` comes from throttled `[EVAL]` log samples, so it is approximate but useful for detecting over-frequent recovery switching.")
    lines.append("- `heading_abs_mean_rad` and `mean_abs_lateral_velocity_mps` are drift/heading-quality indicators for the next behavior-calibration pass.")
    lines.append("- `STUCK_RECOVERY_LOOP` means timeout with high recovery ratio, many recovery entries, and poor goal progress; treat it as a pressure-test/local-navigation failure mode.")
    lines.append("- `INVALID_SCENE_SPAWN` / `invalid_spawn_failure` means a scene obstacle violates the configured spawn-clearance precheck; do not count it as ABS policy failure.")
    lines.append("- Flat-ground runs should have low recovery ratio; obstacle scenes should show RA/recovery activity before close obstacle proximity.")
    lines.append("")

    md_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Evaluation session directory or a summary.json file")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--csv-name", default="aggregate_report.csv")
    parser.add_argument("--md-name", default="aggregate_report.md")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summaries = load_summaries(args.input)
    if not summaries:
        print(f"No summary.json files found under {args.input}")
        return 1

    output_dir = args.output_dir or (args.input if args.input.is_dir() else args.input.parent)
    csv_path = output_dir / args.csv_name
    md_path = output_dir / args.md_name

    write_csv(summaries, csv_path)
    write_markdown(summaries, md_path)

    total = len(summaries)
    successes = sum(1 for r in summaries if is_success(r))
    print(f"Episodes: {total}")
    print(f"Success rate: {successes}/{total} ({successes / total * 100.0:.1f}%)")
    print(f"CSV: {csv_path}")
    print(f"Markdown: {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
