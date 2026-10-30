#!/usr/bin/env python3
"""Task-specific summary for the frozen 90-run R1 M2 A/B/C confirmation batch."""
from __future__ import annotations

import csv
import json
import math
import os
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median

os.environ.setdefault("MPLCONFIGDIR", "/tmp/r1-m2-abc90-matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams["font.family"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


ROOT = Path(__file__).resolve().parents[5]
BATCH = ROOT / "logs/09_R1_switch_constraints/m2_ABC_repeat_90/20261030_r1_m2_abc90"
OUT = Path(__file__).resolve().parent
MAPS = ["d01.xml", "d03.xml", "m01.xml"]
GROUPS = ["A", "B", "C"]
GOAL = (7.5, 0.0)
Z = 1.959963984540054


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def wilson(k: int, n: int) -> tuple[float, float]:
    if not n:
        return (float("nan"), float("nan"))
    p = k / n
    den = 1 + Z * Z / n
    center = (p + Z * Z / (2 * n)) / den
    half = Z * math.sqrt(p * (1 - p) / n + Z * Z / (4 * n * n)) / den
    return center - half, center + half


def find_one(root: Path, filename: str) -> Path:
    found = list(root.rglob(filename))
    if len(found) != 1:
        raise RuntimeError(f"expected one {filename} under {root}; found {len(found)}")
    return found[0]


def task_frames(path: Path) -> list[dict]:
    frames = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("kind") != "frame":
                continue
            p = record.get("payload") or {}
            if p.get("rl_active") != 1 or p.get("sim_clock_valid") is False:
                continue
            pose = p.get("world_pose") or []
            if len(pose) < 2 or not isinstance(p.get("sim_time_s"), (int, float)):
                continue
            elapsed = float(p["sim_time_s"])
            frames.append({
                "sim_time_s": elapsed,
                "x": float(pose[0]), "y": float(pose[1]),
                "yaw": float(pose[2]) if len(pose) > 2 else float("nan"),
                "distance_m": math.hypot(GOAL[0] - float(pose[0]), GOAL[1] - float(pose[1])),
                "ra": p.get("ra_value"),
                "mode": p.get("policy_state_name") or p.get("mode_after") or "UNKNOWN",
                "hold_elapsed_s": p.get("hold_elapsed_sim_s"),
            })
    if not frames:
        raise RuntimeError(f"no active task frames in {path}")
    t0 = frames[0]["sim_time_s"]
    for frame in frames:
        frame["elapsed_s"] = frame["sim_time_s"] - t0
    return frames


def actual_config(context: dict) -> dict:
    validation = context.get("runtime_source_validation") or {}
    return validation.get("evidence") or {}


def expected_config(group: str) -> tuple[float, float, float, str, str]:
    if group == "B":
        return (-0.05, -0.10, 0.0, "true", "false")
    if group == "C":
        return (-0.05, -0.05, 0.20, "false", "true")
    return (-0.05, -0.05, 0.0, "false", "false")


def first_measured_pose(frames: list[dict]) -> tuple[float, float, float]:
    # The runtime frame stores x, y, yaw in world_pose; this is the first persisted RL-active sample.
    # Its initial state is measured, not a deterministic replay of the joint state.
    return frames[0]["x"], frames[0]["y"], frames[0]["yaw"]


def collect() -> tuple[dict, list[dict]]:
    plan = load_json(BATCH / "run_plan.json")
    if plan.get("status") != "COMPLETE" or len(plan["schedule"]["items"]) != 90:
        raise RuntimeError("the frozen run plan is not complete at 90 items")
    rows = []
    for item in plan["schedule"]["items"]:
        run_dir = BATCH / "runs" / item["plan_item_id"] / "result"
        metrics = load_json(find_one(run_dir, "metrics.json"))
        context = load_json(find_one(run_dir, "run_context.json"))
        terminal = load_json(find_one(run_dir, "terminal.json"))
        facts = load_json(find_one(run_dir, "process_facts.json"))
        frames = task_frames(find_one(run_dir, "runtime_record.jsonl"))
        group, map_name = item["group"], item["map"]
        evidence = actual_config(context)
        E, X, H, hysteresis, hold = expected_config(group)
        checks = {
            "group": evidence.get("group") == group,
            "E": evidence.get("entry_threshold") is not None and abs(float(evidence["entry_threshold"]) - E) < 1e-6,
            "X": evidence.get("exit_threshold") is not None and abs(float(evidence["exit_threshold"]) - X) < 1e-6,
            "H": evidence.get("recovery_min_hold_s") is not None and abs(float(evidence["recovery_min_hold_s"]) - H) < 1e-6,
            "hysteresis": evidence.get("hysteresis") == hysteresis,
            "hold": evidence.get("hold") == hold,
            "goal": evidence.get("goal_world_m") == list(GOAL),
            "scene": metrics.get("scene") == map_name and metrics.get("scene_id") == item["scene_id"],
            "source_validation": context.get("runtime_source_validation", {}).get("status") == "confirmed",
            "record": metrics.get("record_validity") == "VALID",
            "shutdown": facts.get("shutdown_complete") is True and facts.get("closeout_complete") is True,
        }
        if not all(checks.values()):
            raise RuntimeError(f"runtime contract/record check failed for {item['plan_item_id']}: {checks}")

        result = metrics.get("navigation_terminal_result") or terminal.get("run_terminal_result") or "UNKNOWN"
        safety_status = metrics.get("safety_task_completion_status")
        if safety_status is True:
            safety_label = "TRUE"
        elif safety_status is False:
            safety_label = "FALSE"
        else:
            safety_label = "UNKNOWN"
        saf = metrics.get("task_window_safety") or {}
        intervals = metrics.get("recovery_intervals") or []
        recovery_durations = [float(x["duration_s"]) for x in intervals if isinstance(x, dict) and isinstance(x.get("duration_s"), (int, float))]
        start_distance = metrics.get("target_distance_start_m")
        final_distance = metrics.get("target_distance_final_m")
        timeout = "SIM_TIME_LIMIT" in result
        reached = result == "ARRIVED"
        stand_done = any(x.get("event") == "STANDING_HOLD_OBSERVATION_COMPLETE" for x in (facts.get("cleanup_events") or []))
        pose = first_measured_pose(frames)
        collision_objects = []
        collision_event_values = terminal.get("collision_events")
        for event in collision_event_values if isinstance(collision_event_values, list) else []:
            if isinstance(event, dict):
                collision_objects.extend(str(event.get(k)) for k in ("obstacle_geom_name", "robot_geom_name") if event.get(k))
        if not collision_objects:
            contact_values = terminal.get("collision_contacts")
            for contact in contact_values if isinstance(contact_values, list) else []:
                if isinstance(contact, dict):
                    collision_objects.extend(str(contact.get(k)) for k in ("obstacle_geom_name", "robot_geom_name") if contact.get(k))
        rows.append({
            "sequence": item["sequence"], "round": item["round"], "map": map_name,
            "scene_id": item["scene_id"], "group": group, "plan_item_id": item["plan_item_id"],
            "run_id": metrics.get("run_id"), "terminal_result": result,
            "record_validity": metrics.get("record_validity"), "arrived": reached,
            "safe_completion": safety_label, "collision": "COLLISION" in result,
            "fall": "FALL" in result or metrics.get("motion_fall_result") is True,
            "sim_timeout": timeout, "wall_timeout": "WALL" in result and "TIME" in result,
            "safety_abort": "SAFETY_ABORT" in result or bool(facts.get("external_safety_events")),
            "arrival_standing_observation_complete": bool(stand_done) if reached else "NA",
            "arrival_time_s": float(metrics["elapsed_task_duration_s"]) if reached and isinstance(metrics.get("elapsed_task_duration_s"), (int, float)) else None,
            "task_duration_s": metrics.get("elapsed_task_duration_s"),
            "distance_start_m": start_distance, "distance_final_m": final_distance,
            "distance_progress_m": (float(start_distance) - float(final_distance)) if isinstance(start_distance, (int, float)) and isinstance(final_distance, (int, float)) else None,
            "recovery_share": metrics.get("recovery_share_valid_motion_time"),
            "recovery_episode_count": len(intervals),
            "recovery_episode_median_s": median(recovery_durations) if recovery_durations else None,
            "recovery_episode_max_s": max(recovery_durations) if recovery_durations else None,
            "mode_transition_count": metrics.get("mode_transitions"),
            "short_reverse_switch_count": metrics.get("short_reverse_switch_count"),
            "foot_contact_count": metrics.get("foot_contact_count"), "foot_impact_count": metrics.get("foot_impact_count"),
            "fall_result": metrics.get("motion_fall_result"),
            "safety_coverage_complete": saf.get("coverage_complete_for_motion_interval"),
            "actual_initial_x_m": pose[0], "actual_initial_y_m": pose[1],
            "actual_initial_yaw_rad": pose[2],
            "initial_joint_state_status": (context.get("initial_state", {}).get("motion_start_measured_state", {}) or {}).get("status"),
            "collision_object_hints": ";".join(sorted(set(collision_objects))),
            "actual_E": evidence.get("entry_threshold"), "actual_X": evidence.get("exit_threshold"),
            "actual_H_s": evidence.get("recovery_min_hold_s"),
            "runtime_contract_check": "PASS",
        })
    return plan, rows


def q(values: list[float], p: float) -> float | None:
    vals = sorted(float(x) for x in values if isinstance(x, (int, float)) and math.isfinite(float(x)))
    if not vals:
        return None
    return float(np.quantile(vals, p))


def known_sum(values: list[object]) -> tuple[int, int]:
    known = [int(v) for v in values if isinstance(v, (int, float)) and math.isfinite(float(v))]
    return sum(known), len(values) - len(known)


def summarize(rows: list[dict]) -> tuple[list[dict], list[dict]]:
    cell_rows = []
    group_rows = []
    result_keys = ["ARRIVED", "COLLISION_TERMINATION", "DIAGNOSTIC_FALL_TERMINATION",
                   "DIAGNOSTIC_SIM_TIME_LIMIT", "DIAGNOSTIC_WALL_TIME_LIMIT", "SAFETY_ABORT"]
    for map_name in MAPS:
        for group in GROUPS:
            subset = [r for r in rows if r["map"] == map_name and r["group"] == group]
            terminals = Counter(r["terminal_result"] for r in subset)
            arrivals = sum(r["arrived"] for r in subset)
            lo, hi = wilson(arrivals, len(subset))
            foot_contacts, foot_contact_unknown = known_sum([r["foot_contact_count"] for r in subset])
            foot_impacts, foot_impact_unknown = known_sum([r["foot_impact_count"] for r in subset])
            cell = {"map": map_name.removesuffix(".xml"), "group": group,
                    "planned": 10, "actual_valid": sum(r["record_validity"] == "VALID" for r in subset),
                    "arrived": arrivals, "arrival_rate": arrivals / len(subset),
                    "arrival_wilson_low": lo, "arrival_wilson_high": hi,
                    "safe_complete": sum(r["safe_completion"] == "TRUE" for r in subset),
                    "safe_complete_unknown": sum(r["safe_completion"] == "UNKNOWN" for r in subset),
                    "collisions": sum(r["collision"] for r in subset),
                    "falls": sum(r["fall"] for r in subset),
                    "sim_timeouts": sum(r["sim_timeout"] for r in subset),
                    "wall_timeouts": sum(r["wall_timeout"] for r in subset),
                    "safety_aborts": sum(r["safety_abort"] for r in subset),
                    "other_terminal": sum(terminals[k] for k in terminals if k not in result_keys),
                    "arrival_time_n": sum(r["arrival_time_s"] is not None for r in subset),
                    "arrival_time_median_s": median([r["arrival_time_s"] for r in subset if r["arrival_time_s"] is not None]) if arrivals else None,
                    "arrival_time_q25_s": q([r["arrival_time_s"] for r in subset], .25),
                    "arrival_time_q75_s": q([r["arrival_time_s"] for r in subset], .75),
                    "end_distance_median_m": median([r["distance_final_m"] for r in subset]),
                    "timeout_n": sum(r["sim_timeout"] for r in subset),
                    "timeout_distance_change_median": median([r["distance_progress_m"] for r in subset if r["sim_timeout"]]) if any(r["sim_timeout"] for r in subset) else None,
                    "recovery_share_median": median([r["recovery_share"] for r in subset if isinstance(r["recovery_share"], (int, float))]),
                    "recovery_episode_median_s": median([r["recovery_episode_median_s"] for r in subset if isinstance(r["recovery_episode_median_s"], (int, float))]),
                    "mode_transitions_median": median([r["mode_transition_count"] for r in subset if isinstance(r["mode_transition_count"], (int, float))]),
                    "short_reverse_median": median([r["short_reverse_switch_count"] for r in subset if isinstance(r["short_reverse_switch_count"], (int, float))]),
                    "foot_contacts": foot_contacts, "foot_contact_unknown_runs": foot_contact_unknown,
                    "foot_impacts": foot_impacts, "foot_impact_unknown_runs": foot_impact_unknown,
                    "foot_impact_runs": sum(isinstance(r["foot_impact_count"], (int, float)) and r["foot_impact_count"] > 0 for r in subset)}
            cell_rows.append(cell)
    for group in GROUPS:
        subset = [r for r in rows if r["group"] == group]
        arrivals = sum(r["arrived"] for r in subset)
        lo, hi = wilson(arrivals, len(subset))
        foot_contacts, foot_contact_unknown = known_sum([r["foot_contact_count"] for r in subset])
        foot_impacts, foot_impact_unknown = known_sum([r["foot_impact_count"] for r in subset])
        group_rows.append({"group": group, "planned": 30, "actual_valid": sum(r["record_validity"] == "VALID" for r in subset),
                           "arrived": arrivals, "arrival_rate": arrivals / len(subset), "arrival_wilson_low": lo, "arrival_wilson_high": hi,
                           "safe_complete": sum(r["safe_completion"] == "TRUE" for r in subset),
                           "safe_complete_unknown": sum(r["safe_completion"] == "UNKNOWN" for r in subset),
                           "collisions": sum(r["collision"] for r in subset), "falls": sum(r["fall"] for r in subset),
                           "sim_timeouts": sum(r["sim_timeout"] for r in subset), "wall_timeouts": sum(r["wall_timeout"] for r in subset),
                           "safety_aborts": sum(r["safety_abort"] for r in subset),
                           "arrival_time_n": arrivals,
                           "arrival_time_median_s": median([r["arrival_time_s"] for r in subset if r["arrival_time_s"] is not None]) if arrivals else None,
                           "end_distance_median_m": median([r["distance_final_m"] for r in subset]),
                           "recovery_share_median": median([r["recovery_share"] for r in subset if isinstance(r["recovery_share"], (int, float))]),
                           "recovery_episode_median_s": median([r["recovery_episode_median_s"] for r in subset if isinstance(r["recovery_episode_median_s"], (int, float))]),
                           "mode_transitions_median": median([r["mode_transition_count"] for r in subset if isinstance(r["mode_transition_count"], (int, float))]),
                           "short_reverse_median": median([r["short_reverse_switch_count"] for r in subset if isinstance(r["short_reverse_switch_count"], (int, float))]),
                           "foot_contacts": foot_contacts, "foot_contact_unknown_runs": foot_contact_unknown,
                           "foot_impacts": foot_impacts, "foot_impact_unknown_runs": foot_impact_unknown})
    return cell_rows, group_rows


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_maps(cell_rows: list[dict]) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), sharey=True)
    for ax, map_name in zip(axes, [m.removesuffix(".xml") for m in MAPS]):
        cells = [next(x for x in cell_rows if x["map"] == map_name and x["group"] == g) for g in GROUPS]
        x = np.arange(3)
        arr = np.array([c["arrived"] for c in cells], dtype=float)
        safe = np.array([c["safe_complete"] for c in cells], dtype=float)
        err = np.array([[max(0.0, arr[i] - 10*cells[i]["arrival_wilson_low"]),
                         max(0.0, 10*cells[i]["arrival_wilson_high"] - arr[i])] for i in range(3)]).T
        ax.bar(x - .18, arr, width=.34, color="#3D76A8", label="到达", yerr=err, capsize=3)
        ax.bar(x + .18, safe, width=.34, color="#65A878", label="确认安全完成")
        for i, c in enumerate(cells):
            ax.text(i, max(arr[i], safe[i]) + .35, f"{int(arr[i])}/10", ha="center", fontsize=9)
        ax.set_title(map_name)
        ax.set_xticks(x, GROUPS)
        ax.set_ylim(0, 11.5)
        ax.grid(axis="y", alpha=.2)
    axes[0].set_ylabel("次数（每格计划10次）")
    axes[0].legend(loc="lower right", fontsize=8)
    fig.suptitle("M2三张开发地图：到达与确认安全完成（到达率Wilson 95%区间）")
    fig.tight_layout()
    fig.savefig(OUT / "地图到达与安全完成.png", dpi=180)
    plt.close(fig)


def plot_failures(rows: list[dict]) -> None:
    cats = [("collision", "碰撞", "#C44E52"), ("fall", "跌倒", "#8172B2"),
            ("sim_timeout", "仿真限时", "#DD8452"), ("wall_timeout", "墙钟保护", "#937860"),
            ("safety_abort", "安全中止", "#64B5CD")]
    labels = [f"{m.removesuffix('.xml')}\n{g}" for m in MAPS for g in GROUPS]
    fig, ax = plt.subplots(figsize=(12, 5))
    x = np.arange(9)
    base = np.zeros(9)
    for key, label, color in cats:
        vals = np.array([sum(bool(r[key]) for r in rows if r["map"] == m and r["group"] == g) for m in MAPS for g in GROUPS])
        ax.bar(x, vals, bottom=base, label=label, color=color, width=.72)
        for i, val in enumerate(vals):
            if val:
                ax.text(i, base[i] + val / 2, str(int(val)), ha="center", va="center", color="white", fontsize=8, fontweight="bold")
        base += vals
    ax.set_xticks(x, labels)
    ax.set_ylim(0, max(10.5, float(base.max()) + 1))
    ax.set_ylabel("失败次数")
    ax.set_title("失败类型分布（每单元10次；合法终止均保留）")
    ax.grid(axis="y", alpha=.2)
    ax.legend(ncol=5, loc="upper center", bbox_to_anchor=(.5, -0.12), frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "失败类型分布.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_time_distance(rows: list[dict]) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(12, 11))
    palette = {"A": "#4C72B0", "B": "#DD8452", "C": "#55A868"}
    for i, map_name in enumerate(MAPS):
        ax_t, ax_d = axes[i]
        for group in GROUPS:
            subset = [r for r in rows if r["map"] == map_name and r["group"] == group]
            times = [r["arrival_time_s"] for r in subset if r["arrival_time_s"] is not None]
            dists = [r["distance_final_m"] for r in subset if isinstance(r["distance_final_m"], (int, float))]
            xpos = GROUPS.index(group)
            jitter = np.linspace(-.07, .07, max(1, len(times)))
            ax_t.scatter(np.full(len(times), xpos) + jitter, times, color=palette[group], label=group, s=28, alpha=.8)
            djitter = np.linspace(-.08, .08, len(dists))
            ax_d.scatter(np.full(len(dists), xpos) + djitter, dists, color=palette[group], s=26, alpha=.75)
        ax_t.set_title(f"{map_name.removesuffix('.xml')}：成功到达耗时")
        ax_t.set_ylabel("任务窗仿真秒")
        ax_t.set_xticks(range(3), GROUPS)
        ax_t.grid(alpha=.2)
        ax_d.set_title(f"{map_name.removesuffix('.xml')}：全部运行终点目标距离")
        ax_d.axhline(.5, color="black", linestyle="--", linewidth=1, label="到达半径0.5m")
        ax_d.set_ylabel("米")
        ax_d.set_xticks(range(3), GROUPS)
        ax_d.grid(alpha=.2)
        if i == 0:
            ax_t.legend(title="组别", ncol=3, fontsize=8)
            ax_d.legend(fontsize=8)
    fig.suptitle("成功到达耗时与全部终点距离；耗时只在成功样本中比较")
    fig.tight_layout(rect=(0, 0, 1, .98))
    fig.savefig(OUT / "到达耗时与终点距离.png", dpi=180)
    plt.close(fig)


def choose_case(rows: list[dict], map_name: str, group: str, terminal: str) -> dict:
    candidates = [r for r in rows if r["map"] == map_name and r["group"] == group and terminal in r["terminal_result"]]
    if not candidates:
        raise RuntimeError(f"no representative case {map_name}/{group}/{terminal}")
    return max(candidates, key=lambda r: float(r["task_duration_s"] or 0))


def plot_timelines(rows: list[dict]) -> list[dict]:
    chosen = [choose_case(rows, "d01.xml", "A", "COLLISION"),
              choose_case(rows, "m01.xml", "B", "SIM_TIME_LIMIT"),
              choose_case(rows, "d03.xml", "A", "SIM_TIME_LIMIT")]
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    for ax, row in zip(axes, chosen):
        item_dir = BATCH / "runs" / row["plan_item_id"] / "result"
        data = task_frames(find_one(item_dir, "runtime_record.jsonl"))
        t = [x["elapsed_s"] for x in data]
        dist = [x["distance_m"] for x in data]
        ra = [float(x["ra"]) if isinstance(x["ra"], (int, float)) else float("nan") for x in data]
        modes = [str(x["mode"]).upper() for x in data]
        for j in range(len(data) - 1):
            if "RECOVERY" in modes[j]:
                ax.axvspan(t[j], t[j+1], color="#F2C14E", alpha=.22, lw=0)
        ax.plot(t, dist, color="#3D76A8", lw=1.6, label="目标距离（按位姿计算）")
        ax.set_ylabel("目标距离 m", color="#3D76A8")
        ax.tick_params(axis="y", labelcolor="#3D76A8")
        ax.axhline(.5, color="#3D76A8", linestyle=":", lw=.8)
        twin = ax.twinx()
        twin.plot(t, ra, color="#C44E52", lw=1.0, alpha=.9, label="RA")
        twin.axhline(-.05, color="#C44E52", linestyle="--", lw=.8, alpha=.7, label="E=−0.05")
        if row["group"] == "B":
            twin.axhline(-.10, color="#8172B2", linestyle="--", lw=.8, alpha=.7, label="X=−0.10")
        twin.set_ylabel("RA")
        ax.set_title(f"{row['map']} {row['group']} / {row['terminal_result']} / {row['plan_item_id']}", loc="left", fontsize=10)
        ax.set_xlabel("RL任务窗仿真秒；黄色背景为Recovery")
        ax.grid(alpha=.18)
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = twin.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8, ncol=2)
    fig.suptitle("代表运行：目标距离、RA与Recovery状态时间线")
    fig.tight_layout(rect=(0, 0, 1, .98))
    fig.savefig(OUT / "代表运行时间线.png", dpi=180)
    plt.close(fig)
    return [{k: r[k] for k in ("plan_item_id", "map", "group", "run_id", "terminal_result", "task_duration_s", "distance_final_m")} for r in chosen]


def fmt_pct(value: float | None) -> str:
    return "UNKNOWN" if value is None else f"{100*value:.1f}%"


def write_report(plan: dict, rows: list[dict], cells: list[dict], groups: list[dict], cases: list[dict]) -> None:
    batch_bytes = sum(p.stat().st_size for p in BATCH.rglob("*") if p.is_file())
    terminal = Counter(r["terminal_result"] for r in rows)
    safe_unknown = sum(r["safe_completion"] == "UNKNOWN" for r in rows)
    safe_true = sum(r["safe_completion"] == "TRUE" for r in rows)
    startup_event = load_json(BATCH / "dispatch_events.jsonl") if (BATCH / "dispatch_events.jsonl").exists() else None
    md = []
    md += ["# R1-4：M2三图A/B/C重复确认实验", "", "更新：2026-10-30。R1下的开发确认阶段；不是独立验证。分析使用本批运行器生成的metrics、context、terminal、process_facts和策略帧记录。", ""]
    md += ["## 实验执行", "", f"- 计划/有效完成：90/90；实际场景启动：{plan['counters']['actual_scene_starts']}；场景补做：{plan['counters']['replacement_scene_starts']}；合法导航结果未补跑。", f"- 调度：10轮交错；种子{plan['schedule']['seed']}。场景终止计数：到达{terminal['ARRIVED']}、碰撞{terminal['COLLISION_TERMINATION']}、仿真限时{terminal['DIAGNOSTIC_SIM_TIME_LIMIT']}、墙钟保护{terminal['DIAGNOSTIC_WALL_TIME_LIMIT']}、跌倒{sum(r['fall'] for r in rows)}、安全中止{sum(r['safety_abort'] for r in rows)}。", f"- 安全完成：确认{safe_true}；UNKNOWN {safe_unknown}。安全完成按已有` safety_task_completion_status `判定；足端接触/撞击另行统计。", f"- 到达后的2秒站立观察：{sum(r['arrival_standing_observation_complete'] is True for r in rows)}次到达完成；未完成/未知{sum(r['arrived'] and r['arrival_standing_observation_complete'] is not True for r in rows)}次。", f"- 本批目录占用约{batch_bytes / (1024**3):.2f} GiB（{batch_bytes:,} bytes）；完成时启动器检查约{plan.get('storage_checkpoints', [{}])[-1].get('filesystem_free_bytes', 0)/(1024**3):.1f} GiB可用。", "- 启动前沙箱GLFW预检失败一次，未启动场景；在同一用户的现有DISPLAY/Xauthority上下文中预检通过后执行。见`dispatch_events.jsonl`。", ""]
    md += ["地图身份固定为M2 selected_v2：`unitree_mujoco/unitree_robots/go2/d01.xml`→m2v2_d01，`d03.xml`→m2v2_d03，`m01.xml`→m2v2_m01；地图版本`m2-selected-development/2.1`。m01沿用M2 selected_v2中的`m2dev04-r5-20261018`布局修订。启动器逐项核对实际组别、E/X/H、机制开关、目标和场景身份；90项均为VALID，关闭清理确认均通过。使用隐藏MuJoCo窗口、关闭RViz和帧缓冲截图，没有截图或录像。", "", "## 地图选择依据", "", "三图都根据R1-3的具体A/B/C差异选入，而非按本批预期结果筛选：", "", "- **d01：**此前A有一次碰撞、B/C各2/2到达；A代表轨迹从约7.5米推进到约3.5米后长时间停滞，适合复查推进不足是否重复。", "- **d03：**此前A有一次Recovery内部碰撞，B/C各2/2到达，适合确认Recovery仍工作时的碰撞和任务结局是否重复。", "- **m01：**此前A有一次碰撞，碰撞前最近退出RA约−0.053、退出后约0.07秒碰撞；B/C各2/2到达，适合复查与退出时机相符的差异。", "", "## 逐地图结果", "", "到达率为到达数/10，括号为Wilson 95%区间。确认安全完成率单列；UNKNOWN不记为False。", "", "| 地图 | 组 | 计划/有效完成 | 到达 | 到达率（Wilson 95%） | 确认安全完成 | UNKNOWN | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 |", "|---|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for c in cells:
        md.append(f"| {c['map']} | {c['group']} | {c['planned']}/{c['actual_valid']} | {c['arrived']} | {fmt_pct(c['arrival_rate'])} ({100*c['arrival_wilson_low']:.1f}–{100*c['arrival_wilson_high']:.1f}%) | {c['safe_complete']} | {c['safe_complete_unknown']} | {c['collisions']} | {c['falls']} | {c['sim_timeouts']} | {c['wall_timeouts']} | {c['safety_aborts']} |")
    md += ["", "## 每组30项汇总", "", "| 组 | 计划/有效 | 到达 | 到达率（Wilson 95%） | 确认安全完成 | UNKNOWN | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 成功到达耗时中位数（样本数） | 终点目标距离中位数 | Recovery占比中位数 | 单次Recovery驻留中位数 | 模式切换中位数 | 短反向中位数 | 足端接触/撞击（已知数；UNKNOWN运行数） |", "|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for g in groups:
        md.append(f"| {g['group']} | {g['planned']}/{g['actual_valid']} | {g['arrived']} | {fmt_pct(g['arrival_rate'])} ({100*g['arrival_wilson_low']:.1f}–{100*g['arrival_wilson_high']:.1f}%) | {g['safe_complete']} | {g['safe_complete_unknown']} | {g['collisions']} | {g['falls']} | {g['sim_timeouts']} | {g['wall_timeouts']} | {g['safety_aborts']} | {g['arrival_time_median_s']:.2f}s (n={g['arrival_time_n']}) | {g['end_distance_median_m']:.2f}m | {100*g['recovery_share_median']:.1f}% | {g['recovery_episode_median_s']:.2f}s | {g['mode_transitions_median']:.0f} | {g['short_reverse_median']:.0f} | {g['foot_contacts']}/{g['foot_impacts']} ({g['foot_contact_unknown_runs']}/{g['foot_impact_unknown_runs']}) |")
    md += ["", "图表：", "", "- ![三图到达和安全完成](地图到达与安全完成.png)", "- ![失败类型](失败类型分布.png)", "- ![到达耗时和终点距离](到达耗时与终点距离.png)", "- ![代表时间线](代表运行时间线.png)", "", "## 任务推进和行为代价", "", "成功到达耗时仅在到达运行中比较，组间成功样本并不相同，不能据此单独判定整体效率。所有运行终点目标距离汇总于`逐次运行明细.csv`；3项仿真限时的距离变化（起点距离减终点距离，正数表示推进）如下："]
    for r in rows:
        if r["sim_timeout"]:
            md.append(f"- {r['map']} {r['group']} {r['plan_item_id']}：{r['distance_progress_m']:.2f}m（终点{r['distance_final_m']:.2f}m）。")
    all_durations = [r["recovery_episode_median_s"] for r in rows if isinstance(r["recovery_episode_median_s"], (int, float))]
    global_recovery_median = median([r["recovery_share"] for r in rows if isinstance(r["recovery_share"], (int, float))])
    all_episode = [r["recovery_episode_median_s"] for r in rows if isinstance(r["recovery_episode_median_s"], (int, float))]
    md += ["", f"90项中Recovery占比的运行级中位数为{global_recovery_median*100:.1f}%；每次运行内Recovery驻留时长中位数的总体中位数为{median(all_episode):.2f}s。模式切换、短反向中位数和足端接触/撞击总数按组列在上表，完整运行值保存在CSV。切换指标用于描述行为，不单独代表效果。", "", "## 按地图观察", ""]
    narrative = {
        "d01": "A到达8/10，B 5/10，C 6/10；本次重复中A到达多于B/C。B/C的失败以碰撞为主（B 5次、C 4次），A为2次碰撞；先前“A撞、B/C到达”的组合没有稳定重复。",
        "d03": "A到达8/10并有2次仿真限时，B和C均10/10到达。本次A的两次超时仍体现推进不足，但重复中未出现碰撞；B/C在此开发图重复达到目标。",
        "m01": "A、C均9/10到达，B为8/10；A/C各有1次碰撞，B有1次碰撞和1次仿真限时。此前B/C均到达的现象只部分重复，B本次出现推进超时。",
    }
    for name in ("d01", "d03", "m01"):
        md.append(f"- **{name}：** {narrative[name]}")
    md += ["", "这些地图是按此前M2结果挑选的开发地图，比较用于确认所见差异是否能在这些场景复现，不能作为对一般地图的独立验证。每格10次的区间较宽；本批未证明总体组别优劣。三图合计A/B/C均为25/30安全到达（B为23/30）；此汇总不能取代逐地图比较。", "", "## 代表时间线", "", "按地图挑选A组d01碰撞、B组m01仿真限时、A组d03仿真限时的长任务运行。目标距离由记录的世界位姿计算；RA与模式直接来自实际策略帧，黄色背景标示Recovery。它们用于展示过程，不作为因果证据。", "", "| 地图/组 | 计划项 | 终止 | 任务时长 | 终点目标距离 |", "|---|---|---|---:|---:|"]
    for c in cases:
        md.append(f"| {c['map']} {c['group']} | {c['plan_item_id']} | {c['terminal_result']} | {float(c['task_duration_s']):.2f}s | {float(c['distance_final_m']):.2f}m |")
    md += ["", "## 解释边界", "", "- A/B/C的初始关节状态没有严格重放；逐次上下文保留了关节状态采样，逐次明细记录了第一个RL有效帧的实际x/y/yaw。没有额外加入随机扰动，仍存在运行间初态差异。", "- 成功到达的任务窗安全判定均为确认TRUE，未见UNKNOWN；安全完成只代表既有任务窗字段支持到达且 required safety dimensions clear。足端接触和足端撞击另行保留，不并入碰撞或当作安全完成失败。", "- 3次仿真限时均为20秒有效任务窗；总仿真时钟因场景准备更早开始，终端原始时刻会更大。墙钟保护未触发。", "- 没有跌倒或安全中止；没有场景补做。启动前一次GLFW失败在任何场景启动前解决。", "- m01使用M2 selected_v2中的r5修订版，未用M3同源地图替代。", "", "## 数据位置", "", f"- 原始批次：`logs/09_R1_switch_constraints/m2_ABC_repeat_90/{BATCH.name}/`（当前约{batch_bytes / 1024**3:.2f} GiB）。", "- 逐项统计：`逐次运行明细.csv`。", "- 9格结果：`地图组别汇总.csv`。", "- 组别汇总：`组别汇总.csv`。", "- 本报告和图表保存在当前目录。", ""]
    (OUT / "分析报告.md").write_text("\n".join(md), encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    plan, rows = collect()
    cells, groups = summarize(rows)
    write_csv(OUT / "逐次运行明细.csv", rows)
    write_csv(OUT / "地图组别汇总.csv", cells)
    write_csv(OUT / "组别汇总.csv", groups)
    plot_maps(cells)
    plot_failures(rows)
    plot_time_distance(rows)
    cases = plot_timelines(rows)
    (OUT / "代表案例.json").write_text(json.dumps(cases, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_report(plan, rows, cells, groups, cases)
    print(f"Analyzed {len(rows)} VALID records; outputs saved to {OUT}")
    print("Cells:", [(c["map"], c["group"], c["arrived"], c["safe_complete"], c["collisions"], c["sim_timeouts"]) for c in cells])
    print("Groups:", [(g["group"], g["arrived"], g["safe_complete"], g["collisions"], g["sim_timeouts"]) for g in groups])


if __name__ == "__main__":
    main()
