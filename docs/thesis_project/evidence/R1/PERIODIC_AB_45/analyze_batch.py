#!/usr/bin/env python3
"""Summarize the frozen R1 45-item A/B Periodic batch without per-run plots."""
from __future__ import annotations

import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
BATCH = ROOT / "logs/09_R1_switch_constraints/periodic_ab_45/20261103_r1_periodic_ab_45"
OUT = Path(__file__).resolve().parent
MAPS = ["r1_d09_gap_narrow_v2.xml", "r1_m02_space_middle_v2.xml", "challenge27.xml"]
SETTINGS = ["A_REF", "B_E04_X07", "B_E05_X10"]
TERMINALS = ["ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
             "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE", "SYSTEM_SAFETY_ABORT"]


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def q(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    xs = sorted(values)
    pos = (len(xs) - 1) * fraction
    lo, hi = math.floor(pos), math.ceil(pos)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def wilson(k: int, n: int, z: float = 1.959963984540054) -> tuple[float, float] | None:
    if n == 0:
        return None
    p = k / n
    den = 1 + z * z / n
    center = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return center - half, center + half


def difference_interval(k1: int, n1: int, k2: int, n2: int) -> tuple[float, float] | None:
    a, b = wilson(k1, n1), wilson(k2, n2)
    if a is None or b is None:
        return None
    p1, p2 = k1 / n1, k2 / n2
    d = p1 - p2
    return d - math.sqrt((p1 - a[0]) ** 2 + (b[1] - p2) ** 2), \
        d + math.sqrt((a[1] - p1) ** 2 + (p2 - b[0]) ** 2)


def periodic_facts(run_dir: Path) -> dict:
    path = run_dir / "ros_launch.log"
    text = path.read_text(encoding="utf-8", errors="replace") if path.is_file() else ""
    configured = "[TWIST-UPDATE-CONFIG] mode=Periodic period_s=0.060" in text
    intervals, optimizer_ms = [], []
    count = recovery_writes = 0
    for line in text.splitlines():
        if "[TWIST-UPDATE]" not in line or "mode=Periodic type=periodic" not in line:
            continue
        count += 1
        if "written_to_recovery=1" in line:
            recovery_writes += 1
        for marker, target, scale in (("elapsed_since_update_s=", intervals, 1.0),
                                      ("optimizer_ns=", optimizer_ms, 1e-6)):
            start = line.find(marker)
            if start < 0:
                continue
            token = line[start + len(marker):].split()[0]
            try:
                target.append(float(token) * scale)
            except ValueError:
                pass
    return {"periodic_config_confirmed": configured, "periodic_update_count": count,
            "periodic_recovery_writes": recovery_writes,
            "periodic_interval_median_s": median(intervals),
            "periodic_interval_p95_s": q(intervals, .95),
            "periodic_interval_min_s": min(intervals) if intervals else None,
            "periodic_interval_max_s": max(intervals) if intervals else None,
            "periodic_optimizer_median_ms": median(optimizer_ms),
            "periodic_optimizer_p95_ms": q(optimizer_ms, .95)}


def run_row(item: dict) -> dict:
    attempts = item.get("attempts", [])
    attempt = attempts[-1] if attempts else {}
    row = {"sequence": item["sequence"], "round": item["round"], "plan_item_id": item["plan_item_id"],
           "map": item["map"], "scene_id": item["scene_id"], "setting": item["setting_id"],
           "group": item["group"], "planned_attempts": len(attempts), "plan_status": item["status"],
           "actual_scene_started": attempt.get("scene_started_confirmed"),
           "run_id": item.get("run_id") or attempt.get("run_id"),
           "terminal": item.get("terminal_result") or attempt.get("terminal_result"),
           "record_validity": attempt.get("record_validity"), "safe_completion": "UNKNOWN",
           "actual_entry_threshold": None, "actual_exit_threshold": None, "actual_hold": None,
           "actual_hold_s": None, "actual_ray_source": None, "actual_goal_x_m": None,
           "actual_goal_y_m": None, "actual_scene_filename": None,
           "actual_arrival_radius_m": None, "actual_sim_limit_s": None,
           "actual_wall_cap_s": None, "actual_standing_observation_s": None,
           "runtime_config_confirmed": None,
           "task_start_x_m": None, "task_start_y_m": None, "task_start_yaw_rad": None,
           "task_start_vx_mps": None, "task_start_vy_mps": None, "task_start_vz_mps": None,
           "task_start_planar_speed_mps": None,
           "safe_completion_reason": "task never reached a valid task window" if item["status"].startswith("STARTUP_FAILURE") else "",
           "start_distance_m": None, "end_distance_m": None, "net_goal_progress_m": None,
           "arrival_time_s": None, "recovery_share": None, "switch_count": None,
           "switch_count_coverage": None,
           "periodic_config_confirmed": None, "periodic_update_count": None,
           "periodic_recovery_writes": None, "periodic_interval_median_s": None,
           "periodic_interval_p95_s": None, "periodic_interval_min_s": None,
           "periodic_interval_max_s": None, "periodic_optimizer_median_ms": None,
           "periodic_optimizer_p95_ms": None,
           "startup_failure_reason": "; ".join(item.get("failure_reasons", attempt.get("failure_reasons", [])))}
    metrics_path = attempt.get("metrics_path")
    if metrics_path and Path(metrics_path).is_file():
        mpath = Path(metrics_path)
        metrics = read_json(mpath)
        safety = metrics.get("safety_task_completion") or {}
        status = safety.get("status", metrics.get("safety_task_completion_status", "UNKNOWN"))
        row["safe_completion"] = ("TRUE" if status is True else "FALSE" if status is False else "UNKNOWN")
        reasons = safety.get("reasons") or []
        row["safe_completion_reason"] = "; ".join(reasons) if reasons else (
            "; ".join(safety.get("record_validity", {}).get("reasons", [])) or "")
        row["record_validity"] = metrics.get("record_validity")
        row["terminal"] = metrics.get("navigation_terminal_result") or row["terminal"]
        context_path = mpath.parent / "run_context.json"
        context = read_json(context_path) if context_path.is_file() else {}
        settings = context.get("settings", {})
        first_active = context.get("initial_state", {}).get("first_active_policy_cycle", {})
        position = first_active.get("world_position_xy_m") or [None, None]
        velocity = first_active.get("body_linear_velocity_m_s") or [None, None, None]
        row["task_start_x_m"], row["task_start_y_m"] = position[:2]
        row["task_start_yaw_rad"] = first_active.get("world_yaw_rad")
        row["task_start_vx_mps"], row["task_start_vy_mps"], row["task_start_vz_mps"] = velocity[:3]
        if all(isinstance(v, (int, float)) for v in velocity[:2]):
            row["task_start_planar_speed_mps"] = math.hypot(velocity[0], velocity[1])
        row["actual_entry_threshold"] = settings.get("entry_threshold")
        row["actual_exit_threshold"] = settings.get("exit_threshold")
        row["actual_hold"] = settings.get("hold")
        row["actual_hold_s"] = settings.get("recovery_min_hold_s")
        row["actual_ray_source"] = settings.get("ray_source")
        row["actual_arrival_radius_m"] = settings.get("arrival_threshold_m")
        row["actual_sim_limit_s"] = settings.get("diagnostic_sim_limit_after_motion_s")
        row["actual_standing_observation_s"] = settings.get("standing_observation_sim_s")
        command = attempt.get("command", [])
        row["actual_wall_cap_s"] = (float(command[command.index("--wall-cap-s") + 1])
                                     if "--wall-cap-s" in command else None)
        row["actual_goal_x_m"] = settings.get("goal_x_m")
        row["actual_goal_y_m"] = settings.get("goal_y_m")
        goal = settings.get("goal_world_m") or [None, None]
        row["actual_goal_x_m"] = goal[0]
        row["actual_goal_y_m"] = goal[1]
        row["actual_scene_filename"] = settings.get("scene_filename")
        expected = {"A_REF": (-.05, -.05), "B_E04_X07": (-.04, -.07), "B_E05_X10": (-.05, -.10)}[row["setting"]]
        row["runtime_config_confirmed"] = (
            isinstance(row["actual_entry_threshold"], (int, float)) and
            isinstance(row["actual_exit_threshold"], (int, float)) and
            math.isclose(row["actual_entry_threshold"], expected[0], abs_tol=1e-6) and
            math.isclose(row["actual_exit_threshold"], expected[1], abs_tol=1e-6) and
            str(row["actual_hold"]).lower() == "false" and row["actual_hold_s"] == 0.0 and
            row["actual_ray_source"] == "geometric" and
            isinstance(row["actual_goal_x_m"], (int, float)) and
            isinstance(row["actual_goal_y_m"], (int, float)) and
            math.isclose(row["actual_goal_x_m"], 7.5, abs_tol=1e-6) and
            math.isclose(row["actual_goal_y_m"], 0.0, abs_tol=1e-6) and
            row["actual_arrival_radius_m"] == 0.5 and row["actual_sim_limit_s"] == 20.0 and
            row["actual_wall_cap_s"] == 120.0 and row["actual_standing_observation_s"] == 2.0 and
            row["actual_scene_filename"] == row["map"])
        start = metrics.get("target_distance_start_m")
        end = metrics.get("target_distance_final_m")
        row["start_distance_m"], row["end_distance_m"] = start, end
        if isinstance(start, (int, float)) and isinstance(end, (int, float)):
            row["net_goal_progress_m"] = start - end
        if row["terminal"] == "ARRIVED":
            row["arrival_time_s"] = metrics.get("elapsed_motion_duration_s")
        row["recovery_share"] = metrics.get("recovery_share_valid_motion_time")
        row["switch_count"] = metrics.get("mode_transitions")
        row["switch_count_coverage"] = metrics.get("mode_transition_count_coverage")
        facts = periodic_facts(mpath.parent)
        row.update(facts)
    return row


def main() -> None:
    plan = read_json(BATCH / "run_plan.json")
    rows = [run_row(item) for item in plan["schedule"]["items"]]
    fields = list(rows[0])
    with (OUT / "per_run_results.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)

    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for row in rows:
        groups[(row["map"], row["setting"])].append(row)
    summaries = []
    for map_name in MAPS:
        for setting in SETTINGS:
            cell = groups[(map_name, setting)]
            valid = [r for r in cell if r["record_validity"] == "VALID"]
            terminals = Counter(r["terminal"] for r in valid)
            safe = Counter(r["safe_completion"] for r in cell)
            progress = [r["net_goal_progress_m"] for r in valid if isinstance(r["net_goal_progress_m"], (int, float))]
            enddist = [r["end_distance_m"] for r in valid if isinstance(r["end_distance_m"], (int, float))]
            arrivals = [r for r in valid if r["terminal"] == "ARRIVED"]
            shares = [r["recovery_share"] for r in valid if isinstance(r["recovery_share"], (int, float))]
            switches = [r["switch_count"] for r in valid if isinstance(r["switch_count"], (int, float))]
            pcounts = [r["periodic_update_count"] for r in valid if isinstance(r["periodic_update_count"], int)]
            intervals = [r["periodic_interval_median_s"] for r in valid if isinstance(r["periodic_interval_median_s"], (int, float))]
            summaries.append({
                "map": map_name, "setting": setting, "planned_n": len(cell), "valid_task_n": len(valid),
                "startup_failure_n": sum(r["plan_status"].startswith("STARTUP_FAILURE") for r in cell),
                "safe_true_n": safe["TRUE"], "safe_false_n": safe["FALSE"], "safe_unknown_n": safe["UNKNOWN"],
                **{f"{term.lower()}_n": terminals[term] for term in TERMINALS},
                "net_goal_progress_mean_m": mean(progress), "net_goal_progress_median_m": median(progress),
                "end_distance_median": median(enddist), "arrival_time_n": len(arrivals),
                "arrival_time_median_s": median([r["arrival_time_s"] for r in arrivals
                                                   if isinstance(r["arrival_time_s"], (int, float))]),
                "recovery_share_mean": mean(shares), "recovery_share_median": median(shares),
                "switch_count_mean": mean(switches), "switch_count_median": median(switches),
                "periodic_config_confirmed_n": sum(r["periodic_config_confirmed"] is True for r in valid),
                "periodic_updates_total": sum(pcounts), "periodic_updates_per_run_median": median(pcounts),
                "periodic_interval_run_median_s": median(intervals),
                "periodic_written_to_recovery_n": sum(r["periodic_recovery_writes"] or 0 for r in valid),
            })
    summary_fields = list(summaries[0])
    with (OUT / "map_setting_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=summary_fields)
        writer.writeheader(); writer.writerows(summaries)

    contrast_rows = []
    for map_name in MAPS:
        cells = {s: next(r for r in summaries if r["map"] == map_name and r["setting"] == s) for s in SETTINGS}
        for setting_a, setting_b, label in (("A_REF", "B_E04_X07", "B_E04_X07-A_REF"),
                                             ("A_REF", "B_E05_X10", "B_E05_X10-A_REF"),
                                             ("B_E04_X07", "B_E05_X10", "B_E05_X10-B_E04_X07")):
            a, b = cells[setting_a], cells[setting_b]
            ra = a["arrived_n"] / a["valid_task_n"] if a["valid_task_n"] else None
            rb = b["arrived_n"] / b["valid_task_n"] if b["valid_task_n"] else None
            ci = difference_interval(b["arrived_n"], b["valid_task_n"], a["arrived_n"], a["valid_task_n"])
            contrast_rows.append({"map": map_name, "contrast": label,
                                  "A_valid_task_n": a["valid_task_n"], "B_valid_task_n": b["valid_task_n"],
                                  "A_arrivals": a["arrived_n"], "B_arrivals": b["arrived_n"],
                                  "arrival_rate_delta_B_minus_A_pp": (rb - ra) * 100 if ra is not None and rb is not None else None,
                                  "arrival_delta_Newcombe95_low_pp": ci[0] * 100 if ci else None,
                                  "arrival_delta_Newcombe95_high_pp": ci[1] * 100 if ci else None,
                                  "collision_rate_delta_B_minus_A_pp": (
                                      (b["collision_termination_n"] / b["valid_task_n"] -
                                       a["collision_termination_n"] / a["valid_task_n"]) * 100
                                      if a["valid_task_n"] and b["valid_task_n"] else None),
                                  "sim_timeout_rate_delta_B_minus_A_pp": (
                                      (b["diagnostic_sim_time_limit_n"] / b["valid_task_n"] -
                                       a["diagnostic_sim_time_limit_n"] / a["valid_task_n"]) * 100
                                      if a["valid_task_n"] and b["valid_task_n"] else None),
                                  "net_goal_progress_delta_B_minus_A_m": (
                                      b["net_goal_progress_mean_m"] - a["net_goal_progress_mean_m"]
                                      if a["net_goal_progress_mean_m"] is not None and b["net_goal_progress_mean_m"] is not None else None),
                                  "startup_failures_A_B": f"{a['startup_failure_n']}/{b['startup_failure_n']}"})
    with (OUT / "contrasts.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(contrast_rows[0]))
        writer.writeheader(); writer.writerows(contrast_rows)

    report = []
    report.append("# R1 A/B双阈值 Periodic 比较（45项）\n")
    report.append("## 主要发现\n")
    config_confirmed = sum(r["runtime_config_confirmed"] is True for r in rows if r["record_validity"] == "VALID")
    total_terminals = Counter(r["terminal"] for r in rows if r["record_validity"] == "VALID")
    safe_counts = Counter(r["safe_completion"] for r in rows)
    report.append(f"本批按5轮冻结交错排程运行45项，得到43项有效任务结果；序号001因排程设置名更新后overlay路径仍引用旧设置名，未启动场景；序号002启动MuJoCo后，DDS在沙箱内无法枚举网络接口，在任务开始前退出。两项均记录为启动故障，未重试或替代。其余43项地图、实际E/X、保持关闭、几何射线、目标、到达半径0.5m、20/120秒任务上限和2秒到达后观察均由运行上下文/命令核对（{config_confirmed}/43）；Periodic P=0.06另由controller日志核实。实际场景进程启动44次，无任务期系统安全中止或有效任务配置不符。有效任务终局共到达{total_terminals['ARRIVED']}、碰撞{total_terminals['COLLISION_TERMINATION']}、仿真限时{total_terminals['DIAGNOSTIC_SIM_TIME_LIMIT']}、跌倒{total_terminals['FALL_TERMINATION']}、墙钟保护{total_terminals['WALL_CLOCK_GUARD_INCOMPLETE']}、安全中止{total_terminals['SYSTEM_SAFETY_ABORT']}；安全完成TRUE/FALSE/UNKNOWN为{safe_counts['TRUE']}/{safe_counts['FALSE']}/{safe_counts['UNKNOWN']}，UNKNOWN均为两次未进入有效任务窗的启动失败。\n")
    report.append("逐图看，d09窄档的B_E04_X07在4项有效运行中4次到达且无碰撞，A为4/5到达、1次碰撞；但B_E04_X07有1项启动失败，且同时改变E与X。m02中档B_E04_X07到达2/5（A为1/4），碰撞3/5（A为3/4），是局部改善线索；B_E05_X10在m02到达1/5、碰撞4/5。challenge27中A到达2/5，两档B各到达1/5、仿真限时4/5。B_E05_X10没有在三图形成一致改善。每格计划5次、有效任务结果4或5次；所有地图都是开发图，本批不支持普遍优越或等效结论。\n")
    report.append("\n## 9格地图×设置汇总\n")
    table_cols = ["map", "setting", "valid_task_n", "startup_failure_n", "safe_true_n", "safe_false_n", "safe_unknown_n",
                  "arrived_n", "collision_termination_n", "fall_termination_n", "diagnostic_sim_time_limit_n",
                  "wall_clock_guard_incomplete_n", "system_safety_abort_n", "net_goal_progress_mean_m", "end_distance_median",
                  "arrival_time_n", "arrival_time_median_s", "recovery_share_median", "switch_count_median",
                  "periodic_updates_total", "periodic_interval_run_median_s"]
    report.append("| " + " | ".join(table_cols) + " |\n|" + "|".join(["---"] * len(table_cols)) + "|")
    for r in summaries:
        vals = [r.get(c) for c in table_cols]
        report.append("| " + " | ".join("UNKNOWN" if v is None else (f"{v:.3f}" if isinstance(v, float) else str(v)) for v in vals) + " |")
    report.append("\n## 同期差异\n")
    report.append("`contrasts.csv`列出两档B−A及B_E05_X10−B_E04_X07的到达率差、Newcombe–Wilson 95%区间、碰撞/限时率差和平均净推进差。到达率区间很宽并跨0。B_E04_X07在d09和m02有局部积极线索，但在challenge27到达更少且推进均值较低；由于E和X同时改变，不能单独归因于滞回。B_E05_X10与A共用E，更接近增加滞回的对照；它在d09到达率相同，在m02碰撞更多，在challenge27限时更多，没有支持其跨图改善的证据。现有结果不足以据此选择B进入新布局验证；若后续仍要验证，应先审阅典型策略帧并预先指定单一候选和布局。\n")
    report.append("\n## Periodic更新与记录范围\n")
    updates = [r for r in rows if r["record_validity"] == "VALID"]
    all_intervals = []
    all_optimizer = []
    all_update_count = 0
    all_recovery_writes = 0
    configs_confirmed = 0
    for r in updates:
        all_update_count += r["periodic_update_count"] or 0
        all_recovery_writes += r["periodic_recovery_writes"] or 0
        configs_confirmed += int(r["periodic_config_confirmed"] is True)
        if r["periodic_interval_median_s"] is not None:
            all_intervals.append(r["periodic_interval_median_s"])
        if r["periodic_optimizer_median_ms"] is not None:
            all_optimizer.append(r["periodic_optimizer_median_ms"])
    report.append(f"controller日志确认Periodic P=0.06的有效任务 {configs_confirmed}/{len(updates)}；记录到type=periodic更新共{all_update_count}次，其中写入Recovery {all_recovery_writes}次。逐次中位间隔的中位数为{median(all_intervals):.3f}s（样本{len(all_intervals)}项），优化耗时逐次中位数的中位数为{median(all_optimizer):.3f}ms。切换次数使用metrics中的`mode_transitions`，每次切换是否完整可见`mode_transition_count_coverage`字段；有效运行间的切换统计见逐次CSV，仅用于行为描述，不以切换减少判定有效性。实际Periodic间隔受控制器策略周期离散影响；这些是本批描述值。\n")
    report.append("\n## 限制\n")
    report.append("任务状态分别报告ARRIVED、COLLISION_TERMINATION、FALL_TERMINATION、仿真限时、墙钟保护和系统安全中止。安全完成UNKNOWN保留在汇总内，并在逐次CSV给出分类原因；启动失败因没有有效任务窗而为UNKNOWN。到达时间只对到达样本报告。逐次CSV记录了每个有效运行首个Active策略帧的位置、朝向与机身速度；初态未严格重放。各格样本少，不能据此证明参数因果或迁移到新布局。旧Frozen结果不并入本批。没有清理历史记录或追加运行。\n")
    (OUT / "ANALYSIS_REPORT.md").write_text("\n".join(report), encoding="utf-8")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch
        fig, ax = plt.subplots(figsize=(12, 5.5), constrained_layout=True)
        colors = {"ARRIVED": "#2e8b57", "COLLISION_TERMINATION": "#c44e52",
                  "FALL_TERMINATION": "#8172b3", "DIAGNOSTIC_SIM_TIME_LIMIT": "#dd9b3d",
                  "WALL_CLOCK_GUARD_INCOMPLETE": "#937860", "STARTUP_FAILURE": "#777777"}
        labels = {"ARRIVED": "Arrival", "COLLISION_TERMINATION": "Collision",
                  "FALL_TERMINATION": "Fall", "DIAGNOSTIC_SIM_TIME_LIMIT": "Sim timeout",
                  "WALL_CLOCK_GUARD_INCOMPLETE": "Wall timeout", "STARTUP_FAILURE": "Startup failure"}
        ylabels = []
        for i, s in enumerate(SETTINGS):
            for j, m in enumerate(MAPS):
                cell = groups[(m, s)]
                y = i * 3 + j
                ylabels.append(f"{m.replace('.xml','')} | {s}")
                count = Counter(("STARTUP_FAILURE" if r["plan_status"].startswith("STARTUP_FAILURE") else r["terminal"])
                                for r in cell)
                left = 0
                for cat in ("ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
                            "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE", "STARTUP_FAILURE"):
                    n = count[cat]
                    if n:
                        ax.barh(y, n, left=left, height=.65, color=colors[cat])
                        left += n
        ax.set_yticks(range(9), ylabels)
        ax.invert_yaxis(); ax.set_xlim(0, 5); ax.set_xlabel("Planned items (5 per map × setting cell)")
        ax.set_title("R1 A/B Periodic outcomes: 43 valid task results, 2 startup failures")
        present = [cat for cat in colors if any(
            ("STARTUP_FAILURE" if r["plan_status"].startswith("STARTUP_FAILURE") else r["terminal"]) == cat
            for r in rows)]
        ax.legend([Patch(facecolor=colors[cat]) for cat in present],
                  [labels[cat] for cat in present], ncol=3, loc="lower right")
        ax.grid(axis="x", alpha=.2)
        fig.savefig(OUT / "outcomes_by_map_setting.png", dpi=170)
        plt.close(fig)
    except ImportError:
        pass
    print(f"wrote {OUT / 'ANALYSIS_REPORT.md'}; {len(rows)} scheduled items, {len(updates)} valid task results")


if __name__ == "__main__":
    main()
