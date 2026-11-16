#!/usr/bin/env python3
"""Analyze the frozen single-layout four-setting R1 x05 batch."""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import analyze_r1_ac_hold_confirm_120 as common  # noqa: E402

MAP = "r1_ch15_left_x05_v2.xml"
MAPS = (MAP,)
LEVELS = (("A", None), ("C", 0.02), ("C", 0.10), ("C", 0.12))
LEVEL_KEYS = ["A", "C_H0.02", "C_H0.10", "C_H0.12"]
LEVEL_NUM = {key: index for index, key in enumerate(LEVEL_KEYS)}
common.LEVELS = LEVELS
common.LEVEL_KEYS = LEVEL_KEYS
common.LEVEL_NUM = LEVEL_NUM
common.MAPS = MAPS
common.LAYERS = (("local_development_variant", 1),)
common.BOOTSTRAP_SEED = 2026110202


def poses(run_dir: Path) -> list[tuple[float, float]]:
    out = []
    path = run_dir / "runtime_record.jsonl"
    if not path.is_file():
        return out
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            frame = json.loads(line)
        except json.JSONDecodeError:
            continue
        point = ((frame.get("payload") or {}).get("world_pose"))
        if isinstance(point, (list, tuple)) and len(point) >= 2:
            out.append((float(point[0]), float(point[1])))
    return out


def route_side(points: list[tuple[float, float]]) -> str:
    ys = [y for x, y in points if x >= 2.5]
    if not ys:
        return "UNKNOWN"
    y = statistics.mean(ys)
    return "left (+Y)" if y > 0.15 else "right (-Y)" if y < -0.15 else "center/unclear"


def obstacles() -> list[tuple[str, float, float, float]]:
    path = ROOT / "unitree_mujoco/unitree_robots/go2" / MAP
    root = ET.parse(path).getroot()
    out = []
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if name.startswith("ppt_obstacle_"):
            pos = [float(v) for v in geom.get("pos", "0 0 0").split()]
            size = [float(v) for v in geom.get("size", "0 0 0").split()]
            out.append((name, pos[0], pos[1], size[0]))
    return out


def wilson(k: int, n: int) -> tuple[float, float]:
    z = 1.959963984540054
    p = k / n
    den = 1 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return mid - half, mid + half


def newcombe(c: int, nc: int, a: int, na: int) -> tuple[float, float]:
    pc, pa = c / nc, a / na
    lc, uc = wilson(c, nc)
    la, ua = wilson(a, na)
    delta = pc - pa
    return (delta - math.sqrt((pc - lc) ** 2 + (ua - pa) ** 2),
            delta + math.sqrt((uc - pc) ** 2 + (pa - la) ** 2))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    plan = common.load_json(args.plan)
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows, episodes, collisions = [], [], []
    starts, route_rows, representative = [], [], {}
    for item in plan["items"]:
        attempts = item.get("attempts", [])
        if not attempts:
            continue
        attempt = attempts[-1]
        row, _, collision = common.extract_row(item, attempt)
        rows.append(row)
        if item["group"] == "C":
            item_episodes, _, _ = common.hold_episodes(item, attempt)
            episodes.extend(item_episodes)
        if collision:
            collisions.append({"sequence": item["sequence"], "level": row["level"], **collision})
        run_dir = Path(attempt["run_directory"])
        side = route_side(poses(run_dir))
        route_rows.append({"sequence": item["sequence"], "level": row["level"],
                           "terminal": row["terminal"], "route_side": side})
        if row["level"] not in representative:
            representative[row["level"]] = (item, attempt, run_dir)
        starts.append({"sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
            "repeat": item["repeat"], "position": item["position"], "map": item["map"],
            "level": row["level"], "attempt_number": attempt.get("attempt_number"),
            "run_id": attempt.get("run_id"), "status": attempt.get("status"),
            "terminal_result": attempt.get("terminal_result"), "run_directory": attempt.get("run_directory"),
            "started_at": attempt.get("started_at"), "finished_at": attempt.get("finished_at"),
            "mujoco_started": attempt.get("mujoco_started"),
            "process_shutdown_complete": attempt.get("process_shutdown_complete"),
            "closeout_complete": attempt.get("closeout_complete")})
    if not rows:
        raise RuntimeError("no attempted scenes to analyze")

    common.write_csv(out / "per_run_results.csv", rows)
    common.write_csv(out / "actual_schedule.csv", [{k: item.get(k) for k in
        ("sequence", "repeat", "block_index", "position", "map", "scene_id", "group", "hold_s", "plan_item_id", "status")}
        for item in plan["items"]])
    common.write_csv(out / "actual_scene_starts.csv", starts)
    common.write_csv(out / "route_side_all_runs.csv", route_rows)
    cells = common.build_map_table(rows, plan["items"])
    common.write_csv(out / "map_setting_4.csv", cells)
    summary = common.build_level_summary(rows, planned_maps=plan["mapset"]["maps"])
    common.write_csv(out / "setting_summary_4.csv", summary)
    common.write_csv(out / "hold_episodes.csv", episodes)
    hold_rows = common.hold_summary(episodes, rows)
    common.write_csv(out / "hold_audit_summary.csv", hold_rows)
    common.write_csv(out / "collision_context.csv", collisions)
    residence_rows = common.recovery_residence(plan, out)

    # One setting-rate comparison uses the same two independent binomial samples
    # and Newcombe-Wilson interval as the existing R1 analysis.
    setting_deltas = []
    ref = next(r for r in summary if r["level"] == "A")
    for result in summary:
        if result["level"] == "A":
            setting_deltas.append({"level": "A", "reference": True})
            continue
        low, high = newcombe(result["ARRIVED"], result["valid"], ref["ARRIVED"], ref["valid"])
        clo, chi = newcombe(result["collision_count"], result["valid"],
                            ref["collision_count"], ref["valid"])
        setting_deltas.append({"level": result["level"], "A_arrived": ref["ARRIVED"],
            "A_valid": ref["valid"], "C_arrived": result["ARRIVED"], "C_valid": result["valid"],
            "arrival_rate_difference": result["arrival_rate"] - ref["arrival_rate"],
            "arrival_rate_difference_ci95_low": low, "arrival_rate_difference_ci95_high": high,
            "A_collisions": ref["collision_count"], "C_collisions": result["collision_count"],
            "collision_rate_difference": result["collision_rate_known"] - ref["collision_rate_known"],
            "collision_rate_difference_ci95_low": clo,
            "collision_rate_difference_ci95_high": chi})
    common.write_csv(out / "setting_deltas_vs_A.csv", setting_deltas)

    obs = obstacles()
    route_summary = []
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 5.5))
    colors = {"A": "#2474b5", "C_H0.02": "#d65f2e", "C_H0.10": "#19966d", "C_H0.12": "#c58b00"}
    for name, x, y, radius in obs:
        ax.add_patch(plt.Circle((x, y), radius, color="#b3b0a7", alpha=0.4, linewidth=0))
    for level in LEVEL_KEYS:
        item, attempt, run_dir = representative[level]
        points = poses(run_dir)
        color = colors[level]
        if points:
            ax.plot([p[0] for p in points], [p[1] for p in points], color=color,
                    linewidth=1.5, label=f"{level} first scheduled (seq {item['sequence']})")
        distances = {name: min((math.hypot(px-x, py-y) for px, py in points), default=None)
                     for name, x, y, _ in obs}
        metrics = common.load_json(run_dir / "metrics.json")
        safety = metrics.get("task_window_safety") or {}
        events = safety.get("collision_events") or []
        route_summary.append({"level": level, "sequence": item["sequence"],
            "terminal": attempt.get("terminal_result"), "route_side": route_side(points),
            "collision_object": events[0].get("obstacle_geom_name") if events else None,
            "closest_base_link_to_01_center_m": distances.get("ppt_obstacle_01"),
            "closest_base_link_to_09_center_m": distances.get("ppt_obstacle_09"),
            "closest_base_link_to_02_center_m": distances.get("ppt_obstacle_02"),
            "closest_base_link_to_03_center_m": distances.get("ppt_obstacle_03")})
    ax.plot([0, 7.5], [0, 0], color="#666666", linestyle="--", linewidth=0.8)
    ax.scatter([0, 7.5], [0, 0], c=["black", "green"], s=20, zorder=5)
    ax.set(xlim=(-0.5, 8.0), ylim=(-3, 3), xlabel="x (m)", ylabel="y (m)",
           title="x05 base-link paths: first scheduled run per setting")
    ax.grid(alpha=0.2); ax.legend(fontsize=8); fig.tight_layout()
    chart_dir = out / "charts"
    chart_dir.mkdir(exist_ok=True)
    fig.savefig(chart_dir / "representative_trajectories.png", dpi=150)
    plt.close(fig)
    common.write_csv(out / "representative_route_summary.csv", route_summary)

    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    for i, (level, color) in enumerate(colors.items()):
        s = next(r for r in summary if r["level"] == level)
        lo, hi = wilson(s["ARRIVED"], s["valid"])
        p = s["arrival_rate"]
        ax.errorbar(i, p * 100, yerr=[[max(0, p-lo)*100], [max(0, hi-p)*100]],
                    marker="o", capsize=4, color=color, label=level)
    ax.set_xticks(range(4), LEVEL_KEYS); ax.set_ylim(0, 100)
    ax.set_ylabel("arrivals / valid runs (%)")
    ax.set_title("x05 arrivals by hold setting (Wilson 95% intervals)")
    ax.grid(axis="y", alpha=0.25); fig.tight_layout()
    fig.savefig(chart_dir / "arrivals_by_setting.png", dpi=150)
    plt.close(fig)

    # Summarize the collision terminal's last strategy state from existing frame-derived context.
    mode_counts = Counter(c.get("last_policy_frame_state") or "UNKNOWN" for c in collisions)
    collision_objects = Counter(c.get("obstacle") or "UNKNOWN" for c in collisions)
    route_counts = Counter(r["route_side"] for r in route_rows)
    lines = ["# R1：challenge15图05 A/C保持时间比较（40项）", "",
        f"- 计划/实跑/有效记录：40/ {len(rows)}/ {sum(r['record_validity']=='VALID' for r in rows)}；场景启动：{plan['counters']['confirmed_scene_starts']}；补做：{plan['counters'].get('engineering_repairs',0)}；系统安全中止：{sum(r['terminal']=='SYSTEM_SAFETY_ABORT' for r in rows)}；未启动：{sum(not i.get('attempts') for i in plan['items'])}；批次状态：{plan.get('status')}。",
        "- 仅使用已安装登记的`r1_ch15_left_x05_v2.xml`（03 x=6.362843,y=0.305638m；11号圆柱；09中心(3.683290,1.473857)m；11个障碍，无长墙）。同期组别为A H=0和C H=0.02/0.10/0.12，各10次；E=−0.05、实际X由E导出、滞回关闭。目标、到达半径及仿真/墙钟/到达后观察条件均沿用固定设置。",
        "", "## 四设置汇总", "",
        "|设置|有效/计划|安全完成T/F/U|到达|碰撞终止|碰撞记录数|跌倒终止|仿真限时|墙钟保护|安全中止|平均推进(m)|平均终点距(m)|终局耗时均值(n)|成功耗时均值(n)|Recovery占比|平均切换数|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for s in summary:
        fmt = lambda v: "UNKNOWN" if v is None else f"{v:.3f}"
        n_arr = sum(r["terminal"] == "ARRIVED" and r["level"] == s["level"] for r in rows)
        durations = [r["task_duration_s"] for r in rows if r["level"] == s["level"] and r["task_duration_s"] is not None]
        terminal_duration = statistics.mean(durations) if durations else None
        lines.append(f"|{s['level']}|{s['valid']}/{s['planned']}|{s['safe_completion_true']}/{s['safe_completion_false']}/{s['safe_completion_unknown']}|{s['ARRIVED']}|{s['COLLISION_TERMINATION']}|{s['collision_count']}|{s['FALL_TERMINATION']}|{s['DIAGNOSTIC_SIM_TIME_LIMIT']}|{s['WALL_CLOCK_GUARD_INCOMPLETE']}|{s['SYSTEM_SAFETY_ABORT']}|{fmt(s['task_progress_mean_m'])}|{fmt(s['arrival_distance_mean_m'])}|{fmt(terminal_duration)} (n={len(durations)})|{fmt(s['arrival_time_mean_s'])} (n={n_arr})|{fmt(s['recovery_share_mean'])}|{fmt(s['total_switches_mean'])}|")
    lines.extend(["", "## C相对同期A", "",
        "率差为独立二项样本的Newcombe–Wilson 95%区间。每设置10次，区间宽；跨0不代表等效，点估计高也不代表优越。",
        "", "|设置|到达/有效|C−A到达率差|Newcombe–Wilson 95%区间|",
        "|---|---:|---:|---:|"])
    for d in setting_deltas[1:]:
        pp = lambda v: f"{100*v:.1f}%"
        lines.append(f"|{d['level']}|{d['C_arrived']}/{d['C_valid']}|{pp(d['arrival_rate_difference'])}|{pp(d['arrival_rate_difference_ci95_low'])}至{pp(d['arrival_rate_difference_ci95_high'])}|")
    lines.extend(["", "碰撞终止率也以每档10次有效记录计算，区间同为Newcombe–Wilson 95%：",
        "", "|设置|碰撞/有效|C−A碰撞率差|Newcombe–Wilson 95%区间|",
        "|---|---:|---:|---:|"])
    for d in setting_deltas[1:]:
        pp = lambda v: f"{100*v:.1f}%"
        lines.append(f"|{d['level']}|{d['C_collisions']}/{d['C_valid']}|{pp(d['collision_rate_difference'])}|{pp(d['collision_rate_difference_ci95_low'])}至{pp(d['collision_rate_difference_ci95_high'])}|")
    all_residence = {r["level"]: r for r in residence_rows if r["layer"] == "all"}
    lines.extend(["", "## 任务耗时与Recovery驻留", "",
        "终局耗时对全部有效场景计算；成功耗时只对到达子集计算。两者样本数不同，成功耗时不代表全任务效率。Recovery驻留由已有策略帧重建，episode不足以将这些时长解释为完整物理状态重放。",
        "", "|设置|H审计episode（观察/退出/删失）|平均Recovery驻留(s)|中位数(s)|P90(s)|",
        "|---|---:|---:|---:|---:|"])
    for key in LEVEL_KEYS:
        h = next(r for r in hold_rows if r["level"] == key)
        residence = all_residence[key]
        lines.append(f"|{key}|{h['episodes']} / {h['exits_observed']} / {h['censored']}|{residence['mean_duration_s']:.3f}|{residence['median_duration_s']:.3f}|{residence['p90_duration_s']:.3f}|")
    lines.extend(["", "## 与上一批图05分开对照", "",
        "上一批同一图05：A到达0/10、C H=0.02到达2/10，碰撞均记录为障碍03。该批数据不进入本批分母。本批A到达3/10、H=0.02到达1/10；旧批的正向差异没有重复。排程、初态并非严格物理重放。",
        "", "## 保持行为、碰撞与路线", "",
        f"- 按x≥2.5m后的base_link中心y均值（±0.15m界限）粗分路线：{dict(route_counts)}。代表轨迹取排程中每种设置的首个场景，见`charts/representative_trajectories.png`；最近障碍中心距离见`representative_route_summary.csv`，只是中心位姿距离，不是整机净空证明。",
        f"- 碰撞记录的障碍对象计数：{dict(collision_objects)}；碰撞前最近策略帧模式计数：{dict(mode_counts)}。策略帧与碰撞时间接近不证明因果。详见`collision_context.csv`。",
        "- H及驻留审计见`hold_audit_summary.csv`、`recovery_residence.csv`和`hold_episodes.csv`。H从Recovery入口计时；H到期仍需满足RA退出条件。H=0.02有4/59个episode在H到期前RA已低于E并随后退出，未见提前退出违规；对应实际驻留均值/中位数见`recovery_residence.csv`。较长H降低切换频率，但可增加Recovery驻留；这些行为仅作解释，不单独判断收益。",
        "- 启动前曾因先暂存overlay、后冻结排程而遇到非空根目录拒绝启动；该次未启动场景，随后冻结原40项排程后继续，计入`run_plan.json`的编排事件，不是工程补做。运行器进度文本仍打印`/120`，但冻结计划、40次确认启动及40条结果均为本批40项。",
        "- 每次运行的终点距离、推进、耗时、有效性、安全T/F/U和实际配置见`per_run_results.csv`；到达耗时只基于到达样本，不代表整体效率。保持UNKNOWN如实保留。",
        "", "## 结论范围", "",
        "图05本批中，H=0.10和0.12各到达2/10，比H=0.02的1/10多一次；但同期A为3/10，且C各档碰撞均比A多1至2次。率差区间很宽并跨0，不能据此认为较长H已提高完成机会，也不能说与A等效。相对H=0.02，较长H的点估计略好、切换较少；0.12的平均驻留更长，属于可能的代价。上一批H=0.02的2/10到达没有重现（本批1/10，A从0/10变为3/10）。目前仍无法区分代表H，也不据本图选优。结论仅限这张已选开发图，不代表其他地图。"])
    (out / "RESULTS_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
