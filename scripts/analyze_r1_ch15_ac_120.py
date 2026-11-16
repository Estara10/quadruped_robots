#!/usr/bin/env python3
"""Analyze the frozen six-layout challenge15 A/C batch using R1 production metrics."""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import analyze_r1_ac_hold_confirm_120 as common  # noqa: E402

MAPS = tuple(f"r1_ch15_left_x{i:02d}_v2.xml" for i in range(1, 7))
common.LEVELS = (("A", None), ("C", 0.02))
common.LEVEL_KEYS = ["A", "C_H0.02"]
common.LEVEL_NUM = {key: i for i, key in enumerate(common.LEVEL_KEYS)}
common.LAYERS = (("local_development_variant", 1),)
common.BOOTSTRAP_SEED = 2026110201


def poses(run_dir: Path) -> list[tuple[float, float]]:
    points = []
    path = run_dir / "runtime_record.jsonl"
    if not path.is_file():
        return points
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        payload = row.get("payload") or {}
        # Production runtime frames store planar pose as [x, y, yaw].
        xy = payload.get("world_pose")
        if isinstance(xy, (tuple, list)) and len(xy) >= 2:
            try:
                points.append((float(xy[0]), float(xy[1])))
            except (TypeError, ValueError):
                pass
    return points


def obstacles(filename: str) -> list[tuple[str, float, float, float]]:
    path = ROOT / "unitree_mujoco/unitree_robots/go2" / filename
    root = ET.parse(path).getroot()
    out = []
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if not name.startswith("ppt_obstacle_"):
            continue
        pos = [float(v) for v in geom.get("pos", "0 0 0").split()]
        size = [float(v) for v in geom.get("size", "0 0 0").split()]
        out.append((name, pos[0], pos[1], size[0]))
    return out


def route_side(path_points: list[tuple[float, float]]) -> str:
    ys = [y for x, y in path_points if x >= 2.5]
    if not ys:
        return "UNKNOWN"
    mean_y = sum(ys) / len(ys)
    return "left (+Y)" if mean_y > 0.15 else "right (-Y)" if mean_y < -0.15 else "center/unclear"


def route_summary(plan: dict, rows: list[dict], out: Path) -> tuple[list[dict], Counter]:
    # Deterministic illustrative paths: first scheduled A and C run for each map.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    selected = {}
    for item in plan["items"]:
        key = (item["map"], item["group"])
        if item.get("attempts") and key not in selected:
            selected[key] = (item, item["attempts"][-1])
    figures, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True, sharey=True)
    axes = list(axes.flat)
    summaries = []
    side_rows = []
    side_counts = Counter()
    for item in plan["items"]:
        if not item.get("attempts"):
            continue
        attempt = item["attempts"][-1]
        path_points = poses(Path(attempt["run_directory"]))
        side = route_side(path_points)
        side_counts[(item["group"], side)] += 1
        side_rows.append({"sequence": item["sequence"], "map": item["map"],
            "group": item["group"], "terminal": attempt.get("terminal_result"),
            "route_side_center_y": side, "base_link_path_points": len(path_points)})
    common.write_csv(out / "route_side_all_runs.csv", side_rows)
    colors = {"A": "#2474b5", "C": "#d65f2e"}
    for axis, filename in zip(axes, MAPS):
        obs = obstacles(filename)
        for name, x, y, radius in obs:
            axis.add_patch(plt.Circle((x, y), radius, color="#b3b0a7", alpha=0.42, linewidth=0))
        for group in ("A", "C"):
            found = selected.get((filename, group))
            if not found:
                continue
            item, attempt = found
            run_dir = Path(attempt["run_directory"])
            path_points = poses(run_dir)
            if path_points:
                axis.plot([p[0] for p in path_points], [p[1] for p in path_points],
                          color=colors[group], linewidth=1.25,
                          label=f"{group} first scheduled (seq {item['sequence']})")
            metrics = common.load_json(run_dir / "metrics.json")
            safety = metrics.get("task_window_safety") or {}
            side = route_side(path_points)
            nearest = []
            for name, x, y, radius in obs:
                distances = [math.hypot(px - x, py - y) for px, py in path_points]
                nearest.append((name, min(distances) if distances else None, radius))
            summaries.append({"map": filename, "group": group, "sequence": item["sequence"],
                "terminal": attempt.get("terminal_result"), "route_side_center_y": side,
                "collision_geom": ((safety.get("collision_events") or [{}])[0].get("obstacle_geom_name")),
                "closest_base_link_to_01_center_m": next((d for n, d, _ in nearest if n.endswith("_01")), None),
                "closest_base_link_to_09_center_m": next((d for n, d, _ in nearest if n.endswith("_09")), None),
                "closest_base_link_to_02_center_m": next((d for n, d, _ in nearest if n.endswith("_02")), None),
                "closest_base_link_to_03_center_m": next((d for n, d, _ in nearest if n.endswith("_03")), None),
                "closest_base_link_to_11_center_m": next((d for n, d, _ in nearest if n.endswith("_11")), None),
                "obstacle_center_distances_m": json.dumps({n: d for n, d, _ in nearest}, ensure_ascii=False),
                "base_link_path_points": len(path_points)})
        axis.plot([0, 7.5], [0, 0], color="#666666", linewidth=0.7, linestyle="--", alpha=0.5)
        axis.scatter([0, 7.5], [0, 0], c=["black", "green"], s=16, zorder=5)
        axis.set_xlim(-0.5, 8.0)
        axis.set_ylim(-3, 3)
        axis.set_title(f"{filename.split('_x')[-1][:2]}  |  03 x={5.862843 + (int(filename.split('_x')[-1][:2])-1)*0.1 + (0.1 if int(filename.split('_x')[-1][:2]) >= 4 else 0):.3f} m", fontsize=8)
        axis.grid(alpha=0.18)
    axes[0].legend(fontsize=7, loc="upper left")
    figures.supxlabel("x (m)")
    figures.supylabel("y (m)")
    figures.suptitle("Base-link trajectories: first scheduled A and C run per layout; illustrative only")
    figures.tight_layout()
    chart_dir = out / "charts"
    chart_dir.mkdir(exist_ok=True)
    figures.savefig(chart_dir / "representative_center_trajectories.png", dpi=150)
    plt.close(figures)
    common.write_csv(out / "representative_route_summary.csv", summaries)
    return summaries, side_counts


def plot_position(out: Path, cells: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    x = [-0.30, -0.20, -0.10, 0.10, 0.20, 0.30]
    fig, ax = plt.subplots(figsize=(8.5, 4.5))
    for group, level, color in (("A", "A", "#2474b5"), ("C", "C_H0.02", "#d65f2e")):
        ys = []
        lows = []
        highs = []
        for filename in MAPS:
            row = next(r for r in cells if r["map"] == filename and r["level"] == level)
            n, k = row["valid"], row["ARRIVED"]
            ys.append((k / n) if n else math.nan)
            if n:
                d = common.bootstrap([{"map": filename, "level": level, "terminal": "ARRIVED",
                                       "record_validity": "VALID"}], []) if False else None
                z = 1.959963984540054
                p = k / n
                den = 1 + z*z/n
                mid = (p + z*z/(2*n))/den
                half = z*math.sqrt(p*(1-p)/n + z*z/(4*n*n))/den
                lows.append((mid-half))
                highs.append((mid+half))
            else:
                lows.append(math.nan); highs.append(math.nan)
        ax.plot(x, [v*100 for v in ys], marker="o", color=color, label=group)
        ax.fill_between(x, [v*100 for v in lows], [v*100 for v in highs], color=color, alpha=0.16)
    ax.set_xlabel("challenge15 obstacle 03 x offset from original (m)")
    ax.set_ylabel("arrivals / valid runs (%)")
    ax.set_xticks(x)
    ax.grid(alpha=0.25)
    ax.legend(title="setting")
    ax.set_title("Task outcome by 03 position (Wilson 95% intervals per layout)")
    fig.tight_layout()
    (out / "charts").mkdir(exist_ok=True)
    fig.savefig(out / "charts" / "position_vs_arrival.png", dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    plan = common.load_json(args.plan)
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows, episodes, collisions = [], [], []
    for item in plan["items"]:
        if not item.get("attempts"):
            continue
        attempt = item["attempts"][-1]
        row, _, collision = common.extract_row(item, attempt)
        rows.append(row)
        if item["group"] == "C":
            item_episodes, _, _ = common.hold_episodes(item, attempt)
            episodes.extend(item_episodes)
        if collision:
            collisions.append({"sequence": item["sequence"], "map": item["map"],
                               "level": row["level"], **collision})
    if not rows:
        raise RuntimeError("no attempted scenes to analyze")
    common.write_csv(out / "per_run_results.csv", rows)
    common.write_csv(out / "actual_schedule.csv", [{k: item.get(k) for k in
        ("sequence", "repeat", "block_index", "position", "map", "scene_id", "group", "hold_s", "plan_item_id", "status")}
        for item in plan["items"]])
    starts = []
    for item in plan["items"]:
        for attempt in item.get("attempts", []):
            starts.append({"sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
                "map": item["map"], "level": common.level_key(item["group"], item["hold_s"]),
                "attempt_number": attempt.get("attempt_number"), "run_id": attempt.get("run_id"),
                "status": attempt.get("status"), "terminal_result": attempt.get("terminal_result"),
                "run_directory": attempt.get("run_directory"), "started_at": attempt.get("started_at"),
                "finished_at": attempt.get("finished_at"), "mujoco_started": attempt.get("mujoco_started"),
                "process_shutdown_complete": attempt.get("process_shutdown_complete"),
                "closeout_complete": attempt.get("closeout_complete")})
    common.write_csv(out / "actual_scene_starts.csv", starts)
    cells = common.build_map_table(rows, plan["items"])
    common.write_csv(out / "map_setting_12.csv", cells)
    summary = common.build_level_summary(rows, planned_maps=plan["mapset"]["maps"])
    common.write_csv(out / "setting_summary_2.csv", summary)
    deltas = common.bootstrap(rows, cells)
    common.write_csv(out / "setting_deltas_vs_A.csv", [
        {"level": "A", "reference": True}, deltas["C_H0.02"]])
    common.write_csv(out / "map_deltas_vs_A.csv", common.map_deltas(rows))
    common.write_csv(out / "collision_context.csv", collisions)
    common.write_csv(out / "hold_episodes.csv", episodes)
    hold_rows = common.hold_summary(episodes, rows)
    common.write_csv(out / "hold_audit_summary.csv", hold_rows)
    routes, route_sides = route_summary(plan, rows, out)
    plot_position(out, cells)
    collision_objects = Counter(c.get("obstacle", "UNKNOWN") or "UNKNOWN" for c in collisions)

    terminals = Counter(r["terminal"] for r in rows if r["record_validity"] == "VALID")
    delta = deltas["C_H0.02"]
    summary_md = ["# R1：challenge15六张同源布局A/C比较（H=0.02，120项）", "",
        f"- 计划项：120；有实际运行记录：{len(rows)}；有效记录：{sum(r['record_validity']=='VALID' for r in rows)}；确认场景启动：{plan['counters']['confirmed_scene_starts']}；工程补做：{plan['counters'].get('engineering_repairs',0)}；系统安全中止：{terminals['SYSTEM_SAFETY_ABORT']}；未启动：{sum(not i.get('attempts') for i in plan['items'])}；状态：{plan.get('status')}。",
        "- 六张图是challenge15模板的同源开发布局，不是独立验证图集；本批A/C同期运行，历史challenge15结果不进入本批分母。原地图、M1/M2/M3及旧结果未改。上一批原challenge15上A为2/8到达、C_H0.02为5/8；本批同源局部图合计A为13/60、C为14/60，旧图方向没有在总体率上重现，不能跨批合并或把地图改动差异归因于H。",
        "- 固定E=−0.05、有效X=E、关闭滞回；A保持关闭（H=0），C保持开启（H=0.02仿真秒）。目标(7.5,0)m、到达半径0.5m、仿真/墙钟上限20/120秒，到达后观察2仿真秒。初态按现有reset/起立流程记录，不代表严格物理状态重放。",
        "- 03的六个x偏移为−0.30、−0.20、−0.10、+0.10、+0.20、+0.30m；各图均有11个障碍，新增圆柱11、09外移0.20m，未含长墙。实际选择右绕不作无效处理。",
        "", "## 两设置汇总", "",
        "|设置|有效/计划|安全完成T/F/U|到达|碰撞终止|安全窗碰撞|跌倒终止|仿真限时|墙钟保护|安全中止|平均推进(m)|平均终点距(m)|成功耗时均值(n)|Recovery占比|平均切换数|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for r in summary:
        n_arr = sum(x["level"] == r["level"] and x["terminal"] == "ARRIVED" for x in rows)
        fmt = lambda v: "UNKNOWN" if v is None else f"{v:.3f}"
        summary_md.append(f"|{r['level']}|{r['valid']}/{r['planned']}|{r['safe_completion_true']}/{r['safe_completion_false']}/{r['safe_completion_unknown']}|{r['ARRIVED']}|{r['COLLISION_TERMINATION']}|{r['collision_count']}|{r['FALL_TERMINATION']}|{r['DIAGNOSTIC_SIM_TIME_LIMIT']}|{r['WALL_CLOCK_GUARD_INCOMPLETE']}|{r['SYSTEM_SAFETY_ABORT']}|{fmt(r['task_progress_mean_m'])}|{fmt(r['arrival_distance_mean_m'])}|{fmt(r['arrival_time_mean_s'])} (n={n_arr})|{fmt(r['recovery_share_mean'])}|{fmt(r['total_switches_mean'])}|")
    fmt_pp = lambda v: "UNKNOWN" if v is None else f"{100*v:.1f}%"
    summary_md.extend(["", "## C相对同期A", "",
        f"- 到达率差（C−A）：{fmt_pp(delta['arrival_rate_difference'])}；Newcombe–Wilson 95%区间：{fmt_pp(delta['arrival_rate_difference_ci95_low'])} 至 {fmt_pp(delta['arrival_rate_difference_ci95_high'])}。六图等权差：{fmt_pp(delta['map_equal_mean_difference'])}；按布局重抽样95%区间：{fmt_pp(delta['selected_map_bootstrap_ci95_low'])} 至 {fmt_pp(delta['selected_map_bootstrap_ci95_high'])}。区间跨越0时不据此认定等效或优越。",
        "", "## 每张布局的A/C结果", "",
        "|03偏移|A安全T/F/U|A到达|A碰撞终止/安全窗碰撞|A仿真限时/墙钟保护|C安全T/F/U|C到达|C碰撞终止/安全窗碰撞|C仿真限时/墙钟保护|C−A到达率差|",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"])
    offsets = [-0.30, -0.20, -0.10, 0.10, 0.20, 0.30]
    for filename, offset in zip(MAPS, offsets):
        a = next(r for r in cells if r["map"] == filename and r["level"] == "A")
        c = next(r for r in cells if r["map"] == filename and r["level"] == "C_H0.02")
        diff = (c["arrival_rate"] - a["arrival_rate"]) if c["arrival_rate"] is not None and a["arrival_rate"] is not None else None
        summary_md.append(f"|{offset:+.2f}m|{a['safe_completion_true']}/{a['safe_completion_false']}/{a['safe_completion_unknown']}|{a['ARRIVED']}/{a['valid']}|{a['COLLISION_TERMINATION']}/{a['collision_count']}|{a['DIAGNOSTIC_SIM_TIME_LIMIT']}/{a['WALL_CLOCK_GUARD_INCOMPLETE']}|{c['safe_completion_true']}/{c['safe_completion_false']}/{c['safe_completion_unknown']}|{c['ARRIVED']}/{c['valid']}|{c['COLLISION_TERMINATION']}/{c['collision_count']}|{c['DIAGNOSTIC_SIM_TIME_LIMIT']}/{c['WALL_CLOCK_GUARD_INCOMPLETE']}|{fmt_pp(diff)}|")
    c_hold = next((r for r in hold_rows if r["level"] == "C_H0.02"), {})
    summary_md.extend(["", "## 保持、路线与解释范围", "",
        f"- H=0.02审计：Recovery episode={c_hold.get('episodes',0)}；到期前RA已低于E的episode={c_hold.get('risk_clear_before_H_expiry',0)}，随后退出={c_hold.get('risk_clear_before_H_then_exit',0)}，H到期前退出={c_hold.get('early_exits_before_H',0)}。1804个可观测episode驻留中位数0.022s、均值0.040s、范围0.020–0.562s；16个episode被终局截断。C平均Recovery占比0.167、平均切换60.4次，A分别为0.161和71.0次。H到期后仍须满足RA退出条件，驻留可能长于H。",
        f"- 碰撞终局92次对应的碰撞对象为：障碍03 {collision_objects['ppt_obstacle_03']}、04 {collision_objects['ppt_obstacle_04']}、02 {collision_objects['ppt_obstacle_02']}、09 {collision_objects['ppt_obstacle_09']}次。按x≥2.5m后的base_link中心y均值粗分路线侧（阈值±0.15m），全批A左绕(+Y)/右绕(−Y)/中间或不清楚/UNKNOWN={route_sides[('A','left (+Y)')]}/{route_sides[('A','right (-Y)')]}/{route_sides[('A','center/unclear')]}/{route_sides[('A','UNKNOWN')]}；C={route_sides[('C','left (+Y)')]}/{route_sides[('C','right (-Y)')]}/{route_sides[('C','center/unclear')]}/{route_sides[('C','UNKNOWN')]}。代表轨迹取每张图排程中最先启动的A与C项，见`charts/representative_center_trajectories.png`；逐次粗分见`route_side_all_runs.csv`。这12条代表轨迹距01、09、02、03障碍中心最近距离范围分别为0.63–0.76、0.65–0.78、0.67–0.88、0.43–0.54m；均为中心点距离，不能证明整机净空。",
        "- 位置—到达对照见`charts/position_vs_arrival.png`；分母为该图该设置有效记录。逐次运行、12格表、两设置汇总、全部场景启动和清理清单均随报告保存。",
        "- 结果仅描述这六张围绕既有challenge15改造的同源开发图，不能宣称普遍泛化。跨批次challenge15结果只列作背景，不与本批合并；新增11、外移09和局部几何变化也使旧图成绩不能用于归因H。",
        "- 有效记录和安全完成状态分别报告；UNKNOWN保留。合法碰撞、跌倒、限时和墙钟保护均作为结果，不重跑。成功耗时只基于各设置成功子集，不代表全任务效率。切换与Recovery占比用于解释，不作为单独有效性判据。",
        "", "## 初步结论", "",
        "H=0.02确实约束了退出时机，但未形成稳定的任务收益：六图中三图到达率提高、两图下降、一图持平；总碰撞终止与A相同，且总体率差区间较宽并跨0。可将H=0.02保留为已评估的短保持候选，不足以指定为推荐或代表H。结果只适用于这六张同源开发图，不能支持独立泛化或普遍优势。"])
    (out / "RESULTS_SUMMARY.md").write_text("\n".join(summary_md) + "\n", encoding="utf-8")
    print(f"analysis written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
