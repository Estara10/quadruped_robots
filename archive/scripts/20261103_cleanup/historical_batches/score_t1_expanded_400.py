#!/usr/bin/env python3
"""Post-hoc scoring of the fixed 400-run T1 table; never opens raw run files."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import random
import statistics
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BATCH = ROOT / "logs/t1/b_exit_validation_400/20261016_131943_26"
WEIGHTS = {"arrival": .40, "safety": .30, "efficiency": .20, "stability": .10}
TIME_CAP = 20.0
F_REF = 257.8  # User-approved rounded A reference, not recomputed per candidate.
BOOTSTRAP_SEED = 20261017
LEVELS = [-.05, -.07, -.09, -.11, -.13, -.15, -.17, -.19]


def mean(values):
    return statistics.mean(values)


def numeric(row, field):
    value = float(row[field])
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{row['plan_item_id']}: invalid {field}")
    return value


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def interval(values):
    values = sorted(values)
    return values[int(.025*(len(values)-1))], values[int(.975*(len(values)-1))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=BATCH / "expanded_run_results.csv")
    parser.add_argument("--output-dir", type=Path, default=BATCH / "weighted_score_v1")
    args = parser.parse_args()
    source, out = args.source.resolve(), args.output_dir.resolve()
    source_digest = hashlib.sha256(source.read_bytes()).hexdigest()
    with source.open(encoding="utf-8-sig", newline="") as f:
        raw = list(csv.DictReader(f))
    if len(raw) != 400 or len({r["plan_item_id"] for r in raw}) != 400:
        raise ValueError("Expected exactly 400 distinct planned results")
    if len({r["run_id"] for r in raw}) != 400:
        raise ValueError("Duplicate run identity")
    maps = sorted({r["map"] for r in raw})
    cells = Counter((r["map"], float(r["exit_threshold_X"])) for r in raw)
    if len(maps) != 10 or len(cells) != 80 or set(cells.values()) != {5}:
        raise ValueError("Expected 10 maps x 8 thresholds x 5 repeats")
    if {x for _, x in cells} != set(LEVELS):
        raise ValueError("Unexpected threshold levels")
    scored = []
    for r in raw:
        if r["status"] != "VALID_TASK_RESULT" or r["record_validity"] != "VALID":
            raise ValueError(f"Not a valid task result: {r['plan_item_id']}")
        if r["motion_collision_result"] not in ("True", "False"):
            raise ValueError("Collision UNKNOWN: do not silently award safety points")
        if r["short_reverse_switch_coverage"] != "complete":
            raise ValueError("Incomplete switch frequency: do not silently score it")
        if abs(numeric(r, "task_valid_time_coverage_fraction")-1.0) > 1e-9:
            raise ValueError("Incomplete task time coverage")
        if numeric(r, "rl_step_gaps") or numeric(r, "invalid_clock_frames"):
            raise ValueError("Cycle or policy clock coverage is incomplete")
        arrived = r["terminal_result"] == "ARRIVED"
        elapsed = numeric(r, "elapsed_task_duration_s")
        f = numeric(r, "short_reverse_per_valid_min")
        duration = numeric(r, "valid_motion_duration_s")
        count = numeric(r, "short_reverse_switch_count")
        if duration <= 0 or not math.isclose(f, count/duration*60, rel_tol=1e-9, abs_tol=1e-8):
            raise ValueError("Short reversal frequency does not match count / valid task time")
        s = {"plan_item_id": r["plan_item_id"], "run_id": r["run_id"],
             "map": r["map"], "group": r["group"], "exit_threshold_X": float(r["exit_threshold_X"]),
             "repeat": int(r["repeat"]), "terminal_result": r["terminal_result"],
             "elapsed_task_duration_s": elapsed, "short_reverse_per_valid_min": f,
             "motion_collision_result": r["motion_collision_result"],
             "motion_fall_result": r["motion_fall_result"],
             "foot_impact_count": int(r["foot_impact_count"]),
             "standing_observation_status": r["standing_observation_status"],
             "arrival_score_0_1": float(arrived),
             "safety_score_0_1": float(r["motion_collision_result"] == "False"),
             "efficiency_score_0_1": max(0.0, 1.0-elapsed/TIME_CAP) if arrived else 0.0,
             "stability_score_0_1": 1.0/(1.0+f/F_REF),
             "arrival_over_time_cap": arrived and elapsed > TIME_CAP}
        for key, weight in WEIGHTS.items():
            s[key+"_points"] = 100*weight*s[key+"_score_0_1"]
        s["total_score_100"] = sum(s[key+"_points"] for key in WEIGHTS)
        scored.append(s)

    grouped = defaultdict(list)
    for s in scored:
        grouped[(s["map"], s["exit_threshold_X"])].append(s)
    map_rows = []
    for m in maps:
        for x in LEVELS:
            rr = grouped[(m,x)]
            row = {"map": m, "exit_threshold_X": x, "n": len(rr)}
            for key in WEIGHTS:
                row[key+"_score_0_1"] = mean(r[key+"_score_0_1"] for r in rr)
                row[key+"_points"] = mean(r[key+"_points"] for r in rr)
            row["total_score_100"] = sum(row[key+"_points"] for key in WEIGHTS)
            map_rows.append(row)
    by_cell = {(r["map"],r["exit_threshold_X"]):r for r in map_rows}
    rng = random.Random(BOOTSTRAP_SEED)
    draws = [[rng.randrange(len(maps)) for _ in maps] for _ in range(10000)]
    summary = []
    for x in LEVELS:
        rr = [r for r in scored if r["exit_threshold_X"] == x]
        mm = [by_cell[(m,x)] for m in maps]
        row = {"exit_threshold_X": x, "group": "A_reference" if x == -.05 else "B",
               "n": len(rr), "maps": len(mm),
               "arrived_n": sum(r["terminal_result"] == "ARRIVED" for r in rr),
               "collision_n": sum(r["motion_collision_result"] == "True" for r in rr),
               "fall_true_n": sum(r["motion_fall_result"] == "True" for r in rr),
               "fall_unknown_n": sum(r["motion_fall_result"] == "UNKNOWN" for r in rr),
               "timeout_n": sum(r["terminal_result"] == "DIAGNOSTIC_SIM_TIME_LIMIT" for r in rr),
               "arrived_with_foot_impact_n": sum(r["terminal_result"] == "ARRIVED" and r["foot_impact_count"] > 0 for r in rr),
               "standing_incomplete_n": sum(r["standing_observation_status"] == "INCOMPLETE" for r in rr)}
        for key in WEIGHTS:
            row[key+"_score_0_1"] = mean(r[key+"_score_0_1"] for r in mm)
            row[key+"_points"] = mean(r[key+"_points"] for r in mm)
        row["total_score_100"] = mean(r["total_score_100"] for r in mm)
        values = [r["total_score_100"] for r in mm]
        diffs = [by_cell[(m,x)]["total_score_100"]-by_cell[(m,-.05)]["total_score_100"] for m in maps]
        row["difference_vs_A_points"] = mean(diffs)
        row["score_ci95_low"], row["score_ci95_high"] = interval([mean(values[i] for i in draw) for draw in draws])
        row["difference_vs_A_ci95_low"], row["difference_vs_A_ci95_high"] = interval([mean(diffs[i] for i in draw) for draw in draws])
        # Independent arithmetic reconciliation, not an additional experiment.
        expected = 40*row["arrived_n"]/50 + 30*(1-row["collision_n"]/50) + row["efficiency_points"] + row["stability_points"]
        if not math.isclose(row["total_score_100"], expected, abs_tol=1e-10):
            raise ValueError("Score does not reconcile to counts and components")
        summary.append(row)
    for rank,row in enumerate(sorted(summary, key=lambda r:-r["total_score_100"]),1):
        row["rank"] = rank
    out.mkdir(parents=True, exist_ok=True)
    write_csv(out/"per_run_scores_400.csv", scored)
    write_csv(out/"map_threshold_scores_80.csv", map_rows)
    write_csv(out/"threshold_scores_8.csv", sorted(summary,key=lambda r:r["rank"]))
    metadata = {"version": "t1-weighted-score/1.0", "retrospective": True,
                "source": str(source.relative_to(ROOT)), "source_sha256": source_digest,
                "weights": WEIGHTS, "time_cap_s": TIME_CAP, "frequency_reference_per_min": F_REF,
                "aggregation": "transform each run; average five runs per map; equal mean of ten map scores",
                "bootstrap": {"unit": "map", "paired_B_minus_A": True, "repetitions": 10000, "seed": BOOTSTRAP_SEED},
                "arrival_over_time_cap": [r["plan_item_id"] for r in scored if r["arrival_over_time_cap"]],
                "safety_component_scope": "task-window terminal nonfoot obstacle collision; fall and foot impacts remain separate",
                "raw_files_modified": False}
    (out/"scoring_method.json").write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="t1-score-mpl-") as mpl_dir:
        os.environ["MPLCONFIGDIR"] = mpl_dir
        plots(summary, maps, by_cell, out)
    report(summary, maps, by_cell, out, metadata)
    if hashlib.sha256(source.read_bytes()).hexdigest() != source_digest:
        raise RuntimeError("Input changed during scoring")
    for r in sorted(summary,key=lambda r:r["rank"]):
        print(f"{r['rank']} X={r['exit_threshold_X']:.2f} score={r['total_score_100']:.6f} "
              f"A-difference={r['difference_vs_A_points']:+.6f}")
    print(f"Output: {out}")


def plots(summary, maps, by_cell, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    plt.rcParams["axes.unicode_minus"] = False
    labels = [f"{r['exit_threshold_X']:.2f}" for r in summary]
    fig,ax = plt.subplots(figsize=(12,6.8))
    bottom = [0.0]*8
    for key,label,color in (("arrival","到达（40分）","#2867a1"),("safety","无终止性碰撞（30分）","#31967e"),
                            ("efficiency","效率（20分）","#e3a146"),("stability","切换稳定性（10分）","#8061a6")):
        vals = [r[key+"_points"] for r in summary]
        ax.bar(labels, vals, bottom=bottom, label=label, color=color,width=.65)
        bottom = [a+b for a,b in zip(bottom,vals)]
    for i,r in enumerate(summary):
        ax.text(i,r["total_score_100"]+1,f"{r['total_score_100']:.2f}",ha="center",fontsize=11)
    ax.set_ylim(0,100); ax.set_ylabel("综合分（满分100）",fontproperties=font)
    ax.set_xlabel("退出阈值；−0.05为A组参考",fontproperties=font)
    ax.set_title("400次开发实验：各项加权得分",fontproperties=font,fontsize=17,pad=15)
    ax.legend(prop=font,loc="upper center",ncol=2,frameon=False)
    ax.set_axisbelow(True);ax.grid(axis="y",alpha=.18)
    fig.text(.5,.015,"每档50次，10张地图等权。事后辅助评分，不代表已证明最优或满足安全约束。",ha="center",fontproperties=font,fontsize=11)
    fig.tight_layout(rect=(0,.04,1,1));fig.savefig(out/"weighted_score_components.png",dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,7))
    data=[[by_cell[(m,x)]["total_score_100"] for x in LEVELS] for m in maps]
    image=ax.imshow(data,cmap="YlGnBu",vmin=0,vmax=100,aspect="auto")
    ax.set_xticks(range(8),labels);ax.set_yticks(range(10),[m.removesuffix(".xml") for m in maps])
    for i,row in enumerate(data):
        for j,v in enumerate(row):
            ax.text(j,i,f"{v:.1f}",ha="center",va="center",color="white" if v>65 else "#17212b")
    ax.set_title("逐地图综合分：每格5次运行的平均",fontproperties=font,fontsize=17,pad=15)
    ax.set_xlabel("退出阈值；−0.05为A组参考",fontproperties=font)
    fig.colorbar(image,ax=ax,label="Score / 100")
    fig.tight_layout();fig.savefig(out/"map_threshold_weighted_scores.png",dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(11,6))
    for i,r in enumerate(summary[1:]):
        d=r["difference_vs_A_points"]
        ax.errorbar(d,i,xerr=[[d-r["difference_vs_A_ci95_low"]],[r["difference_vs_A_ci95_high"]-d]],fmt="o",capsize=4,color="#2867a1")
    ax.axvline(0,color="#777",linestyle="--",linewidth=1)
    ax.set_yticks(range(7),labels[1:]);ax.invert_yaxis()
    ax.set_xlabel("相对A组的分数差及95%区间",fontproperties=font)
    ax.set_ylabel("B组退出阈值",fontproperties=font)
    ax.set_title("按地图成组重抽：综合分与A组的差异",fontproperties=font,fontsize=16)
    fig.text(.5,.01,"10张开发图、10,000次重抽；未校正多候选选择，不能外推任意地图。",ha="center",fontproperties=font)
    fig.tight_layout(rect=(0,.04,1,1));fig.savefig(out/"score_difference_vs_A.png",dpi=160);plt.close(fig)


def report(summary,maps,by_cell,out,metadata):
    lines=["# T1：400次实验的加权评分", "", "状态：已完成事后派生计算；T1保持ACTIVE，未冻结最终阈值。", "",
           "## 计算规则", "", "按用户确认的权重：到达40%、无终止性碰撞30%、效率20%、切换稳定性10%。所有400条VALID任务结果均参与，不删除合法失败，不纳入原96项粗选或运行前解析失败。", "",
           "逐run计算四项0～1分：到达为1，否则0；任务期无终止性碰撞为1，否则0；到达效率=max(0,1−任务到达用时/20)，未到达为0；稳定性=1/(1+短反向切换频率/257.8)。总分=40×到达分+30×安全分+20×效率分+10×稳定性分。", "",
           "先平均每张地图同档的5次得分，再对10张地图等权平均。稳定性先逐run变换再平均，不能把平均频率代入非线性公式；257.8固定使用先前确认的四舍五入A参考，不替换成某个阈值或本次重算的数。分项不提前四舍五入。", "",
           "全部400条碰撞结论明确、切换覆盖完整、任务时间覆盖为1且无周期/时钟缺口，因此本次没有因缺失指标剔除任何run。非到达效率为0；到达与效率共同惩罚未完成，是本评分有意采用的重复影响。超时不再额外加权。", "",
           "## 排名与分项贡献", "", "以下分项均为已乘权重后的分数，四项合计满分100。", "",
           "| 排名 | X | 到达/50 | 碰撞/50 | 到达贡献/40 | 安全贡献/30 | 效率贡献/20 | 稳定贡献/10 | 总分 | 相对A |",
           "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for r in sorted(summary,key=lambda r:r["rank"]):
        lines.append(f"| {r['rank']} | {r['exit_threshold_X']:.2f} | {r['arrived_n']} | {r['collision_n']} | "
                     f"{r['arrival_points']:.3f} | {r['safety_points']:.3f} | {r['efficiency_points']:.3f} | "
                     f"{r['stability_points']:.3f} | **{r['total_score_100']:.3f}** | {r['difference_vs_A_points']:+.3f} |")
    lines += ["", "## 区间与解释", "", "以地图为聚类单位做10,000次bootstrap，种子20261017；各档与B−A使用相同的地图抽样索引，保留每图5次重复的组内关系。区间仅描述这10张有目的选取开发图下的不确定性，未校正多候选选择，也不是严格同初态配对。", "",
              "| X | 总分 | 相对A分差 | 分差95%区间 |", "|---:|---:|---:|---:|"]
    for r in summary:
        lines.append(f"| {r['exit_threshold_X']:.2f} | {r['total_score_100']:.3f} | {r['difference_vs_A_points']:+.3f} | [{r['difference_vs_A_ci95_low']:+.3f}, {r['difference_vs_A_ci95_high']:+.3f}] |")
    lines += ["", "−0.07在此评分规则和本批数据下排第一。−0.19、−0.09和−0.13分数接近，不把小分差解释成稳定优劣。所有B−A分差95%区间都包含0，不能据此确认稳定优于A。不得把综合分第一直接写成全局最优阈值或安全约束已满足。", "",
              "−0.07的主要得分优势来自到达与无终止性碰撞两项。其逐run稳定性变换的平均贡献为6.053分，低于A的6.242分；这与平均短反向频率较低并不矛盾，因为该变换非线性，且各地图/运行分布不同。不得混用先平均频率再变换的另一种评分口径。", "",
              "## 必须同时保留的事实", "",
              "- 评分是在400次实验完成、结果已知后由用户确定的，是事后辅助分析；不能改称运行前预注册的选择规则。若未来采用同一规则，应在下一轮开始前固定。",
              "- 安全分只表示既定任务窗口的终止性障碍碰撞口径，不包括跌倒惩罚、足端接触/撞击、越界或收尾接触。不得把该分数称为整机安全分。",
              "- −0.11含1次确认跌倒，−0.13含1条跌倒UNKNOWN；这些运行保持原样参与已约定公式，同时作为独立风险标记。没有悄然增加跌倒惩罚或将UNKNOWN当False。",
              "- 34条ARRIVED记录伴随足端撞击；它们的到达和终止性碰撞字段不追改，但不能称为无撞击到达。",
              "- 第257项任务终局为仿真限时；随后收尾出现机器人与障碍接触，站立观察INCOMPLETE并转为PASSIVE。属于收尾事件，不倒算入任务期碰撞分。具体接触强度及部位不由本评分推断。",
              "- 第363项原始ARRIVED用时20.02秒，略超20秒评分上限；效率分取0，原始到达终局与时长保留。",
              "- 不新增安全通过门槛，不因分数自动淘汰地图、冻结阈值或授权第二轮。M1留出集未使用，M2未运行。", "",
              "## 图表与逐项数据", "",
              "- [分项得分图](weighted_score_components.png)",
              "- [逐地图得分热图](map_threshold_weighted_scores.png)",
              "- [相对A分差及区间](score_difference_vs_A.png)",
              "- [400条逐run得分](per_run_scores_400.csv)",
              "- [80格逐地图×阈值得分](map_threshold_scores_80.csv)",
              "- [8档总排名及区间](threshold_scores_8.csv)",
              "- [公式与来源标识](scoring_method.json)",
              "- [输入400条结果](../expanded_run_results.csv)", "",
              "复算：`rtk proxy /home/lidio/anaconda3/bin/python3 scripts/score_t1_expanded_400.py`（从仓库根目录执行）。", "",
              "本轮仅读取输入CSV并生成独立派生表图，输入SHA-256在计算前后相同。未改原始run、旧图或旧指标，未运行仿真、ROS/controller、训练或测试套件，未提交Git。分析进程已结束，没有本轮仿真进程需关闭。"]
    (out/"REPORT.md").write_text("\n".join(lines)+"\n",encoding="utf-8")


if __name__ == "__main__":
    main()
