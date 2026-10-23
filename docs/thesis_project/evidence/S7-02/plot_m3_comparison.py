"""Render the existing M3 batch summary; starts no simulation."""
from pathlib import Path
import csv
import json
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/m3_comparison_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / "logs/08_formal_experiment/M3_ABCD_240/m3-abcd240-20261023/analysis/formal_summary"
OUT = Path(__file__).resolve().parent
data = json.loads((SOURCE / "formal_summary.json").read_text())
metrics = data["comparisons"]["metrics"]
with (SOURCE / "primary_plan_items.csv").open() as f:
    rows = list(csv.DictReader(f))

groups = list("ABCD")
colors = ["#64748b", "#2563a6", "#118579", "#c47526"]
labels = ["A 原始单阈值", "B 双阈值", "C 最小保持", "D 两者组合"]
def values(key, scale=1):
    return [metrics[key]["groups"][g]["map_equal_value"] * scale for g in groups]

plt.rcParams.update({"font.family": "WenQuanYi Micro Hei", "font.size": 11,
                     "axes.spines.top": False, "axes.spines.right": False,
                     "axes.unicode_minus": False, "figure.facecolor": "white"})
fig, axes = plt.subplots(2, 2, figsize=(13, 9.5))
fig.subplots_adjust(left=.075, right=.97, top=.86, bottom=.15, wspace=.25, hspace=.45)
fig.suptitle("M3 v1.4：四组挑战地图比较", fontsize=21, x=.075, ha="left", y=.98)
fig.text(.075, .92, "30张地图 × 每组2次；每组60项｜E = −0.05，B/D：X = −0.10，C/D：H = 0.20秒", fontsize=11, color="#475569")

def bars(ax, key, scale, title, unit, limit, fmt, counts=False):
    vals = values(key, scale)
    ax.bar(groups, vals, color=colors, width=.58, zorder=3)
    ax.set_title(title, loc="left", fontsize=14, pad=18)
    ax.set_ylabel(unit)
    ax.set_ylim(0, limit)
    ax.grid(axis="y", color="#e2e8f0", zorder=0)
    for i, v in enumerate(vals):
        text = fmt.format(v)
        if counts:
            n = sum(r["group"] == groups[i] and r["unified_safety_task_completion"] == "true" for r in rows)
            text += f"\n{n}/60"
        ax.text(i, v + limit*.025, text, ha="center", va="bottom", fontsize=11)

bars(axes[0,0], "safe_complete", 100, "① 确认安全完成率 ↑", "%", 100, "{:.1f}%", True)
bars(axes[0,1], "short_reverse_per_min", 1, "② 短反向切换频率 ↓", "次 / 有效分钟", 390, "{:.1f}")
bars(axes[1,0], "recovery_share", 100, "③ Recovery时间占比（机制代价）", "%", 36, "{:.1f}%")

ax = axes[1,1]
ax.set_title("④ 未安全完成的结果构成", loc="left", fontsize=14, pad=18)
bottom = np.zeros(4)
categories = [("碰撞", "COLLISION_TERMINATION", "#b34e59"),
              ("仿真超时", "DIAGNOSTIC_SIM_TIME_LIMIT", "#d69a34"),
              ("跌倒", "FALL_TERMINATION", "#805aaa"),
              ("安全中止 / 到达但越界", None, "#64748b")]
for label, terminal, color in categories:
    counts = []
    for g in groups:
        counts.append(sum(r["group"] == g and (r["selected_terminal"] == terminal if terminal
            else r["selected_terminal"] == "SYSTEM_SAFETY_ABORT" or
                 (r["selected_terminal"] == "ARRIVED" and r["unified_safety_task_completion"] == "false")) for r in rows))
    ax.bar(groups, counts, bottom=bottom, color=color, width=.58, label=label, zorder=3)
    for i, count in enumerate(counts):
        if count:
            ax.text(i, bottom[i]+count/2, str(count), ha="center", va="center", color="white", fontsize=10)
    bottom += counts
ax.set_ylim(0, 28)
ax.set_ylabel("项")
ax.grid(axis="y", color="#e2e8f0", zorder=0)
ax.legend(frameon=False, fontsize=9, loc="upper left", bbox_to_anchor=(0, 1.01), ncol=2)

fig.text(.075, .09, "A 原始单阈值     B 双阈值     C 最小保持     D 两者组合", fontsize=11)
fig.text(.075, .055, "图示为点估计，不能仅凭柱高判断显著优劣。安全完成率差值的95%区间均跨0。\nA有1项安全中止的切换/Recovery指标未知，相关均值按已知值地图等权；M3为历史筛选挑战集。", fontsize=9, color="#475569")
for suffix in ("png", "svg"):
    path = OUT / f"M3_ABCD_COMPARISON.{suffix}"
    fig.savefig(path, dpi=170)
    print(path)
plt.close(fig)
