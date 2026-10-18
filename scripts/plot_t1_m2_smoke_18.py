#!/usr/bin/env python3
"""Simple static diagnostic charts for the T1 new-M2 18-map smoke batch."""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

# Reference categorical slots (validated order); used in fixed order.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
MUTED = "#52514e"
GRID = "#dddcd6"
TERMINALS = ["ARRIVED", "COLLISION_TERMINATION", "DIAGNOSTIC_SIM_TIME_LIMIT",
             "FALL_TERMINATION", "WALL_CLOCK_GUARD_INCOMPLETE"]
TERMINAL_COLOR = dict(zip(TERMINALS, SERIES))
SHORT = {"ARRIVED": "ARRIVED", "COLLISION_TERMINATION": "COLLISION",
         "DIAGNOSTIC_SIM_TIME_LIMIT": "SIM LIMIT", "FALL_TERMINATION": "FALL",
         "WALL_CLOCK_GUARD_INCOMPLETE": "WALL CLOCK"}


def number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def load(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(axis="x", color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8, length=0)
    ax.yaxis.label.set_color(MUTED)
    ax.xaxis.label.set_color(MUTED)
    ax.title.set_color(TEXT)


def fig(figsize, ncols=1):
    figure, axes = plt.subplots(1, ncols, figsize=figsize, facecolor=SURFACE)
    return figure, axes


def terminal_chart(rows, out: Path):
    figure, ax = fig((7.2, 6.4))
    labels = [f"{row['map']} · {row['difficulty'][:1].upper()}" for row in rows]
    y = list(range(len(rows)))[::-1]
    for index, terminal in enumerate(TERMINALS):
        xs = [index for row in rows if row["terminal"] == terminal and row["status"] == "VALID_TASK_RESULT"]
        ys = [y[i] for i, row in enumerate(rows)
              if row["terminal"] == terminal and row["status"] == "VALID_TASK_RESULT"]
        if xs:
            ax.scatter(xs, ys, s=90, color=TERMINAL_COLOR[terminal], edgecolor=SURFACE,
                       linewidth=1.5, zorder=3)
    invalid_y = [y[i] for i, row in enumerate(rows) if row["status"] != "VALID_TASK_RESULT"]
    for value in invalid_y:
        ax.scatter([2.5], [value], s=90, facecolor=SURFACE, edgecolor=MUTED,
                   linewidth=1.2, zorder=3)
    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    ax.set_xticks(range(len(TERMINALS)))
    ax.set_xticklabels([SHORT[t] for t in TERMINALS], rotation=20, ha="right")
    ax.set_xlim(-0.55, len(TERMINALS) - 0.45)
    ax.set_ylim(-0.7, len(rows) - 0.3)
    style(ax)
    ax.grid(axis="y", visible=False)
    ax.set_title("Terminal outcome per map (1 run each, B / X=-0.07)", fontsize=10, loc="left")
    handles = [Line2D([], [], marker="o", linestyle="", color=TERMINAL_COLOR[t],
                      markersize=7, label=SHORT[t]) for t in TERMINALS]
    handles.append(Line2D([], [], marker="o", linestyle="", markerfacecolor=SURFACE,
                          markeredgecolor=MUTED, markersize=7, label="NO VALID RESULT"))
    ax.legend(handles=handles, frameon=False, fontsize=7.5, loc="upper center",
              bbox_to_anchor=(0.5, -0.13), ncol=3)
    figure.tight_layout()
    figure.savefig(out / "terminal_outcomes.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)


def bars(ax, rows, key, title, xlabel, reference=None):
    labels = [row["map"] for row in rows]
    values = [number(row[key]) for row in rows]
    y = list(range(len(rows)))[::-1]
    ax.barh(y, [0 if math.isnan(v) else v for v in values], height=0.62,
            color=SERIES[0], edgecolor=SURFACE, linewidth=1.2, zorder=2)
    for pos, value, row in zip(y, values, rows):
        if math.isnan(value):
            ax.text(0.01, pos, "UNKNOWN", va="center", fontsize=7, color=MUTED)
    if reference is not None:
        ax.axvline(reference, color=MUTED, linewidth=1.0, linestyle=(0, (3, 3)), zorder=1)
    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    style(ax)
    ax.set_title(title, fontsize=10, loc="left")
    ax.set_xlabel(xlabel, fontsize=8)


def metric_chart(rows, out: Path):
    figure, axes = fig((10.5, 6.0), ncols=2)
    bars(axes[0], rows, "arrival_distance_m", "Final distance to goal", "m", 0.5)
    bars(axes[1], rows, "task_duration_s", "Task duration (sim time)", "s", 20.0)
    figure.suptitle("Task result per map", fontsize=11, color=TEXT, x=0.01, ha="left")
    figure.tight_layout()
    figure.savefig(out / "task_metrics.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)


def switching_chart(rows, out: Path):
    figure, axes = fig((10.5, 6.0), ncols=2)
    share = [number(row["recovery_share"]) * 100.0 for row in rows]
    labels = [row["map"] for row in rows]
    y = list(range(len(rows)))[::-1]
    axes[0].barh(y, [0 if math.isnan(v) else v for v in share], height=0.62,
                 color=SERIES[0], edgecolor=SURFACE, linewidth=1.2, zorder=2)
    axes[0].set_yticks(y)
    axes[0].set_yticklabels(labels)
    style(axes[0])
    axes[0].set_title("Recovery share of valid task time", fontsize=10, loc="left")
    axes[0].set_xlabel("%", fontsize=8)
    reversals = [number(row["short_reversals"]) for row in rows]
    axes[1].barh(y, [0 if math.isnan(v) else v for v in reversals], height=0.62,
                 color=SERIES[0], edgecolor=SURFACE, linewidth=1.2, zorder=2)
    axes[1].set_yticks(y)
    axes[1].set_yticklabels(labels)
    style(axes[1])
    axes[1].set_title("Short reverse switches (< 0.5 s)", fontsize=10, loc="left")
    axes[1].set_xlabel("count", fontsize=8)
    known = [v for v in reversals if not math.isnan(v)]
    label_x = (max(known) if known else 0) + 1.5
    for pos, row in zip(y, rows):
        if row["terminal"] in ("COLLISION_TERMINATION", "FALL_TERMINATION"):
            tag = "COLLISION" if row["terminal"] == "COLLISION_TERMINATION" else "FALL"
            axes[1].text(label_x, pos, tag, va="center", ha="left", fontsize=6.5, color=MUTED)
    figure.suptitle("Switching activity per map", fontsize=11, color=TEXT, x=0.01, ha="left")
    figure.tight_layout()
    figure.savefig(out / "switching_activity.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)


def main() -> int:
    csv_path = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    rows = load(csv_path)
    if not rows:
        return 0
    terminal_chart(rows, out)
    metric_chart(rows, out)
    switching_chart(rows, out)
    print(f"charts written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
