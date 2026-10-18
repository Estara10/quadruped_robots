#!/usr/bin/env python3
"""Analysis for the T2 A/C minimum-hold function check (4 runs on m02)."""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
E = -0.05
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
MUTED = "#52514e"
GRID = "#dddcd6"


def load_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def fnum(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def mean(values):
    values = [v for v in values if v is not None]
    return statistics.mean(values) if values else None


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_frames(run_dir: Path) -> list[dict]:
    path = run_dir / "runtime_record.jsonl"
    if not path.is_file():
        return []
    frames = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("kind") == "frame" and isinstance(row.get("payload"), dict):
            payload = row["payload"]
            if fnum(payload.get("sim_time_s")) is not None:
                frames.append(payload)
    return sorted(frames, key=lambda p: float(p["sim_time_s"]))


def state(payload: dict) -> int | None:
    value = payload.get("policy_state")
    return int(value) if isinstance(value, (int, float)) else None


def build_run_row(item: dict, attempt: dict) -> dict:
    run_dir = Path(attempt["run_directory"])
    metrics = load_json(run_dir / "metrics.json")
    context = load_json(run_dir / "run_context.json")
    safety = metrics.get("task_window_safety") or {}
    lifecycle = (context.get("lifecycle") or {}).get("standing_observation") or {}
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    cycle0 = (context.get("initial_state") or {}).get("first_active_policy_cycle") or {}
    audit = metrics.get("recovery_hold_audit") or {}
    return {
        "sequence": item["sequence"], "item": item["plan_item_id"],
        "group": item["group"], "H_s": item["hold_s"],
        "hold_consumed": item["hold_consumed"], "scene": item["scene"],
        "scene_id": item["scene_id"], "run_id": attempt.get("run_id"),
        "terminal": attempt.get("terminal_result"), "record_validity": attempt.get("record_validity"),
        "config_source": (context.get("runtime_source_validation") or {}).get("status"),
        "actual_group": source.get("group"), "actual_entry": source.get("entry_threshold"),
        "actual_exit": source.get("exit_threshold"), "actual_hold": source.get("hold"),
        "actual_recovery_min_hold_s": source.get("recovery_min_hold_s"),
        "hold_audit_status": audit.get("status"),
        "hold_episode_count": len(audit.get("episodes") or []),
        "hold_violation_count": len(audit.get("violations") or []),
        "arrival_distance_m": fnum(metrics.get("target_distance_final_m")),
        "task_duration_s": fnum(metrics.get("elapsed_task_duration_s")),
        "path_length_m": fnum(metrics.get("path_length_m")),
        "policy_frequency_hz": fnum(metrics.get("policy_frequency_hz_overall")),
        "recovery_entries": fnum(metrics.get("agile_to_recovery")),
        "recovery_exits": fnum(metrics.get("recovery_to_agile")),
        "recovery_share": fnum(metrics.get("recovery_share_valid_motion_time")),
        "short_reversals": fnum(metrics.get("short_reverse_switch_count")),
        "cycle_gaps": metrics.get("motion_policy_cycle_gaps"),
        "invalid_clock_frames": metrics.get("invalid_clock_frames"),
        "nonfoot_collision": safety.get("nonfoot_collision_failure"),
        "collision": safety.get("collision"), "fall": safety.get("fall"),
        "safety_coverage": safety.get("coverage_complete_for_motion_interval"),
        "standing_observation": metrics.get("standing_observation_status"),
        "standing_observed_s": lifecycle.get("observed_elapsed_sim_s"),
        "standing_sim_range": json.dumps(lifecycle.get("observed_sim_time_s")),
        "shutdown_complete": attempt.get("process_shutdown_complete"),
        "closeout_complete": attempt.get("closeout_complete"),
        "window_status": (attempt.get("muJoCo_window_log") or {}).get("status"),
        "window_line": (attempt.get("muJoCo_window_log") or {}).get("log_line"),
        "initial_sim_time_s": cycle0.get("sim_time_s"),
        "initial_xy": json.dumps(cycle0.get("world_position_xy_m")),
        "initial_yaw": cycle0.get("world_yaw_rad"),
    }


def build_episode_rows(item: dict, attempt: dict) -> tuple[list[dict], list[dict]]:
    run_dir = Path(attempt["run_directory"])
    metrics = load_json(run_dir / "metrics.json")
    audit = metrics.get("recovery_hold_audit") or {}
    frames = read_frames(run_dir)
    by_cycle = {int(f["rl_step"]): f for f in frames if isinstance(f.get("rl_step"), (int, float))}
    episodes = []
    violations = list(audit.get("violations") or [])
    for index, episode in enumerate(audit.get("episodes") or [], start=1):
        enter_cycle = episode.get("enter_cycle")
        exit_cycle = episode.get("exit_cycle")
        t_enter = fnum(episode.get("t_enter_sim_s"))
        H = fnum(episode.get("H_s"))
        entry = by_cycle.get(int(enter_cycle)) if isinstance(enter_cycle, (int, float)) else None
        exit_frame = by_cycle.get(int(exit_cycle)) if isinstance(exit_cycle, (int, float)) else None
        later = [f for f in frames if t_enter is not None and float(f["sim_time_s"]) >= t_enter
                 and (enter_cycle is None or int(f.get("rl_step", -1)) >= int(enter_cycle))
                 and (exit_cycle is None or int(f.get("rl_step", -1)) <= int(exit_cycle))]
        first_clear = next((f for f in later
                            if fnum(f.get("ra_value")) is not None
                            and float(f["ra_value"]) < E
                            and (state(f) == 1
                                 or (exit_cycle is not None
                                     and int(f.get("rl_step", -1)) == int(exit_cycle)))), None)
        expiry = t_enter + H if t_enter is not None and H is not None else None
        first_eligible = None
        for frame in later:
            if (state(frame) == 1 and expiry is not None
                    and float(frame["sim_time_s"]) >= expiry
                    and fnum(frame.get("ra_value")) is not None
                    and float(frame["ra_value"]) < E):
                first_eligible = frame
                break
        residence = None
        if exit_frame is not None and t_enter is not None:
            residence = float(exit_frame["sim_time_s"]) - t_enter
        if exit_frame is not None:
            reason = "exited"
            if residence is not None and H is not None and residence < H - 1e-9:
                reason = "exited_before_H"
            elif (fnum(exit_frame.get("ra_value")) is not None
                  and float(exit_frame["ra_value"]) >= E):
                reason = "exited_with_RA_not_below_E"
            elif first_eligible is not None and int(exit_frame.get("rl_step", -1)) > int(first_eligible.get("rl_step", -1)):
                reason = "exited_later_than_first_eligible_cycle"
        else:
            last = later[-1] if later else None
            if last is None:
                reason = "no_frames_after_entry"
            elif state(last) == 1:
                if expiry is not None and float(last["sim_time_s"]) < expiry:
                    reason = "censored_before_H_at_task_terminal"
                elif first_eligible is None:
                    reason = "risk_not_below_E_at_task_terminal"
                else:
                    reason = "did_not_exit_after_H_and_RA_below_E"
            else:
                reason = "no_exit_observed_unknown"
        expiry_frame = next((f for f in later if expiry is not None
                             and float(f["sim_time_s"]) >= expiry), None)
        ra_at_expiry = fnum(expiry_frame.get("ra_value")) if expiry_frame else None
        risk_high_at_expiry = (ra_at_expiry is not None and ra_at_expiry >= E)
        continued_high = bool(risk_high_at_expiry and (
            exit_cycle is None or int(expiry_frame.get("rl_step", -1)) < int(exit_cycle)))
        entry_time_changed = any(
            fnum(f.get("hold_enter_sim_time_s")) is not None and t_enter is not None
            and abs(float(f["hold_enter_sim_time_s"]) - t_enter) > 1e-9
            for f in later)
        seen_low = False
        risk_re_rise = False
        for f in later:
            ra = fnum(f.get("ra_value"))
            if ra is None:
                continue
            if ra < E:
                seen_low = True
            elif seen_low:
                risk_re_rise = True
        episodes.append({
            "run_item": item["plan_item_id"], "run_id": attempt.get("run_id"),
            "group": item["group"], "H_s": H, "episode_index": index,
            "enter_cycle": enter_cycle, "enter_sim_s": t_enter,
            "H_expiry_sim_s": expiry,
            "first_risk_clear_sim_s": fnum(first_clear.get("sim_time_s")) if first_clear else None,
            "first_risk_clear_cycle": first_clear.get("rl_step") if first_clear else None,
            "first_eligible_after_H_sim_s": fnum(first_eligible.get("sim_time_s")) if first_eligible else None,
            "first_eligible_after_H_cycle": first_eligible.get("rl_step") if first_eligible else None,
            "exit_cycle": exit_cycle,
            "exit_sim_s": fnum(exit_frame.get("sim_time_s")) if exit_frame else None,
            "observed_residence_s": residence,
            "residence_minus_H_s": (residence - H) if residence is not None and H is not None else None,
            "exit_ra": fnum(exit_frame.get("ra_value")) if exit_frame else None,
            "censored": bool(episode.get("censored")),
            "clock_coverage": episode.get("clock_coverage"),
            "hold_rule": episode.get("hold_rule"),
            "exit_reason": reason,
            "ra_at_expiry": ra_at_expiry,
            "risk_high_at_expiry": risk_high_at_expiry,
            "continued_recovery_after_expiry_high_risk": continued_high,
            "entry_time_changed_within_episode": entry_time_changed,
            "risk_re_rise_within_episode": risk_re_rise,
            "reentered_after_exit": None,
        })
    # Mark re-entry within 0.5 s after each exit, for context only.
    for episode in episodes:
        if episode["exit_sim_s"] is None:
            continue
        window = [f for f in frames if episode["exit_sim_s"] < float(f["sim_time_s"])
                  <= episode["exit_sim_s"] + 0.5]
        episode["reentered_after_exit"] = any(state(f) == 1 for f in window)
    return episodes, violations


def semantic_checks(run_rows: list[dict], episodes: list[dict], violations: list[dict]) -> dict:
    c_rows = [r for r in run_rows if r["group"] == "C"]
    a_rows = [r for r in run_rows if r["group"] == "A"]
    early = [e for e in episodes if e["exit_reason"] == "exited_before_H"]
    low_risk_no_exit = [e for e in episodes if e["exit_reason"] in
                        ("did_not_exit_after_H_and_RA_below_E", "exited_later_than_first_eligible_cycle")]
    high_risk_expired = [e for e in episodes if e["exit_reason"] == "risk_not_below_E_at_task_terminal"]
    clock_incomplete = [e for e in episodes if e["clock_coverage"] != "complete"]
    return {
        "a_runs": len(a_rows),
        "a_hold_enabled_any": any(str(r["actual_hold"]).lower() in ("true", "1") for r in a_rows),
        "a_hold_field_nonzero_any": any((fnum(r["actual_recovery_min_hold_s"]) or 0.0) != 0.0 for r in a_rows),
        "c_runs": len(c_rows),
        "c_hold_enabled_all": all(str(r["actual_hold"]).lower() in ("true", "1") for r in c_rows),
        "c_hold_values": sorted({fnum(r["actual_recovery_min_hold_s"]) for r in c_rows}),
        "c_episodes": len(episodes),
        "early_exits_before_H": len(early),
        "expired_low_risk_without_prompt_exit": len(low_risk_no_exit),
        "expired_high_risk_continued_recovery": sum(
            1 for e in episodes if e["continued_recovery_after_expiry_high_risk"]),
        "episodes_high_risk_at_expiry": sum(1 for e in episodes if e["risk_high_at_expiry"]),
        "episodes_entry_time_changed_within_episode": sum(
            1 for e in episodes if e["entry_time_changed_within_episode"]),
        "episodes_with_risk_re_rise_within_episode": sum(
            1 for e in episodes if e["risk_re_rise_within_episode"]),
        "risk_re_rise_episodes_with_entry_time_changed": sum(
            1 for e in episodes if e["risk_re_rise_within_episode"] and e["entry_time_changed_within_episode"]),
        "distinct_entry_times": len({round(float(e["enter_sim_s"]), 9) for e in episodes
                                     if e["enter_sim_s"] is not None}),
        "clock_incomplete_episodes": len(clock_incomplete),
        "audit_violations": len(violations),
        "audit_statuses": sorted({r["hold_audit_status"] for r in c_rows if r["hold_audit_status"]}),
        "safety_abort_runs": sum(1 for r in run_rows if r["terminal"] == "SYSTEM_SAFETY_ABORT"),
        "episodes_with_first_clear_before_H": sum(
            1 for e in episodes if e["first_risk_clear_sim_s"] is not None
            and e["H_expiry_sim_s"] is not None
            and e["first_risk_clear_sim_s"] < e["H_expiry_sim_s"]),
        "episodes_with_first_clear_after_H": sum(
            1 for e in episodes if e["first_risk_clear_sim_s"] is not None
            and e["H_expiry_sim_s"] is not None
            and e["first_risk_clear_sim_s"] >= e["H_expiry_sim_s"]),
        "exits_with_reentry_within_0_5s": sum(1 for e in episodes if e["reentered_after_exit"] is True),
        "exits_observed": sum(1 for e in episodes if e["exit_sim_s"] is not None),
    }


def plots(out: Path, run_rows: list[dict], episodes: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    chart_dir = out / "charts"
    chart_dir.mkdir(exist_ok=True)

    def style(ax):
        ax.set_facecolor(SURFACE)
        ax.grid(color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.tick_params(colors=MUTED, labelsize=8)

    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.0), facecolor=SURFACE)
    labels = [("A" if r["group"] == "A" else f"C H={r['H_s']:.2f}") for r in run_rows]
    for ax, key, title in ((axes[0], "recovery_share", "Recovery share of valid task time"),
                           (axes[1], "short_reversals", "Short reverse switches")):
        values = [fnum(r[key]) for r in run_rows]
        ax.bar(range(len(run_rows)), [0 if v is None else v for v in values],
               color=SERIES[:len(run_rows)], edgecolor=SURFACE, linewidth=1.2)
        ax.set_xticks(range(len(run_rows)))
        ax.set_xticklabels(labels, rotation=15, ha="right")
        style(ax)
        ax.set_title(title, fontsize=10, loc="left")
    figure.tight_layout()
    figure.savefig(chart_dir / "run_metrics.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)

    if episodes:
        figure, ax = plt.subplots(figsize=(7.5, 4.2), facecolor=SURFACE)
        for hold, color in zip((0.10, 0.30, 0.50), SERIES[:3]):
            rows = [e for e in episodes if e["H_s"] == hold]
            xs = [i for i, _ in enumerate(rows)]
            ax.scatter([hold] * len(rows),
                       [e["observed_residence_s"] for e in rows],
                       color=color, s=32, label=f"H={hold:.2f}", zorder=3)
        ax.plot([0.08, 0.55], [0.08, 0.55], color=MUTED, linewidth=1.0, linestyle=(0, (3, 3)),
                label="residence = H", zorder=1)
        style(ax)
        ax.set_xlabel("configured H (sim s)", fontsize=8)
        ax.set_ylabel("observed residence (sim s)", fontsize=8)
        ax.set_title("Recovery residence versus configured hold", fontsize=10, loc="left")
        ax.legend(frameon=False, fontsize=7)
        figure.tight_layout()
        figure.savefig(chart_dir / "hold_residence.png", dpi=160, facecolor=SURFACE)
        plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    plan = load_json(args.plan)
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    run_rows = []
    episodes = []
    violations = []
    for item in plan["items"]:
        if not item.get("attempts"):
            continue
        attempt = item["attempts"][-1]
        run_rows.append(build_run_row(item, attempt))
        if item["group"] == "C":
            episode_rows, item_violations = build_episode_rows(item, attempt)
            episodes.extend(episode_rows)
            for violation in item_violations:
                violations.append({"run_item": item["plan_item_id"], **violation})
    if len(run_rows) != 4:
        raise ValueError(f"expected 4 run rows, found {len(run_rows)}")
    write_csv(out / "run_results.csv", run_rows)
    write_csv(out / "hold_episodes.csv", episodes)
    write_csv(out / "hold_violations.csv", violations)
    checks = semantic_checks(run_rows, episodes, violations)
    (out / "semantic_checks.json").write_text(
        json.dumps(checks, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    plots(out, run_rows, episodes)

    lines = ["# T2 A/C 保持时间功能试跑：自动汇总", ""]
    lines.append("| 顺序 | 组别 | H(s) | 终局 | 记录 | 实际配置 | 保持审计 | 到达距离m | 任务s | Recovery占比 | 短反向 | 站立 | 清理 |")
    lines.append("|---|---|---:|---|---|---|---|---|---:|---:|---:|---:|---|---|")
    for row in run_rows:
        config = (f"{row['actual_group']}/E={row['actual_entry']}/"
                  f"exit={row['actual_exit']}/hold={row['actual_hold']}/"
                  f"H={row['actual_recovery_min_hold_s']}")
        lines.append("| " + " | ".join(str(v) for v in (
            row["sequence"], row["group"], f"{row['H_s']:.2f}" if row["H_s"] is not None else "off",
            row["terminal"], row["record_validity"], config, row["hold_audit_status"],
            row["arrival_distance_m"], row["task_duration_s"], row["recovery_share"],
            row["short_reversals"], row["standing_observation"],
            f"{row['shutdown_complete']}/{row['closeout_complete']}")) + " |")
    lines.append("")
    lines.append("## 保持 episode 审计")
    lines.append(f"- C 的 episode 数：{checks['c_episodes']}")
    lines.append(f"- H 前提前退出：{checks['early_exits_before_H']}")
    lines.append(f"- 到期且 RA<E 但未及时退出：{checks['expired_low_risk_without_prompt_exit']}")
    lines.append(f"- 到期但风险仍高而继续 Recovery：{checks['expired_high_risk_continued_recovery']}")
    lines.append(f"- 时钟覆盖不完整 episode：{checks['clock_incomplete_episodes']}")
    lines.append(f"- 审计违规：{checks['audit_violations']}")
    lines.append("")
    lines.append("单次功能试跑，不比较H优劣、不选择最佳H、不证明总体成功率或实机计时。")
    (out / "RESULTS_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"analysis written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
