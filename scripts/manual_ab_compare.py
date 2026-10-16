#!/usr/bin/env python3
"""一键运行非正式 A/B/C/D 仿真对比；默认场景04，每组一次，随后自动分析。"""

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from uuid import uuid4

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "scripts"))
from s1_run_diagnostic import overlay_configuration_matches  # noqa: E402


def validate_switch_parameters(entry_threshold: float, exit_threshold: float,
                               hold_s: float) -> None:
    if not all(math.isfinite(value) for value in (entry_threshold, exit_threshold, hold_s)):
        raise ValueError("switching parameters must be finite")
    if exit_threshold >= entry_threshold:
        raise ValueError("B/D exit threshold X must be below entry threshold E")
    if hold_s <= 0:
        raise ValueError("C/D hold H must be greater than zero")


def execute(command, log_path, env=None):
    """保留实时终端输出；中断时让诊断器执行自身的进程清理。"""
    last_result = None
    with log_path.open("w", encoding="utf-8") as log:
        child = subprocess.Popen(command, cwd=PROJECT, env=env, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            for line in child.stdout:
                print(line, end="", flush=True)
                log.write(line)
                log.flush()
                try:
                    value = json.loads(line)
                    if isinstance(value, dict) and "run_id" in value:
                        last_result = value
                except json.JSONDecodeError:
                    pass
            code = child.wait()
        except KeyboardInterrupt:
            print("\n已中断对比，等待当前诊断器清理；不会启动下一组。", flush=True)
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=60)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                print("尚未确认诊断器退出，请检查本次项目进程。", flush=True)
            raise
    if code:
        raise RuntimeError(f"命令退出码为 {code}，详情：{log_path}")
    return last_result


def prepare_overlay(source, destination, group, entry_threshold, exit_threshold, hold_s):
    # 解引用所有符号链接，避免写配置时修改到源码或原安装目录。
    shutil.copytree(source, destination, symlinks=False)
    config = destination / "share/go2_description/config/abs/config.yaml"
    text = config.read_text(encoding="utf-8")
    block = (f'  switching:\n    group: "{group}"\n'
             f'    entry_threshold: {entry_threshold:.17g}\n')
    if group in ("B", "D"):
        block += f"    exit_threshold: {exit_threshold:.17g}\n"
    if group in ("C", "D"):
        block += f"    recovery_min_hold_s: {hold_s:.17g}\n"
    text, count = re.subn(r"(?m)^  switching:\s*\n(?:^    [^\n]*\n)+",
                         lambda _: block, text)
    if count != 1:
        raise RuntimeError("没有找到唯一 abs.switching 配置，停止。")
    config.write_text(text, encoding="utf-8")


def set_overlay_goal(config_path, goal_x, goal_y):
    text = config_path.read_text(encoding="utf-8")
    for key, number in (("goal_x", f"{goal_x:.17g}"), ("goal_y", f"{goal_y:.17g}")):
        text, count = re.subn(rf"(?m)^  {key}:.*$", f"  {key}: {number}", text)
        if count != 1:
            raise RuntimeError(f"没有找到唯一 {key} 配置，停止。")
    config_path.write_text(text, encoding="utf-8")


def resolve_group_overlay(root, group, prepared_overlay=None):
    """Return the exact detached overlay selected for both preflight and launch."""
    return prepared_overlay.resolve() if prepared_overlay else root / "overlays" / group


def verify_overlay_detached(source, destination):
    """Ensure effective config and all three deployed artifacts are copied files."""
    relative = (
        "share/go2_description/config/abs/config.yaml",
        "share/go2_description/config/abs/policy.pt",
        "share/go2_description/config/abs/ra_value.pt",
        "share/go2_description/config/rec/policy.pt",
    )
    checked = []
    for item in relative:
        original = source / item
        copied = destination / item
        if (not original.is_file() or not copied.is_file() or copied.is_symlink() or
                os.path.samefile(original, copied)):
            raise RuntimeError(f"overlay is not an independent file copy: {item}")
        checked.append(item)
    return checked


def show(value):
    return "UNKNOWN" if value is None else f"{value:.3f}" if isinstance(value, float) else str(value)


def summarize_startup_without_runtime_record(result: dict, context: dict,
                                            scene: str) -> dict:
    """Represent a pre-recorder startup failure without fabricating runtime metrics."""
    preflight = context.get("preflight", {})
    initial = context.get("initial_state", {})
    return {"run_id": result.get("run_id"), "record_validity": "INVALID",
            "record_validity_reasons": ["runtime_record_not_started"],
            "supervisor_terminal_result": result.get("result", "UNKNOWN"),
            "navigation_terminal_result": result.get("result", "UNKNOWN"),
            "scene": scene, "session_id": context.get("session_id"), "frame_count": 0,
            "task_window_safety": {"collision": "UNKNOWN", "fall": "UNKNOWN"},
            "s6_preparation_readiness": {
                "first_active_policy_frame_exactly_persisted": False,
                "recorder_ready_before_rl": preflight.get("recorder_readiness", {}).get("status") == "ready_readable_and_writable",
                "pre_rl_standing_confirmed": initial.get("pre_rl_standing_preparation", {}).get("status") == "STABLE",
                "pre_task_safety_boundary_live": initial.get("pre_task_safety_boundary", {}).get("status") == "LIVE"},
            "startup_error": context.get("lifecycle", {}).get("startup_error"),
            "task_metrics": "UNKNOWN: strategy recorder never started"}


def enrich_run_metrics(metric: dict, run_directory: Path, facts: dict) -> dict:
    """Join analyzer metrics with closeout and actual gain-trace evidence."""
    try:
        context = json.loads((run_directory / "run_context.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        context = {}
    lifecycle = context.get("lifecycle", {})
    standing = lifecycle.get("standing_observation") or {}
    metric["standing_observation_status"] = (
        "COMPLETE" if standing.get("complete") is True else
        "INCOMPLETE" if standing else "NOT_REACHED_OR_UNKNOWN")
    stop = lifecycle.get("post_terminal_strategy_stop") or {}
    metric["passive_controller_transition_confirmed"] = stop.get("controller_transition_confirmed")
    metric["process_shutdown_complete"] = facts.get("shutdown_complete")
    metric["closeout_complete"] = facts.get("closeout_complete")

    audit = metric.get("recovery_hold_audit") or {}
    episodes = audit.get("episodes", [])
    metric["recovery_hold_audit_status"] = audit.get("status", "UNKNOWN")
    metric["recovery_hold_episodes_complete"] = sum(
        episode.get("censored") is False and episode.get("clock_coverage") == "complete"
        for episode in episodes)
    metric["recovery_hold_episodes_incomplete"] = (
        len(episodes) - metric["recovery_hold_episodes_complete"])
    metric["recovery_hold_violations"] = len(audit.get("violations", []))

    trace_path = run_directory / "startup_gain_transition.jsonl"
    trace_events = []
    if trace_path.is_file():
        for line in trace_path.read_text(encoding="utf-8").splitlines():
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(event, dict):
                trace_events.append(event)
    metric["startup_gain_trace_status"] = "CAPTURED" if trace_events else "MISSING"
    metric["startup_gain_trace_event_count"] = len(trace_events)
    return metric


def save_comparison(root, rows, groups, parameters=None, classification=None):
    plan_path = root / "batch_plan.json"
    try:
        plan = json.loads(plan_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        plan = {}
    planned_per_group = int(plan.get("repeats_per_group", 1))
    planned_statuses = plan.get("run_statuses", [])
    fields = [("record_validity", "记录有效性"),
              ("supervisor_terminal_result", "supervisor原始终局"),
              ("navigation_terminal_result", "结束原因"),
              ("task_window_safety.collision", "任务窗口碰撞判定"),
              ("task_window_safety.fall", "任务窗口跌倒判定"),
              ("task_window_safety.nonfoot_collision_failure", "非足端碰撞"),
              ("task_window_safety.foot_contact_count", "足端接触次数"),
              ("task_window_safety.foot_impact_count", "足端撞击次数"),
              ("target_distance_final_m", "终点距目标/m"),
              ("elapsed_motion_duration_s", "首个活跃策略周期至终局经过时间/s"),
              ("mode_transitions", "全部模式切换次数"),
              ("transitions_with_valid_sim_time", "具备有效仿真时间的切换次数"),
              ("short_reverse_switch_count", "短时反向切换次数"),
              ("short_reverse_switch_coverage", "短反向统计覆盖"),
              ("agile_cycles", "Agile 周期数"),
              ("recovery_cycles", "Recovery 周期数"),
              ("policy_frequency_hz_overall", "全记录策略频率/Hz"),
              ("rl_task_metric_boundary.elapsed_sim_s", "RL活跃起点至终局经过时间/s"),
              ("rl_task_metric_boundary.valid_continuous_sim_s", "RL活跃窗口有效连续时间/s"),
              ("rl_task_metric_boundary.valid_continuous_path_length_m", "RL活跃窗口有效路径/m"),
              ("rl_task_metric_boundary.mean_valid_continuous_speed_mps", "RL活跃窗口有效平均速度/(m/s)"),
              ("task_valid_time_coverage_fraction", "任务窗口有效仿真时间覆盖率"),
              ("rl_task_metric_boundary.mode_transition_count_all_cycles", "RL活跃窗口模式边沿数"),
              ("risk_to_recovery_response.paired_Agile_to_Recovery_count", "配对风险→Recovery事件数"),
              ("risk_to_recovery_response.median_ns", "风险→模式决定延迟中位数/ns"),
              ("valid_motion_duration_s", "有效连续运动时间/s"),
              ("recovery_sim_duration_s", "有效片段 Recovery 时间/s"),
              ("path_length_m", "有效片段路径/m"),
              ("mean_path_speed_mps", "有效片段平均速度/(m/s)"),
              ("peak_interval_path_speed_mps", "有效区间峰值路径速度/(m/s)")]
    fields.extend([
        ("recovery_share_valid_motion_time", "Recovery/有效连续运动时间占比"),
        ("recovery_hold_audit_status", "Recovery保持审计状态"),
        ("recovery_hold_episodes_complete", "保持审计完整episode数"),
        ("recovery_hold_episodes_incomplete", "保持审计不完整episode数"),
        ("recovery_hold_violations", "保持规则违规数"),
        ("startup_gain_trace_status", "实际启动增益轨迹"),
        ("startup_gain_trace_event_count", "实际增益记录事件数"),
        ("standing_observation_status", "到达后站立观察"),
        ("passive_controller_transition_confirmed", "PASSIVE控制器转换确认"),
        ("process_shutdown_complete", "本次进程退出确认"),
        ("closeout_complete", "收尾记录完整"),
    ])
    lines = [f"# 手动 {'/'.join(groups)} 运行汇总", "",
             f"计划参数：{json.dumps(parameters or {}, ensure_ascii=False, sort_keys=True)}。",
             "预实验准备批次不是正式实验。各组样本数、INVALID、未运行及合法终局单独计数；UNKNOWN 不按零处理。",
             "任务主窗口从首个 rl_entered=1 且 rl_active=1 周期到真实终局（含终止周期）；历史0.10m位移窗口仅作辅助。有效连续时间、路径和速度不冒充完整经过时间。", "",
             "| 指标 | " + " | ".join(groups) + " |", "|---|" + "---|" * len(groups)]
    summary = {"classification": classification or plan.get("classification", "manual diagnostic; not a formal experiment"),
               "parameters": parameters or {}, "groups": {}}
    for group in groups:
        items = rows.get(group, [])
        terminals = {}
        validity = {}
        collision = {}
        fall = {}
        run_status_counts = {}
        for status_item in planned_statuses:
            if status_item.get("group") == group:
                status = status_item.get("status", "UNKNOWN")
                run_status_counts[status] = run_status_counts.get(status, 0) + 1
        for item in items:
            for target, key in ((terminals, "navigation_terminal_result"),
                                (validity, "record_validity"),
                                (collision, "task_window_safety.collision"),
                                (fall, "task_window_safety.fall")):
                value = item
                for part in key.split("."):
                    value = value.get(part) if isinstance(value, dict) else None
                label = "UNKNOWN" if value is None else str(value)
                target[label] = target.get(label, 0) + 1
        summary["groups"][group] = {
            "planned": planned_per_group,
            "started": planned_per_group - run_status_counts.get("NOT_STARTED", 0),
            "not_started": run_status_counts.get("NOT_STARTED", 0),
            "run_status_counts": run_status_counts,
            "valid": sum(item.get("record_validity") == "VALID" for item in items),
            "invalid": sum(item.get("record_validity") == "INVALID" for item in items),
            "validity_unknown": sum(item.get("record_validity") not in ("VALID", "INVALID") for item in items),
            "preparation_ready": sum(
                all((item.get("s6_preparation_readiness") or {}).get(key) is True for key in (
                    "first_active_policy_frame_exactly_persisted", "recorder_ready_before_rl",
                    "pre_rl_standing_confirmed", "pre_task_safety_boundary_live")) for item in items),
            "preparation_not_confirmed": sum(
                not all((item.get("s6_preparation_readiness") or {}).get(key) is True for key in (
                    "first_active_policy_frame_exactly_persisted", "recorder_ready_before_rl",
                    "pre_rl_standing_confirmed", "pre_task_safety_boundary_live")) for item in items),
            "terminal_counts": terminals, "record_validity_counts": validity,
            "task_window_collision_counts": collision, "task_window_fall_counts": fall,
            "run_ids": [item.get("run_id") for item in items],
            "runs": items,
        }
    (root / "batch_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    for key, label in fields:
        row_values = []
        for group in groups:
            values = rows.get(group, [])
            extracted = []
            for item in values:
                value = item
                for part in key.split("."):
                    value = value.get(part) if isinstance(value, dict) else None
                extracted.append(value)
            if len(extracted) == 1:
                text = show(extracted[0])
            elif not extracted:
                text = "未运行"
            else:
                finite = [v for v in extracted if isinstance(v, (int, float)) and math.isfinite(v)]
                text = (f"n={len(extracted)}; mean={sum(finite)/len(finite):.3f} ({len(finite)} known)"
                        if finite else f"n={len(extracted)}; UNKNOWN")
            row_values.append(text)
        lines.append("| " + label + " | " + " | ".join(row_values) + " |")
    lines.extend(["", "每次运行的完整指标在 `batch_summary.json`；各 run 的原始记录、指标和 timeline.png 位于对应组/run_id 子目录。",
                  "ARRIVED 与正常站立收尾分开；任务窗安全 UNKNOWN 不按零处理。INVALID 和未运行状态另行保留。"])
    (root / "comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="dev01.xml")
    parser.add_argument("--goal-x", type=float, default=7.5,
                        help="world-frame goal x; used in overlay, controller validation, run context and analysis")
    parser.add_argument("--goal-y", type=float, default=0.0,
                        help="world-frame goal y; used in overlay, controller validation, run context and analysis")
    parser.add_argument("--sim-limit-s", type=float, default=20.0)
    parser.add_argument("--wall-cap-s", type=float, default=120.0)
    parser.add_argument("--result-root", type=Path, default=None,
                        help="optional unique output root for one resumable M1 map/repeat")
    parser.add_argument("--prepared-overlay", type=Path, default=None,
                        help="reuse a previously detached overlay whose config and model files are revalidated before each run")
    parser.add_argument("--disable-framebuffer-capture", action="store_true",
                        help="suppress optional live/standing PPM writes for storage-bounded batches")
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    default_xauthority = os.environ.get("XAUTHORITY") or os.path.join(
        os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"), "gdm", "Xauthority")
    parser.add_argument("--xauthority", default=default_xauthority)
    parser.add_argument("--plot-python", default=os.environ.get("ABS_PLOT_PYTHON", sys.executable),
                        help="绘图解释器；可用 ABS_PLOT_PYTHON 覆盖")
    parser.add_argument("--classification", default="S6-06 four-group workflow verification; not a formal experiment",
                        help="run classification recorded in each output; not a formal experiment")
    parser.add_argument("--groups", nargs="+", choices=("A", "B", "C", "D"),
                        default=("A", "B", "C", "D"), help="运行组及顺序，默认 A B C D")
    parser.add_argument("--repeats", type=int, default=1,
                        help="每个指定组的有限重复次数（1..20）；每次独立 run_id")
    parser.add_argument("--entry-threshold", type=float, default=-0.05,
                        help="共用进入阈值 E")
    parser.add_argument("--exit-threshold", type=float, default=-0.08,
                        help="B/D 共用退出阈值 X；仅 B/D overlay 写入")
    parser.add_argument("--hold-s", type=float, default=0.30,
                        help="C/D 共用最小保持 H，单位仿真秒；仅 C/D overlay 写入")
    args = parser.parse_args()
    if not Path(args.plot_python).is_file():
        parser.error("绘图 Python 不存在，请用 --plot-python 指定。")
    if (not math.isfinite(args.goal_x) or not math.isfinite(args.goal_y) or
            not math.isfinite(args.sim_limit_s) or args.sim_limit_s <= 0 or
            not math.isfinite(args.wall_cap_s) or args.wall_cap_s <= 0):
        parser.error("goal and runtime limits must be finite; limits must be positive")
    source = PROJECT / "quadruped_ros2_control_humble/install/go2_description"
    batch = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid4().hex[:6]
    root = args.result_root.resolve() if args.result_root else PROJECT / "logs/manual_abcd_compare" / batch
    root.mkdir(parents=True, exist_ok=False)
    groups = tuple(args.groups)
    if len(set(groups)) != len(groups):
        parser.error("--groups cannot repeat a group")
    if not 1 <= args.repeats <= 20:
        parser.error("--repeats must be in [1, 20]")
    try:
        validate_switch_parameters(args.entry_threshold, args.exit_threshold, args.hold_s)
    except ValueError as error:
        parser.error(str(error))
    parameters = {"entry_threshold_E": args.entry_threshold,
                  "exit_threshold_X_BD": args.exit_threshold,
                  "recovery_min_hold_H_CD_sim_s": args.hold_s}
    plan = {"classification": args.classification,
            "scene": args.scene, "goal_world_m": [args.goal_x, args.goal_y], "arrival_radius_m": 0.5,
            "sim_limit_s": args.sim_limit_s, "wall_cap_s": args.wall_cap_s,
            "standing_observation_sim_s": 5.0, "standing_wall_cap_s": 30.0,
            "groups_order": list(groups), "repeats_per_group": args.repeats,
            "parameters": parameters,
            "run_statuses": [{"group": group, "repeat": repeat_index,
                              "status": "NOT_STARTED", "run_id": None}
                             for group in groups for repeat_index in range(1, args.repeats + 1)]}
    (root / "batch_plan.json").write_text(json.dumps(plan, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"本次结果目录：{root}\n计划运行 {'、'.join(groups)}，每组 {args.repeats} 次；每次到达后站立观察5秒。", flush=True)
    print(f"诊断参数：E={args.entry_threshold:g}，X={args.exit_threshold:g}，H={args.hold_s:g}s（按组别只消费适用字段）。", flush=True)
    rows = {}
    try:
        for group in groups:
            overlay = resolve_group_overlay(root, group, args.prepared_overlay)
            if not args.prepared_overlay:
                prepare_overlay(source, overlay, group, args.entry_threshold,
                                args.exit_threshold, args.hold_s)
            set_overlay_goal(overlay / "share/go2_description/config/abs/config.yaml", args.goal_x, args.goal_y)
            detached_files = verify_overlay_detached(source, overlay)
            config_path = overlay / "share/go2_description/config/abs/config.yaml"
            config_text = config_path.read_text(encoding="utf-8")
            if not overlay_configuration_matches(config_text, group, args.entry_threshold,
                                                 args.exit_threshold, args.hold_s, args.goal_x, args.goal_y):
                raise RuntimeError(f"generated {group} overlay failed production configuration validation: {config_path}")
            plan.setdefault("overlay_preflight", {})[group] = {
                "status": "PRODUCTION_CONFIG_VALIDATED",
                "validator": "s1_run_diagnostic.overlay_configuration_matches",
                "config_path": str(config_path),
                "group": group, "entry_threshold_E": args.entry_threshold,
                "exit_threshold_X": args.exit_threshold if group in ("B", "D") else None,
                "hold_H_sim_s": args.hold_s if group in ("C", "D") else None,
                "goal_world_m": [args.goal_x, args.goal_y],
                "overlay_is_independent_tree": True,
                "overlay_reused": bool(args.prepared_overlay),
                "independent_regular_files": detached_files,
            }
            (root / "batch_plan.json").write_text(json.dumps(plan, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        for group in groups:
            rows[group] = []
            for repeat_index in range(1, args.repeats + 1):
                print(f"\n开始 {group} 组第 {repeat_index}/{args.repeats} 次：{args.scene}", flush=True)
                run_root = root / group
                run_root.mkdir(parents=True, exist_ok=True)
                run_out = run_root / f"rep_{repeat_index:02d}"
                run_out.mkdir(parents=True, exist_ok=False)
                status_item = next(item for item in plan["run_statuses"]
                                   if item["group"] == group and item["repeat"] == repeat_index)
                status_item.update({"status": "RUNNING", "output_directory": str(run_out)})
                (root / "batch_plan.json").write_text(json.dumps(plan, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
                command = ["/usr/bin/python3", "-u", str(PROJECT / "scripts/s1_run_diagnostic.py"),
                           "--group", group, "--scene", args.scene, "--goal-x", f"{args.goal_x:.17g}", "--goal-y", f"{args.goal_y:.17g}",
                           "--entry-threshold", f"{args.entry_threshold:.17g}",
                           "--exit-threshold", f"{args.exit_threshold:.17g}",
                           "--hold-s", f"{args.hold_s:.17g}",
                           "--sim-limit-s", f"{args.sim_limit_s:.17g}", "--wall-cap-s", f"{args.wall_cap_s:.17g}",
                           "--standing-observation-sim-s", "5", "--standing-observation-wall-cap-s", "30",
                           "--startup-gain-trace",
                           "--overlay", str(resolve_group_overlay(root, group, args.prepared_overlay)),
                           "--output-root", str(run_out), "--display", args.display,
                           "--xauthority", args.xauthority,
                           "--classification", args.classification]
                if args.disable_framebuffer_capture:
                    command.append("--disable-framebuffer-capture")
                try:
                    result = execute(command, root / f"{group}_rep_{repeat_index:02d}_console.log")
                except Exception as error:
                    status_item.update({"status": "RUNNER_FAILED_OR_UNKNOWN", "reason": str(error)})
                    (root / "batch_plan.json").write_text(json.dumps(plan, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
                    raise
                facts = (result or {}).get("process_facts", {})
                if not result or not facts.get("shutdown_complete") or not facts.get("closeout_complete"):
                    status_item.update({"status": "RUNNER_FAILED_OR_CLEANUP_UNCONFIRMED",
                                        "run_id": (result or {}).get("run_id"),
                                        "result": (result or {}).get("result"),
                                        "shutdown_complete": facts.get("shutdown_complete"),
                                        "closeout_complete": facts.get("closeout_complete")})
                    (root / "batch_plan.json").write_text(json.dumps(plan, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
                    raise RuntimeError(f"{group} 组未确认完整收尾，不再启动下一次。")
                analysis_root = run_out / "analysis"
                run_directory = run_out / result["run_id"]
                runtime_record = run_directory / "runtime_record.jsonl"
                status_item.update({"status": "FINISHED", "run_id": result.get("run_id"),
                                    "result": result.get("result"),
                                    "output_directory": str(run_directory)})
                (root / "batch_plan.json").write_text(json.dumps(plan, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
                if runtime_record.is_file():
                    plot_env = os.environ.copy()
                    # The workstation user site may provide NumPy 2 while the
                    # system Matplotlib extension was built against NumPy 1.
                    # Keep this compatibility choice local to analysis only.
                    plot_env["PYTHONNOUSERSITE"] = "1"
                    plot_env.setdefault("MPLCONFIGDIR", "/tmp/manual_ab_compare_mpl")
                    execute([args.plot_python, str(PROJECT / "scripts/s1_analyze_run.py"),
                             "--runs-root", str(run_out), "--output-dir", str(analysis_root),
                             "--task-label", f"S6-06-{group}-rep-{repeat_index:02d}"],
                            root / f"{group}_rep_{repeat_index:02d}_analysis.log", env=plot_env)
                    metrics = run_directory / "metrics.json"
                    metric = json.loads(metrics.read_text(encoding="utf-8"))
                else:
                    context_path = run_directory / "run_context.json"
                    context = json.loads(context_path.read_text(encoding="utf-8")) if context_path.is_file() else {}
                    metric = summarize_startup_without_runtime_record(result, context, args.scene)
                    metric.update({"process_shutdown_complete": facts.get("shutdown_complete"),
                                   "closeout_complete": facts.get("closeout_complete")})
                    run_directory.mkdir(parents=True, exist_ok=True)
                    (run_directory / "metrics.json").write_text(
                        json.dumps(metric, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
                metric = enrich_run_metrics(metric, run_directory, facts)
                metric["supervisor_terminal_result"] = (result.get("result") or
                    metric.get("run_terminal_result_context") or "UNKNOWN")
                metric["supervisor_terminal_source"] = "s1_run_diagnostic stdout result, cross-checked with run_context/terminal when available"
                (run_directory / "metrics.json").write_text(
                    json.dumps(metric, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
                rows[group].append(metric)
                status_item["record_validity"] = metric.get("record_validity")
                (root / "batch_plan.json").write_text(json.dumps(plan, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
                save_comparison(root, rows, groups, parameters, args.classification)
                print(f"{group}组第 {repeat_index} 次结束：{result.get('result')}；本次运行进程已由诊断器确认退出。", flush=True)
                if result.get("startup_error") or metric.get("record_validity") != "VALID":
                    raise RuntimeError(f"{group} 组记录无效，停止后续运行；保留原始记录。")
                if metric.get("s6_preparation_readiness", {}).get("first_active_policy_frame_exactly_persisted") is not True:
                    raise RuntimeError(f"{group} 首个活跃策略周期未与 companion 精确配对持久化，停止后续组；保留原始记录。")
    except KeyboardInterrupt:
        save_comparison(root, rows, groups, parameters, args.classification)
        print(f"对比已中断，已有记录保留：{root}")
        return 130
    except Exception as error:
        save_comparison(root, rows, groups, parameters, args.classification)
        print(f"对比停止：{error}\n已有记录保留：{root}", file=sys.stderr)
        return 1
    print(f"\n{'/'.join(groups)} 批次诊断及绘图已完成。\n对照表：{root / 'comparison.md'}\n机器可读汇总：{root / 'batch_summary.json'}")
    print("这是预实验准备核验，不用于成功率或性能结论；本脚本不会启动正式实验。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
