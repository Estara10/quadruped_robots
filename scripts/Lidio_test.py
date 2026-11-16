#!/usr/bin/env python3
"""Interactive, visible, single-run Go2 simulation for manual A/B/C/D viewing."""

from datetime import datetime
from pathlib import Path
import json
import os
import re
import shutil
import sys
import tempfile
from uuid import uuid4

import manual_ab_compare as manual
from abs_scene import resolve_scene


# Change this switch in this file to keep or discard each completed viewing run.
SAVE_RUN_RECORDS = False

# Manual viewing settings. A/C use the single threshold; B/D use X; C/D use H.
ENTRY_THRESHOLD = -0.05
EXIT_THRESHOLD = -0.07
HOLD_S = 0.02
PERIODIC_MODE = "Periodic"
PERIODIC_PERIOD_S = 0.06  # Fixed in the controller implementation.
GOAL_X, GOAL_Y = 7.5, 0.0
SIM_LIMIT_S, WALL_CAP_S = 20.0, 120.0
STANDING_OBSERVATION_S = 1.0
DEFAULT_GROUP = "A"

PROJECT = Path(__file__).resolve().parents[1]
INSTALL = PROJECT / "quadruped_ros2_control_humble/install/go2_description"
RECORD_ROOT = PROJECT / "logs/manual_visual_abcd"
DISPLAY = os.environ.get("DISPLAY", ":0")
XAUTHORITY = os.environ.get("XAUTHORITY") or os.path.join(
    os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"), "gdm", "Xauthority")
NORMAL_TERMINALS = {
    "ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
    "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE",
}


def choose_group():
    print("\n选择要观看的组：")
    print(f"A：单阈值 E={ENTRY_THRESHOLD:g}")
    print(f"B：双阈值 E={ENTRY_THRESHOLD:g}, X={EXIT_THRESHOLD:g}")
    print(f"C：最小保持 E={ENTRY_THRESHOLD:g}, H={HOLD_S:g}s")
    print(f"D：组合 E={ENTRY_THRESHOLD:g}, X={EXIT_THRESHOLD:g}, H={HOLD_S:g}s")
    while True:
        value = input(f"输入A/B/C/D，回车选{DEFAULT_GROUP}，q退出： ").strip().upper()
        if value in ("Q", "QUIT", "EXIT"):
            return None
        group = {"": DEFAULT_GROUP, "A": "A", "B": "B", "C": "C", "D": "D",
                 "1": "A", "2": "B", "3": "C", "4": "D"}.get(value)
        if group:
            return group
        print("请输入A、B、C、D或1、2、3、4。")


def choose_scene():
    while True:
        value = input("输入地图编号文件名（如M1_01.xml、M2_20.xml、M3_01.xml或R1_01.xml；旧名仍兼容），q退出： ").strip()
        if value.lower() in ("q", "quit", "exit"):
            return None
        if not value:
            continue
        filename = value if value.endswith(".xml") else value + ".xml"
        try:
            return resolve_scene(filename)
        except (OSError, ValueError) as error:
            print(f"地图不能用于当前仿真入口：{error}")


def periodic_overlay_config(config_path):
    config = config_path.read_text(encoding="utf-8")
    block = f'  twist_update:\n    mode: "{PERIODIC_MODE}"\n'
    if re.search(r"(?m)^  twist_update:", config):
        config = re.sub(r"(?m)^  twist_update:[^\n]*\n(?:^    [^\n]*\n)*",
                        lambda _: block, config)
    else:
        config = config.replace("  switching:\n", block + "\n  switching:\n", 1)
    config_path.write_text(config, encoding="utf-8")


def closeout_confirmed(result, temporary):
    if result:
        facts = result.get("process_facts", {})
        if facts.get("shutdown_complete") is True and facts.get("closeout_complete") is True:
            return True
    facts_files = list((temporary / "runs").glob("*/process_facts.json"))
    if len(facts_files) == 1:
        try:
            facts = json.loads(facts_files[0].read_text(encoding="utf-8"))
            return facts.get("shutdown_complete") is True and facts.get("closeout_complete") is True
        except (OSError, ValueError):
            return False
    return False


def save_run(temporary, scene, group, result):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_label = f"{stamp}_{scene.scene_id}_{group}_{uuid4().hex[:6]}"
    destination = RECORD_ROOT / run_label
    destination.mkdir(parents=True, exist_ok=False)
    runs = temporary / "runs"
    if runs.exists():
        shutil.move(str(runs), str(destination / "runs"))
    console = temporary / "console.log"
    if console.is_file():
        shutil.copy2(console, destination / "console.log")
    settings = {
        "purpose": "manual visual inspection; not a formal experiment",
        "scene": scene.filename,
        "scene_id": scene.scene_id,
        "group": group,
        "entry_threshold": ENTRY_THRESHOLD,
        "exit_threshold": EXIT_THRESHOLD if group in ("B", "D") else ENTRY_THRESHOLD,
        "hold_s": HOLD_S if group in ("C", "D") else 0.0,
        "ray_source": "geometric",
        "twist_update": PERIODIC_MODE,
        "twist_update_period_s": PERIODIC_PERIOD_S,
        "goal_m": [GOAL_X, GOAL_Y],
        "sim_limit_s": SIM_LIMIT_S,
        "wall_cap_s": WALL_CAP_S,
        "standing_observation_s": STANDING_OBSERVATION_S,
        "result": (result or {}).get("result", "UNKNOWN"),
    }
    (destination / "manual_settings.json").write_text(
        json.dumps(settings, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return destination


def run_once(scene, group):
    temporary = Path(tempfile.mkdtemp(prefix="manual-visual-abcd-"))
    result = None
    dispatched = False
    completed = False
    try:
        overlay = temporary / "overlay"
        manual.prepare_overlay(INSTALL, overlay, group, ENTRY_THRESHOLD, EXIT_THRESHOLD, HOLD_S)
        config = overlay / "share/go2_description/config/abs/config.yaml"
        manual.set_overlay_goal(config, GOAL_X, GOAL_Y)
        periodic_overlay_config(config)

        command = [
            "/usr/bin/python3", "-u", str(PROJECT / "scripts/s1_run_diagnostic.py"),
            "--scene", scene.filename, "--group", group,
            "--entry-threshold", str(ENTRY_THRESHOLD), "--exit-threshold", str(EXIT_THRESHOLD),
            "--hold-s", str(HOLD_S), "--goal-x", str(GOAL_X), "--goal-y", str(GOAL_Y),
            "--sim-limit-s", str(SIM_LIMIT_S), "--wall-cap-s", str(WALL_CAP_S),
            "--standing-observation-sim-s", str(STANDING_OBSERVATION_S),
            "--overlay", str(overlay), "--output-root", str(temporary / "runs"),
            "--display", DISPLAY, "--xauthority", XAUTHORITY,
            "--disable-framebuffer-capture", "--no-rviz", "--render-backend", "glfw",
            "--ray-source", "geometric",
            "--classification", "manual visual inspection; not a formal experiment",
        ]
        print(f"\n运行 {scene.filename} / {group}组；几何射线，Periodic P={PERIODIC_PERIOD_S:.2f}s。"
              "MuJoCo可见，RViz关闭。没有自动重跑。", flush=True)
        dispatched = True
        result = manual.execute(command, temporary / "console.log")
        outcome = result.get("result", "UNKNOWN")
        summary = result.get("summary", {})
        print(f"\n本次结果：{outcome}；距目标：{summary.get('distance_to_goal_m', 'UNKNOWN')} m。",
              flush=True)
        completed = outcome in NORMAL_TERMINALS
    except KeyboardInterrupt:
        print("\n已中断本次观看；诊断器正在执行正常收尾。", flush=True)
    except Exception as error:
        print(f"运行未完成：{error}", flush=True)
    finally:
        outcome = (result or {}).get("result", "")
        preflight_rejected = (
            outcome == "SCENE_PREFLIGHT_REJECTED" or
            outcome.endswith("_SCENE_SETTINGS_REJECTED")
        )
        clean = not dispatched or preflight_rejected or closeout_confirmed(result, temporary)
        if SAVE_RUN_RECORDS and dispatched and clean:
            saved = save_run(temporary, scene, group, result)
            shutil.rmtree(temporary, ignore_errors=True)
            print(f"本次记录保存在：{saved}", flush=True)
        elif clean:
            shutil.rmtree(temporary)
            if preflight_rejected:
                print("地图预检拒绝，未启动场景；本次临时记录已清理。", flush=True)
            else:
                print("本次临时记录已清理；仿真收尾已确认。", flush=True)
        else:
            print(f"未能确认进程完整收尾；资料保留在：{temporary}", flush=True)
    return clean and completed


def main():
    print("手动可视化测试：复用完整仿真诊断流程，仅用于观看，不构成正式实验。")
    print(f"结果记录开关 SAVE_RUN_RECORDS={SAVE_RUN_RECORDS}；地图与组别每次运行前选择。")
    print("需要可用的桌面DISPLAY/XAUTHORITY；本脚本使用可见GLFW窗口。")
    while True:
        try:
            group = choose_group()
            if group is None:
                break
            scene = choose_scene()
            if scene is None:
                break
            if not run_once(scene, group):
                print("本次未以正常任务终局并完成收尾；停止继续派发。", flush=True)
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\n已退出手动观看。")
            return 0
    print("手动观看结束。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
