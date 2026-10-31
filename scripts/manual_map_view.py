#!/usr/bin/env python3
"""交互式手动看图；设置集中在文件顶部，复用现有诊断器与清理逻辑。"""
from pathlib import Path
import argparse
import json
import os
import shutil
import sys
import tempfile

import manual_ab_compare as manual


# 以后修改手动测试设置，只改这里。A 不消费退出阈值和保持时间。
GROUP = "A"
ENTRY_THRESHOLD = -0.05
EXIT_THRESHOLD = -0.10
HOLD_S = 0.20
GOAL_X, GOAL_Y = 7.5, 0.0
SIM_LIMIT_S, WALL_CAP_S = 20, 120
STANDING_S = 2
DISPLAY = os.environ.get("DISPLAY") or ":0"
XAUTHORITY = os.environ.get("XAUTHORITY") or "/run/user/1000/gdm/Xauthority"

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "unitree_mujoco/unitree_robots/go2"


def scene_name(value):
    value = value.strip()
    if value.isdecimal():
        number = int(value)
        if not 1 <= number <= 30:
            raise ValueError("M3地图编号范围是1–30。")
        value = f"challenge{number:02d}.xml"
    elif not value.endswith(".xml"):
        value += ".xml"
    if Path(value).name != value or not (SCENES / value).is_file():
        raise ValueError("地图不存在。请输入1–30，或地图文件名，例如m02.xml。")
    return value


def cleanup_confirmed(result, temporary):
    if result:
        facts = result.get("process_facts", {})
        return facts.get("shutdown_complete") is True and facts.get("closeout_complete") is True
    # Ctrl+C后，execute会等待诊断器退出；从其落盘结果确认收尾。
    facts_files = list((temporary / "runs").glob("*/process_facts.json"))
    if len(facts_files) == 1:
        try:
            facts = json.loads(facts_files[0].read_text())
            return facts.get("shutdown_complete") is True and facts.get("closeout_complete") is True
        except (OSError, ValueError):
            pass
    return False


def view(scene, group=GROUP):
    temporary = Path(tempfile.mkdtemp(prefix="go2-map-view-"))
    result = None
    dispatched = False
    interrupted = False
    try:
        overlay = temporary / "overlay"
        manual.prepare_overlay(ROOT / "quadruped_ros2_control_humble/install/go2_description",
                               overlay, group, ENTRY_THRESHOLD, EXIT_THRESHOLD, HOLD_S)
        manual.set_overlay_goal(overlay / "share/go2_description/config/abs/config.yaml", GOAL_X, GOAL_Y)
        command = ["/usr/bin/python3", "-u", str(ROOT / "scripts/s1_run_diagnostic.py"),
                   "--scene", scene, "--group", group,
                   "--entry-threshold", str(ENTRY_THRESHOLD), "--exit-threshold", str(EXIT_THRESHOLD),
                   "--hold-s", str(HOLD_S), "--goal-x", str(GOAL_X), "--goal-y", str(GOAL_Y),
                   "--sim-limit-s", str(SIM_LIMIT_S), "--wall-cap-s", str(WALL_CAP_S),
                   "--standing-observation-sim-s", str(STANDING_S),
                   "--overlay", str(overlay), "--output-root", str(temporary / "runs"),
                   "--display", DISPLAY, "--xauthority", XAUTHORITY,
                   "--disable-framebuffer-capture",
                   "--classification", "manual visual inspection; temporary data; not a formal experiment"]
        print(f"\n打开 {scene}，{group}组；显示视窗、不截图、不做事后绘图。", flush=True)
        dispatched = True
        result = manual.execute(command, temporary / "console.log")
    except KeyboardInterrupt:
        interrupted = True
    except Exception as error:
        print(f"运行未完成：{error}", flush=True)
    finally:
        clean = not dispatched or cleanup_confirmed(result, temporary)
        if clean:
            shutil.rmtree(temporary)
            print("本次临时数据已删除。", flush=True)
        else:
            print(f"尚未确认完整收尾，停止继续选图；临时资料保留在：{temporary}", flush=True)
    return clean and not interrupted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--group", type=str.upper, choices=("A", "B", "C", "D"),
                        default=GROUP, help="运行组别，默认A")
    args = parser.parse_args()
    print(f"手动看图：当前{args.group}组，M3输入1–30；其他地图输入文件名。输入q退出。")
    print(f"E={ENTRY_THRESHOLD:g}；X={EXIT_THRESHOLD if args.group in ('B', 'D') else ENTRY_THRESHOLD:g}；"
          f"H={HOLD_S if args.group in ('C', 'D') else 0:g}仿真秒。")
    while True:
        try:
            value = input("\n运行哪张地图？ ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已退出。")
            return 0
        if value.lower() in ("q", "quit", "exit"):
            return 0
        if not value:
            continue
        try:
            scene = scene_name(value)
        except ValueError as error:
            print(error)
            continue
        if not view(scene, args.group):
            return 1


if __name__ == "__main__":
    sys.exit(main())
