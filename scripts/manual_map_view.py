#!/usr/bin/env python3
"""默认演示固定三张地图，逐图回车开始；--interactive才手动输入地图。"""
from pathlib import Path
import argparse
import json
import os
import re
import shutil
import sys
import tempfile

import manual_ab_compare as manual
from abs_scene import canonical_scene_filename


# 以后修改手动测试设置，只改这里。A 不消费退出阈值和保持时间。
GROUP = "A"
ENTRY_THRESHOLD = -0.05
EXIT_THRESHOLD = -0.07
HOLD_S = 0.02
GOAL_X, GOAL_Y = 7.5, 0.0
SIM_LIMIT_S, WALL_CAP_S = 20, 120
STANDING_S = 2
DEMO_MAPS = ("M2_21.xml", "M2_24.xml", "M2_27.xml")
RAY_MODEL = Path(__file__).resolve().parents[1] / "archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt"
RAY_PYTHON = "/home/lidio/anaconda3/envs/abs/bin/python"
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
        value = f"M3_{number:02d}.xml"
    elif not value.endswith(".xml"):
        value += ".xml"
    value = canonical_scene_filename(value)
    if Path(value).name != value or not (SCENES / value).is_file():
        raise ValueError("地图不存在。请输入1–30，或新编号文件名，例如M3_01.xml；旧文件名仍可用。")
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


def view(scene, group=GROUP, ray_source="geometric"):
    temporary = Path(tempfile.mkdtemp(prefix="go2-map-view-"))
    result = None
    dispatched = False
    interrupted = False
    completed = False
    try:
        overlay = temporary / "overlay"
        manual.prepare_overlay(ROOT / "quadruped_ros2_control_humble/install/go2_description",
                               overlay, group, ENTRY_THRESHOLD, EXIT_THRESHOLD, HOLD_S)
        config = overlay / "share/go2_description/config/abs/config.yaml"
        manual.set_overlay_goal(config, GOAL_X, GOAL_Y)
        text = config.read_text(encoding="utf-8")
        periodic = '  twist_update:\n    mode: "Periodic"\n'
        if re.search(r"(?m)^  twist_update:", text):
            text = re.sub(r"(?m)^  twist_update:[^\n]*\n(?:^    [^\n]*\n)*",
                          lambda _: periodic, text)
        else:
            text = text.replace("  switching:\n", periodic + "\n  switching:\n", 1)
        config.write_text(text, encoding="utf-8")
        command = ["/usr/bin/python3", "-u", str(ROOT / "scripts/s1_run_diagnostic.py"),
                   "--scene", scene, "--group", group,
                   "--entry-threshold", str(ENTRY_THRESHOLD), "--exit-threshold", str(EXIT_THRESHOLD),
                   "--hold-s", str(HOLD_S), "--goal-x", str(GOAL_X), "--goal-y", str(GOAL_Y),
                   "--sim-limit-s", str(SIM_LIMIT_S), "--wall-cap-s", str(WALL_CAP_S),
                   "--standing-observation-sim-s", str(STANDING_S),
                   "--overlay", str(overlay), "--output-root", str(temporary / "runs"),
                   "--display", DISPLAY, "--xauthority", XAUTHORITY,
                   "--disable-framebuffer-capture", "--no-rviz", "--render-backend", "glfw",
                   "--ray-source", ray_source,
                   "--classification", "manual visual inspection; temporary data; not a formal experiment"]
        if ray_source == "prediction":
            command.extend(["--ray-model", str(RAY_MODEL), "--ray-python", RAY_PYTHON,
                            "--ray-device", "cuda"])
        print(f"\n打开 {scene}，{group}组，Periodic P=0.06s，射线={ray_source}；"
              "显示MuJoCo窗口，RViz关闭。Ctrl+C结束本轮演示。", flush=True)
        dispatched = True
        result = manual.execute(command, temporary / "console.log")
        completed = result.get("result") in {
            "ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION", "DIAGNOSTIC_SIM_TIME_LIMIT"}
        summary = result.get("summary", {})
        print(f"\n本图结果：{result.get('result')}；距目标：{summary.get('distance_to_goal_m')} m。",
              flush=True)
    except KeyboardInterrupt:
        interrupted = True
    except Exception as error:
        print(f"运行未完成：{error}", flush=True)
    finally:
        clean = not dispatched or cleanup_confirmed(result, temporary)
        if clean and (not dispatched or (result or {}).get("result") == "ARRIVED"):
            shutil.rmtree(temporary)
            print("本次临时数据已删除。", flush=True)
        elif clean:
            print(f"本图未到达或运行中断，临时记录保留在：{temporary}", flush=True)
        else:
            print(f"尚未确认完整收尾，停止继续选图；临时资料保留在：{temporary}", flush=True)
    return clean and completed and not interrupted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--group", type=str.upper, choices=("A", "B", "C", "D"),
                        default=None, help="直接指定组别；未指定时显示A/B/C选择菜单")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--demo", dest="demo", action="store_true", help="三图演示（默认），每张回车开始")
    mode.add_argument("--interactive", dest="demo", action="store_false", help="手动输入地图名")
    parser.set_defaults(demo=True)
    parser.add_argument("--maps", nargs=3, metavar="MAP", default=DEMO_MAPS,
                        help="三图演示地图，默认M2_21.xml M2_24.xml M2_27.xml；旧文件名仍兼容")
    parser.add_argument("--ray-source", choices=("geometric", "prediction"), default="geometric",
                        help="默认几何射线；prediction使用现存预测模型和CUDA")
    args = parser.parse_args()
    if args.ray_source == "prediction" and (not RAY_MODEL.is_file() or not Path(RAY_PYTHON).is_file()):
        parser.error("预测模型或预测Python不存在，请修改文件顶部的RAY_MODEL/RAY_PYTHON。")
    if args.group is None:
        print("\n请选择演示组别：")
        print(f"  1 / A：单阈值，E=X={ENTRY_THRESHOLD:g}，H=0")
        print(f"  2 / B：双阈值，E={ENTRY_THRESHOLD:g}，X={EXIT_THRESHOLD:g}，H=0")
        print(f"  3 / C：单阈值＋保持，E=X={ENTRY_THRESHOLD:g}，H={HOLD_S:g}秒")
        while args.group is None:
            try:
                choice = input(f"输入1/2/3或A/B/C，回车默认{GROUP}，q退出： ").strip().upper()
            except (EOFError, KeyboardInterrupt):
                print("\n已退出，未启动仿真。")
                return 0
            if choice in ("Q", "QUIT", "EXIT"):
                return 0
            args.group = {"": GROUP, "1": "A", "2": "B", "3": "C",
                          "A": "A", "B": "B", "C": "C"}.get(choice)
            if args.group is None:
                print("请输入A、B、C或对应数字。")
    print(f"三图录屏演示：当前{args.group}组。" if args.demo else
          f"手动看图：当前{args.group}组，M3输入1–30；其他地图输入文件名。输入q退出。")
    print(f"E={ENTRY_THRESHOLD:g}；X={EXIT_THRESHOLD if args.group in ('B', 'D') else ENTRY_THRESHOLD:g}；"
          f"H={HOLD_S if args.group in ('C', 'D') else 0:g}仿真秒。")
    print(f"Twist更新：Periodic P=0.06s；射线：{args.ray_source}；"
          "窗口需要可用的桌面DISPLAY/XAUTHORITY，EGL隐藏模式不能用于屏幕录制。")
    if args.demo:
        try:
            scenes = [scene_name(name) for name in args.maps]
        except ValueError as error:
            parser.error(str(error))
        print("\n本轮仅作演示，不保证每次到达，碰撞和超时不自动重跑。")
        for index, scene in enumerate(scenes, 1):
            try:
                answer = input(f"\n[{index}/3] {scene}：准备好录屏后按回车开始；s跳过，q退出： ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print("\n演示已结束。")
                return 0
            if answer in ("q", "quit", "exit"):
                return 0
            if answer == "s":
                continue
            if answer:
                print("未输入回车确认，本轮结束；没有启动该地图。")
                return 0
            if not view(scene, args.group, args.ray_source):
                return 1
        print("\n三图演示已结束。")
        return 0
    while True:
        try:
            value = input("\n运行哪张地图？输入M1/M2/M3/R1新编号文件名（旧名仍兼容）： ").strip()
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
        if not view(scene, args.group, args.ray_source):
            return 1


if __name__ == "__main__":
    sys.exit(main())
