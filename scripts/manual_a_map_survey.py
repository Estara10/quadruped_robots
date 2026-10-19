#!/usr/bin/env python3
"""Run A once on each of the twelve new S6-11 development maps."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "docs/thesis_project/designs/S6-11/scene_manifest_s6_11.json"
STRUCTURES = ("short_local_bypass", "forward_offset_short_obstacles")
DENSITIES = ("sparse", "medium", "dense")


def selected_scenes():
    doc = json.loads(MANIFEST.read_text(encoding="utf-8"))
    scenes = [s for s in doc["active_scenes"]
              if s["split"] == "development" and s["route_structure"] in STRUCTURES]
    expected = {(d, r): 2 for d in DENSITIES for r in STRUCTURES}
    observed = {key: sum((s["density"], s["route_structure"]) == key for s in scenes)
                for key in expected}
    if (len(scenes) != 12 or observed != expected or
            len({s["scene_id"] for s in scenes}) != 12 or
            len({s["filename"] for s in scenes}) != 12):
        raise ValueError("清单不是预期的12张独立开发图，停止；没有启动仿真。")
    for scene in scenes:
        name = scene["filename"]
        if (Path(name).name != name or not name.startswith("scene_random_s6_11_")
                or not (ROOT / "unitree_mujoco/unitree_robots/go2" / name).is_file()):
            raise ValueError(f"场景文件缺失或不属于本轮：{name}")
    return sorted(scenes, key=lambda s: (DENSITIES.index(s["density"]),
                                        STRUCTURES.index(s["route_structure"]), s["filename"]))


def save_index(root, doc):
    (root / "survey.json").write_text(json.dumps(doc, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = ["# A组12张开发地图手动观察", "", f"状态：{doc['status']}", "",
             "每图一次；流程完成不等于到达成功。碰撞、跌倒和超时保留；未知不当作零。", "",
             "| 序号 | 地图 | 状态 | 运行结果目录 |", "|---|---|---|---|"]
    for i, item in enumerate(doc["scenes"], 1):
        batch = item.get("batch_directory")
        target = f"[结果]({batch}/comparison.md)" if batch else "尚无"
        lines.append(f"| {i} | {item['filename']} | {item['status']} | {target} |")
    (root / "survey.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="A组遍历12张新短障碍开发图，排除留出集和家具。")
    parser.add_argument("--list", action="store_true", help="只列出地图，不创建日志或启动仿真")
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    parser.add_argument("--xauthority", default=os.environ.get("XAUTHORITY", "/run/user/1000/gdm/Xauthority"))
    parser.add_argument("--plot-python", default="/home/lidio/anaconda3/bin/python3")
    args = parser.parse_args()
    scenes = selected_scenes()
    print("本轮12张：两种短障碍结构 × 三种密度 × 每种两张；仅开发集。", flush=True)
    for i, scene in enumerate(scenes, 1):
        print(f"{i:02d}. {scene['filename']}", flush=True)
    if args.list:
        return 0
    if not Path(args.plot_python).is_file():
        parser.error("绘图Python不存在，请指定 --plot-python")
    root = ROOT / "logs/manual_a_map_survey" / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid4().hex[:6])
    root.mkdir(parents=True)
    doc = dict(status="RUNNING", group="A", entry_threshold=-.05,
               classification="manual development map viewing; not a formal experiment",
               manifest=str(MANIFEST), scenes=[dict(s, status="NOT_STARTED") for s in scenes])
    save_index(root, doc)
    print(f"\n本轮索引：{root / 'survey.md'}", flush=True)
    for i, item in enumerate(doc["scenes"], 1):
        print(f"\n===== 地图 {i}/12：{item['filename']} =====", flush=True)
        item["status"] = "RUNNING"
        save_index(root, doc)
        command = ["/usr/bin/python3", "-u", str(ROOT / "scripts/manual_ab_compare.py"),
                   "--groups", "A", "--repeats", "1", "--scene", item["filename"],
                   "--entry-threshold", "-0.05", "--plot-python", args.plot_python,
                   "--display", args.display, "--xauthority", args.xauthority,
                   "--classification", doc["classification"]]
        proc = None
        try:
            with (root / f"{i:02d}_{item['scene_id']}.log").open("w", encoding="utf-8") as log:
                proc = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, bufsize=1)
                for line in proc.stdout:
                    print(line, end="", flush=True)
                    log.write(line)
                    log.flush()
                    if line.startswith("本次结果目录："):
                        item["batch_directory"] = line.split("：", 1)[1].strip()
                        save_index(root, doc)
                code = proc.wait()
        except KeyboardInterrupt:
            # Terminal SIGINT reaches the child too; its supervisor owns simulator cleanup.
            if proc is not None and proc.poll() is None:
                try:
                    proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    item["cleanup_status"] = "UNKNOWN; child still running; no further scene launched"
                    print("子运行尚未退出，清理状态未知；不会启动下一张，请保留终端检查。", flush=True)
            item["status"] = "INTERRUPTED"
            doc["status"] = "INTERRUPTED"
            save_index(root, doc)
            return 130
        except Exception as error:
            item["status"] = "WRAPPER_ERROR"
            item["error"] = str(error)
            doc["status"] = "STOPPED"
            save_index(root, doc)
            raise
        item["return_code"] = code
        item["status"] = "WORKFLOW_COMPLETED" if code == 0 else "STOPPED_ERROR_OR_INVALID"
        if code != 0:
            doc["status"] = "STOPPED"
            save_index(root, doc)
            print(f"\n本张发生故障/无效记录，后续未运行。索引：{root / 'survey.md'}", flush=True)
            return code if code > 0 else 1
        save_index(root, doc)
    doc["status"] = "COMPLETED"
    save_index(root, doc)
    print(f"\n12张流程完成。结果索引：{root / 'survey.md'}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
