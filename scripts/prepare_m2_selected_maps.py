#!/usr/bin/env python3
"""Prepare the approved 18-map M2 development selection; never run policies."""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import re

from PIL import Image, ImageDraw
import generate_m2_maps as g

ROOT = g.ROOT
OUT = ROOT / "docs/thesis_project/designs/M2/selected_v2"
M1_DOC = ROOT / "docs/thesis_project/designs/M1/library_v1/layouts.json"
M2_DOC = g.OUT / "layouts.json"
VERSION = "m2-selected-development/2.1"
# Historical selection aliases are retained in provenance; current filenames use difficulty.
COPIES = [
    (7, 3, "high_candidate", "两档最近3到达1碰撞；旧400次也有碰撞和超时"),
    (8, 4, "high_candidate", "两档最近3到达1碰撞，终局有差异"),
    (9, 8, "high_candidate", "两档最近3到达1碰撞，终局有差异"),
    (10, 11, "high_candidate", "两档最近2到达2碰撞；旧实验有较多超时"),
    (11, 19, "high_candidate", "两档最近3到达1碰撞，Recovery占用较高"),
    (12, 24, "high_candidate", "两档最近3到达1碰撞，终局有差异"),
    (13, 13, "medium_switching", "最近4次均到达，但较宽滞回的短反向更多"),
    (14, 17, "medium_switching", "最近4次均到达、切换频繁；历史有跌倒记录"),
    (15, 26, "medium_switching", "最近4次均到达，两档短反向频率差异明显"),
    (16, 29, "medium_switching", "最近4次均到达、切换频繁；旧实验有碰撞/超时"),
    (17, 1, "low_reference", "低风险参考，最近两档均无Recovery"),
    (18, 12, "low_reference", "中等几何密度下的低风险参考"),
    (19, 22, "low_reference", "较密布局下的低风险参考，有一定Recovery活动"),
]
KEPT = {
    2: ("high_candidate", "用户指定difficult；旧M2第二轮23/24到达，切换活动较多"),
    3: ("high_candidate", "旧M2第二轮17到达7碰撞；最新两档有超时差异"),
    4: ("challenge_unverified", "采用用户修改后的r5；旧几何成绩不用于新图风险判定"),
    5: ("high_candidate", "旧M2第二轮9到达13碰撞2超时"),
    6: ("medium_switching", "旧M2第二轮24/24到达，但切换活动较多"),
}
NAMES = {
    "m2dev03.xml": ("d01", "difficult"),
    "m2dev05.xml": ("d02", "difficult"),
    "m2dev02.xml": ("d03", "difficult"),
    **{f"m2dev{n:02d}.xml": (f"d{n-3:02d}", "difficult") for n in range(7, 13)},
    "m2dev04.xml": ("m01", "middle"),
    "m2dev06.xml": ("m02", "middle"),
    **{f"m2dev{n:02d}.xml": (f"m{n-10:02d}", "middle") for n in range(13, 17)},
    **{f"m2dev{n:02d}.xml": (f"e{n-16:02d}", "easy") for n in range(17, 20)},
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_index(doc):
    keys = ("filename", "scene_id", "difficulty_label", "previous_m2_filename", "source_set", "source_filename", "selection_group",
            "layout_revision", "source_xml_sha256", "selection_reason")
    with (OUT / "scene_index.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys, lineterminator="\n")
        writer.writeheader()
        for s in doc["scenes"]:
            writer.writerow({k: s.get(k, "") for k in keys})
    lines = ["# 新版M2：18张开发地图", "", "当前入口：e01–e03（easy）、m01–m06（middle）、d01–d09（difficult），均为.xml文件。",
             "M1源地图未移动或修改；旧M2别名保留历史，不与新名重复计入实验。难度是开发标签，不是风险概率或已证实的因果关系。", "",
             "| 当前地图 | 旧M2名 | 来源 | 难度 | 理由 |", "|---|---|---|---|---|"]
    for s in doc["scenes"]:
        lines.append(f"| [{s['filename']}]({s['preview']}) | {s['previous_m2_filename']} | {s['source_set']} {s['source_filename']} | {s['difficulty_label']} | {s['selection_reason']} |")
    lines += ["", "构成：difficult 9张、middle 6张、easy 3张；用户明确原M2 03、05、02作为d01、d02、d03。",
              "m01是修改后的旧M2 dev04，暂归middle但仍待实测；M1 dev15/dev30作为压力备选保留，不纳入此18图。",
              "", "新版dev04为r5，与旧实验不同；01和08障碍间距约0.514m，低于原0.60m候选规则，参考路径净距约0.764m。",
              "复制到M2不产生独立地图样本：后续按源地图/几何区组统计，不能将M1与对应M2别名当成两张独立地图。",
              "没有使用留出集；没有启动216次扫描。今后阈值、筛选规则、顺序和重试规则需在运行前单独固定。",
              "", f"生产登记状态：{doc['registration_status']}。静态加载与构建不等于策略可达性验证。", ""]
    (OUT / "MAP_INDEX.md").write_text("\n".join(lines), encoding="utf-8")


def prepare():
    OUT.mkdir(parents=True, exist_ok=True)
    old_path = OUT / "layouts.json"
    old_scenes = ({s["filename"]: s for s in json.loads(old_path.read_text())["scenes"]}
                  if old_path.exists() else {})
    m1 = json.loads(M1_DOC.read_text())["scenes"]
    m2 = json.loads(M2_DOC.read_text())["scenes"]
    all_scenes = []
    for number, (group, reason) in KEPT.items():
        filename = f"m2dev{number:02d}.xml"
        s = copy.deepcopy(next(s for s in m2 if s["filename"] == filename))
        s.update(source_set="M2", source_filename=filename, source_xml_sha256=digest(g.m1.SCENE_DIR / filename),
                 selection_group=group, selection_reason=reason, mapset_version=VERSION,
                 display_name=f"M2-{number:02d}", split="development_selection")
        all_scenes.append(s)
    for number, source_number, group, reason in COPIES:
        source_name, filename = f"dev{source_number:02d}.xml", f"m2dev{number:02d}.xml"
        source = g.m1.SCENE_DIR / source_name
        s = copy.deepcopy(next(s for s in m1 if s["filename"] == source_name))
        s.pop("production_scene_binding", None)
        s.update(filename=filename, scene_id=f"m2d{number:02d}", model_name=f"m2dev{number:02d}",
                 preview=f"m2dev{number:02d}.png", display_name=f"M2-{number:02d}（源{source_name[:-4]}）",
                 source_set="M1", source_filename=source_name, source_scene_id=s["scene_id"],
                 source_xml_sha256=digest(source), selection_group=group, selection_reason=reason,
                 mapset_version=VERSION, split="development_selection",
                 layout_revision="source-M1-1.0-geometry-unchanged")
        all_scenes.append(s)
    for s in all_scenes:
        previous_name = s["filename"]
        stem, difficulty = NAMES[previous_name]
        s.pop("production_scene_binding", None)
        s.update(previous_m2_filename=previous_name, filename=stem+".xml",
                 scene_id="m2v2_"+stem, model_name="m2_"+stem, preview=stem+".png",
                 display_name=stem+" · "+difficulty, difficulty_label=difficulty)
        source = g.m1.SCENE_DIR / s["source_filename"]
        text, n = re.subn(r'(<mujoco\s+model=")[^"]+', rf'\g<1>go2 m2_{stem}', source.read_text(), count=1)
        if n != 1:
            raise ValueError(f"No unique model name in {source}")
        (g.m1.SCENE_DIR / s["filename"]).write_text(text)
    all_scenes.sort(key=lambda s: ("emd".index(s["filename"][0]), s["filename"]))
    # A repeat preparation must not replace already registered, unchanged models with zeros.
    rows = []
    pending = []
    for s in all_scenes:
        prior = old_scenes.get(s["filename"], {}).get("production_scene_binding", {})
        unchanged = prior.get("root_sha256") == digest(g.m1.SCENE_DIR / s["filename"])
        fp = prior.get("model_fingerprint") if unchanged else None
        row = g.m1.s611.catalog_rows([s], {s["filename"]: fp} if fp else {})[0]
        if fp and row["closure_sha256"] != prior.get("closure_sha256"):
            fp = None
            row["model_fingerprint"] = "0" * 64
        if fp:
            s["production_scene_binding"] = {**row, "runtime_mujoco_version": prior["runtime_mujoco_version"]}
        else:
            pending.append(s["filename"])
        rows.append(row)
    g.m1.s611.append_or_replace_catalog(rows, upsert=True)
    g.m1.BOUNDS = ((-0.5, 8.0), (-3.0, 3.0))
    overview = Image.new("RGB", (1830, 2810), "#edf1f4")
    d = ImageDraw.Draw(overview)
    d.text((22, 15), "M2 · easy 3张 / middle 6张 / difficult 9张 · m01修改后待确认", fill="#17212b", font=g.m1.font(26))
    for i, s in enumerate(all_scenes):
        im = g.render(s)
        im.save(OUT / s["preview"])
        overview.paste(im.resize((600, 439)), (10 + (i % 3)*610, 55 + (i // 3)*459))
    overview.save(OUT / "overview_18.png")
    doc = {"mapset_version": VERSION, "status": "approved_selection_not_experiment_matrix",
           "scene_count": 18, "policy_runs_this_preparation": 0, "m1_modified": False,
           "held_out_used": False, "excluded_historical_m2": ["m2dev01.xml"],
           "naming_revision": "easy-middle-difficult-20261018",
           "registration_status": ("pending runtime identities: " + ", ".join(pending)
                                   if pending else "registered identities preserved; no policy runs"),
           "scenes": all_scenes}
    (OUT / "layouts.json").write_text(json.dumps(doc, ensure_ascii=False, indent=2)+"\n")
    write_index(doc)
    print("Prepared 18-map selection; M1 untouched; no policy runs.")


def register(path):
    p = OUT / "layouts.json"
    doc = json.loads(p.read_text())
    entries = {}
    for line in path.read_text().splitlines():
        match = re.fullmatch(r"SCENE filename=(\S+) id=(\S+) obstacle_count=(\d+) fingerprint=([0-9a-f]{64}) mujoco_version=(\S+)", line)
        if match:
            name, sid, count, fp, version = match.groups()
            entries[name] = (sid, int(count), fp, version)
    if set(entries) != {s["filename"] for s in doc["scenes"]}:
        raise ValueError("Runtime identities must cover exactly all 18 selected maps")
    for s in doc["scenes"]:
        sid, count, fp, version = entries[s["filename"]]
        if sid != s["scene_id"] or count != s["obstacle_count"]:
            raise ValueError(f"Runtime selection mismatch: {s['filename']}")
        row = g.m1.s611.catalog_rows([s], {s["filename"]: fp})[0]
        g.m1.s611.append_or_replace_catalog([row], upsert=True)
        s["production_scene_binding"] = {**row, "runtime_mujoco_version": version}
    doc["registration_status"] = "18 static runtime identities registered; rebuild simulator required; no policy runs"
    p.write_text(json.dumps(doc, ensure_ascii=False, indent=2)+"\n")
    (OUT / "runtime_identities.txt").write_text(path.read_text())
    write_index(doc)
    print("Registered all 18 production runtime identities; no physics stepping or policies.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--register-from", type=Path)
    args = parser.parse_args()
    register(args.register_from) if args.register_from else prepare()
