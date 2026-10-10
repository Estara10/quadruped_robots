#!/usr/bin/env python3
"""PPT-only Go2 obstacle scene generator and launcher; not experiment evidence."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
GO2_DIR = REPO / "unitree_mujoco/unitree_robots/go2"
LAUNCHER = REPO / "scripts/launch_abs_obstacle.sh"

# Each tuple is (x, y, half_length_x, half_width_y, height), in metres.
# Go2 starts at (0, 0), faces +X. Stagger boxes toward the middle while
# retaining a straight, robot-width clearance around y=0 for this visual demo.
SCENES = {
    "sparse": [
        (1.9, 1.48, .36, .30, .42),
        (3.3, -0.78, .40, .25, .42),
        (5.0, 0.74, .32, .24, .42),
        (6.5, -1.48, .38, .28, .42),
    ],
    "medium": [
        (1.7, 1.53, .32, .28, .42),
        (2.4, -0.80, .34, .25, .42),
        (3.2, 0.77, .32, .25, .42),
        (4.0, -1.55, .33, .27, .42),
        (4.9, 1.38, .35, .29, .42),
        (5.8, -0.75, .32, .24, .42),
        (6.7, 0.85, .35, .26, .42),
    ],
    "dense": [
        (1.55, 1.49, .28, .25, .42),
        (2.05, -0.74, .30, .24, .42),
        (2.65, 0.82, .29, .26, .42),
        (3.18, -1.52, .28, .25, .42),
        (3.75, 1.19, .29, .26, .42),
        (4.28, -0.75, .30, .24, .42),
        (4.83, 0.73, .27, .23, .42),
        (5.36, -1.22, .29, .25, .42),
        (5.92, 1.55, .30, .25, .42),
        (6.47, -0.78, .29, .25, .42),
        (7.02, 0.90, .28, .25, .42),
    ],
}

# Shared frame: 12 m x 7 m visible floor, fixed free-camera defaults.
CAMERA = {"center": "2.8 0 0.1", "extent": "5.0",
          "azimuth": "135", "elevation": "-40"}
OBSTACLE_RGBA = "0.26 0.28 0.31 1"


def make_scene(name: str) -> Path:
    root = ET.Element("mujoco", {"model": f"go2 ppt {name}"})
    ET.SubElement(root, "include", {"file": "go2.xml"})
    ET.SubElement(root, "statistic", {"center": CAMERA["center"],
                                      "extent": CAMERA["extent"]})
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", {"diffuse": "0.6 0.6 0.6",
                                        "ambient": "0.3 0.3 0.3",
                                        "specular": "0 0 0"})
    ET.SubElement(visual, "rgba", {"haze": "0.15 0.25 0.35 1"})
    ET.SubElement(visual, "global", {"azimuth": CAMERA["azimuth"],
                                     "elevation": CAMERA["elevation"]})
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", {"type": "skybox", "builtin": "gradient",
                                     "rgb1": "0.3 0.5 0.7",
                                     "rgb2": "0 0 0",
                                     "width": "512", "height": "3072"})
    ET.SubElement(asset, "texture", {"type": "2d", "name": "ppt_groundplane",
                                     "builtin": "checker", "mark": "edge",
                                     "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3",
                                     "markrgb": "0.8 0.8 0.8",
                                     "width": "300", "height": "300"})
    ET.SubElement(asset, "material", {"name": "ppt_groundplane",
                                      "texture": "ppt_groundplane",
                                      "texuniform": "true", "texrepeat": "5 5",
                                      "reflectance": "0.2"})
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", {"pos": "1 0 7", "dir": "0 0 -1",
                                   "directional": "true"})
    ET.SubElement(world, "geom", {"name": "floor", "type": "plane",
                                  "pos": "3 0 0", "size": "6 3.5 0.05",
                                  "material": "ppt_groundplane"})
    for index, (x, y, half_x, half_y, height) in enumerate(SCENES[name], 1):
        ET.SubElement(world, "geom", {
            "name": f"ppt_obstacle_{index:02d}", "type": "box",
            "pos": f"{x} {y} {height / 2}",
            "size": f"{half_x} {half_y} {height / 2}",
            "rgba": OBSTACLE_RGBA,
        })
    ET.indent(root, space="  ")
    path = GO2_DIR / f"scene_ppt_{name}.xml"
    path.write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, choices=SCENES)
    parser.add_argument("--camera", choices=("ppt",), default="ppt")
    parser.add_argument("--prepare-only", action="store_true",
                        help="generate the XML without launching MuJoCo/ROS2")
    args = parser.parse_args()
    if not (GO2_DIR / "go2.xml").is_file():
        parser.error("Go2 model missing")
    scene = make_scene(args.scene)
    print(f"PPT demo: {args.scene}, {len(SCENES[args.scene])} static boxes; {scene}", flush=True)
    if args.prepare_only:
        return 0
    if not LAUNCHER.is_file():
        parser.error(f"launcher missing: {LAUNCHER}")
    env = os.environ.copy()
    env["MUJOCO_SCENE"] = scene.name
    # Existing launcher owns MuJoCo, ROS2 controller, and auto-forward command.
    try:
        return subprocess.call(["bash", str(LAUNCHER)], cwd=REPO, env=env)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
