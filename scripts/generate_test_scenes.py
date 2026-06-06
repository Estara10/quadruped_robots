#!/usr/bin/env python3
"""Generate multiple random obstacle scenes for Go2 recovery testing."""

import xml.etree.ElementTree as xml_et
import numpy as np
import os
import shutil

ROBOT = "go2"
BASE_SCENE = "/home/lidio/quadruped_robots/unitree_mujoco/unitree_robots/go2/go2.xml"
OUTPUT_DIR = "/home/lidio/quadruped_robots/unitree_mujoco/unitree_robots/go2/"
NUM_SCENES = 5
SEED = 42

def euler_to_quat(roll, pitch, yaw):
    cx, sx = np.cos(roll/2), np.sin(roll/2)
    cy, sy = np.cos(pitch/2), np.sin(pitch/2)
    cz, sz = np.cos(yaw/2), np.sin(yaw/2)
    return np.array([cx*cy*cz+sx*sy*sz, sx*cy*cz-cx*sy*sz,
                     cx*sy*cz+sx*cy*sz, cx*cy*sz-sx*sy*cz], dtype=np.float64)

def list_to_str(vec):
    return " ".join(f"{s:.6g}" for s in vec)

def generate_scene(name, obstacles):
    """Generate a scene XML with given obstacles."""
    root = xml_et.Element("mujoco", model=f"go2 {name}")
    xml_et.SubElement(root, "include", file="go2.xml")

    visual = xml_et.SubElement(root, "visual")
    xml_et.SubElement(visual, "headlight", diffuse="0.6 0.6 0.6", ambient="0.3 0.3 0.3", specular="0 0 0")
    xml_et.SubElement(visual, "rgba", haze="0.15 0.25 0.35 1")
    xml_et.SubElement(visual, "global", azimuth="-130", elevation="-20")

    asset = xml_et.SubElement(root, "asset")
    xml_et.SubElement(asset, "texture", type="skybox", builtin="gradient",
                       rgb1="0.3 0.5 0.7", rgb2="0 0 0", width="512", height="3072")
    xml_et.SubElement(asset, "texture", type="2d", name="groundplane", builtin="checker",
                       mark="edge", rgb1="0.2 0.3 0.4", rgb2="0.1 0.2 0.3",
                       markrgb="0.8 0.8 0.8", width="300", height="300")
    xml_et.SubElement(asset, "material", name="groundplane", texture="groundplane",
                       texuniform="true", texrepeat="5 5", reflectance="0.2")

    worldbody = xml_et.SubElement(root, "worldbody")
    xml_et.SubElement(worldbody, "light", pos="0 0 1.5", dir="0 0 -1", directional="true")
    xml_et.SubElement(worldbody, "geom", name="floor", size="0 0 0.05", type="plane", material="groundplane")

    for obs in obstacles:
        geo = xml_et.SubElement(worldbody, "geom")
        geo.attrib["type"] = obs["type"]
        geo.attrib["pos"] = list_to_str(obs["pos"])
        geo.attrib["size"] = list_to_str(obs["size"])
        if "quat" in obs:
            geo.attrib["quat"] = list_to_str(obs["quat"])
        if "rgba" in obs:
            geo.attrib["rgba"] = obs["rgba"]

    # Save (pretty-print manually since Python 3.8 lacks xml.etree.ElementTree.indent)
    path = os.path.join(OUTPUT_DIR, f"scene_{name}.xml")
    raw = xml_et.tostring(root, encoding="utf-8")
    import xml.dom.minidom
    dom = xml.dom.minidom.parseString(raw)
    with open(path, "w", encoding="utf-8") as f:
        f.write(dom.toprettyxml(indent="  "))
    return path

def main():
    np.random.seed(SEED)

    for i in range(NUM_SCENES):
        obstacles = []

        # Random number of box obstacles (3-8)
        n_boxes = np.random.randint(3, 9)
        for _ in range(n_boxes):
            x = np.random.uniform(0.5, 5.0)     # forward range
            y = np.random.uniform(-3.0, 3.0)    # lateral range
            z = np.random.uniform(0.05, 0.5)    # height
            sx = np.random.uniform(0.1, 0.6)    # half-length
            sy = np.random.uniform(0.1, 0.6)    # half-width
            sz = np.random.uniform(0.05, z)     # half-height <= z
            obstacles.append({
                "type": "box",
                "pos": [x, y, z],
                "size": [sx, sy, sz],
            })

        # 1-3 cylinders (pillars)
        n_cyl = np.random.randint(1, 4)
        for _ in range(n_cyl):
            x = np.random.uniform(1.0, 4.0)
            y = np.random.uniform(-2.5, 2.5)
            h = np.random.uniform(0.2, 0.8)
            r = np.random.uniform(0.1, 0.35)
            obstacles.append({
                "type": "cylinder",
                "pos": [x, y, h],
                "size": [r, h, h],  # MuJoCo cylinder: radius, half-height x2
            })

        path = generate_scene(f"test{i+1}", obstacles)
        print(f"  {path}  ({len(obstacles)} obstacles)")

    print(f"\nGenerated {NUM_SCENES} test scenes.")
    print("Usage:")
    for i in range(NUM_SCENES):
        print(f"  MUJOCO_SCENE_OVERRIDE=scene_test{i+1}.xml ./scripts/launch_abs_terrain.sh")

if __name__ == "__main__":
    main()
