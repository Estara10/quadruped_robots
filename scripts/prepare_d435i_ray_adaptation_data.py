#!/usr/bin/env python3
"""Prepare a bounded, split-by-layout D435i ray-adaptation dataset (offline).

Reuses the existing renderer, geometric label function, center sampler and
librealsense 2.58.4 source-equivalent hole fill. It does not train, touch the
network, connect to a device, or write shared memory.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import pickle
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import mujoco  # noqa: E402

import compare_ray_pred as compare  # noqa: E402
import generate_mujoco_ray_dataset as generation  # noqa: E402
from analyze_d435i_new_camera_hole_candidates import librealsense_2584_mode1_source  # noqa: E402
from realsense_depth_to_abs_rays import downsample_depth_centers  # noqa: E402

CAMERA = compare.CameraRenderConfig(
    position_body_m=(.305, 0., .140),
    pitch_down_deg=4.0,
    width=480, height=270,
    fx=244.134, fy=244.134, ppx=245.186, ppy=134.22,
)
PITCH_RANGE_DEG = (3., 5.)
BASE_Z_RANGE_M = (.35, .36)
RAY_ANGLES = [-45. + 9. * i for i in range(11)]
POSE_X_RANGE = (-.8, 3.5)
POSE_Y_RANGE = (-1., 1.)
POSE_YAW_RANGE = (-.5, .5)
BIN_NEAR = (.45, 1.5)
BIN_MID = (1.5, 3.0)
BIN_FAR = (3.0, 5.999)
MIN_CLEARANCE_M = .45
MAX_CAP_BEAMS = {  # reject poses whose view is mostly unexplained 6 m caps
    "near": 5, "mid": 6, "far": 6,
}

MASK_GROUPS = {
    "train": [
        "logs/real_go2/20261113_d435i_remounted_cabinet/capture",
        "logs/real_go2/20261113_remount_adaptation_capture/capture",
        "logs/real_go2/20261113_d435i_existing_mount_standing_cabinet/capture",
        "logs/real_go2/20261113_d435i_1m_distance_capture/capture",
    ],
    "validation": ["logs/real_go2/20261113_d435i_80cm_distance_capture/capture"],
    "test": [
        "logs/real_go2/20261113_d435i_2m_distance_capture/capture/data_20261113_143447",
        "logs/real_go2/20261112_d435i_native_fill_field/capture",
    ],
}


def box(name: str, x: float, y: float, sx: float, sy: float, z: float = .6, sz: float = .6) -> str:
    return (f'<geom name="{name}" type="box" pos="{x:.4f} {y:.4f} {z:.4f}" '
            f'size="{sx:.4f} {sy:.4f} {sz:.4f}" rgba=".62 .62 .62 1"/>')


def cyl(name: str, x: float, y: float, radius: float, half_height: float = .6, z: float = .6) -> str:
    return (f'<geom name="{name}" type="cylinder" pos="{x:.4f} {y:.4f} {z:.4f}" '
            f'size="{radius:.4f} {half_height:.4f}" rgba=".55 .55 .55 1"/>')


def wall(name: str, x: float, half_y: float = 4.0) -> str:
    return (f'<geom name="{name}" type="box" pos="{x:.4f} 0 1" '
            f'size=".05 {half_y:.3f} 1" rgba=".5 .5 .5 1"/>')


def random_obstacles(seed: int, wall_x: float | None) -> list[str]:
    rng = np.random.default_rng(seed)
    parts = [wall("backdrop", wall_x)] if wall_x is not None else []
    for j in range(7):
        x, y = rng.uniform(1.2, 5.2), rng.uniform(-2.1, 2.1)
        sx, sy = rng.uniform(.12, .4), rng.uniform(.12, .4)
        if j % 2:
            parts.append(box(f"box{j}", x, y, sx, sy))
        else:
            parts.append(cyl(f"cyl{j}", x, y, sx))
    return parts


def build_layouts(test_layout_set: str = "legacy") -> list[dict]:
    """Whole layouts own a split; quota = fraction of near/mid/far pose classes."""
    layouts = [
        {"split": "train", "name": "train_near_center", "count": 105, "quota": (.55, .30, .15), "parts": [
            wall("backdrop", 5.2), box("near_center", .95, .10, .18, .25),
            box("left_near", 1.35, .75, .15, .20), box("right_mid", 2.6, -.9, .25, .30),
            box("left_far", 3.8, 1.2, .30, .30), cyl("right_cyl", 3.2, -1.4, .25)]},
        {"split": "train", "name": "train_near_left", "count": 105, "quota": (.55, .30, .15), "parts": [
            wall("backdrop", 5.4), box("left_near", 1.0, .55, .18, .22),
            box("left_mid", 1.9, 1.0, .20, .25), cyl("center", 1.6, -.15, .18),
            box("right_far", 3.4, -.95, .30, .30), cyl("right_far_cyl", 4.2, -.4, .28)]},
        {"split": "train", "name": "train_near_right", "count": 105, "quota": (.55, .30, .15), "parts": [
            wall("backdrop", 5.4), box("right_near", 1.0, -.55, .18, .22),
            box("right_mid", 1.9, -1.0, .20, .25), cyl("center", 1.6, .15, .18),
            box("left_far", 3.4, .95, .30, .30), cyl("left_far_cyl", 4.2, .4, .28)]},
        {"split": "train", "name": "train_occlusion_pair", "count": 105, "quota": (.50, .30, .20), "parts": [
            wall("backdrop", 5.8), box("front", 1.0, 0., .15, .25), box("behind", 2.0, 0., .15, .25),
            box("left", 2.4, .9, .15, .25), box("right", 1.9, -.8, .20, .20),
            cyl("cyl", 3.5, .4, .30)]},
        {"split": "train", "name": "train_corridor", "count": 105, "quota": (.55, .30, .15), "parts": [
            '<geom name="side_left" type="box" pos="2.0 2.6 1" size="2.4 .05 1" rgba=".5 .5 .5 1"/>',
            '<geom name="side_right" type="box" pos="2.0 -2.6 1" size="2.4 .05 1" rgba=".5 .5 .5 1"/>',
            '<geom name="end_wall" type="box" pos="5.0 0 1" size=".05 3.5 1" rgba=".5 .5 .5 1"/>',
            box("center_near", 1.6, .15, .18, .22), box("right_mid", 2.9, -.2, .20, .25),
            box("left_far", 4.0, .3, .25, .30), cyl("left_cyl", 2.3, .9, .25)]},
        {"split": "train", "name": "train_open_mixed", "count": 105, "quota": (.30, .35, .35), "parts": [
            wall("backdrop", 5.6), box("a", 2.2, -.6, .20, .25), box("b", 3.6, .9, .25, .25),
            cyl("c", 4.6, -.4, .30), box("d", 3.0, -1.3, .25, .30)]},
        {"split": "train", "name": "train_random_a", "count": 105, "quota": (.50, .30, .20), "parts":
            random_obstacles(20261115, 5.2)},
        {"split": "train", "name": "train_random_b", "count": 105, "quota": (.45, .35, .20), "parts":
            random_obstacles(20261116, 5.8)},
        {"split": "validation", "name": "validation_mixed", "count": 90, "quota": (.45, .35, .20), "parts": [
            wall("backdrop", 5.4), box("left_near", 1.1, .6, .18, .22),
            box("center", 1.6, -.15, .20, .25), box("right_mid", 2.4, .9, .20, .25),
            cyl("right_cyl", 2.9, -.8, .25), box("far", 4.0, .3, .30, .30)]},
        {"split": "validation", "name": "validation_open", "count": 90, "quota": (.30, .35, .35), "parts": [
            wall("backdrop", 5.0), box("a", 2.5, .7, .20, .25), box("b", 3.4, -.8, .25, .30),
            cyl("c", 4.2, .4, .30), box("d", 4.5, -.2, .20, .20)]},
        {"split": "test", "name": "test_mixed", "count": 90, "quota": (.45, .35, .20), "parts": [
            wall("backdrop", 5.6), box("right_near", .9, -.25, .18, .22),
            box("left_mid", 1.4, .7, .15, .20), box("front", 1.8, .1, .15, .22),
            box("behind", 2.6, .1, .15, .22), cyl("right_cyl", 3.3, -.9, .28)]},
        {"split": "test", "name": "test_open", "count": 90, "quota": (.30, .35, .35), "parts": [
            wall("backdrop", 5.9), box("a", 2.8, .8, .20, .25), box("b", 3.9, -.9, .25, .30),
            cyl("c", 4.8, .2, .30), box("d", 4.4, -.3, .20, .20)]},
    ]
    if test_layout_set == "current_mount_v2":
        layouts = [layout for layout in layouts if layout["split"] != "test"]
        layouts.extend([
            {"split": "test", "name": "test_current_mount_offset_occlusion_a", "count": 90,
             "quota": (.45, .35, .20), "parts": [
                 wall("backdrop", 5.7), box("front_left", 1.05, .58, .16, .22),
                 box("front_right", 1.35, -.48, .18, .24), box("center_behind", 2.3, .12, .22, .28),
                 cyl("left_far", 3.8, 1.35, .30), box("right_far", 4.4, -1.25, .28, .32)]},
            {"split": "test", "name": "test_current_mount_cross_lane_b", "count": 90,
             "quota": (.30, .35, .35), "parts": [
                 '<geom name="side_left" type="box" pos="2.8 2.35 1" size="2.6 .06 1" rgba=".5 .5 .5 1"/>',
                 '<geom name="side_right" type="box" pos="2.5 -2.45 1" size="2.4 .06 1" rgba=".5 .5 .5 1"/>',
                 '<geom name="end_wall" type="box" pos="5.5 0 1" size=".06 3.4 1" rgba=".5 .5 .5 1"/>',
                 box("center_mid", 2.0, -.18, .20, .26), cyl("left_mid", 2.7, .85, .27),
                 box("right_far", 4.0, -.65, .30, .30), cyl("left_far", 4.5, 1.0, .28)]},
        ])
    elif test_layout_set == "final_holdout_v1":
        # Only final-test layouts: never regenerate training/validation here.
        layouts = [
            {"split": "test", "name": "final_wide_wall_offset", "count": 90,
             "quota": (.45, .35, .20), "parts": [
                 wall("backdrop", 6.15), box("near_right", 1.18, -.72, .23, .17),
                 cyl("left_front", 1.85, .42, .21), box("left_mid", 3.15, 1.18, .32, .19),
                 box("right_mid", 3.45, -.63, .17, .36), cyl("center_far", 4.85, .18, .34)]},
            {"split": "test", "name": "final_staggered_occlusion", "count": 90,
             "quota": (.45, .35, .20), "parts": [
                 '<geom name="left_wall" type="box" pos="2.1 2.05 1" size="2.9 .07 1" rgba=".5 .5 .5 1"/>',
                 '<geom name="right_wall" type="box" pos="2.8 -2.15 1" size="2.2 .07 1" rgba=".5 .5 .5 1"/>',
                 wall("backdrop", 6.3), cyl("right_front", 1.42, -.32, .23),
                 box("left_front", 2.15, .66, .19, .29), box("right_behind", 2.65, -.32, .35, .18),
                 cyl("left_behind", 3.35, .66, .31), box("far_center", 4.75, -.10, .28, .31)]},
        ]
    elif test_layout_set != "legacy":
        raise ValueError(f"unknown test layout set: {test_layout_set}")
    return layouts


def build_coverage_supplement_probe_layouts() -> list[dict]:
    """A few new whole layouts to probe size, lateral placement and occlusion coverage."""
    return [
        {"split": "train", "name": "supp_near_size_mix", "count": 60,
         "quota": (.65, .25, .10), "parts": [
             wall("backdrop", 5.7), box("wide_near", 1.18, .05, .30, .27),
             cyl("narrow_left", 1.42, .78, .13), box("small_right", 1.72, -.78, .12, .18),
             cyl("mid_left", 2.8, 1.15, .28), box("mid_right", 3.3, -.9, .24, .31)]},
        {"split": "train", "name": "supp_near_partial_left", "count": 60,
         "quota": (.65, .25, .10), "parts": [
             wall("backdrop", 5.8), box("front_left", 1.08, .64, .17, .24),
             box("rear_left", 1.55, .83, .34, .36), cyl("near_center", 1.25, -.12, .16),
             box("right_mid", 2.4, -.85, .22, .29), cyl("far_right", 4.1, -.55, .31)]},
        {"split": "train", "name": "supp_near_partial_right", "count": 60,
         "quota": (.65, .25, .10), "parts": [
             wall("backdrop", 5.9), box("front_right", 1.14, -.61, .14, .20),
             cyl("rear_right", 1.62, -.86, .29), box("near_center", 1.38, .18, .18, .30),
             cyl("left_mid", 2.55, .9, .24), box("far_left", 4.25, .48, .33, .27)]},
        {"split": "validation", "name": "supp_validation_staggered_sizes", "count": 90,
         "quota": (.55, .30, .15), "parts": [
             wall("backdrop", 5.6), cyl("near_left", 1.22, .58, .19),
             box("behind_left", 1.82, .79, .28, .31), box("near_right", 1.35, -.42, .23, .16),
             cyl("mid_right", 2.7, -1.05, .32), box("far_center", 4.2, .1, .35, .30)]},
    ]


def build_independent_eval_layouts() -> list[dict]:
    """New whole layouts for one-shot independent checkpoint comparison.

    All layouts are evaluation-only: no train or test split is generated, and none
    of the names or obstacle placements below appear in adaptation_v1, the
    current-mount test sets, final_holdout_v1 or coverage_supplement_v1.
    """
    return [
        {"split": "validation", "name": "indep_eval_offset_pair_a", "count": 60,
         "quota": (.60, .25, .15), "parts": [
             wall("backdrop", 5.75), box("near_left_wide", 1.12, .68, .26, .21),
             cyl("near_right_narrow", 1.46, -.52, .12), box("behind_left", 1.95, .92, .33, .26),
             cyl("mid_right", 2.9, -.95, .30), box("far_left", 4.35, .45, .32, .29)]},
        {"split": "validation", "name": "indep_eval_offset_pair_b", "count": 60,
         "quota": (.55, .30, .15), "parts": [
             wall("backdrop", 6.05), cyl("near_right", 1.04, -.61, .18),
             box("rear_right_wide", 1.68, -.88, .37, .24), box("near_left_small", 1.31, .38, .13, .19),
             box("mid_center", 2.6, .12, .29, .33), cyl("far_right", 4.5, -.72, .27)]},
        {"split": "validation", "name": "indep_eval_open_mix_c", "count": 60,
         "quota": (.35, .35, .30), "parts": [
             wall("backdrop", 6.25), cyl("near_center_small", 1.52, .16, .15),
             box("mid_left", 2.35, .86, .24, .30), cyl("mid_right", 3.05, -.82, .26),
             box("far_center", 4.6, -.06, .30, .27), box("far_left", 5.25, .95, .22, .24)]},
    ]


def build_scene_coverage_v1_layouts() -> list[dict]:
    """Bounded, distinct layouts for the front-mounted D435i scene supplement."""
    layouts = [
        {"split": "train", "category": "wide_wall", "name": "scv1_train_wall_front",
         "count": 60, "quota": (.82, .18, 0.),
         "pose_ranges": {"x": (-.45, .85), "y": (-.30, .30), "yaw_rad": (-.05, .05)},
         "parts": [wall("wide_front_wall", 1.65, 4.0)]},
        {"split": "train", "category": "wall_corner", "name": "scv1_train_wall_oblique_corner",
         "count": 50, "quota": (.70, .30, 0.),
         "pose_ranges": {"x": (-.30, .45), "y": (-.40, .40), "yaw_rad": (-.12, .12)},
         "parts": [
             '<geom name="oblique_wall" type="box" pos="1.78 0 1" euler="0 0 0.122173" size=".05 4 1" rgba=".5 .5 .5 1"/>',
             '<geom name="corner_return" type="box" pos="3.2 2.05 1" euler="0 0 1.570796" size=".05 1.4 1" rgba=".5 .5 .5 1"/>',
             wall("backdrop", 5.8)]},
        {"split": "train", "category": "furniture", "name": "scv1_train_furniture_cabinet_sofa_table",
         "count": 60, "quota": (.68, .32, 0.),
         "pose_ranges": {"x": (-.25, .40), "y": (-.50, .50), "yaw_rad": (-.20, .20)},
         "parts": [
             wall("backdrop", 5.8),
             box("cabinet_facade", 1.72, 0.0, .16, .62, .72, .72),
             box("sofa_seat", 2.15, .95, .42, .68, .40, .20),
             box("sofa_back", 2.48, .95, .10, .68, .77, .38),
             box("table_top", 2.38, -.92, .48, .58, .62, .07),
             cyl("table_leg_fl", 1.98, -1.34, .045, .31), cyl("table_leg_fr", 1.98, -.50, .045, .31),
             cyl("table_leg_rl", 2.78, -1.34, .045, .31), cyl("table_leg_rr", 2.78, -.50, .045, .31)]},
        {"split": "train", "category": "narrow_obstacle", "name": "scv1_train_posts_chairlegs_doorway",
         "count": 50, "quota": (.65, .35, 0.),
         "pose_ranges": {"x": (-.20, .35), "y": (-.25, .25), "yaw_rad": (-.12, .12)},
         "parts": [
             '<geom name="corridor_left" type="box" pos="2.7 1.35 1" size="2.3 .05 1" rgba=".5 .5 .5 1"/>',
             '<geom name="corridor_right" type="box" pos="2.7 -1.35 1" size="2.3 .05 1" rgba=".5 .5 .5 1"/>',
             wall("corridor_end", 5.0, 1.7),
             box("door_jamb_left", 1.72, .56, .10, .08, .95, .95),
             box("door_jamb_right", 1.72, -.56, .10, .08, .95, .95),
             box("narrow_box", 1.38, .88, .13, .14, .46, .46),
             cyl("pole", 2.0, -.82, .075),
             cyl("chair_leg_a", 2.55, .58, .045), cyl("chair_leg_b", 2.55, -.58, .045),
             cyl("chair_leg_c", 3.12, .58, .045), cyl("chair_leg_d", 3.12, -.58, .045)]},
        {"split": "train", "category": "open_side_far", "name": "scv1_train_open_side_far_wall",
         "count": 50, "quota": (0., .26, .74),
         "pose_ranges": {"x": (-.25, .55), "y": (-.20, .20), "yaw_rad": (-.10, .10)},
         "parts": [wall("far_wall", 6.3), box("left_side_item", 2.35, 1.55, .18, .24),
                   cyl("right_side_pole", 2.7, -1.55, .10), box("left_far_box", 4.0, 1.75, .25, .28)]},
        {"split": "train", "category": "open", "name": "scv1_train_clear_no_hit",
         "count": 20, "acceptance": "no_hit", "parts": []},
        {"split": "train", "category": "open_side_only", "name": "scv1_train_forward_clear_side_items",
         "count": 20, "acceptance": "forward_clear",
         "pose_ranges": {"x": (-.12, .22), "y": (-.06, .06), "yaw_rad": (-.03, .03)},
         "parts": [box("train_left_side", 1.55, 1.45, .24, .25), cyl("train_right_side", 1.55, -1.45, .14)]},
        {"split": "validation", "category": "wide_wall_corridor", "name": "scv1_val_wall_corridor",
         "count": 40, "quota": (.72, .28, 0.),
         "pose_ranges": {"x": (.10, .78), "y": (-.25, .25), "yaw_rad": (-.10, .10)},
         "parts": [
             '<geom name="val_wall" type="box" pos="2.02 0 1" euler="0 0 -0.069813" size=".06 4.2 1" rgba=".5 .5 .5 1"/>',
             '<geom name="val_corridor_left" type="box" pos="2.8 1.65 1" size="2.1 .05 1" rgba=".5 .5 .5 1"/>',
             '<geom name="val_corridor_right" type="box" pos="2.8 -1.65 1" size="2.1 .05 1" rgba=".5 .5 .5 1"/>']},
        {"split": "validation", "category": "far_wall", "name": "scv1_val_far_wall_unseen",
         "count": 20, "quota": (0., 0., 1.),
         "pose_ranges": {"x": (-.15, .15), "y": (-.20, .20), "yaw_rad": (-.05, .05)},
         "parts": [wall("val_far_wall", 5.0, 6.0)]},
        {"split": "validation", "category": "furniture", "name": "scv1_val_furniture_console_table",
         "count": 40, "quota": (.65, .35, 0.),
         "pose_ranges": {"x": (.02, .55), "y": (-.45, .45), "yaw_rad": (-.20, .20)},
         "parts": [
             wall("backdrop", 5.9), box("console_side_left", 1.92, .75, .12, .10, .55, .55),
             box("console_side_right", 1.92, -.75, .12, .10, .55, .55),
             box("console_top", 1.92, 0, .12, .84, 1.08, .08),
             box("sofa_base", 2.35, 1.05, .38, .60, .38, .18),
             box("sofa_back", 2.66, 1.05, .10, .60, .72, .34),
             box("table_top", 2.55, -.98, .38, .50, .58, .06),
             cyl("table_leg1", 2.18, -1.38, .04, .29), cyl("table_leg2", 2.18, -.58, .04, .29),
             cyl("table_leg3", 2.92, -1.38, .04, .29), cyl("table_leg4", 2.92, -.58, .04, .29)]},
        {"split": "validation", "category": "open", "name": "scv1_val_clear_no_hit",
         "count": 30, "acceptance": "no_hit", "parts": []},
        {"split": "validation", "category": "open_side_only", "name": "scv1_val_forward_clear_side_items",
         "count": 30, "acceptance": "forward_clear",
         "pose_ranges": {"x": (-.15, .25), "y": (-.08, .08), "yaw_rad": (-.035, .035)},
         "parts": [box("left_side_only", 1.55, 1.45, .24, .25),
                   cyl("right_side_only", 1.55, -1.45, .14)]},
        {"split": "test", "category": "wide_wall_corner", "name": "scv1_test_wall_corner_unseen",
         "count": 40, "quota": (.70, .30, 0.),
         "pose_ranges": {"x": (.22, .88), "y": (-.30, .30), "yaw_rad": (-.12, .12)},
         "parts": [wall("test_wide_wall", 2.18, 4.5),
                   '<geom name="test_corner_return" type="box" pos="3.5 2.35 1" euler="0 0 1.570796" size=".06 1.5 1" rgba=".5 .5 .5 1"/>']},
        {"split": "test", "category": "far_wall", "name": "scv1_test_far_wall_unseen",
         "count": 20, "quota": (0., 0., 1.),
         "pose_ranges": {"x": (-.12, .18), "y": (-.18, .18), "yaw_rad": (-.05, .05)},
         "parts": [wall("test_far_wall", 5.15, 6.0)]},
        {"split": "test", "category": "furniture", "name": "scv1_test_furniture_staggered",
         "count": 40, "quota": (.63, .37, 0.),
         "pose_ranges": {"x": (.10, .65), "y": (-.40, .40), "yaw_rad": (-.22, .22)},
         "parts": [wall("backdrop", 5.7),
                   box("test_cabinet", 1.95, -.10, .20, .52, .78, .78),
                   box("test_sofa_seat", 2.25, 1.05, .43, .62, .39, .17),
                   box("test_sofa_back", 2.58, 1.05, .11, .62, .75, .36),
                   box("test_table_top", 2.56, -.95, .43, .54, .60, .06),
                   cyl("test_table_leg1", 2.11, -1.38, .045, .30),
                   cyl("test_table_leg2", 2.11, -.52, .045, .30),
                   cyl("test_table_leg3", 3.01, -1.38, .045, .30),
                   cyl("test_table_leg4", 3.01, -.52, .045, .30)]},
        {"split": "test", "category": "open", "name": "scv1_test_open_no_hit",
         "count": 25, "acceptance": "no_hit", "parts": []},
        {"split": "test", "category": "open_side_only", "name": "scv1_test_forward_clear_side_items",
         "count": 25, "acceptance": "forward_clear",
         "pose_ranges": {"x": (-.10, .20), "y": (-.06, .06), "yaw_rad": (-.03, .03)},
         "parts": [box("test_left_side", 1.55, 1.45, .24, .25), cyl("test_right_pole", 1.55, -1.45, .14)]},
    ]
    for layout in layouts:
        category = layout["category"]
        layout["family"] = ("wall" if "wall" in category or "corridor" in category else
                            "furniture" if category == "furniture" else
                            "open" if category.startswith("open") else "narrow_obstacle")
    return layouts


DIAGNOSTIC_TARGETS = [0.25, 0.35]
CHECK_TARGETS = [(0., 1.0), (18., 1.2), (-18., 1.4), (45., 2.0), (-45., 2.2)]


def scene_xml(parts: list[str]) -> str:
    return (
        '<mujoco model="d435i_ray_adaptation"><compiler angle="radian"/>'
        '<visual><global offwidth="480" offheight="270"/></visual>'
        '<worldbody><geom name="floor" type="plane" size="20 20 .1" rgba=".25 .25 .25 1"/>'
        + "".join(parts)
        + '<body name="base_link" pos="0 0 .355"><freejoint/>'
          '<inertial pos="0 0 0" mass="1" diaginertia=".01 .01 .01"/></body></worldbody></mujoco>'
    )


def target_xml(angle_deg: float, distance: float) -> str:
    """Finite box whose front face meets the given ABS ray at distance."""
    angle = math.radians(angle_deg)
    half_depth, half_width = .08, .12
    cx = -.05 + (distance + half_depth) * math.cos(angle)
    cy = (distance + half_depth) * math.sin(angle)
    return (f'<geom name="target" type="box" pos="{cx:.9f} {cy:.9f} .70" '
            f'euler="0 0 {angle:.12f}" size="{half_depth} {half_width} .70" '
            'rgba=".65 .65 .65 1"/>')


def projected_pixel(point_world, body_pos, body_rot, config: compare.CameraRenderConfig):
    cam = body_pos + body_rot @ np.asarray(config.position_body_m)
    delta = body_rot.T @ (point_world - cam)
    pitch = math.radians(config.pitch_down_deg)
    optical = np.asarray([
        -delta[1],
        -math.sin(pitch) * delta[0] - math.cos(pitch) * delta[2],
        math.cos(pitch) * delta[0] - math.sin(pitch) * delta[2],
    ])
    if optical[2] <= 0:
        return math.nan, math.nan, float(optical[2])
    return (config.fx * optical[0] / optical[2] + config.ppx,
            config.fy * optical[1] / optical[2] + config.ppy,
            float(optical[2]))


def classify(label_m: np.ndarray) -> str | None:
    near = int(np.count_nonzero((label_m >= BIN_NEAR[0]) & (label_m < BIN_NEAR[1])))
    mid = int(np.count_nonzero((label_m >= BIN_MID[0]) & (label_m < BIN_MID[1])))
    far = int(np.count_nonzero((label_m >= BIN_FAR[0]) & (label_m < BIN_FAR[1])))
    cap = int(np.count_nonzero(label_m >= 5.999))
    for name, ok in (
        ("near", near >= 2),
        ("mid", near < 2 and mid >= 2),
        ("far", near < 2 and mid < 2 and far >= 2),
    ):
        if ok and cap <= MAX_CAP_BEAMS[name]:
            return name
    return None


def quota_counts(total: int, quota: tuple[float, float, float]) -> dict[str, int]:
    near = int(round(total * quota[0]))
    mid = int(round(total * quota[1]))
    return {"near": near, "mid": mid, "far": total - near - mid}


def inside_obstacle(model, data, body_id: int, scene_filename: str) -> bool:
    point = data.xpos[body_id, :2]
    inside = False
    for gid in compare.obstacle_geom_ids(mujoco, model, scene_filename):
        delta = point - data.geom_xpos[gid, :2]
        if int(model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_BOX):
            inside |= bool(np.all(np.abs(delta) <= model.geom_size[gid, :2] + .2))
        else:
            inside |= bool(np.linalg.norm(delta) <= model.geom_size[gid, 0] + .2)
    return inside


def set_candidate_frustum(renderer, config: compare.CameraRenderConfig) -> None:
    # MuJoCo 3.2.3 mono rendering averages both virtual eyes; set both.
    for gl_camera in renderer.scene.camera:
        near = float(gl_camera.frustum_near)
        gl_camera.frustum_center = ((config.width - 1.) * .5 - config.ppx) * near / config.fx
        gl_camera.frustum_top = (config.ppy + .5) * near / config.fy
        gl_camera.frustum_bottom = (config.ppy + .5 - config.height) * near / config.fy


def render_native_checked(model, data, renderer, body_id: int, config) -> np.ndarray:
    """Render native depth; retry once if the first frame comes back empty."""
    for attempt in range(2):
        native = generation.render_depth(mujoco, compare, model, data, renderer, body_id,
                                         config, return_native=True)
        if float((native < .1005).mean()) <= .95:
            return native
        if attempt == 0:
            print("[prepare] empty render detected; retrying once", flush=True)
    raise RuntimeError("render returned an empty frame twice; refusing to save it")


def warm_up(model, data, depth_renderer, seg_renderer) -> None:
    data.qpos[:3] = 0.
    data.qpos[2] = .355
    data.qpos[3:7] = (1., 0., 0., 0.)
    mujoco.mj_forward(model, data)
    depth_renderer.update_scene(data, camera=compare.camera_from_body(
        mujoco, data, compare.find_body_id(mujoco, model), CAMERA))
    set_candidate_frustum(depth_renderer, CAMERA)
    depth_renderer.render()
    seg_renderer.update_scene(data, camera=compare.camera_from_body(
        mujoco, data, compare.find_body_id(mujoco, model), CAMERA))
    set_candidate_frustum(seg_renderer, CAMERA)
    seg_renderer.render()


def visibility_rows(model, data, body_id, scene_name, key, split, pose, config,
                     label_m, clean_input, seg_renderer):
    seg_renderer.update_scene(data, camera=compare.camera_from_body(
        mujoco, data, body_id, config))
    set_candidate_frustum(seg_renderer, config)
    seg = seg_renderer.render()
    obs_ids = list(compare.obstacle_geom_ids(mujoco, model, Path(scene_name).name))
    visible_geom = {gid: int(np.count_nonzero((seg[:, :, 0] == gid) &
                                              (seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))))
                    for gid in obs_ids}
    body_pos = data.xpos[body_id].copy()
    body_rot = data.xmat[body_id].reshape(3, 3).copy()
    origin = body_pos + body_rot @ np.asarray([compare.RAY2D_X0, compare.RAY2D_Y0, 0.])
    rows = []
    for ray_index, angle_deg in enumerate(RAY_ANGLES):
        angle = math.radians(angle_deg)
        direction = body_rot @ np.asarray([math.cos(angle), math.sin(angle), 0.])
        # ABS labels are 2-D XY footprints (see compute_geometric_ray_log),
        # independent of geom height. Keep a separate 3-D ray at base height
        # as a diagnostic; furniture/table tops can intentionally differ.
        world_theta = math.atan2(float(direction[1]), float(direction[0]))
        ctheta, stheta = math.cos(world_theta), math.sin(world_theta)
        best_distance, label_geom_id = 6.0, -1
        for candidate_id in obs_ids:
            geom_type = int(model.geom_type[candidate_id])
            center = data.geom_xpos[candidate_id]
            size = model.geom_size[candidate_id]
            if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
                candidate_distance = compare.ray_box_distance(
                    origin[:2], direction[:2], center, data.geom_xmat[candidate_id], size)
            else:
                candidate_distance = compare.ray_round_distance(
                    mujoco, model, candidate_id, float(origin[0]), float(origin[1]),
                    ctheta, stheta, center, size)
            if math.isfinite(candidate_distance):
                candidate_distance = max(candidate_distance, compare.RAY2D_MIN_DIST)
                if candidate_distance < best_distance:
                    best_distance, label_geom_id = float(candidate_distance), candidate_id
        target_m = best_distance
        if label_geom_id < 0:
            hit_name, u, v, zcam = "range_cap", math.nan, math.nan, math.nan
            inside, sampled_depth, sample_u, sample_v = False, math.nan, -1, -1
            object_visible, visible_pixel_count, seg_at_hit = False, 0, -1
        else:
            geom_id = label_geom_id
            hit = origin + direction * target_m
            u, v, zcam = projected_pixel(hit, body_pos, body_rot, config)
            inside = bool(zcam > 0 and -.5 <= u < config.width - .5 and -.5 <= v < config.height - .5)
            hit_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom{geom_id}"
            visible_pixel_count = visible_geom.get(geom_id, 0)
            object_visible = visible_pixel_count > 0
            if inside:
                model_x = int(np.clip(np.rint((u - 1.) / 3.), 0, 159))
                model_y = int(np.clip(np.rint((v - 1.) / 3.), 0, 89))
                sample_u, sample_v = 3 * model_x + 1, 3 * model_y + 1
                sampled_depth = float(clean_input[model_y, model_x])
                px, py = int(np.clip(np.rint(u), 0, config.width - 1)), int(np.clip(np.rint(v), 0, config.height - 1))
                seg_at_hit = int(seg[py, px, 0])
            else:
                sampled_depth, sample_u, sample_v, seg_at_hit = math.nan, -1, -1, -1
        if abs(target_m - float(label_m[ray_index])) > 1e-4:
            raise RuntimeError(f"label mismatch {scene_name} {key} ray {angle_deg}: "
                               f"{target_m} vs {label_m[ray_index]}")
        height_gid = np.asarray([-1], dtype=np.int32)
        height_distance = float(mujoco.mj_ray(model, data, origin, direction, None, 1, body_id, height_gid))
        height_distance = height_distance if 0. <= height_distance <= 6. else 6.0
        rows.append({
            "split": split, "scene": scene_name, "sample": key,
            "pose_x_m": pose.x, "pose_y_m": pose.y, "pose_z_m": pose.z, "pose_yaw_deg": math.degrees(pose.yaw),
            "pitch_down_deg": config.pitch_down_deg,
            "ray_index": ray_index, "ray_angle_deg": angle_deg,
            "label_m": float(label_m[ray_index]), "hit_geom": hit_name,
            "base_height_3d_raycast_m": height_distance,
            "base_height_3d_raycast_geom": (mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM, int(height_gid[0])) or f"geom{int(height_gid[0])}"
                if int(height_gid[0]) >= 0 else "range_cap"),
            "label_is_height_independent_xy_footprint": True,
            "label_hit_u_px": float(u), "label_hit_v_px": float(v),
            "label_hitpoint_in_image": inside,
            "obstacle_has_visible_pixels_in_render": object_visible,
            "visible_geometry_pixels": visible_pixel_count,
            "segmentation_geom_at_hitpoint_pixel": seg_at_hit,
            "nearest_center_sampler_source_u_px": sample_u,
            "nearest_center_sampler_source_v_px": sample_v,
            "model_input_depth_at_hitpoint_m": sampled_depth,
            "projected_hit_optical_z_m": zcam,
        })
    return rows


def save_pair(destination: Path, key: str, split: str, masks, mask_counter: dict,
              native, label_m, labels: dict, records: dict, pose, scene_name):
    raw = np.rint(native * 1000.).clip(0, 65535).astype("<u2")
    clean, _ = downsample_depth_centers(raw.astype(np.float32) * .001, np.ones(raw.shape, dtype=bool))
    clean = np.clip(clean, compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST).astype(np.float32)
    group = masks[split]
    mask_path, raw_valid = group[mask_counter[split] % len(group)]
    mask_counter[split] += 1
    masked_u16 = raw.copy()
    masked_u16[~raw_valid] = 0
    filled = librealsense_2584_mode1_source(masked_u16)
    added = (~raw_valid) & (filled > 0)
    remaining = filled == 0
    prepared = filled.astype(np.float32) * .001
    prepared[remaining] = 6.
    masked, _ = downsample_depth_centers(prepared, np.ones(prepared.shape, dtype=bool))
    masked = np.clip(masked, compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST).astype(np.float32)
    sampled_unknown, _ = downsample_depth_centers(remaining.astype(np.float32), np.ones(prepared.shape, dtype=bool))
    for kind, image in (("clean", clean), ("masked", masked)):
        name = f"{key}_{kind}"
        np.save(destination / f"{name}.npy", image)
        labels[name] = label_m
    records[key] = {
        "key": key, "split": split, "layout": scene_name, "pose": {
            "x": pose.x, "y": pose.y, "z": pose.z, "yaw_rad": pose.yaw,
            "pitch_down_deg": None},
        "mask_source": str(mask_path.relative_to(ROOT)),
        "paired_clean_masked_same_label": True,
        "raw_unknown_fraction": float((~raw_valid).mean()),
        "sdk_mode1_filled_fraction": float(added.mean()),
        "remaining_unknown_fraction": float(remaining.mean()),
        "model_input_unknown_fraction_after_fill": float(sampled_unknown.mean()),
        "augmentation_note": "real captured zero mask transplanted onto synthetic geometry; not a sensor physics simulation",
    }
    return clean, masked


def run_layout(mujoco_module, compare_module, generation_module, layout, output, masks,
               mask_counter, rng, max_attempts, poses_override, all_rows, records):
    scene_path = output / "scenes" / f"{layout['name']}.xml"
    scene_path.write_text(scene_xml(layout["parts"]))
    model = mujoco_module.MjModel.from_xml_path(str(scene_path))
    data = mujoco_module.MjData(model)
    mujoco_module.mj_forward(model, data)
    body_id = compare_module.find_body_id(mujoco_module, model)
    config = CAMERA
    depth_renderer = compare_module.make_depth_renderer(mujoco_module, model, config)
    seg_renderer = mujoco_module.Renderer(model, height=270, width=480)
    seg_renderer.enable_segmentation_rendering()
    warm_up(model, data, depth_renderer, seg_renderer)
    split = layout["split"]
    destination = output / "data" / split
    destination.mkdir(parents=True, exist_ok=True)
    labels = {}
    label_path = destination / "label.pkl"
    if label_path.exists():
        with label_path.open("rb") as f:
            labels = pickle.load(f)
    total = poses_override or layout["count"]
    quota = ( {"open": total} if layout.get("acceptance") else
              quota_counts(total, layout["quota"]) )
    accepted = {name: 0 for name in quota}
    accepted_total = 0
    attempts = 0
    try:
        while any(accepted[c] < quota[c] for c in accepted) and attempts < max_attempts:
            attempts += 1
            pose_ranges = layout.get("pose_ranges", {})
            x = float(rng.uniform(*pose_ranges.get("x", POSE_X_RANGE)))
            y = float(rng.uniform(*pose_ranges.get("y", POSE_Y_RANGE)))
            yaw = float(rng.uniform(*pose_ranges.get("yaw_rad", POSE_YAW_RANGE)))
            z = float(rng.uniform(*BASE_Z_RANGE_M))
            pitch = float(rng.uniform(*PITCH_RANGE_DEG))
            pose = compare_module.PoseSample(attempts, "d435i_ray_adaptation", x, y, z, yaw)
            compare_module.apply_sample(mujoco_module, model, data, pose)
            if inside_obstacle(model, data, body_id, scene_path.name):
                continue
            label_m = np.clip(np.exp2(compare_module.compute_geometric_ray_log(
                mujoco_module, model, data, body_id, scene_path.name)),
                compare_module.RAY2D_MIN_DIST, compare_module.RAY2D_MAX_DIST).astype(np.float32)
            if float(label_m.min()) < MIN_CLEARANCE_M:
                continue
            acceptance = layout.get("acceptance")
            if acceptance == "no_hit":
                cls = "open" if np.all(label_m >= 5.999) else None
            elif acceptance == "forward_clear":
                # Central five rays (−18°…+18°) must remain clear; at least one
                # outer ray must hit a side object so this is not another empty scene.
                cls = ("open" if np.all(label_m[3:8] >= 5.999) and
                       np.any(label_m[[0, 1, 2, 8, 9, 10]] < 5.999) else None)
            elif acceptance:
                raise ValueError(f"unknown layout acceptance: {acceptance}")
            else:
                cls = classify(label_m)
            if cls is None or accepted[cls] >= quota[cls]:
                continue
            pose_config = replace(config, pitch_down_deg=pitch)
            native = render_native_checked(model, data, depth_renderer, body_id, pose_config)
            key = f"{layout['name']}_pose{accepted_total:03d}"
            clean, _masked = save_pair(destination, key, split, masks, mask_counter,
                                       native, label_m, labels, records, pose, layout["name"])
            records[key]["pose"]["pitch_down_deg"] = pitch
            records[key]["pose_class"] = cls
            records[key]["category"] = layout.get("category", "unspecified")
            rows = visibility_rows(model, data, body_id, scene_path.name, key, split,
                                   pose, pose_config, label_m, clean, seg_renderer)
            all_rows.extend(rows)
            accepted[cls] += 1
            accepted_total += 1
            print(f"[prepare] {layout['name']} {key} class={cls} attempts={attempts}", flush=True)
    finally:
        depth_renderer.close()
        seg_renderer.close()
        seg_renderer._mjr_context = None
    if any(accepted[c] < quota[c] for c in accepted):
        raise RuntimeError(f"{layout['name']}: unmet quota {accepted} vs {quota} after {attempts} attempts")
    with label_path.open("wb") as f:
        pickle.dump(labels, f)
    return {"scene": layout["name"], "split": split,
            "category": layout.get("category", "unspecified"), "attempts": attempts,
            "accepted": accepted, "images": 2 * sum(accepted.values())}


def run_diagnostics(output):
    """0.1-0.45 m is a visibility diagnostic only; labels keep the ABS 0.1 m floor."""
    diag_dir = output / "diagnostic"
    diag_dir.mkdir(parents=True, exist_ok=True)
    results = {"use": "visibility diagnostic only; excluded from train/validation/test",
               "samples": []}
    for distance in DIAGNOSTIC_TARGETS:
        name = f"diagnostic_center_{distance:.2f}m"
        scene_path = output / "scenes" / f"{name}.xml"
        scene_path.write_text(scene_xml([wall("backdrop", 4.0), target_xml(0., distance)]))
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        body_id = compare.find_body_id(mujoco, model)
        diagnostic_base_z = float(BASE_Z_RANGE_M[1])
        pose = compare.PoseSample(0, "diagnostic", 0., 0., diagnostic_base_z, 0.)
        compare.apply_sample(mujoco, model, data, pose)
        config = replace(CAMERA, pitch_down_deg=float(sum(PITCH_RANGE_DEG) / 2.0))
        renderer = compare.make_depth_renderer(mujoco, model, config)
        detail_renderer = mujoco.Renderer(model, height=270, width=480)
        detail_renderer.enable_segmentation_rendering()
        try:
            warm_up(model, data, renderer, detail_renderer)
            compare.apply_sample(mujoco, model, data, pose)
            native = render_native_checked(model, data, renderer, body_id, config)
            native_raw = np.rint(native * 1000.).clip(0, 65535).astype("<u2")
            clean, _ = downsample_depth_centers(native_raw.astype(np.float32) * .001,
                                                np.ones(native_raw.shape, dtype=bool))
            label_m = np.exp2(compare.compute_geometric_ray_log(
                mujoco, model, data, body_id, scene_path.name)).astype(np.float32)
            rows = visibility_rows(model, data, body_id, scene_path.name, name, "diagnostic",
                                   pose, config, label_m, clean, detail_renderer)
            np.save(diag_dir / f"{name}_clean.npy", clean)
            center = rows[5]
            results["samples"].append({
                "sample": name, "target_distance_m": distance,
                "label_m": float(label_m[5]),
                "center_hit_geom": center["hit_geom"],
                "label_hitpoint_in_image": center["label_hitpoint_in_image"],
                "label_hit_u_px": center["label_hit_u_px"],
                "label_hit_v_px": center["label_hit_v_px"],
                "projected_hit_optical_z_m": center["projected_hit_optical_z_m"],
                "obstacle_has_visible_pixels_in_render": center["obstacle_has_visible_pixels_in_render"],
            })
        finally:
            renderer.close()
            detail_renderer.close()
            detail_renderer._mjr_context = None
    (diag_dir / "diagnostic.json").write_text(json.dumps(results, indent=2))


def run_geometry_checks(output):
    """A few known angles/distances, one target per scene, to check sign/order/projection."""
    check_dir = output / "geometry_checks"
    check_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for angle_deg, distance in CHECK_TARGETS:
        scene_path = check_dir / f"check_{angle_deg:+.0f}_{distance:.1f}m.xml"
        # No backdrop: every other beam must stay at the 6 m range cap.
        scene_path.write_text(scene_xml([target_xml(angle_deg, distance)]))
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        body_id = compare.find_body_id(mujoco, model)
        geometry_base_z = float(BASE_Z_RANGE_M[1])
        pose = compare.PoseSample(0, "geometry_check", 0., 0., geometry_base_z, 0.)
        compare.apply_sample(mujoco, model, data, pose)
        config = replace(CAMERA, pitch_down_deg=float(sum(PITCH_RANGE_DEG) / 2.0))
        renderer = compare.make_depth_renderer(mujoco, model, config)
        detail_renderer = mujoco.Renderer(model, height=270, width=480)
        detail_renderer.enable_segmentation_rendering()
        try:
            warm_up(model, data, renderer, detail_renderer)
            compare.apply_sample(mujoco, model, data, pose)
            native = render_native_checked(model, data, renderer, body_id, config)
            raw = np.rint(native * 1000.).clip(0, 65535).astype("<u2")
            clean, _ = downsample_depth_centers(raw.astype(np.float32) * .001,
                                                np.ones(raw.shape, dtype=bool))
            label_m = np.exp2(compare.compute_geometric_ray_log(
                mujoco, model, data, body_id, scene_path.name)).astype(np.float32)
            rows = visibility_rows(model, data, body_id, scene_path.name, scene_path.stem,
                                   "geometry_check", pose, config, label_m, clean, detail_renderer)
            ray_index = int(round((angle_deg + 45.) / 9.))
            row = rows[ray_index]
            hit_expected = all(abs(float(label_m[i]) - 6.) < 1e-3 for i in range(11) if i != ray_index)
            entry = {
                "angle_deg": angle_deg, "distance_m": distance, "ray_index": ray_index,
                "label_m": float(label_m[ray_index]),
                "label_matches_distance": bool(abs(float(label_m[ray_index]) - distance) < 1e-3),
                "other_beams_are_range_cap": hit_expected,
                "hit_geom": row["hit_geom"],
                "hit_u_px": row["label_hit_u_px"], "hit_v_px": row["label_hit_v_px"],
                "hitpoint_in_image": row["label_hitpoint_in_image"],
                # A body-forward ray need not project to ppx when the camera is laterally offset.
                "u_side_consistent": bool((angle_deg > 0 and row["label_hit_u_px"] < CAMERA.ppx) or
                                          (angle_deg < 0 and row["label_hit_u_px"] > CAMERA.ppx) or
                                          angle_deg == 0),
                "model_input_depth_at_hitpoint_m": row["model_input_depth_at_hitpoint_m"],
                "projected_hit_optical_z_m": row["projected_hit_optical_z_m"],
            }
            entries.append(entry)
            if not (entry["label_matches_distance"] and entry["other_beams_are_range_cap"]
                    and entry["u_side_consistent"]):
                raise RuntimeError(f"geometry check failed: {entry}")
        finally:
            renderer.close()
            detail_renderer.close()
            detail_renderer._mjr_context = None
    # Broad-wall check: front face x=1.60 m, ABS origin x=-0.05 m.
    # Thus the 0-degree label must be 1.65 m while camera-center Z is ~1.30 m.
    wall_path = check_dir / "check_wide_wall_analytic.xml"
    wall_path.write_text(scene_xml([wall("analytic_wide_wall", 1.65, 4.0)]))
    wall_model = mujoco.MjModel.from_xml_path(str(wall_path))
    wall_data = mujoco.MjData(wall_model)
    wall_body_id = compare.find_body_id(mujoco, wall_model)
    wall_pose = compare.PoseSample(0, "wide_wall_check", 0., 0., float(BASE_Z_RANGE_M[0]), 0.)
    compare.apply_sample(mujoco, wall_model, wall_data, wall_pose)
    wall_labels = np.clip(np.exp2(compare.compute_geometric_ray_log(
        mujoco, wall_model, wall_data, wall_body_id, wall_path.name)),
        compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST)
    if abs(float(wall_labels[5]) - 1.65) > 1e-5:
        raise RuntimeError(f"wide wall analytic 0-degree label mismatch: {wall_labels[5]}")
    # Empty world except floor: every beam must be an ABS 6 m range-cap label.
    open_path = check_dir / "check_open_no_hit.xml"
    open_path.write_text(scene_xml([]))
    open_model = mujoco.MjModel.from_xml_path(str(open_path))
    open_data = mujoco.MjData(open_model)
    open_body_id = compare.find_body_id(mujoco, open_model)
    compare.apply_sample(mujoco, open_model, open_data,
                         compare.PoseSample(0, "open_check", 0., 0., float(BASE_Z_RANGE_M[0]), 0.))
    open_labels = np.clip(np.exp2(compare.compute_geometric_ray_log(
        mujoco, open_model, open_data, open_body_id, open_path.name)),
        compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST)
    if not np.allclose(open_labels, 6.0, atol=1e-6):
        raise RuntimeError(f"open-scene labels are not all 6 m: {open_labels}")
    side_path = check_dir / "check_open_forward_clear_side_items.xml"
    side_path.write_text(scene_xml([box("left_side", 1.55, 1.45, .24, .25),
                                    cyl("right_side", 1.55, -1.45, .14)]))
    side_model = mujoco.MjModel.from_xml_path(str(side_path))
    side_data = mujoco.MjData(side_model)
    side_body_id = compare.find_body_id(mujoco, side_model)
    compare.apply_sample(mujoco, side_model, side_data,
                         compare.PoseSample(0, "side_open_check", 0., 0., float(BASE_Z_RANGE_M[0]), 0.))
    side_labels = np.clip(np.exp2(compare.compute_geometric_ray_log(
        mujoco, side_model, side_data, side_body_id, side_path.name)),
        compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST)
    if not (np.all(side_labels[3:8] >= 5.999) and
            np.any(side_labels[[0, 1, 2, 8, 9, 10]] < 5.999)):
        raise RuntimeError(f"side-only open scene did not leave central rays clear: {side_labels}")
    entries.extend([
        {"check": "wide_wall_analytic", "wall_front_x_m": 1.60,
         "abs_ray_origin_x_m": compare.RAY2D_X0, "expected_zero_degree_label_m": 1.65,
         "actual_zero_degree_label_m": float(wall_labels[5]),
         "camera_center_forward_distance_m": 1.30, "passed": True},
        {"check": "empty_open_scene", "ray_labels_m": open_labels.tolist(),
         "all_beams_range_cap_6m": True},
        {"check": "side_only_forward_clear", "ray_labels_m": side_labels.tolist(),
         "central_minus18_to_plus18_clear": True, "outer_beam_hit": True},
    ])
    (check_dir / "geometry_checks.json").write_text(json.dumps({
        "camera": {"housing_proxy_base_m": list(CAMERA.position_body_m),
                   "pitch_down_deg": float(sum(PITCH_RANGE_DEG) / 2.0),
                   "base_z_m": float(BASE_Z_RANGE_M[1]),
                   "intrinsics": [CAMERA.width, CAMERA.height, CAMERA.fx, CAMERA.fy, CAMERA.ppx, CAMERA.ppy]},
        "rays_deg": RAY_ANGLES, "entries": entries,
        "note": "one synthetic target per scene; checks label distance, beam index/sign and projection, not real-camera accuracy",
    }, indent=2))


def audit_generated_data(output, records, layouts, args):
    splits = tuple(s for s in ("train", "validation", "test") if any(l["split"] == s for l in layouts))
    by_split = {split: [key for key, rec in records.items() if rec["split"] == split]
                for split in splits}
    layouts_by_split = {split: {rec["layout"] for rec in records.values() if rec["split"] == split}
                        for split in splits}
    if any(layouts_by_split[a] & layouts_by_split[b]
           for i, a in enumerate(splits) for b in splits[i + 1:]):
        raise RuntimeError("layout appears in more than one split")
    expected_counts = {
        split: sum((args.poses_per_layout or layout["count"])
                   for layout in layouts if layout["split"] == split)
        for split in splits
    }
    if {s: len(by_split[s]) for s in splits} != expected_counts:
        raise RuntimeError(f"pose counts differ from layout plan: {by_split} vs {expected_counts}")
    if args.coverage_supplement_probe:
        if any(split == "test" for split in splits):
            raise RuntimeError("coverage supplement probe must not generate a test split")
    if args.test_layout_set == "current_mount_v2" and not args.coverage_supplement_probe:
        new_test_names = {layout["name"] for layout in layouts if layout["split"] == "test"}
        if not new_test_names or new_test_names & {"test_mixed", "test_open"}:
            raise RuntimeError("current-mount test layouts must be distinct from adaptation_v1 layouts")
    if args.scene_coverage_v1:
        families_by_split = {split: {layout["family"] for layout in layouts if layout["split"] == split}
                             for split in splits}
        if any("wall" not in families_by_split[s] or "furniture" not in families_by_split[s]
               for s in ("train", "validation", "test")):
            raise RuntimeError(f"wall/furniture layout coverage missing by split: {families_by_split}")
        if len({layout["name"] for layout in layouts}) != len(layouts):
            raise RuntimeError("scene-coverage layout names are not unique")

    split_audit = {}
    for split in splits:
        data_dir = output / "data" / split
        with (data_dir / "label.pkl").open("rb") as f:
            labels = pickle.load(f)
        expected_image_names = {f"{key}_{kind}" for key in by_split[split]
                                for kind in ("clean", "masked")}
        found_image_names = {path.stem for path in data_dir.glob("*.npy")}
        if found_image_names != expected_image_names:
            raise RuntimeError(f"{split}: clean/masked image keys differ")
        expected_label_names = {f"{key}_{kind}" for key in by_split[split]
                                for kind in ("clean", "masked")}
        if set(labels) != expected_label_names:
            raise RuntimeError(f"{split}: label keys differ from paired images")
        empty_frames = 0
        for key in by_split[split]:
            clean = np.load(data_dir / f"{key}_clean.npy").astype(np.float32)
            masked = np.load(data_dir / f"{key}_masked.npy").astype(np.float32)
            if clean.shape != (90, 160) or masked.shape != clean.shape:
                raise RuntimeError(f"{key}: wrong model input shape")
            if not np.isfinite(clean).all() or not np.isfinite(masked).all():
                raise RuntimeError(f"{key}: non-finite model input")
            if clean.min() < .1 or clean.max() > 6. or masked.min() < .1 or masked.max() > 6.:
                raise RuntimeError(f"{key}: model input outside [0.1,6] m")
            if np.all(clean <= .1005) or np.all(masked <= .1005):
                empty_frames += int(np.all(clean <= .1005)) + int(np.all(masked <= .1005))
                raise RuntimeError(f"{key}: empty clipped-depth frame")
            clean_label = np.asarray(labels[f"{key}_clean"], dtype=np.float32)
            masked_label = np.asarray(labels[f"{key}_masked"], dtype=np.float32)
            if clean_label.shape != (11,) or masked_label.shape != (11,):
                raise RuntimeError(f"{key}: expected 11 ray labels")
            if not np.isfinite(clean_label).all() or not np.array_equal(clean_label, masked_label):
                raise RuntimeError(f"{key}: invalid or mismatched paired labels")
            if clean_label.min() < compare.RAY2D_MIN_DIST or clean_label.max() > compare.RAY2D_MAX_DIST:
                raise RuntimeError(f"{key}: labels outside ABS range")
            if args.scene_coverage_v1:
                layout = next(item for item in layouts if item["name"] == records[key]["layout"])
                acceptance = layout.get("acceptance")
                if acceptance == "no_hit" and not np.all(clean_label >= 5.999):
                    raise RuntimeError(f"{key}: no-hit layout contains a non-cap ray")
                if acceptance == "forward_clear" and not (
                        np.all(clean_label[3:8] >= 5.999) and
                        np.any(clean_label[[0, 1, 2, 8, 9, 10]] < 5.999)):
                    raise RuntimeError(f"{key}: forward-clear side-only label condition failed")
        split_audit[split] = {
            "poses": len(by_split[split]),
            "paired_images": 2 * len(by_split[split]),
            "layouts": sorted(layouts_by_split[split]),
            "empty_input_frames": empty_frames,
            "all_inputs_finite_shape_range_ok": True,
            "paired_labels_equal_11d_meters": True,
        }
    return {"passed": True, "expected_pose_counts": expected_counts,
            "split_audit": split_audit,
        "test_layout_set": ("scene_coverage_v1" if args.scene_coverage_v1 else
                            None if args.coverage_supplement_probe else args.test_layout_set),
            "test_split_generated": any(layout["split"] == "test" for layout in layouts),
            "test_layouts_new_relative_to_adaptation_v1": (
                True if args.scene_coverage_v1 else False if args.coverage_supplement_probe
                else args.test_layout_set in ("current_mount_v2", "final_holdout_v1"))}


def summarize(output, all_rows, records, layout_stats, masks, elapsed_s, bytes_saved, args, layouts, audit):
    splits = tuple(s for s in ("train", "validation", "test") if any(l["split"] == s for l in layouts))
    bin_counts = {s: {"below_0p45": 0, "near_045_1p5": 0, "mid_1p5_3": 0,
                      "far_3_6": 0, "cap_6": 0} for s in splits}
    pose_classes = {s: {"near": 0, "mid": 0, "far": 0, "open": 0} for s in splits}
    for record in records.values():
        pose_classes[record["split"]][record["pose_class"]] += 1
    per_ray = {s: [{"count": 0, "near": 0, "cap": 0, "sum": 0., "min": 99., "max": 0.} for _ in RAY_ANGLES]
               for s in splits}
    for row in all_rows:
        if row["split"] not in splits:
            continue
        d = row["label_m"]
        key = ("cap_6" if d >= 5.999 else "far_3_6" if d >= BIN_FAR[0]
               else "mid_1p5_3" if d >= BIN_MID[0] else "near_045_1p5" if d >= BIN_NEAR[0] else "below_0p45")
        bin_counts[row["split"]][key] += 1
        stat = per_ray[row["split"]][row["ray_index"]]
        stat["count"] += 1
        stat["near"] += int(row["label_m"] < 1.5)
        stat["cap"] += int(d >= 5.999)
        stat["sum"] += d
        stat["min"] = min(stat["min"], d)
        stat["max"] = max(stat["max"], d)
    for s in splits:
        for stat in per_ray[s]:
            stat["mean"] = stat["sum"] / max(1, stat["count"]) if stat["count"] else None
            stat["min"] = stat["min"] if stat["count"] else None
            stat["max"] = stat["max"] if stat["count"] else None
            stat.pop("sum", None)
    hit_rows = [r for r in all_rows if r["split"] in splits and r["hit_geom"] != "range_cap"]
    body_height_disagreements = [r for r in hit_rows
                                 if abs(float(r["base_height_3d_raycast_m"]) - float(r["label_m"])) > .02]
    outside = [r for r in hit_rows if not r["label_hitpoint_in_image"]]
    visibility = {
        "hit_label_count": len(hit_rows),
        "hitpoint_inside_image_count": len(hit_rows) - len(outside),
        "hitpoint_outside_image_count": len(outside),
        "outside_hitpoint_but_some_obstacle_surface_visible": sum(
            r["obstacle_has_visible_pixels_in_render"] for r in outside),
        "outside_hitpoint_and_no_obstacle_pixels_in_render": sum(
            not r["obstacle_has_visible_pixels_in_render"] for r in outside),
        "inside_hitpoint_but_hit_geom_pixel_absent": sum(
            r["segmentation_geom_at_hitpoint_pixel"] < 0 for r in hit_rows if r["label_hitpoint_in_image"]),
        "method": "MuJoCo segmentation rendering with candidate K; each split counted separately",
    }
    per_split_visibility = {}
    for s in splits:
        rows = [r for r in hit_rows if r["split"] == s]
        out = [r for r in rows if not r["label_hitpoint_in_image"]]
        per_split_visibility[s] = {
            "hit_label_count": len(rows), "inside": len(rows) - len(out), "outside": len(out),
            "outside_but_surface_visible": sum(r["obstacle_has_visible_pixels_in_render"] for r in out),
            "outside_and_no_surface": sum(not r["obstacle_has_visible_pixels_in_render"] for r in out),
        }
    mask_stats = {}
    for s in splits:
        vals = [r for r in records.values() if r["split"] == s]
        mask_stats[s] = {
            "poses": len(vals),
            "mask_sources_used": len({r["mask_source"] for r in vals}),
            "raw_unknown_fraction_mean": float(np.mean([r["raw_unknown_fraction"] for r in vals])),
            "raw_unknown_fraction_min": float(np.min([r["raw_unknown_fraction"] for r in vals])),
            "raw_unknown_fraction_max": float(np.max([r["raw_unknown_fraction"] for r in vals])),
            "filled_fraction_mean": float(np.mean([r["sdk_mode1_filled_fraction"] for r in vals])),
            "remaining_unknown_fraction_mean": float(np.mean([r["remaining_unknown_fraction"] for r in vals])),
            "model_input_unknown_fraction_mean": float(
                np.mean([r["model_input_unknown_fraction_after_fill"] for r in vals])),
        }
    total_rays = {s: sum(stat["count"] for stat in per_ray[s]) for s in splits}
    category_counts = {}
    for layout in layouts:
        category_counts[layout.get("category", "unspecified")] = (
            category_counts.get(layout.get("category", "unspecified"), 0) + layout["count"])
    frontal_wall = []
    for key, record in records.items():
        if record["layout"] != "scv1_train_wall_front":
            continue
        pose = record["pose"]
        yaw = float(pose["yaw_rad"])
        camera_x = float(pose["x"]) + CAMERA.position_body_m[0] * math.cos(yaw) - CAMERA.position_body_m[1] * math.sin(yaw)
        camera_distance = (1.60 - camera_x) / max(1e-9, math.cos(yaw))
        image = np.load(output / "data" / record["split"] / f"{key}_clean.npy")
        roi = image[20:70, 50:110]
        center_median = float(np.median(roi))
        frontal_wall.append({"sample": key, "camera_to_wall_horizontal_m": camera_distance,
                             "center_roi_median_m": center_median,
                             "center_roi_fraction_0p7_1p3m": float(np.mean((roi >= .7) & (roi <= 1.3))),
                             "center_roi_fraction_within_0p05m_of_median": float(np.mean(np.abs(roi - center_median) <= .05))})
    summary = {
        "training_performed": False,
        "dataset_kind": "offline geometry/mask adaptation candidate for a later bounded training run",
        "seed": args.seed,
        "layout_selection": ("scene_coverage_v1" if args.scene_coverage_v1 else
                             "independent_eval_v1" if args.independent_eval else
                             "coverage_supplement_probe_v1" if args.coverage_supplement_probe
                             else args.test_layout_set),
        "test_layout_set": args.test_layout_set,
        "split_layouts": {split: [layout["name"] for layout in layouts if layout["split"] == split]
                           for split in splits},
        "dataset_audit": audit,
        "elapsed_seconds": elapsed_s,
        "bytes_on_disk": bytes_saved,
        "poses_total": len(records),
        "images_total": 2 * len(records),
        "poses_by_split": {s: sum(1 for r in records.values() if r["split"] == s) for s in splits},
        "layout_stats": layout_stats,
        "planned_poses_by_category": category_counts,
        "layout_plan": [{"name": layout["name"], "split": layout["split"],
                         "category": layout.get("category"), "family": layout.get("family"),
                         "planned_poses": layout["count"], "acceptance": layout.get("acceptance", "near_mid_far_quota"),
                         "pose_ranges": layout.get("pose_ranges", "global default"),
                         "geometry_summary": layout.get("geometry_summary", "see scene XML")}
                        for layout in layouts],
        "frontal_wall_coverage": {
            "layout": "scv1_train_wall_front", "front_face_x_m": 1.60,
            "target_camera_distance_band_m": [0.7, 1.3],
            "samples_in_band": sum(.7 <= r["camera_to_wall_horizontal_m"] <= 1.3 for r in frontal_wall),
            "samples_total": len(frontal_wall), "samples": frontal_wall,
            "center_roi_model_input_pixels": "rows 20:70, columns 50:110 (50x60 center region)",
            "this_is_coverage_not_prediction_accuracy": True},
        "pose_class_counts": pose_classes,
        "label_bin_counts": bin_counts,
        "label_bin_fraction": {s: {k: v / max(1, total_rays[s]) for k, v in bin_counts[s].items()}
                               for s in splits},
        "cap_fraction": {s: bin_counts[s]["cap_6"] / max(1, total_rays[s]) for s in splits},
        "near_fraction": {s: bin_counts[s]["near_045_1p5"] / max(1, total_rays[s]) for s in splits},
        "per_ray_label_stats": per_ray,
        "hitpoint_visibility": visibility,
        "label_geometry_semantics": {
            "labels": "existing ABS horizontal XY footprint rays, independent of obstacle geom height",
            "visibility_cross_check": "label hitpoint is projected from the 2D XY intersection; base-height 3D MuJoCo ray is retained separately",
            "hit_label_count_with_base_height_3d_difference_over_0p02m": len(body_height_disagreements),
            "examples": [{"sample": r["sample"], "ray_deg": r["ray_angle_deg"],
                          "label_xy_m": r["label_m"], "base_height_3d_raycast_m": r["base_height_3d_raycast_m"],
                          "label_geom": r["hit_geom"], "base_height_3d_geom": r["base_height_3d_raycast_geom"]}
                         for r in body_height_disagreements[:20]],
        },
        "per_split_visibility": per_split_visibility,
        "mask_augmentation": mask_stats,
        "camera": {"housing_proxy_base_m": list(CAMERA.position_body_m),
                   "pitch_range_deg": list(PITCH_RANGE_DEG),
                   "base_height_range_m": list(BASE_Z_RANGE_M),
                   "parameter_source": "explicit command-line candidate; position is left-IR front-glass proxy, not calibrated optical center",
                   "yaw_roll": "assumed zero; not measured",
                   "depth_intrinsics": [CAMERA.width, CAMERA.height, CAMERA.fx, CAMERA.fy,
                                        CAMERA.ppx, CAMERA.ppy]},
        "contract": {"rays_deg": RAY_ANGLES, "ray_origin_base_m": [compare.RAY2D_X0, compare.RAY2D_Y0],
                     "label_range_m": [compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST],
                     "pose_uniform_ranges_default": {"x": list(POSE_X_RANGE), "y": list(POSE_Y_RANGE),
                                                     "yaw_rad": list(POSE_YAW_RANGE)}},
        "processing": {
            "training_and_inference_values": "native Z metres -> mm Z16 -> real zero mask -> librealsense 2.58.4 source-equivalent mode1 fill -> remaining zero to 6 m -> centre sample (3i+1,3j+1) -> clip 0.1-6 m -> log2 -> 3 channels",
            "field_candidate_equivalent": "depth mm * 0.001 -> device SDK mode1 fill -> centre sample -> residual zero to 6 m -> clip/log2/3 channels",
            "field_default_strict_path": "rejects a frame when the centre pixel is still unknown; the 6 m replacement is not the default",
        },
        "limitations": [
            "housing centre is only an optical-origin proxy; pitch and base height are candidates, not precision extrinsic calibration",
            "camera yaw/roll and glass-to-optical-center offset are unmeasured",
            "mask transplant is an augmentation, not a sensor physics simulation; mask frames are reused across poses",
            "residual unknown filled to 6 m is a candidate numeric policy, not measured free space",
            "all labels come from scene geometry; no neural prediction is used as a label",
            "0.1-0.45 m is a separate visibility diagnostic and is not part of train/validation/test",
            "synthetic held-out layouts are development data, not real-world accuracy evidence",
        ],
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def save_representative_inputs(output, records, layouts):
    """Save a compact panel of the actual 160x90 clean training inputs."""
    import matplotlib.pyplot as plt

    desired = [layout["name"] for layout in layouts if layout["name"] in {
        "scv1_train_wall_front", "scv1_train_wall_oblique_corner",
        "scv1_train_furniture_cabinet_sofa_table", "scv1_train_posts_chairlegs_doorway",
        "scv1_train_open_side_far_wall", "scv1_val_far_wall_unseen", "scv1_val_clear_no_hit",
        "scv1_val_forward_clear_side_items", "scv1_test_wall_corner_unseen",
        "scv1_test_far_wall_unseen", "scv1_test_furniture_staggered", "scv1_test_open_no_hit",
    }]
    selected = []
    for layout_name in desired:
        candidates = [(name, record) for name, record in records.items()
                      if record["layout"] == layout_name]
        if layout_name == "scv1_train_wall_front":
            in_band = []
            for name, record in candidates:
                pose = record["pose"]
                yaw = float(pose["yaw_rad"])
                cam_x = (float(pose["x"]) + CAMERA.position_body_m[0] * math.cos(yaw)
                         - CAMERA.position_body_m[1] * math.sin(yaw))
                distance = (1.60 - cam_x) / max(1e-9, math.cos(yaw))
                if .7 <= distance <= 1.3:
                    in_band.append((abs(distance - 1.0), name, record))
            key = min(in_band)[1] if in_band else (candidates[0][0] if candidates else None)
            record = next((item for name, item in candidates if name == key), None)
        else:
            key, record = candidates[0] if candidates else (None, None)
        if key and record:
            selected.append((key, record))
    if not selected:
        raise RuntimeError("no representative clean inputs found")
    fig, axes = plt.subplots(3, 4, figsize=(15, 9), constrained_layout=True)
    for ax, (key, record) in zip(axes.flat, selected):
        image = np.load(output / "data" / record["split"] / f"{key}_clean.npy")
        with (output / "data" / record["split"] / "label.pkl").open("rb") as f:
            labels = pickle.load(f)
        label = np.asarray(labels[f"{key}_clean"], dtype=float)
        im = ax.imshow(image, cmap="viridis", vmin=.1, vmax=6., interpolation="nearest")
        ax.set_title(f"{record['layout']}\n0°={label[5]:.2f}m, {record['pose_class']}", fontsize=8)
        ax.set_xticks([])
        ax.set_yticks([])
    for ax in axes.flat[len(selected):]:
        ax.axis("off")
    fig.colorbar(im, ax=axes.ravel().tolist(), label="sampled clean depth (m)", shrink=.78)
    examples = output / "examples"
    examples.mkdir(exist_ok=True)
    fig.savefig(examples / "representative_clean_inputs.png", dpi=150)
    plt.close(fig)
    (examples / "representative_samples.json").write_text(json.dumps([
        {"key": key, "layout": rec["layout"], "split": rec["split"],
         "pose_class": rec["pose_class"]} for key, rec in selected
    ], indent=2))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True,
                        help="new directory; the script refuses to overwrite")
    parser.add_argument("--poses-per-layout", type=int, default=None,
                        help="probe override; default uses each layout's planned count")
    parser.add_argument("--seed", type=int, default=20261115)
    parser.add_argument("--max-attempts", type=int, default=40000)
    parser.add_argument("--skip-geometry-checks", action="store_true")
    parser.add_argument("--camera-position-m", nargs=3, type=float, default=(.305, 0., .140),
                        metavar=("X", "Y", "Z"),
                        help="camera position proxy in base_link, not a calibrated optical center; default preserves adaptation_v1")
    parser.add_argument("--camera-pitch-range-deg", nargs=2, type=float, default=(3., 5.),
                        metavar=("MIN", "MAX"),
                        help="downward pitch sampling range; default preserves adaptation_v1")
    parser.add_argument("--base-height-range-m", nargs=2, type=float, default=(.35, .36),
                        metavar=("MIN", "MAX"),
                        help="base_link height sampling range; default preserves adaptation_v1")
    parser.add_argument("--depth-intrinsics", nargs=6, type=float,
                        default=(480., 270., 244.134, 244.134, 245.186, 134.22),
                        metavar=("WIDTH", "HEIGHT", "FX", "FY", "PPX", "PPY"),
                        help="native depth dimensions and K; default preserves adaptation_v1")
    parser.add_argument("--test-layout-set", choices=("legacy", "current_mount_v2", "final_holdout_v1"), default="legacy",
                        help="final_holdout_v1 generates final-test only; default preserves adaptation_v1")
    parser.add_argument("--coverage-supplement-v1", "--coverage-supplement-probe",
                        dest="coverage_supplement_probe", action="store_true",
                        help="generate the new whole-layout near-coverage supplement; no test split")
    parser.add_argument("--independent-eval-v1", dest="independent_eval", action="store_true",
                        help="generate new eval-only whole layouts for one-shot checkpoint comparison; no train/test split")
    parser.add_argument("--scene-coverage-v1", action="store_true",
                        help="generate bounded front-D435i wall/furniture/narrow/open scene supplement, split by whole layout")
    args = parser.parse_args(argv)
    if sum((args.independent_eval, args.coverage_supplement_probe, args.scene_coverage_v1)) > 1:
        raise ValueError("choose exactly one new-layout mode")
    global CAMERA, PITCH_RANGE_DEG, BASE_Z_RANGE_M
    width_f, height_f, fx, fy, ppx, ppy = args.depth_intrinsics
    if width_f != int(width_f) or height_f != int(height_f):
        raise ValueError("depth image width/height must be integers")
    if (int(width_f), int(height_f)) != (480, 270):
        raise ValueError("current mask augmentation is defined only for 480x270 depth frames")
    if args.camera_pitch_range_deg[0] > args.camera_pitch_range_deg[1]:
        raise ValueError("camera pitch range minimum must not exceed maximum")
    if args.base_height_range_m[0] > args.base_height_range_m[1]:
        raise ValueError("base height range minimum must not exceed maximum")
    PITCH_RANGE_DEG = tuple(float(v) for v in args.camera_pitch_range_deg)
    BASE_Z_RANGE_M = tuple(float(v) for v in args.base_height_range_m)
    CAMERA = compare.CameraRenderConfig(
        position_body_m=tuple(float(v) for v in args.camera_position_m),
        pitch_down_deg=float(sum(PITCH_RANGE_DEG) / 2.),
        width=int(width_f), height=int(height_f), fx=fx, fy=fy, ppx=ppx, ppy=ppy,
    )
    CAMERA.validate()
    out = args.output_root.expanduser().resolve()
    if out.exists():
        raise FileExistsError(out)
    out.mkdir(parents=True)
    (out / "scenes").mkdir()
    rng = np.random.default_rng(args.seed)
    masks = {}
    for split, groups in MASK_GROUPS.items():
        masks[split] = []
        for group in groups:
            for path in sorted((ROOT / group).glob("*_depth.u16")):
                mask = np.fromfile(path, dtype="<u2")
                if mask.size != 480 * 270:
                    raise ValueError(f"mask dimensions invalid: {path}")
                masks[split].append((path, mask.reshape(270, 480) > 0))
        if not masks[split]:
            raise ValueError(f"no raw masks for split {split}")
    mask_counter = {s: 0 for s in masks}
    all_rows, records, layout_stats = [], {}, []
    import time
    t0 = time.time()
    layouts = (build_scene_coverage_v1_layouts() if args.scene_coverage_v1 else
               build_independent_eval_layouts() if args.independent_eval
               else build_coverage_supplement_probe_layouts() if args.coverage_supplement_probe
               else build_layouts(args.test_layout_set))
    if args.scene_coverage_v1:
        plan = {
            "dataset": "front_d435i_scene_coverage_v1",
            "seed": args.seed,
            "planned_pose_count": sum(layout["count"] for layout in layouts),
            "layouts": layouts,
            "camera_candidate": {"position_base_m": list(CAMERA.position_body_m),
                                 "pitch_range_deg": list(PITCH_RANGE_DEG),
                                 "base_height_range_m": list(BASE_Z_RANGE_M),
                                 "K": [CAMERA.width, CAMERA.height, CAMERA.fx, CAMERA.fy,
                                       CAMERA.ppx, CAMERA.ppy],
                                 "yaw_roll": "assumed zero; not measured",
                                 "position_reference": "front glass proxy, not calibrated optical center"},
            "abs_ray_contract": {"angles_deg": RAY_ANGLES,
                                 "origin_base_m": [compare.RAY2D_X0, compare.RAY2D_Y0],
                                 "range_m": [compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST]},
            "preprocessing": "native clean depth -> centre sample (3i+1,3j+1) to 160x90; paired mask path uses existing mode1 augmentation and residual 6m candidate; model training contract clip 0.1-6m, log2, 3 channels",
            "training": False,
            "real_images_in_dataset": False,
        }
        (out / "dataset_plan.json").write_text(json.dumps(plan, indent=2))
        with (out / "layout_splits.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=("layout", "split", "family", "category", "poses", "acceptance"))
            writer.writeheader()
            writer.writerows({"layout": item["name"], "split": item["split"],
                              "family": item["family"], "category": item["category"],
                              "poses": item["count"], "acceptance": item.get("acceptance", "near_mid_far_quota")}
                             for item in layouts)
        # Fail on the small geometry/sign/open checks before rendering the full data batch.
        run_geometry_checks(out)
    for layout in layouts:
        stats = run_layout(mujoco, compare, generation, layout, out, masks, mask_counter,
                           rng, args.max_attempts, args.poses_per_layout, all_rows, records)
        layout_stats.append(stats)
    run_diagnostics(out)
    if not args.skip_geometry_checks and not args.scene_coverage_v1:
        run_geometry_checks(out)
    elapsed = time.time() - t0
    with (out / "ray_projection_visibility.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    (out / "data_records.json").write_text(json.dumps(records, indent=2))
    bytes_saved = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
    audit = audit_generated_data(out, records, layouts, args)
    if args.scene_coverage_v1:
        save_representative_inputs(out, records, layouts)
        bytes_saved = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
    summary = summarize(out, all_rows, records, layout_stats, masks, elapsed, bytes_saved,
                        args, layouts, audit)
    print(json.dumps({k: summary[k] for k in ("poses_total", "images_total", "poses_by_split",
                                              "label_bin_fraction", "cap_fraction", "near_fraction",
                                              "hitpoint_visibility", "elapsed_seconds", "bytes_on_disk")},
                     indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
