#!/usr/bin/env python3
"""One bounded, offline D435i ray-only adaptation pilot; never deploys weights."""
import argparse
import json
import pickle
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

import generate_mujoco_ray_dataset as generation
from analyze_d435i_new_camera_hole_candidates import librealsense_2584_mode1_source
from compare_d435i_ray_model_weights import infer_batch
from realsense_depth_to_abs_rays import downsample_depth_centers

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt"
CAPTURE_ROOT = ROOT / "logs/real_go2"
MASK_GROUPS = {
    "train": ["20261113_d435i_remounted_cabinet/capture"],
    "validation": ["20261113_d435i_1m_distance_capture/capture"],
    "test": ["20261113_d435i_80cm_distance_capture/capture",
             "20261113_d435i_2m_distance_capture/capture/data_20261113_143447"],
}


def metrics(model, dataset, output):
    import torch
    from torch.utils.data import DataLoader
    model.eval()
    predictions, targets = [], []
    with torch.no_grad():
        for depth_log, label_log in DataLoader(dataset, batch_size=16, shuffle=False):
            predictions.append(torch.exp2(model(depth_log.unsqueeze(1).repeat(1, 3, 1, 1))).clamp(.1, 6))
            targets.append(torch.exp2(label_log))
    pred, target = torch.cat(predictions), torch.cat(targets)
    error = pred - target
    result = {
        "samples": len(dataset), "ray_values": target.numel(),
        "mae_m": error.abs().mean().item(), "rmse_m": error.square().mean().sqrt().item(),
        "bias_m": error.mean().item(), "per_ray_mae_m": error.abs().mean(0).tolist(),
        "false_safe_target_le_1p5_overestimate_ge_0p5_count": int(((target <= 1.5) & (error >= .5)).sum()),
        "false_safe_eligible_count": int((target <= 1.5).sum()),
        "underestimate_ge_0p5_count": int((error <= -.5).sum()),
        "per_ray_bias_m": error.mean(0).tolist(),
    }
    if output is not None:
        np.savez_compressed(output, prediction_m=pred.numpy(), target_m=target.numpy())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stage", choices=("all", "generate", "train"), default="all")
    args = parser.parse_args()
    out = args.output_dir.resolve()
    if args.stage == "all":
        # Keep EGL rendering and Torch training in separate processes. The first
        # combined pilot wrote its results but aborted during interpreter cleanup.
        for stage in ("generate", "train"):
            subprocess.run([sys.executable, str(Path(__file__).resolve()),
                            "--output-dir", str(out), "--stage", stage], check=True)
        return
    if args.stage == "generate":
        import mujoco
        out.mkdir(parents=True, exist_ok=False)
        np.random.seed(1311)
        compare = generation.load_compare_module()
        plan = {
            "authorization": "2026-11-13 user requested to begin bounded ray model adaptation after resolution comparison",
            "kind": "development pilot, not thesis formal experiment or robot control",
            "base_model": str(BASE), "epochs": 4, "lr": 0.0001, "batch_size": 16,
            "trainable": "resnet.layer4 and resnet.fc; BatchNorm running statistics frozen in eval mode",
            "split": "three training layouts, one validation layout, two held-out test layouts; no shared poses or raw mask frames across splits",
            "geometry_samples_per_layout": 64,
            "camera_nominal": {"position_housing_proxy_m": [.305, 0, .140], "pitch_range_deg": [3, 5],
                               "base_height_range_m": [.35, .36], "intrinsics": [480, 270, 244.134, 244.134, 245.186, 134.22]},
            "limitations": ["housing origin approximates optical origin", "transplanted masks are augmentation, not a sensor physics model",
                            "held-out layouts are synthetic development data, not general real-world accuracy evidence",
                            "remaining unknown filled to 6m is a candidate numerical policy, not a safety guarantee",
                            "no independently labelled real 11-ray evaluation available"],
            "processing": "native simulated Z in metres -> millimetre Z16 -> apply real raw-zero mask -> source-equivalent 2.58.4 mode1 fill -> remaining zero to 6m -> center sample -> clip .1..6 -> log2 -> three channels",
            "selection": "best validation MAE among exactly four epochs; test not used to select checkpoint",
            "device": "cpu", "seed": 1311, "no_control_output": True,
        }
        (out / "PLAN.json").write_text(json.dumps(plan, indent=2))
        masks = {}
        for split, groups in MASK_GROUPS.items():
            masks[split] = [(path, np.fromfile(path, dtype="<u2").reshape(270, 480) > 0)
                            for group in groups for path in sorted((CAPTURE_ROOT / group).glob("*_depth.u16"))]
            if not masks[split]:
                raise ValueError(f"no raw masks for {split}")
        split_records = {s: [] for s in MASK_GROUPS}
        # Known static ground-plane layouts, including closed, open, left and right
        # obstacles. Each split owns complete layouts, not random neighbouring frames.
        splits = ["train", "train", "train", "validation", "test", "test"]
        scene_dir = out / "scenes"
        scene_dir.mkdir()
        for layout, split in enumerate(splits):
            rng = np.random.default_rng(9100 + layout)
            wall_x = [5.2, None, 6.0, 5.65, 4.8, None][layout]
            obstacles = []
            if wall_x is not None:
                obstacles.append(f'<geom name="wall" type="box" pos="{wall_x} 0 1" size="0.05 4 1"/>')
            for j in range(7):
                x, y = rng.uniform(1.2, 5.2), rng.uniform(-2.1, 2.1)
                sx, sy = rng.uniform(.12, .4), rng.uniform(.12, .4)
                if j % 2:
                    obstacles.append(f'<geom name="box{j}" type="box" pos="{x} {y} .6" size="{sx} {sy} .6"/>')
                else:
                    obstacles.append(f'<geom name="cylinder{j}" type="cylinder" pos="{x} {y} .6" size="{sx} .6"/>')
            xml = ('<mujoco><compiler angle="radian"/><visual><global offwidth="480" offheight="270"/></visual>'
                   '<worldbody><geom name="floor" type="plane" size="15 15 .1"/>' + ''.join(obstacles) +
                   '<body name="base_link" pos="0 0 .355"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia=".01 .01 .01"/></body></worldbody></mujoco>')
            scene_path = scene_dir / f"{split}_{layout}.xml"
            scene_path.write_text(xml)
            model = mujoco.MjModel.from_xml_path(str(scene_path))
            data = mujoco.MjData(model)
            body = compare.find_body_id(mujoco, model)
            config = compare.CameraRenderConfig((.305, 0, .140), 4, 480, 270, 244.134, 244.134, 245.186, 134.22)
            renderer = compare.make_depth_renderer(mujoco, model, config)
            destination = out / "data" / split
            destination.mkdir(parents=True, exist_ok=True)
            labels = {}
            if (destination / "label.pkl").exists():
                with (destination / "label.pkl").open("rb") as f:
                    labels = pickle.load(f)
            accepted = 0
            try:
                for attempt in range(4096):
                    x, y, yaw = rng.uniform(-.8, 3.5), rng.uniform(-1, 1), rng.uniform(-.5, .5)
                    z, pitch = rng.uniform(.35, .36), rng.uniform(3, 5)
                    compare.apply_sample(mujoco, model, data, compare.PoseSample(attempt, "d435i_pilot", x, y, z, yaw))
                    label = np.exp2(compare.compute_geometric_ray_log(mujoco, model, data, body, scene_path.name)).astype(np.float32)
                    # Reject poses inside obstacles as well as too-close forward rays.
                    point = data.xpos[body, :2]
                    inside = False
                    for gid in compare.obstacle_geom_ids(mujoco, model, scene_path.name):
                        delta = point - data.geom_xpos[gid, :2]
                        if int(model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_BOX):
                            inside |= bool(np.all(np.abs(delta) <= model.geom_size[gid, :2] + .2))
                        else:
                            inside |= bool(np.linalg.norm(delta) <= model.geom_size[gid, 0] + .2)
                    if inside or label.min() < .45:
                        continue
                    config = replace(config, pitch_down_deg=pitch)
                    native = generation.render_depth(mujoco, compare, model, data, renderer, body, config, return_native=True)
                    raw = np.rint(native * 1000).astype(np.uint16)
                    mask_path, valid = masks[split][accepted % len(masks[split])]
                    masked = raw.copy()
                    masked[~valid] = 0
                    filled = librealsense_2584_mode1_source(masked)
                    prepared = filled.astype(np.float32) * .001
                    prepared[filled == 0] = 6
                    clean, _ = downsample_depth_centers(raw.astype(np.float32) * .001, np.ones(raw.shape, bool))
                    augmented, _ = downsample_depth_centers(prepared, np.ones(raw.shape, bool))
                    key = f"layout{layout}_pose{accepted:03d}"
                    for kind, image in (("clean", clean), ("masked", augmented)):
                        name = key + "_" + kind
                        np.save(destination / f"{name}.npy", image)
                        labels[name] = label
                    split_records[split].append({"key": key, "layout": layout, "x": x, "y": y, "z": z, "yaw": yaw,
                                                 "pitch": pitch, "mask_source": str(mask_path.relative_to(ROOT)),
                                                 "raw_unknown_fraction": float((~valid).mean()),
                                                 "remaining_unknown_fraction": float((filled == 0).mean())})
                    accepted += 1
                    if accepted == 64:
                        break
                if accepted != 64:
                    raise RuntimeError(f"layout {layout}: only {accepted} accepted poses")
            finally:
                renderer.close()
            with (destination / "label.pkl").open("wb") as f:
                pickle.dump(labels, f)
            print(f"DATA {split} layout={layout} poses={accepted} images={accepted*2}", flush=True)
        (out / "data_records.json").write_text(json.dumps(split_records, indent=2))
        return
    import torch
    from torch.utils.data import DataLoader
    from finetune_mujoco_ray_pred import RayDataset, weighted_ray_loss
    torch.set_num_threads(6)
    torch.manual_seed(1311)
    if (out / "candidate_last.pt").exists():
        raise FileExistsError("Training output already exists; keep this run and generate a new output directory")
    datasets = {s: RayDataset([out / "data" / s]) for s in MASK_GROUPS}
    for s, dataset in datasets.items():
        values = np.stack([label for _, label in dataset.items])
        if values.shape[1] != 11 or not np.isfinite(values).all() or values.min() < .0999 or values.max() > 6.0001:
            raise ValueError("invalid geometric labels")
    model = torch.jit.load(str(BASE), map_location="cpu").eval()
    baseline = {s: metrics(model, datasets[s], out / f"baseline_{s}.npz") for s in ("validation", "test")}
    (out / "baseline_metrics.json").write_text(json.dumps(baseline, indent=2))
    print("BASELINE " + json.dumps(baseline), flush=True)
    for name, parameter in model.named_parameters():
        parameter.requires_grad_(name.startswith("resnet.layer4.") or name.startswith("resnet.fc."))
    optimizer = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad], lr=1e-4, weight_decay=1e-4)
    loader = DataLoader(datasets["train"], batch_size=16, shuffle=True,
                        generator=torch.Generator().manual_seed(1311), num_workers=0)
    best = float("inf")
    history = []
    for epoch in range(1, 5):
        model.eval()  # Keep pretrained BN statistics; gradients still enabled.
        losses = []
        for depth_log, target_log in loader:
            prediction = model(depth_log.unsqueeze(1).repeat(1, 3, 1, 1))
            loss = weighted_ray_loss(prediction, target_log, 4., 8., 0.)
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad], 1.)
            optimizer.step()
            losses.append(loss.item())
        validation = metrics(model, datasets["validation"], None)
        history.append({"epoch": epoch, "training_loss": float(np.mean(losses)), "validation": validation})
        torch.jit.save(model, str(out / "candidate_last.pt"))
        if validation["mae_m"] < best:
            best = validation["mae_m"]
            torch.jit.save(model, str(out / "candidate_best.pt"))
        (out / "history.json").write_text(json.dumps(history, indent=2))
        print(f"EPOCH {epoch}/4 train_loss={np.mean(losses):.5f} validation=" + json.dumps(validation), flush=True)
    candidate = torch.jit.load(str(out / "candidate_best.pt"), map_location="cpu").eval()
    after = {s: metrics(candidate, datasets[s], out / f"candidate_{s}.npz") for s in ("validation", "test")}
    # Real images never contribute training targets or checkpoint selection.
    real = []
    baseline_model = torch.jit.load(str(BASE), map_location="cpu").eval()
    for group in ("20261113_remount_adaptation_capture/capture", "20261113_d435i_1m_distance_capture/capture", "20261113_d435i_80cm_distance_capture/capture"):
        directory = CAPTURE_ROOT / group
        for path in sorted(directory.glob("*_filled.u16")):
            depth = np.fromfile(path, dtype="<u2").reshape(270, 480).astype(np.float32) * .001
            depth[depth == 0] = 6
            sampled, _ = downsample_depth_centers(depth, np.ones(depth.shape, bool))
            before_rays = infer_batch(baseline_model, sampled[None])[0]
            after_rays = infer_batch(candidate, sampled[None])[0]
            real.append({"path": str(path.relative_to(ROOT)), "baseline_rays_m": before_rays.tolist(),
                         "candidate_rays_m": after_rays.tolist(), "diagnostic_only_no_ray_truth": True,
                         "posture": "prone or not reconfirmed; not standing accuracy evaluation"})
    result = {"baseline": baseline, "candidate": after,
              "selected_epoch": min(history, key=lambda h: h["validation"]["mae_m"])["epoch"],
              "real_diagnostics": real, "control_approved": False}
    (out / "RESULT.json").write_text(json.dumps(result, indent=2))
    print("DONE " + json.dumps({"baseline_test": baseline["test"], "candidate_test": after["test"]}), flush=True)


if __name__ == "__main__":
    main()
