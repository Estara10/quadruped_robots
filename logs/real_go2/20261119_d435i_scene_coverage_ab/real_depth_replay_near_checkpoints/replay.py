import csv
import json
import sys
from pathlib import Path

import numpy as np
import torch

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
LOGS = ROOT / "logs/real_go2"
sys.path.insert(0, str(ROOT / "scripts"))
from go2_depth_ray_observer import load_model, prepare_input

model_paths = {
    "epoch13": LOGS / "20261117_d435i_fullnet_preparation/training_runs/d435i_fullnet_300epoch_20261117/ray_pred_mujoco_finetuned_best.pt",
    "B1000": OUT.parent / "expanded_1000ep_seed20261117/ray_pred_mujoco_finetuned_last.pt",
}
near = OUT.parent / "expanded_near_1000ep_seed20261117"
model_paths.update({
    f"near{epoch}": near / f"ray_pred_epoch_{epoch:04}.pt"
    for epoch in (60, 250, 500, 750)
})
model_paths["near1000"] = near / "ray_pred_mujoco_finetuned_last.pt"
models = {k: load_model(v)[0] for k, v in model_paths.items()}
torch.set_num_threads(4)
runs = [("wall_43cm", LOGS / "static_wall430_FEMgc0" / "capture", {0: .78}),
        ("wall_99p5cm", LOGS / "static_wall995_aFzR2g" / "capture", {0: 1.345}),
        ("wall_153p5cm", LOGS / "static_wall1535_BSU8y7" / "capture", {0: 1.885})]
box = LOGS / "20261118_bucket_wall_static"
for g, refs in enumerate(({0: 1.284}, {9: 1.303, 18: 1.336}, {-18: 1.245, -9: 1.245}, {0: 2.105}), 1):
    runs.append((f"box_{g}", box / f"group{g}/capture", refs))
board = LOGS / "20261118_standing_board119_B0bEMC"
board_refs = json.loads((board / "comparison.json").read_text())["ray_reference_m"]
runs.append(("board_119cm", board / "capture", {int(k): float(v) for k, v in board_refs.items()}))
angles = list(range(-45, 46, 9))
frames = []
summary = {}
for name, capture, references in runs:
    metadata = dict(s.split("=", 1) for s in (capture / "metadata.txt").read_text().splitlines() if "=" in s)
    shape = int(metadata["height"]), int(metadata["width"])
    scale = float(metadata["scale"])
    with (capture / "frames.csv").open() as f:
        raw_center = {int(r["i"]): float(r["center_pixel_depth_m"]) for r in csv.DictReader(f)}
    paths = sorted(capture.glob("*_filled.u16"), key=lambda p: int(p.name.split("_", 1)[0]))
    assert len(paths) == (600 if name.startswith("wall") else 3), (name, len(paths))
    outputs = {}
    for start in range(0, len(paths), 16):
        subset = paths[start:start+16]
        depths = []
        unknowns = []
        for path in subset:
            z16 = np.fromfile(path, dtype="<u2").reshape(shape)
            x, _, valid = prepare_input(z16.astype(np.float32) * scale, z16 != 0, 6.)
            depths.append(x)
            unknowns.append(float((~valid).mean()))
        x = torch.frombuffer(bytearray(np.stack(depths).tobytes()), dtype=torch.float32)
        x = x.reshape(len(subset), 1, 90, 160).repeat(1, 3, 1, 1)
        with torch.inference_mode():
            predictions = {k: torch.exp2(model(x)).clamp(.1, 6.).tolist() for k, model in models.items()}
        for i, path in enumerate(subset):
            frame = int(path.name.split("_", 1)[0])
            ray_models = {k: v[i] for k, v in predictions.items()}
            for j, angle in enumerate(angles):
                diffs = {k: v[j] for k, v in ray_models.items()}
                row = dict(case=name, frame=frame, angle_deg=angle, camera_depth_m=raw_center[frame],
                           reference_m=references.get(angle), residual_unknown_fraction=unknowns[i], **diffs)
                assert all(np.isfinite(list(diffs.values())))
                frames.append(row)
                outputs.setdefault(angle, []).append(row)
    focus = {}
    for angle, reference in references.items():
        subset = outputs[angle]
        focus[angle] = {"reference_m": reference}
        for k in models:
            vals = np.asarray([r[k] for r in subset])
            focus[angle][k] = dict(median_m=float(np.median(vals)),
                                  p10_median_p90_m=np.percentile(vals, [10, 50, 90]).tolist(),
                                  report_far_gt_0p5=int(np.count_nonzero(vals - reference > .5)),
                                  report_near_gt_0p5=int(np.count_nonzero(reference - vals > .5)),
                                  median_error_m=float(np.median(vals)-reference))
    summary[name] = dict(capture=str(capture.relative_to(ROOT)), frames=len(paths), focused_rays=focus)
    print(name, len(paths), [(k, round(v["reference_m"],3), round(v["epoch13"]["median_m"],3), round(v["B1000"]["median_m"],3)) for k,v in focus.items()], flush=True)
with (OUT / "per_ray.csv").open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(frames[0]))
    writer.writeheader()
    writer.writerows(frames)
(OUT / "summary.json").write_text(json.dumps(dict(models={k:str(v.relative_to(ROOT)) for k,v in model_paths.items()}, runs=summary), indent=2, allow_nan=False)+"\n")
