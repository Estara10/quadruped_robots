"""Replay matched camera observations with identical offline RA state and goal."""
import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import torch

base = Path(__file__).resolve().parents[1]
root = base.parents[1]
sys.path.insert(0, str(root / "scripts"))
from go2_depth_ray_observer import load_model, prepare_input

spec = importlib.util.spec_from_file_location(
    "geometry", base / "20261118_abs_geometry_network_ab/analyze_geometry_network_ab.py")
geom = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geom)
ray_model, _ = load_model(geom.EPOCH13)
ra_model = torch.jit.load(str(geom.RA_MODEL), map_location="cpu").eval()
threshold = -.05

for run_name in sys.argv[1:]:
    run = base / run_name
    cap = run / "capture"
    states = [json.loads(line) for line in (run / "state.jsonl").read_text().splitlines() if line.startswith("{")]
    aligned = [json.loads(line) for line in (run / "aligned_index.jsonl").read_text().splitlines()]
    camera_rows = list(csv.DictReader((cap / "frames.csv").read_text().splitlines()))
    zero = aligned[0]["time_pc_steady_s"]
    matched = []
    for a in aligned:
        if not {"sport", "low"} <= a["matches"].keys():
            continue
        sport = states[a["matches"]["sport"]["state_index"]]["sport"]
        low = states[a["matches"]["low"]["state_index"]]["low"]
        w, x, y, z = low["quaternion"]
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        matched.append((a["frame"], a["time_pc_steady_s"]-zero, sport, low, yaw))
    moving = [t for _, t, sport, _, _ in matched if math.hypot(*sport["velocity"][:2]) > .08]
    if not moving:
        print(run_name, "no overlapping motion")
        continue
    start, end = moving[0], moving[-1]
    selected = [row for row in matched[::3] if start-2 <= row[1] <= end+2]
    positions = np.asarray([row[2]["position"][:2] for row in matched])
    times = np.asarray([row[1] for row in matched])
    initial = selected[0]
    goal = np.asarray(initial[2]["position"][:2]) + 3.5*np.array([math.cos(initial[4]), math.sin(initial[4])])
    meta = geom.read_meta(cap / "metadata.txt")
    shape = int(meta["height"]), int(meta["width"])
    scale = float(meta["scale"])
    results = []
    for i, t, sport, low, yaw in selected:
        depth = np.fromfile(cap / f"{i}_filled.u16", dtype="<u2").reshape(shape)
        model_input, _, _ = prepare_input(depth.astype(np.float32)*scale, depth != 0, 6.)
        tensor = torch.frombuffer(bytearray(model_input.tobytes()), dtype=torch.float32)
        tensor = tensor.reshape(1,1,90,160).repeat(1,3,1,1)
        with torch.inference_mode():
            net = torch.exp2(ray_model(tensor)).clamp(.1,6.).flatten().tolist()
        distances, counts = geom.load_geom(cap / f"{i}_depth.u16", meta)
        mixed = [estimate if point is None else point for point, estimate in zip(distances, net)]
        before, after = max(times[0],t-.15), min(times[-1],t+.15)
        vel = np.array([(np.interp(after,times,p)-np.interp(before,times,p))/(after-before)
                        for p in positions.T])
        rotation = np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])
        command = rotation @ (goal-np.asarray(sport["position"][:2]))
        vx, vy = rotation @ vel
        prefix = [vx,vy,sport["velocity"][2],*low["gyro"],*command]
        with torch.inference_mode():
            ra = [float(v) for v in ra_model(torch.tensor([
                prefix+[math.log2(max(.1,min(6.,d))) for d in source] for source in (net,mixed)
            ], dtype=torch.float32)).flatten()]
        results.append(dict(frame=i,time_s=t,camera_center_m=float(camera_rows[i]["center_median_m"]),
                            network_0_m=net[5],hybrid_0_m=mixed[5],geometry_0_m=distances[5],
                            geometry_valid_rays=sum(count>=geom.MIN_POINTS and d is not None for count,d in zip(counts,distances)),
                            network_ra=ra[0],hybrid_ra=ra[1]))
    with (run / "dynamic_ab.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=results[0])
        writer.writeheader()
        writer.writerows(results)
    crossings = {}
    for name in ("network","hybrid"):
        flags = [r[f"{name}_ra"]>=threshold for r in results]
        first = next((r for j,r in enumerate(results[:-2]) if all(flags[j:j+3])),None)
        crossings[name] = dict(above=sum(flags),first_three_consecutive=first,
                               peak=max(r[f"{name}_ra"] for r in results))
    summary = dict(frames=len(results), motion_window_s=[start,end], first=results[0], last=results[-1],
                   crossings=crossings, goal="hypothetical fixed world goal 3.5 m forward from initial replay pose",
                   state="host-time aligned, XY velocity from position differences rotated by LowState yaw, raw Z velocity and gyro",
                   note="Static threshold evaluation of recorded frames, not closed-loop behavior.")
    (run / "dynamic_ab.json").write_text(json.dumps(summary,indent=2,allow_nan=False)+"\n")
    print(run_name,json.dumps(summary))
