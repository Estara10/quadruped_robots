"""Evaluate a geometry-first ray input against the existing epoch13 baseline."""
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median

OUT = Path(__file__).resolve().parent
ANGLES = tuple(range(-45, 46, 9))


def read_frames(path, existing_ab):
    frames = defaultdict(dict)
    with path.open() as stream:
        for row in csv.DictReader(stream):
            key = (row["capture"], row["group"], row["frame"]) if existing_ab else (
                "current_mount", "", row["frame"])
            angle = int(float(row["angle_deg"]))
            geometry = float(row["geometry_m"]) if row["geometry_m"] else None
            network = float(row["network_epoch13_m"] if existing_ab else row["epoch13_m"])
            reference = float(row["reference_m"]) if existing_ab and row["reference_m"] else None
            if angle in frames[key]:
                raise ValueError(f"Duplicate ray {angle} in frame {key}")
            if not math.isfinite(network) or (geometry is not None and not math.isfinite(geometry)):
                raise ValueError(f"Non-finite ray {angle} in frame {key}")
            frames[key][angle] = (geometry, network, reference)
    return frames


frames = read_frames(OUT / "frame_ray_ab.csv", existing_ab=True)
frames.update(read_frames(OUT.parent / "standing_current_mount_ab_PN3V7r/per_frame_per_ray.csv", existing_ab=False))
coverage = defaultdict(Counter)
references = defaultdict(list)
changes = defaultdict(list)
with (OUT / "hybrid_frame_rays.jsonl").open("w") as output:
    for (capture, group, frame), rays in frames.items():
        if tuple(sorted(rays)) != ANGLES:
            raise ValueError(f"Invalid angle set in {capture}/{group}/{frame}")
        values, sources = [], []
        for angle in ANGLES:
            geometry, network, reference = rays[angle]
            value = network if geometry is None else geometry
            values.append(min(6.0, max(.1, value)))
            sources.append("epoch13_estimate" if geometry is None else "geometry_sample")
            if geometry is None:
                coverage[capture][angle] += 1
            changes[capture].append(abs(values[-1] - network))
            if reference is not None:
                references[angle].append((reference, values[-1], network))
        output.write(json.dumps(dict(capture=capture, group=group, frame=int(frame), angles_deg=ANGLES,
                                     meters=values, log2_meters=[math.log2(x) for x in values],
                                     source=sources)) + "\n")

summary = dict(
    frames=len(frames),
    per_capture={name: dict(frames=sum(key[0] == name for key in frames),
                            model_fallback_by_angle=dict(sorted(coverage[name].items())),
                            median_absolute_change_m=median(changes[name]),
                            maximum_absolute_change_m=max(changes[name]))
                 for name in sorted(changes)},
    reference_rays={angle: dict(reference_m=median(a for a, _, _ in rows),
                                hybrid_m=median(b for _, b, _ in rows),
                                network_m=median(c for _, _, c in rows))
                    for angle, rows in sorted(references.items())},
)
(OUT / "hybrid_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
print(json.dumps(summary, indent=2))
