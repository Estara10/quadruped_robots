#!/usr/bin/env python3
"""One-time public-name migration; archive before removing retired scene XMLs."""
import csv
import hashlib
import io
import json
from pathlib import Path
import zipfile

import abs_scene
import generate_m1_library as gen
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


def main():
    out = gen.OUT
    path = out / "layouts.json"
    doc = json.loads(path.read_text())
    if doc.get("naming_revision"):
        raise SystemExit("Names already migrated; refusing to repeat archival/deletion.")
    scenes = doc["scenes"]
    assert len(scenes) == 60
    mappings = []
    for s in scenes:
        replica = int(s["scene_id"][-2:])
        name = gen.public_name(s["density"], s["route_structure"], s["split"], replica)
        mappings.append(dict(scene_id=s["scene_id"], old_filename=s["filename"],
                             filename=name+".xml", old_preview=s["preview"], preview=name+".png",
                             display_name=name, density=s["density"], structure=s["route_structure"], split=s["split"]))
    assert len({m["filename"] for m in mappings}) == 60
    for m in mappings:
        assert not (gen.SCENE_DIR / m["filename"]).exists()
    keep = {"scene.xml", "scene_flat.xml", "scene_obstacle.xml"}
    retired = sorted(p for p in gen.SCENE_DIR.glob("scene*.xml") if p.name not in keep)
    archive = ROOT / "archive/maps_before_short_names_20261015.zip"
    archive.parent.mkdir(exist_ok=True)
    if archive.exists():
        raise RuntimeError("archive exists; refusing overwrite")
    # Include unchanged original bytes, old identity catalog and frozen M1 previews.
    with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as z:
        for p in retired:
            z.write(p, p.relative_to(ROOT))
        z.write(abs_scene.CATALOG, abs_scene.CATALOG.relative_to(ROOT))
        for p in out.iterdir():
            if p.is_file():
                z.write(p, p.relative_to(ROOT))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for p in retired:
            assert z.read(str(p.relative_to(ROOT))) == p.read_bytes()
    old_specs = abs_scene.SCENES
    rows = []
    for s, m in zip(scenes, mappings):
        original = gen.SCENE_DIR / m["old_filename"]
        target = gen.SCENE_DIR / m["filename"]
        target.write_bytes(original.read_bytes())
        assert target.read_bytes() == original.read_bytes()
        spec = old_specs[m["old_filename"]]
        s.update(filename=m["filename"], preview=m["preview"], display_name=m["display_name"])
        binding = dict(s.get("production_scene_binding", {}))
        root_hash = hashlib.sha256(target.read_bytes()).hexdigest()
        closure_hash = abs_scene._closure_sha256(target)
        binding.update(filename=target.name, scene_id=s["scene_id"], root_sha256=root_hash,
                       closure_sha256=closure_hash, runtime_model_fingerprint=spec.model_fingerprint)
        s["production_scene_binding"] = binding
        rows.append(f'ABS_SCENE("{target.name}", "{s["scene_id"]}", {s["obstacle_count"]}, kNamedPptObstacles,\n'
                    f'          "{root_hash}", "{closure_hash}",\n'
                    f'          "{spec.model_fingerprint}")\n')
    # Keep only the two explicitly required baseline entries plus current maps.
    entries = abs_scene.ENTRY_RE.finditer(abs_scene.CATALOG.read_text())
    baseline = [match.group(0)+"\n" for match in entries if match.group(1) in keep]
    assert len(baseline) == 2
    abs_scene.CATALOG.write_text("// Current M1 maps; historical scene catalog is in archive/maps_before_short_names_20261015.zip\n" + "".join(baseline+rows))
    doc["naming_revision"] = "dev-holdout/1.0"
    doc["naming_note"] = "Filename/preview migration only; internal identities, XML bytes, geometry, seeds and split unchanged."
    path.write_text(json.dumps(doc, indent=2, ensure_ascii=False)+"\n")
    (out / "name_mapping.json").write_text(json.dumps(mappings, indent=2, ensure_ascii=False)+"\n")
    buffer = io.StringIO()
    fields = ["display_name", "scene_id", "filename", "density", "route_structure", "split", "replica", "seed", "goal_x", "goal_y", "sim_limit_s", "wall_cap_s"]
    writer = csv.DictWriter(buffer, fieldnames=fields)
    writer.writeheader()
    gen.configure_geometry()
    for s in scenes:
        writer.writerow(dict(display_name=s["display_name"],scene_id=s["scene_id"],filename=s["filename"],density=s["density"],route_structure=s["route_structure"],split=s["split"],replica=int(s["scene_id"][-2:]),seed=s["generation_seed"],goal_x=7.5,goal_y=0,sim_limit_s=20,wall_cap_s=120))
        gen.render(s).save(out / s["preview"], optimize=True)
    (out / "scene_index.csv").write_text(buffer.getvalue())
    for split in ("development", "held_out"):
        selected = [s for s in scenes if s["split"] == split]
        sheet = Image.new("RGB", (1500, 1380), "#f2f4f7")
        for i, s in enumerate(selected):
            sheet.paste(gen.render(s).resize((290,218)), ((i%5)*300,(i//5)*230))
        sheet.save(out / f"overview_{split}_30.png", optimize=True)
    # Delete only inventoried files after their byte-for-byte backup was checked.
    for p in retired:
        p.unlink()
    for m in mappings:
        (out / m["old_preview"]).unlink()
    (out / "cleanup_inventory.json").write_text(json.dumps(dict(archive=str(archive.relative_to(ROOT)),removed_scene_files=[str(p.relative_to(ROOT)) for p in retired],preserved_baseline_files=sorted(keep),geometry_changed=False,raw_run_logs_changed=False),indent=2)+"\n")
    print(f"Renamed 60 maps; removed {len(retired)} previous scene XMLs from active folder. Backup: {archive}")


if __name__ == "__main__":
    main()
