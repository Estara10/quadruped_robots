"""Small reader for the shared C++/Python supported-scene catalog."""
from __future__ import annotations

import hashlib
import json
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GO2_SCENE_DIR = ROOT / "unitree_mujoco/unitree_robots/go2"
CATALOG = ROOT / "common/abs_scene_catalog.def"
ENTRY_RE = re.compile(
    r'ABS_SCENE\("([^\"]+)",\s*"([^\"]+)",\s*(\d+),\s*'
    r'(kLegacySignatures|kNoObstacles|kNamedPptObstacles),\s*'
    r'"([0-9a-f]{64})",\s*"([0-9a-f]{64})",\s*"([0-9a-f]{64})"\)')


@dataclass(frozen=True)
class SceneSpec:
    filename: str
    scene_id: str
    obstacle_count: int
    kind: str
    root_sha256: str
    closure_sha256: str
    model_fingerprint: str

    @property
    def path(self) -> Path:
        return (GO2_SCENE_DIR / self.filename).resolve()

    def binding(self) -> dict[str, str | int]:
        return {"filename": self.filename, "scene_id": self.scene_id,
                "obstacle_count": self.obstacle_count,
                "root_sha256": self.root_sha256,
                "closure_sha256": self.closure_sha256,
                "model_fingerprint": self.model_fingerprint}


def scene_catalog() -> dict[str, SceneSpec]:
    entries = {}
    for match in ENTRY_RE.finditer(CATALOG.read_text(encoding="utf-8")):
        filename, scene_id, count, kind, root_hash, closure_hash, fingerprint = match.groups()
        spec = SceneSpec(filename, scene_id, int(count), kind, root_hash, closure_hash, fingerprint)
        if filename in entries or scene_id in {item.scene_id for item in entries.values()}:
            raise ValueError("duplicate scene catalog identity")
        entries[filename] = spec
    if len(entries) != 10:
        raise ValueError(f"expected ten supported scenes, found {len(entries)}")
    return entries


SCENES = scene_catalog()
DEFAULT_SCENE = "scene_obstacle.xml"
LEGACY_BINDING = {
    **SCENES[DEFAULT_SCENE].binding(),
    # Historical v2/v3 records used this earlier collision-authority binding.
    "closure_sha256": "6ca5da14be6909815ac9c41bf6db0f8108e07082aea5aba22c91e833e6181746",
}


def _closure_sha256(root_xml: Path) -> str:
    closure_root = root_xml.parent.resolve()
    records: dict[str, str] = {}
    visited: set[Path] = set()
    file_tags = {"mesh", "hfield", "texture", "skin"}

    def add(path: Path) -> None:
        path = path.resolve()
        path.relative_to(closure_root)
        if not path.is_file():
            raise ValueError(f"scene closure asset missing: {path}")
        records[path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()

    def visit(xml_path: Path) -> None:
        xml_path = xml_path.resolve()
        xml_path.relative_to(closure_root)
        if xml_path in visited:
            return
        visited.add(xml_path)
        add(xml_path)
        document = ET.parse(xml_path).getroot()
        for element in document.iter():
            if element.tag == "include" and element.get("file"):
                visit(xml_path.parent / element.get("file", ""))
            elif element.tag in file_tags and element.get("file"):
                name = element.get("file", "")
                asset = xml_path.parent / "assets" / name
                if not asset.exists():
                    asset = xml_path.parent / name
                add(asset)

    visit(root_xml)
    payload = [{"path": path, "sha256": digest} for path, digest in sorted(records.items())]
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"),
                                    ensure_ascii=False).encode()).hexdigest()


def resolve_scene(filename: str) -> SceneSpec:
    if filename not in SCENES:
        raise ValueError(f"unsupported scene {filename!r}; choose one of {', '.join(SCENES)}")
    spec = SCENES[filename]
    if not spec.path.is_file():
        raise ValueError(f"scene XML is missing: {spec.path}")
    if hashlib.sha256(spec.path.read_bytes()).hexdigest() != spec.root_sha256:
        raise ValueError(f"scene root XML differs from catalog: {spec.path}")
    if _closure_sha256(spec.path) != spec.closure_sha256:
        raise ValueError(f"scene XML closure differs from catalog: {spec.path}")
    return spec
