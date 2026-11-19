import gzip
import json
from collections.abc import Sequence
from pathlib import Path
import os
import re
import xml.etree.ElementTree as ET

from molmospaces_resources import CompactPathTrie, ResourceManager
from molmospaces_resources.behaviors import InstallMode
from molmospaces_resources.constants import COMBINED_TRIES_NAME

from .storage import ResumableR2Storage


DATA_DIR = Path(__file__).resolve().parent / "data"
ASSET_VERSIONS = {
    "scenes": {"ithor": "20251217_with_occupancy"},
    "objects": {"thor": "20251117"},
}


def list_scenes(
    *, data_dir: Path | None = None, source: str = "mujoco-thor-resources",
) -> list[str]:
    """Read the scene catalog without downloading asset archives."""
    root = (DATA_DIR if data_dir is None else Path(data_dir)).expanduser().resolve()
    storage = ResumableR2Storage(source, root / "cache" / "downloads")
    relative_path = Path("scenes/ithor") / ASSET_VERSIONS["scenes"]["ithor"]
    catalog = storage.prepare_catalog(relative_path)
    with gzip.open(catalog / COMBINED_TRIES_NAME, "rt") as stream:
        archives = json.load(stream)
    scenes = set()
    for paths in archives.values():
        for path in CompactPathTrie.from_dict(paths).leaf_paths():
            match = re.fullmatch(r"(FloorPlan[1-9][0-9]*)_physics\.xml", path)
            if match:
                scenes.add(match[1])
    return sorted(scenes, key=lambda name: int(name.removeprefix("FloorPlan")))


def load_scene(
    scene_id: str,
    *,
    data_dir: Path | None = None,
    source: str = "mujoco-thor-resources",
) -> Path:
    """Return a local MJCF path, downloading missing assets.

    data_dir defaults to scenes/data; source accepts a bucket name or HTTP mirror.
    """
    return load_scenes([scene_id], data_dir=data_dir, source=source)[0]


def load_scenes(
    scene_ids: Sequence[str],
    *,
    data_dir: Path | None = None,
    source: str = "mujoco-thor-resources",
) -> list[Path]:
    """Return MJCF paths in input order, deduplicating scene IDs and shared archives."""
    scene_ids = list(dict.fromkeys(scene_ids))
    if not scene_ids:
        raise ValueError("Select at least one scene")
    for scene_id in scene_ids:
        if not re.fullmatch(r"FloorPlan[1-9][0-9]*", scene_id):
            raise ValueError(f"Expected an iTHOR scene ID such as FloorPlan1, got {scene_id!r}")

    root = (DATA_DIR if data_dir is None else Path(data_dir)).expanduser().resolve()
    assets = root / "mjcf"
    cache = root / "cache"
    storage = ResumableR2Storage(source, cache / "downloads")
    manager = ResourceManager(
        storage,
        symlink_dir=assets,
        cache_dir=cache,
        data_type_to_source_to_version=ASSET_VERSIONS,
        source_overrides={("objects", "thor"): {"install_mode": InstallMode.ON_DEMAND}},
    )

    # Stage indexes first so failed downloads cannot leave unregistered directories.
    for data_type, sources in ASSET_VERSIONS.items():
        for name in sources:
            relative_path = manager.relative_path(data_type, name)
            if not (cache / relative_path).is_dir():
                storage.prepare_catalog(relative_path)
    manager.setup()

    filenames = [Path(f"{scene_id}_physics.xml") for scene_id in scene_ids]
    archives = manager.find_archives("scenes", "ithor", filenames)
    manager.install_packages("scenes", {"ithor": archives})
    scene_paths = [assets / "scenes" / "ithor" / filename for filename in filenames]

    # Preserve the symlink tree used by relative object paths.
    thor_root = assets / "objects" / "thor"
    thor_files = set()
    for scene_path in scene_paths:
        for element in ET.parse(scene_path).iter():
            if file := element.get("file"):
                path = Path(os.path.abspath(scene_path.parent / file))
                if path.is_relative_to(thor_root):
                    thor_files.add(path.relative_to(thor_root))

    if thor_files:
        archives = manager.find_archives("objects", "thor", sorted(thor_files))
        manager.install_packages("objects", {"thor": archives})
    return scene_paths
