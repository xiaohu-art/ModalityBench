"""Download a MolmoSpaces scene and optionally place a robot in it."""

import argparse
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def download_scene(scene: str = "FloorPlan1") -> Path:
    from molmospaces_resources import ResourceManager
    from molmospaces_resources.behaviors import InstallMode

    from asset_storage import ResumableR2Storage

    if not scene.isalnum():
        raise ValueError("Expected an iTHOR scene ID such as FloorPlan1")

    assets = Path(os.environ.get("MLSPACES_ASSETS_DIR", "~/.local/share/modalitybench/mjcf")).expanduser().resolve()
    cache = Path(os.environ.get("MLSPACES_CACHE_DIR", "~/.cache/modalitybench/molmospaces")).expanduser().resolve()
    source = os.environ.get("MODALITYBENCH_ASSET_URL", "mujoco-thor-resources")
    storage = ResumableR2Storage(source, cache / "downloads")
    versions = {"scenes": {"ithor": "20251217_with_occupancy"}, "objects": {"thor": "20251117"}}
    manager = ResourceManager(
        storage,
        symlink_dir=assets,
        cache_dir=cache,
        data_type_to_source_to_version=versions,
        source_overrides={("objects", "thor"): {"install_mode": InstallMode.ON_DEMAND}},
    )
    # Stage the small indexes before setup creates a version directory in the cache.
    for data_type, sources in versions.items():
        for name in sources:
            relative_path = manager.relative_path(data_type, name)
            if not (cache / relative_path).is_dir():
                storage.prepare_catalog(relative_path)
    manager.setup()
    filename = Path(f"{scene}_physics.xml")
    archives = manager.find_archives("scenes", "ithor", [filename])
    manager.install_packages("scenes", {"ithor": archives})
    scene_path = assets / "scenes" / "ithor" / filename

    # iTHOR physics XMLs refer to shared THOR files by paths relative to the scene.
    thor_root = assets / "objects" / "thor"
    thor_files = set()
    for element in ET.parse(scene_path).iter():
        if file := element.get("file"):
            path = Path(os.path.abspath(scene_path.parent / file))
            if path.is_relative_to(thor_root):
                thor_files.add(path.relative_to(thor_root))
    archives = manager.find_archives("objects", "thor", sorted(thor_files))
    manager.install_packages("objects", {"thor": archives})
    return scene_path


def load_scene(
    scene_path: Path,
    robot_path: Path | None = None,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    yaw_degrees: float = 0.0,
):
    import mujoco

    if robot_path is None:
        model = mujoco.MjModel.from_xml_path(str(scene_path))
    else:
        if not all(math.isfinite(value) for value in (*xyz, yaw_degrees)):
            raise ValueError("Robot position and yaw must be finite")
        scene = mujoco.MjSpec.from_file(str(scene_path))
        robot = mujoco.MjSpec.from_file(str(robot_path))
        half_yaw = math.radians(yaw_degrees) / 2
        spawn = scene.worldbody.add_frame(
            pos=xyz, quat=(math.cos(half_yaw), 0, 0, math.sin(half_yaw))
        )
        scene.attach(robot, frame=spawn, prefix="robot/")
        model = scene.compile()

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--scene", default="FloorPlan1", help="MolmoSpaces iTHOR scene ID")
    source.add_argument("--scene-mjcf", type=Path, help="Load an existing scene XML")
    parser.add_argument("--download-only", action="store_true")
    parser.add_argument("--robot-mjcf", type=Path, help="Robot model XML with its own assets")
    parser.add_argument("--xyz", nargs=3, type=float, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "Z"))
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--filament-png", type=Path, help="Render a PNG using the Filament MuJoCo build")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--lookat", nargs=3, type=float, default=(0, 0, 1))
    parser.add_argument("--distance", type=float, default=1.5)
    parser.add_argument("--azimuth", type=float, default=120)
    parser.add_argument("--elevation", type=float, default=-15)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.distance <= 0:
        parser.error("Image dimensions and camera distance must be positive")
    if args.filament_png and (args.viewer or args.download_only):
        parser.error("Use --filament-png with scene loading only")

    if args.download_only and (args.scene_mjcf or args.robot_mjcf or args.viewer):
        parser.error("--download-only expects a MolmoSpaces scene and no robot or viewer")
    if args.scene_mjcf is None and not args.scene.isalnum():
        parser.error("--scene must be an iTHOR scene ID such as FloorPlan1")

    scene_path = args.scene_mjcf or download_scene(args.scene)
    print(f"Scene: {scene_path}")
    if args.download_only:
        return

    model, data = load_scene(scene_path, args.robot_mjcf, tuple(args.xyz), args.yaw_deg)
    if args.robot_mjcf:
        print(f"Robot: {args.robot_mjcf}, offset={tuple(args.xyz)}, yaw={args.yaw_deg} degrees")
    print(f"Loaded {model.nbody} bodies, {model.ngeom} geoms and {model.nu} actuators")
    if args.filament_png:
        from scene_render import render_filament

        render_filament(model, data, args.filament_png, args.width, args.height,
                        args.lookat, args.distance, args.azimuth, args.elevation)
    if args.viewer:
        from mujoco import viewer

        viewer.launch(model, data)


if __name__ == "__main__":
    main()
