"""Download a MolmoSpaces scene and optionally place a robot in it."""

import argparse
import math
import os
from pathlib import Path


def download_scene(scene: str = "FloorPlan1") -> Path:
    from molmospaces_resources import R2RemoteStorage, setup_resource_manager

    if not scene.isalnum():
        raise ValueError("Expected an iTHOR scene ID such as FloorPlan1")

    assets = Path(os.environ.get("MLSPACES_ASSETS_DIR", "~/.local/share/modalitybench/mjcf")).expanduser()
    cache = Path(os.environ.get("MLSPACES_CACHE_DIR", "~/.cache/modalitybench/molmospaces")).expanduser()
    manager = setup_resource_manager(
        R2RemoteStorage("mujoco-thor-resources"),
        symlink_dir=assets,
        cache_dir=cache,
        env_prefix="MLSPACES",
        versions={
            "scenes": {"ithor": "20251217_with_occupancy"},
            "objects": {"thor": "20251117"},
        },
    )
    filename = Path(f"{scene}_physics.xml")
    archives = manager.find_archives("scenes", "ithor", [filename])
    if not archives:
        raise FileNotFoundError(f"Scene {scene!r} is not in the MolmoSpaces iTHOR index")
    manager.install_packages("scenes", {"ithor": archives})
    scene_path = assets / "scenes" / "ithor" / filename
    if not scene_path.is_file():
        raise FileNotFoundError(scene_path)
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
    args = parser.parse_args()

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
    if args.viewer:
        from mujoco import viewer

        viewer.launch(model, data)


if __name__ == "__main__":
    main()
