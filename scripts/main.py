from enum import StrEnum
from typing import Annotated

import typer


app = typer.Typer(
    help="ModalityBench: MuJoCo simulation and teleoperation data collection.",
    no_args_is_help=True,
    add_completion=False,
    pretty_exceptions_enable=False,
)


class GraphicsBackend(StrEnum):
    opengl = "opengl_software"
    vulkan = "vulkan_software"


@app.command(help="Select and preview scenes in MuJoCo Studio")
def preview(gfx: GraphicsBackend = GraphicsBackend.opengl):
    from modalitybench.assets.scenes.preview import launch_preview

    launch_preview(gfx.value)


@app.command(help="Place G1 randomly in a scene and keep it static in Studio")
def run(
    scene: str = "FloorPlan1",
    seed: Annotated[int | None, typer.Option(min=0)] = None,
    max_attempts: Annotated[int, typer.Option(min=1)] = 500,
    gfx: GraphicsBackend = GraphicsBackend.opengl,
):
    from modalitybench.assets.scenes import load_scene
    from modalitybench.robots import G1WithHeadCamera
    from modalitybench.simulator import Simulator
    from modalitybench.utils.placement import find_robot_placement

    try:
        print(f"Preparing {scene}; existing assets will be reused.", flush=True)
        scene_path = load_scene(scene)
        robot = G1WithHeadCamera()
        print("Searching for a collision-free bounding-box placement...", flush=True)
        xy, yaw = find_robot_placement(
            scene_path, robot,
            ((-1.5, 1.5), (-1.5, 1.5)), seed=seed, max_attempts=max_attempts,
        )
        simulator = Simulator(scene=scene_path, robot=robot, xy=xy, yaw=yaw)
        print("Opening static view. Physics time is frozen; close the window or press Ctrl+C to exit.", flush=True)
        simulator.launch_static(gfx=gfx.value)
    except KeyboardInterrupt:
        typer.echo("\nStopped.")
    except Exception as exc:
        typer.echo(f"Run failed: {exc}", err=True)
        raise typer.Exit(1) from exc


@app.command(help="Prepare scene assets without opening a viewer")
def download(
    scenes: Annotated[list[str] | None, typer.Option(help="Repeat for each scene ID.")] = None,
    all_scenes: Annotated[bool, typer.Option("--all", help="Download all supported iTHOR scenes.")] = False,
):
    if bool(scenes) == all_scenes:
        raise typer.BadParameter("Choose either --scenes SCENE (repeatable) or --all.")

    from modalitybench.assets.scenes import list_scenes, load_scenes

    try:
        scene_ids = list_scenes() if all_scenes else list(dict.fromkeys(scenes))
        if not scene_ids:
            raise ValueError("No scenes were found in the catalog.")
        print(f"Preparing {len(scene_ids)} scene(s); existing assets will be reused.", flush=True)
        paths = load_scenes(scene_ids)
    except KeyboardInterrupt:
        typer.echo("\nDownload interrupted. Rerun the same command to resume.", err=True)
        raise typer.Exit(130)
    except Exception as exc:
        typer.echo(f"Download failed: {exc}\nRerun the same command to resume.", err=True)
        raise typer.Exit(1) from exc
    for path in paths:
        typer.echo(path)
    typer.echo(f"Ready: {len(paths)} scene(s).")


if __name__ == "__main__":
    app()
