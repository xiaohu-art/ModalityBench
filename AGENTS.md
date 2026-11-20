# Repository Guidelines

## Project Structure & Module Organization

ModalityBench is a MuJoCo environment for robot teleoperation and data collection using MolmoSpaces scenes and Unitree G1. Current functionality covers scene downloads, Studio preview, bounding-box placement, robot state access, and camera rendering. Teleoperation and data recording remain planned functionality.

- `scripts/main.py`: Typer CLI and application assembly.
- `src/modalitybench/assets/scenes/`: MolmoSpaces catalog lookup, asset preparation, resumable storage, and Studio preview.
- `src/modalitybench/assets/robots/g1/`: G1 MJCF and referenced meshes.
- `src/modalitybench/robots/`: articulation configuration, model indexing, state access, G1 variants, and cameras.
- `src/modalitybench/utils/placement.py`: robot bounding boxes, collision queries, and XY/yaw sampling.
- `src/modalitybench/simulator.py`: model composition, state, stepping, Filament rendering, and static Studio display.
- `src/modalitybench/policy/base.py`: policy protocol for future control integration.
- `pyproject.toml` and `uv.lock`: packaging, dependencies, and CLI entry point.
- `README.md`: user-facing setup and usage.

The package uses Hatchling with source code under `src/`. Keep robot XML files alongside their referenced meshes and textures.

## Build, Test, and Development Commands

Use Linux, Python 3.12, and uv. MuJoCo 4.0.1 is installed through the project dependencies. Run commands from the repository root:

```bash
uv sync
uv run modalitybench --help
uv run modalitybench download --scenes FloorPlan1
uv run modalitybench download --scenes FloorPlan1 --scenes FloorPlan2
uv run modalitybench download --all
uv run modalitybench preview
uv run modalitybench run --scene FloorPlan1 --seed 42
```

Preview and Studio windows require a graphical desktop. Preview provides a scene selector. The run command places G1 with a head camera and keeps physics frozen; its Cameras panel switches between third-person and model-camera views. Use `uv run modalitybench <command> --help` for options. Studio supports the CLI's OpenGL and Vulkan graphics choices. Offscreen RGB rendering is exposed through `Simulator.render()`.

## Coding Style & Naming Conventions

Follow existing Python style: four-space indentation, snake_case functions and variables, PascalCase classes, and short module docstrings. Use pathlib.Path and type hints consistent with neighboring functions. Preserve deferred imports for optional rendering and simulation dependencies. No formatter or linter is configured.

## Testing Guidelines

No automated test framework or coverage threshold is configured. Check CLI changes with --help and affected argument combinations. Validate scene preparation with a cached scene or a controlled download. For placement changes, check bounding-box queries and seeded sampling. For robot changes, check joint/body indexing, initial posture, and camera names. Inspect rendered images or Studio views when changing rendering. Exercise interrupted-download recovery when modifying storage. Report validation commands and required environment in pull requests.

## Commit & Pull Request Guidelines

Use concise, imperative commit subjects. Keep commits focused. Pull requests should explain behavior changes, link relevant issues, and record validation results. Include before/after images for rendering changes and update README.md when setup or CLI usage changes.

## Asset Configuration

Scene data defaults to src/modalitybench/assets/scenes/data, with mjcf links and a separate cache subtree. Python asset APIs accept data_dir and source overrides; source accepts the official bucket name or a compatible HTTP mirror. Asset versions are defined by ASSET_VERSIONS in molmospaces.py. Reuse the same cache to resume interrupted downloads. Keep downloaded archives, generated images, and local environments out of commits.
