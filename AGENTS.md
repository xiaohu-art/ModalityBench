# Repository Guidelines

## Project Structure & Module Organization

ModalityBench downloads MolmoSpaces iTHOR assets, loads MuJoCo scenes, places MJCF robots, and renders PNGs with Filament. Source files live at the repository root:

- `mujoco_scene.py`: CLI, scene asset resolution, scene loading, and robot placement.
- `asset_storage.py`: resumable HTTP downloads, caching, and file locking.
- `scene_render.py`: Filament camera setup and offscreen rendering.
- `requirements.txt`: shared Python dependencies; `README.md`: setup and usage examples.

There is currently no test directory or packaged application build. Scene assets are downloaded externally; keep robot XML files alongside their referenced meshes and textures.

## Build, Test, and Development Commands

Use Linux and Python 3.11. Run commands from the repository root:

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python mujoco_scene.py --help
python mujoco_scene.py --scene FloorPlan1 --download-only
python mujoco_scene.py --scene FloorPlan1
```

These commands create an environment, install dependencies, display CLI options, download assets, and load a scene. Add `--viewer` for an interactive window with a graphical display. Use `--scene-mjcf /path/to/scene.xml` for local MJCF input.

Filament rendering requires a separate environment, the compatible MolmoSpaces MuJoCo wheel described in `README.md`, and Vulkan. Render with `--filament-png outputs/floorplan1.png`.

## Coding Style & Naming Conventions

Follow existing Python style: four-space indentation, `snake_case` functions and variables, `PascalCase` classes, and short module docstrings. Use `pathlib.Path` for filesystem paths and type hints consistent with neighboring functions. Preserve deferred imports for optional rendering and simulation dependencies. No formatter or linter is configured.

## Testing Guidelines

No automated test framework or coverage threshold is configured. Check CLI changes with `--help` and affected argument combinations. Validate loading changes against a local MJCF or cached scene; inspect generated PNGs for rendering changes. Exercise interrupted-download recovery when modifying storage. Report exact validation commands and required environment in pull requests.

## Commit & Pull Request Guidelines

History uses concise, imperative commit subjects, such as “Add Filament offscreen rendering for MuJoCo scenes.” Keep commits focused. Pull requests should explain the behavior change, link relevant issues, and record validation results. Include before/after images for rendering changes and update `README.md` when setup or CLI usage changes.

## Asset Configuration

Set `MLSPACES_ASSETS_DIR` and `MLSPACES_CACHE_DIR` to separate directories when overriding defaults. `MODALITYBENCH_ASSET_URL` selects an optional compatible mirror. Keep downloaded archives, generated images, and local environments out of commits.
