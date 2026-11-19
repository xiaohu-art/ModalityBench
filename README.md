# ModalityBench

A MuJoCo environment for robot teleoperation and data collection, using
MolmoSpaces scenes and Unitree G1.

Currently supports scene downloads, preview, collision-free bounding-box
placement and camera views. Teleoperation and data recording are not implemented yet.

## Setup

Linux, Python 3.12 and uv. MuJoCo 4.0.1 is installed with the project dependencies.
Preview and simulation windows require a graphical desktop.

```bash
uv sync
```

## Usage

Preview a scene: select it in the dropdown and click **Preview**.

```bash
uv run modalitybench preview
```

Place G1 with a head camera in FloorPlan1 and display it in Studio:

```bash
uv run modalitybench run
uv run modalitybench run --scene FloorPlan1 --seed 42
```

The robot stays fixed; physics does not advance. Use the **Cameras** panel to
switch between third-person and model camera views.

Download scenes ahead of time:

```bash
uv run modalitybench download --scenes FloorPlan1 --scenes FloorPlan2
uv run modalitybench download --all
```

`--all` downloads the supported iTHOR catalog. Existing assets are reused;
missing assets download automatically when loading a scene. Rerun interrupted
downloads to resume. See `uv run modalitybench <command> --help` for options.

## Structure

```text
scripts/main.py                 # CLI and application assembly
src/modalitybench/
├── assets/                     # Scene downloads and local robot assets
├── robots/                     # Articulation, G1 variants and camera configuration
├── policy/                     # Teleoperation policy interface (placeholder)
├── utils/placement.py          # Bounding-box checks and XY/yaw sampling
└── simulator.py                # MuJoCo state, rendering and Studio window
```

`G1` provides the base robot and default posture. `G1WithHeadCamera` adds the
head camera and is used by `run`. Additional cameras use the shared `CameraCfg`.
