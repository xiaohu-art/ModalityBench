# ModalityBench

A MuJoCo environment for robot teleoperation and data collection, using
MolmoSpaces scenes and Unitree G1.

Currently supports scene downloads, preview, collision-free bounding-box
placement, camera views, and optional GEAR-SONIC keyboard control in MuJoCo.
Data recording is planned.

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

## GEAR-SONIC keyboard control

Set up [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) with its MuJoCo simulation environment at `.venv_sim` and the GEAR-SONIC deployment executable and models. Configure the deployment runtime libraries, including TensorRT, in the controller terminal as required by that checkout.

Start the simulated FloorPlan1 in one terminal:

```bash
uv run modalitybench sonic --sonic-root /path/to/GR00T-WholeBodyControl
```

After it reports ready, start the controller in a second interactive terminal on the same machine:

```bash
uv run modalitybench sonic-keyboard --sonic-root /path/to/GR00T-WholeBodyControl
```

Wait for the controller to load its models. In the controller terminal, use `]` to start control, `Enter` to enable planner mode, and `2` to select walking in the default standing motion set. Use `W/S` for forward/backward, `,` and `.` to strafe left/right, and `A/D` for forward motion with a left/right adjustment. After G1 stabilizes, press `9` in the simulation window to release the startup suspension. Press `O` in the controller terminal to stop the controller. The simulator and controller communicate on loopback. The room keeps its collision geometry with its furniture joints fixed.

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
