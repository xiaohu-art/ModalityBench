# ModalityBench

Download MolmoSpaces iTHOR assets, load scenes in MuJoCo, place an MJCF robot, and render images with Filament.

The commands below use Linux and Python 3.11. Run them from the repository root.

## Install

```bash
git clone https://github.com/xiaohu-art/ModalityBench.git
cd ModalityBench
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Download and load a scene

```bash
python mujoco_scene.py --scene FloorPlan1 --download-only
python mujoco_scene.py --scene FloorPlan1
```

Loading by scene ID downloads missing assets automatically. To open an interactive MuJoCo window, add `--viewer` on a machine with a graphical display.

```bash
python mujoco_scene.py --scene FloorPlan1 --viewer
```

To load an existing MJCF file, use `--scene-mjcf`.

```bash
python mujoco_scene.py --scene-mjcf /path/to/scene.xml
```

## Place a robot

Download the Unitree G1 model from MuJoCo Menagerie.

```bash
git clone --depth 1 --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git ../mujoco_menagerie
git -C ../mujoco_menagerie sparse-checkout set unitree_g1
python mujoco_scene.py --scene FloorPlan1 \
  --robot-mjcf ../mujoco_menagerie/unitree_g1/g1.xml \
  --xyz 0 0 0 --yaw-deg 90
```

Keep each robot XML alongside its referenced meshes and textures. `--xyz` sets a placement frame in world coordinates, in meters. `--yaw-deg` rotates the frame about world Z. The robot model's original root pose is relative to this frame. Choose a clear location and set the frame's Z to the local floor height. The example initializes the robot pose. Balance control and collision-free placement require additional logic.

## Render with Filament

Create a separate environment and install the Filament-enabled MuJoCo wheel matching your Python version and platform. Wheels are provided in the [MolmoSpaces repository](https://github.com/allenai/molmospaces/tree/main/bin/wheels). The example below uses its Python 3.11 Linux x86-64 wheel.

```bash
python3.11 -m venv .venv-filament
source .venv-filament/bin/activate
python -m pip install -r requirements-filament.txt \
  /path/to/mujoco-3.7.1-cp311-cp311-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
python mujoco_scene.py --scene FloorPlan1 \
  --robot-mjcf ../mujoco_menagerie/unitree_g1/g1.xml \
  --filament-png outputs/floorplan1.png \
  --width 960 --height 720 \
  --lookat 0 0 1 --distance 1.5 --azimuth 120 --elevation -15
```

Filament uses Vulkan to save a PNG of the initial scene state. Set `--lookat` in world coordinates, `--distance` in meters, and `--azimuth` and `--elevation` in degrees. Choose a camera position inside the room to keep walls and ceilings from blocking the view. Use `--scene-mjcf` to render an existing scene file.

## Asset storage

By default, assets are linked under `~/.local/share/modalitybench/mjcf` and cached under `~/.cache/modalitybench/molmospaces`. Override these locations with separate directories if needed.

```bash
export MLSPACES_ASSETS_DIR="$PWD/data/mjcf"
export MLSPACES_CACHE_DIR="$PWD/data/cache"
```

Downloads resume from saved bytes when you rerun a command with the same cache directory. Allow space for both compressed archives and extracted files. A first download of `FloorPlan1` requires about 1.1 GB of compressed assets.

An optional HTTP mirror can be selected with `MODALITYBENCH_ASSET_URL`. It must preserve the official directory layout and index files under `scenes/ithor/20251217_with_occupancy` and `objects/thor/20251117`.

For all command-line options, run `python mujoco_scene.py --help`.
