# ModalityBench

## Filament image rendering

Use a separate Python 3.11 environment for the MolmoSpaces Filament MuJoCo wheel. The standard environment uses MuJoCo 3.5, while the Filament wheel tested on the server is 3.7.1. Install `requirements-filament.txt` in the Filament environment, together with the platform-specific wheel from `allenai/molmospaces/bin/wheels`.

```bash
python -m pip install -r requirements-filament.txt /path/to/mujoco-3.7.1-cp311-cp311-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
python mujoco_scene.py --scene-mjcf /path/to/FloorPlan1_physics.xml \
  --robot-mjcf /path/to/unitree_g1/g1.xml \
  --filament-png outputs/floorplan1.png \
  --lookat 0 0 1 --distance 1.5 --azimuth 120 --elevation -15
```

On our server the existing Filament Python is `/home/jnli/.local/share/robot_projects/dependencies/filament_viewer_venv/bin/python`. This path renders through Vulkan and supports offscreen PNG output. `--viewer` continues to launch the MuJoCo viewer. The Filament output captures the initial scene state. Robot control and live state streaming are outside this entry point. Choose camera coordinates inside the room to avoid viewing the ceiling from outside.

The implementation follows the rendering API used by the server's `custom_control/molmospaces/filament_viewer.py`. Tested with FloorPlan1 and G1 at 640 by 480 pixels on an NVIDIA RTX A6000. The upstream Filament build may print a handle-arena warning for the full physics model while falling back to heap allocation.

Load MolmoSpaces iTHOR scenes and place an MJCF robot in them with MuJoCo. The commands below are for Linux with Python 3.11.

## Setup

From this repository root, create a Python environment and install MuJoCo and the MolmoSpaces asset downloader.

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

By default, the downloader links MJCF assets under `~/.local/share/modalitybench/mjcf` and caches files under `~/.cache/modalitybench/molmospaces`. Set `MLSPACES_ASSETS_DIR` and `MLSPACES_CACHE_DIR` to override those paths. Keep the MJCF assets directory separate from any existing USD cache. The downloader fetches the archives referenced by the selected scene and reuses extracted archives on later runs. Shared archives can still be large. `FloorPlan1` requires about 1.1 GB of compressed assets on an empty cache.

If the server cannot reach the official asset host reliably, set `MODALITYBENCH_ASSET_URL` to an HTTP mirror that preserves the MolmoSpaces directory layout (`scenes/ithor/20251217_with_occupancy` and `objects/thor/20251117`, including their index files).

Downloads are staged under `MLSPACES_CACHE_DIR/downloads`. If a transfer stalls, rerun with the same cache directory to continue from the saved bytes. Keep enough disk space for both the archives and the extracted assets.

## Download and load a scene

```bash
python mujoco_scene.py --scene FloorPlan1 --download-only
python mujoco_scene.py --scene FloorPlan1
```

Use `--scene-mjcf /path/to/scene.xml` to load a local MJCF directly. Add `--viewer` on a machine with a graphical display. Without it, the program loads the model and initializes its state without opening a window.

## Place a robot

Pass the robot model XML with its meshes and textures in their original directories. For example, use the Unitree G1 model from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1).

```bash
git clone --depth 1 --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git ../mujoco_menagerie
git -C ../mujoco_menagerie sparse-checkout set unitree_g1
python mujoco_scene.py --scene FloorPlan1 \
  --robot-mjcf ../mujoco_menagerie/unitree_g1/g1.xml \
  --xyz 0 0 0 --yaw-deg 90
```

`--xyz` sets the placement frame in world coordinates, in meters. `--yaw-deg` rotates that frame about world Z, in degrees. The robot model's root pose is relative to this frame. For example, Menagerie's G1 already has a pelvis height of 0.793 m. Choose a clear spot and set the frame's Z to the local floor height. The placement applies to the initial model state. Collision-free placement and balance control are outside this example.
