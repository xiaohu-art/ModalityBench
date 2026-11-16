# ModalityBench

Load MolmoSpaces iTHOR scenes and place an MJCF robot in them with MuJoCo. The commands below are for Linux with Python 3.11.

## Setup

From this repository root, create a Python environment and install MuJoCo and the MolmoSpaces asset downloader.

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

By default, the downloader links MJCF assets under `~/.local/share/modalitybench/mjcf` and caches files under `~/.cache/modalitybench/molmospaces`. Set `MLSPACES_ASSETS_DIR` and `MLSPACES_CACHE_DIR` to override those paths. Keep the MJCF assets directory separate from any existing USD cache. The first download also fetches about 1.5 GB of compressed THOR object assets shared by iTHOR scenes.

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
