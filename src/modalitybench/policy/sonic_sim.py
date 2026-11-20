"""Run the GEAR-SONIC simulation bridge in its existing Python environment."""

import argparse
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import mujoco
import numpy as np


SPAWN = (0.747010221, 1.646574242, 0.793)


def absolute_assets(spec, source):
    for assets, directory in ((spec.meshes, spec.meshdir), (spec.textures, spec.texturedir)):
        for asset in assets:
            if asset.file:
                asset.file = str((source.parent / directory / asset.file).absolute())
    spec.meshdir = ""
    spec.texturedir = ""


def compose(room_path, robot_path):
    robot = mujoco.MjSpec.from_file(str(robot_path))
    baseline = robot.compile()
    absolute_assets(robot, robot_path)
    friction = baseline.geom("floor").friction.copy()
    robot.delete(robot.geom("floor"))
    robot.body("pelvis").pos = SPAWN

    room = mujoco.MjSpec.from_file(str(room_path))
    absolute_assets(room, room_path)
    # SONIC expects its original joint, sensor, and actuator layout.
    for collection in ("actuators", "sensors", "equalities", "tendons", "keys", "joints"):
        for element in list(getattr(room, collection)):
            room.delete(element)
    for geom in room.geoms:
        if geom.name.startswith("ceiling_") and "_visual_" in geom.name:
            geom.group = 5
    robot.attach(room, prefix="room_", frame=robot.worldbody.add_frame())
    model = robot.compile()
    for kind, count in (("joint", "njnt"), ("sensor", "nsensor"), ("actuator", "nu")):
        expected = [getattr(baseline, kind)(i).name for i in range(getattr(baseline, count))]
        actual = [getattr(model, kind)(i).name for i in range(getattr(model, count))]
        if actual != expected:
            raise ValueError(f"Room composition changed SONIC's {kind} layout")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    floors = [i for i in range(model.ngeom)
              if model.geom(i).name.startswith("room_floor") and model.geom_contype[i]]
    if not floors:
        raise ValueError("Expected collision-enabled FloorPlan1 floor geometry")
    for i in range(model.ngeom):
        if any(model.geom_type[i] == model.geom_type[floor]
               and np.allclose(model.geom_size[i], model.geom_size[floor])
               and np.allclose(data.geom_xpos[i], data.geom_xpos[floor])
               and np.allclose(data.geom_xmat[i], data.geom_xmat[floor]) for floor in floors):
            robot.geom(model.geom(i).name).friction = friction
    return robot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sonic_root", type=Path)
    parser.add_argument("scene", type=Path)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    sys.path.insert(0, str(args.sonic_root))
    from gear_sonic.utils.mujoco_sim.base_sim import BaseSimulator
    from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig

    robot_path = args.sonic_root / "gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml"
    with TemporaryDirectory(prefix="modalitybench-sonic-") as directory:
        scene_path = Path(directory) / "floorplan1.xml"
        scene_path.write_text(compose(args.scene, robot_path).to_xml(), encoding="utf-8")
        config = SimLoopConfig(interface="lo", enable_onscreen=not args.headless).load_wbc_yaml()
        config["ROBOT_SCENE"] = str(scene_path)
        config["PRINT_SCENE_INFORMATION"] = False
        sim = BaseSimulator(config, onscreen=not args.headless, offscreen=False)
        env = sim.sim_env
        step = env.sim_step

        def step_with_controller():
            # Publish the initial state while SONIC loads its engines and waits for ].
            if not any(motor.kp > 0 for motor in sim.unitree_bridge.low_cmd.motor_cmd):
                mujoco.mj_forward(env.mj_model, env.mj_data)
                sim.unitree_bridge.PublishLowState(env.prepare_obs())
                return
            step()
            # The bridge resets MjData on a fall; refresh kinematics before the next step.
            if not np.any(env.mj_data.xquat[env.band_attached_link]):
                mujoco.mj_forward(env.mj_model, env.mj_data)

        env.sim_step = step_with_controller
        env.elastic_band.point[:] = (*SPAWN[:2], SPAWN[2] + 0.03)
        if env.viewer:
            env.viewer.cam.lookat[:] = SPAWN
            env.viewer.opt.geomgroup[3:6] = False
        print("SONIC simulation ready. Start the keyboard controller on loopback.", flush=True)
        sim.start()


if __name__ == "__main__":
    main()
