from pathlib import Path
from typing import Any
import math
import time

import mujoco
import numpy as np

from modalitybench.robots.articulation import Articulation


class Simulator:
    def __init__(self, scene: Path | None = None, *, robot: Articulation,
                 xy: tuple[float, float] = (0.0, 0.0), yaw: float = 0.0):
        spec = (mujoco.MjSpec.from_file(str(Path(scene).absolute()))
                if scene is not None else mujoco.MjSpec())
        robot_spec = robot.build_spec()
        spec.attach(robot_spec, prefix=robot.prefix, frame=spec.worldbody.add_frame())
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)

        self.robot = robot
        self.robot.initialize(self.model, self.data)
        q = self.robot.indexing.free_joint_q_adr
        self.data.qpos[q[:2]] = xy
        self.data.qpos[q[3:]] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
        self._initial_qpos = self.data.qpos.copy()
        
        self._renderer = None
        self._render_scene = None
        self.forward()

    @property
    def camera_names(self) -> tuple[str, ...]:
        return tuple(self.model.camera(i).name for i in range(self.model.ncam))

    def forward(self) -> None:
        """Refresh link poses and velocities after editing qpos/qvel directly."""
        mujoco.mj_forward(self.model, self.data)

    def reset(self) -> dict[str, Any]:
        """Restore the configured initial pose and zero velocities."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self._initial_qpos
        self.forward()
        return self.observe()

    def step(self, action: Any = None) -> dict[str, Any]:
        if action is not None:
            raise NotImplementedError("Policy actions and robot actuators are not configured yet.")
        mujoco.mj_step(self.model, self.data)
        # Integration changes qpos after kinematics; refresh before taking snapshots.
        self.forward()
        return self.observe()

    def observe(self) -> dict[str, Any]:
        return {
            "time": float(self.data.time),
            "robot": self.robot.read_state(),
        }

    def _camera(self, camera: str | int | None = None) -> mujoco.MjvCamera:
        view = mujoco.MjvCamera()
        if camera is None:
            view.type = mujoco.mjtCamera.mjCAMERA_FREE
            view.lookat[:] = self.robot.data.root_link_pos_w
            view.distance = 1.8
            view.azimuth = 135
            view.elevation = -15
        else:
            view.type = mujoco.mjtCamera.mjCAMERA_FIXED
            view.fixedcamid = self.model.camera(camera).id
        return view

    def render(self, camera: str | int | None = None, *, width: int = 640,
               height: int = 480) -> np.ndarray:
        """Filament RGB uint8 (H, W, 3). None selects the free third-person view.

        Call render() and close() on the same thread. Physics time is unchanged.
        """
        from mujoco import mjrf
        from mujoco.rendering.filament import renderer
        from mujoco.studio import native_viewer_cc

        view = self._camera(camera)
        if self._renderer is None:
            context = mjrf.Context(mjrf.ContextConfig(
                graphics_api=mjrf.GraphicsApi.GRAPHICS_API_OPENGL,
            ))
            self._renderer = renderer.Renderer(context)
            self._render_scene = native_viewer_cc.ModelScene(
                context, self._renderer.scene("scene"), self.model,
            )
        pixels = np.empty((height, width, 3), dtype=np.uint8)
        options = mujoco.MjvOption()
        options.geomgroup[3:6] = False
        options.sitegroup[:] = False
        self._render_scene.sync(width, height, self.model, self.data,
                                mujoco.MjvPerturb(), view, options, [])
        self._renderer.target("rgb", pixels)
        self._renderer.render([renderer.View(
            scene="scene", target="rgb",
            camera=mujoco.mjv_camera2GLCamera(self.model, self.data, view),
            headlight=self.model.vis.headlight.active != 0,
            headlight_color=tuple(self.model.vis.headlight.diffuse),
            headlight_intensity=self._render_scene.get_headlight_intensity(),
        )])
        return pixels

    def close(self) -> None:
        self._render_scene = None
        self._renderer = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def launch_static(self, *, gfx: str = "opengl_software") -> None:
        """Display a fixed state with all model cameras and a free third-person view."""
        from mujoco.studio import app, launch_native, messages, plugin, sim, ux, viewer_protocol
        from mujoco.studio.dear_imgui import dear_imgui as imgui

        simulator = self
        studio_app = app.App(app.Config(show_options=False, show_inspector=False))

        class FrozenScene(plugin.SimPlugin):
            def __init__(self):
                self.signature = mujoco.mjtState.mjSTATE_INTEGRATION
                self.state = np.empty(mujoco.mj_stateSize(simulator.model, self.signature))
                self.current = np.empty_like(self.state)
                mujoco.mj_getState(simulator.model, simulator.data, self.state, self.signature)

            @messages.handler
            def on_model(self, event: messages.PostModelEvent) -> None:
                self.handle.send_to_viewer(messages.RequestPauseEvent(sim.PauseState.NORMAL_PAUSED))

            @messages.handler
            def on_pause(self, event: messages.RequestPauseEvent) -> bool:
                self.handle.send_to_viewer(messages.RequestPauseEvent(sim.PauseState.NORMAL_PAUSED))
                return True

            @messages.handler
            def on_step(self, event: messages.StepEvent) -> None:
                mujoco.mj_getState(event.model, event.data, self.current, self.signature)
                if not np.array_equal(self.current, self.state):
                    mujoco.mj_setState(event.model, event.data, self.state, self.signature)
                    mujoco.mj_forward(event.model, event.data)
                time.sleep(1 / 60)

        class CameraViews(plugin.ViewerPlugin):
            def __init__(self):
                self.names = ()

            def select_camera(self, index: int) -> None:
                studio_app.ux_state.camera_index = ux.set_camera(
                    self.viewer.model, self.viewer.camera, index,
                )
                if index == ux.TUMBLE_CAMERA_IDX:
                    self.viewer.camera = simulator._camera()

            @messages.handler(priority=messages.Priority.INTERNAL - 1)
            def on_model(self, event: messages.PostModelEvent) -> None:
                # Apply after Studio resets its default camera.
                self.names = tuple(event.model.camera(i).name or f"Camera {i}"
                                   for i in range(event.model.ncam))
                self.viewer.vis_options.geomgroup[3:6] = False
                self.viewer.vis_options.sitegroup[:] = False
                self.select_camera(ux.TUMBLE_CAMERA_IDX)

            @messages.handler
            def on_gui(self, event: messages.BuildGuiEvent) -> None:
                expanded, _ = imgui.Begin("Cameras")
                try:
                    if not expanded:
                        return
                    selected = studio_app.ux_state.camera_index
                    label = self.names[selected] if selected >= 0 else "Third person"
                    if imgui.BeginCombo("View", label):
                        if imgui.Selectable("Third person", selected < 0):
                            self.select_camera(ux.TUMBLE_CAMERA_IDX)
                        for index, name in enumerate(self.names):
                            if imgui.Selectable(f"{name}##camera_{index}", selected == index):
                                self.select_camera(index)
                        imgui.EndCombo()
                    imgui.TextUnformatted(f"{len(self.names)} cameras + third person")
                finally:
                    imgui.End()

        config = viewer_protocol.ViewerConfig(title="ModalityBench - Static Scene", gfx=gfx)
        launch_native.run(
            config, model=self.model, data=self.data,
            plugins=[studio_app, CameraViews(), FrozenScene()],
        )

