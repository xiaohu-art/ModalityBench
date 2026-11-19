from concurrent.futures import Future
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, SimpleQueue
from threading import Thread
import time

import mujoco
from mujoco.studio import app, launch_native, messages, plugin, sim, viewer_protocol
from mujoco.studio.dear_imgui import dear_imgui as imgui

from .molmospaces import list_scenes, load_scene


@dataclass(frozen=True)
class PreviewModelEvent(messages.Event):
    model: mujoco.MjModel
    path: str


class PreviewSimulation(plugin.SimPlugin):
    @messages.handler
    def on_preview_model(self, event: PreviewModelEvent) -> None:
        self.handle.set_model(event.model, path=event.path)
        self.handle.dispatch(messages.RequestPauseEvent(sim.PauseState.NORMAL_PAUSED))


class ScenePreview(plugin.ViewerPlugin):
    def __init__(self):
        self.scenes: list[str] = []
        self.selected = ""
        self.status = "Reading scene catalog..."
        self.future: Future | None = None
        self.job = ""
        self.started = 0.0
        self.awaiting_model = False
        self.progress: SimpleQueue[str] = SimpleQueue()

    @property
    def busy(self) -> bool:
        return self.future is not None or self.awaiting_model

    def _start(self, job: str, work) -> None:
        future = Future()
        self.future = future
        self.job = job
        self.started = time.monotonic()

        def run():
            try:
                future.set_result(work())
            except Exception as exc:
                future.set_exception(exc)

        # Daemon threads let the viewer close during a download.
        Thread(target=run, name="scene-preview-loader", daemon=True).start()

    def read_catalog(self) -> None:
        if self.busy:
            return
        self.status = "Reading scene catalog..."
        self._start("catalog", list_scenes)

    def load_selected(self) -> None:
        if self.busy or not self.selected:
            return
        scene_id = self.selected
        self.status = f"Preparing assets for {scene_id}..."

        def prepare():
            path = load_scene(scene_id)
            self.progress.put(f"Compiling {scene_id} from local assets...")
            model = mujoco.MjModel.from_xml_path(str(path))
            return PreviewModelEvent(model=model, path=str(path))

        self._start("model", prepare)

    @messages.handler
    def on_init(self, event: plugin.ViewerInitEvent) -> None:
        self.read_catalog()

    @messages.handler
    def on_update(self, event: messages.UpdateEvent) -> None:
        # Apply worker progress on the viewer thread.
        while True:
            try:
                self.status = self.progress.get_nowait()
            except Empty:
                break
        if self.future is None or not self.future.done():
            return
        future, self.future = self.future, None
        try:
            result = future.result()
            if self.job == "catalog":
                if not result:
                    raise ValueError("No iTHOR scenes were found in the catalog.")
                self.scenes = result
                if self.selected not in result:
                    self.selected = result[0]
                self.status = f"{len(result)} scenes available. Select a scene and click Preview."
            else:
                self.awaiting_model = True
                self.status = f"Preparing rendering for {self.selected}..."
                self.viewer.send_to_sim(result)
        except Exception as exc:
            self.awaiting_model = False
            self.status = f"Failed: {exc}. You can retry."

    @messages.handler
    def on_model(self, event: messages.PostModelEvent) -> None:
        if self.awaiting_model:
            self.awaiting_model = False
            self.status = f"Loaded {Path(event.path).stem.removesuffix('_physics')} (paused)."
        self.viewer.vis_options.geomgroup[3:6] = False
        self.viewer.vis_options.sitegroup[:] = False

    @messages.handler
    def on_gui(self, event: messages.BuildGuiEvent) -> None:
        imgui.SetNextWindowSize(imgui.Vec2(390, 220), imgui.Cond.FirstUseEver)
        expanded, _ = imgui.Begin("Scene Preview")
        try:
            if not expanded:
                return
            imgui.BeginDisabled(self.busy)
            if imgui.BeginCombo("Scene", self.selected or "Select a scene"):
                for scene_id in self.scenes:
                    if imgui.Selectable(scene_id, scene_id == self.selected):
                        self.selected = scene_id
                imgui.EndCombo()
            imgui.BeginDisabled(not self.scenes)
            if imgui.Button("Preview"):
                self.load_selected()
            imgui.EndDisabled()
            imgui.SameLine()
            if imgui.Button("Reload list"):
                self.read_catalog()
            imgui.EndDisabled()
            imgui.Separator()
            imgui.TextWrapped(self.status)
            imgui.TextWrapped("Local assets are reused. Missing assets are downloaded automatically.")
            if self.busy:
                imgui.TextWrapped(f"Elapsed: {time.monotonic() - self.started:.0f}s.")
        finally:
            imgui.End()


def launch_preview(gfx: str = "opengl_software") -> None:
    config = viewer_protocol.ViewerConfig(title="ModalityBench - Scene Preview", gfx=gfx)
    launch_native.run(config, plugins=[
        app.App(), *app.SIM_PLUGINS,
        ScenePreview(), PreviewSimulation(),
    ])
