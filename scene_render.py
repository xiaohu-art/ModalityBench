"""Render a scene with the MolmoSpaces MuJoCo Filament build."""

from pathlib import Path


def render_filament(model, data, output: Path, width: int, height: int, lookat, distance, azimuth, elevation):
    import mujoco
    import numpy as np
    from PIL import Image

    if getattr(mujoco, "mjRENDERER", "") != "filament":
        raise RuntimeError("Filament rendering requires the MolmoSpaces Filament MuJoCo wheel. See README.md.")
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height
    camera = mujoco.MjvCamera()
    camera.lookat[:] = lookat
    camera.distance = distance
    camera.azimuth = azimuth
    camera.elevation = elevation
    option = mujoco.MjvOption()
    option.geomgroup[3:6] = False
    option.sitegroup[:] = False
    scene = mujoco.MjvScene(model, maxgeom=max(4000, model.ngeom + 100))
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100.value)
    try:
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
        rect = mujoco.MjrRect(0, 0, width, height)
        pixels = np.empty((height, width, 3), dtype=np.uint8)
        for _ in range(8):
            mujoco.mjv_updateScene(model, data, option, None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(rect, scene, context)
        mujoco.mjr_readPixels(pixels, None, rect, context)
        output.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(pixels).save(output)
    finally:
        context.free()
    print(f"Filament {mujoco.__version__}: saved {width}x{height} image to {output}")
