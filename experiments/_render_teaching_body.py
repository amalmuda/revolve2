"""High-resolution MuJoCo render of the teaching body.

Produces two views saved to the thesis figures directory:
  - teaching_body.png      (angled perspective)
  - teaching_body_top.png  (top-down, horizontally mirrored)

Pipeline:
  1. Build a Revolve2 ModularRobotScene with the teaching body + checker floor.
  2. Call scene_to_model to compile to a MuJoCo MjModel.
  3. Save the compiled XML via mj_saveLastXML, inject:
       - sky-blue gradient skybox (no more black background)
       - <global offwidth/offheight> for high-res offscreen rendering
       - larger shadow map
  4. Reload the modified XML to a fresh MjModel and render with mujoco.Renderer.
  5. Top-down view: mirror horizontally via PIL for the requested orientation.
"""
import os
import tempfile

import numpy as np
import mujoco
from PIL import Image

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2

from teaching_body import teaching_body_v1


FIG_DIR = "/home/abdullah/masteroppgave/master_thesis/figures"
OUT_ANGLE_PNG = f"{FIG_DIR}/teaching_body.png"
OUT_ANGLE_PDF = f"{FIG_DIR}/teaching_body.pdf"
OUT_TOP_PNG = f"{FIG_DIR}/teaching_body_top.png"
OUT_TOP_PDF = f"{FIG_DIR}/teaching_body_top.pdf"
W, H = 4096, 4096  # 4K+ — offscreen buffer resized via <global offwidth/offheight>


def build_compiled_model():
    body = teaching_body_v1()
    rng = np.random.default_rng(0)
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    robot = ModularRobot(body=body, brain=brain)

    terrain = Terrain(static_geometry=[GeometryPlane(
        pose=Pose(), mass=0.0, size=Vector2([2.0, 2.0]),
        texture=Texture(
            base_color=Color(200, 200, 200, 255),
            primary_color=Color(200, 200, 200, 255),
            secondary_color=Color(160, 160, 160, 255),
            map_type=MapType.MAP2D,
            reference=TextureReference(builtin="checker"),
            repeat=(14, 14),
        ),
    )], friction=1.0)
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)
    sim_scene, _ = scene.to_simulation_scene()

    model, _ = scene_to_model(
        sim_scene, simulation_timestep=0.001,
        cast_shadows=True, fast_sim=False,
    )
    return model


def inject_sky_and_offscreen(base_model: mujoco.MjModel) -> mujoco.MjModel:
    """Dump last-compiled XML, inject skybox + high-res offscreen buffer, recompile."""
    with tempfile.NamedTemporaryFile(suffix=".xml", mode="w", delete=False) as f:
        tmp = f.name
    try:
        err = mujoco.mj_saveLastXML(tmp, base_model)
        if err:
            print(f"mj_saveLastXML warning: {err}")
        with open(tmp) as f:
            xml = f.read()
    finally:
        os.unlink(tmp)

    # Add skybox asset, offscreen buffer, shadow quality.
    injection = (
        '  <visual>\n'
        f'    <global offwidth="{W}" offheight="{H}"/>\n'
        '    <quality shadowsize="8192" numslices="64" numstacks="32"/>\n'
        '  </visual>\n'
        '  <asset>\n'
        '    <texture type="skybox" builtin="gradient" '
        'rgb1="0.55 0.62 0.72" rgb2="0.86 0.89 0.92" width="256" height="256"/>\n'
        '  </asset>\n'
    )
    # Insert just before the first <worldbody> tag.
    idx = xml.find("<worldbody")
    assert idx != -1, "no <worldbody> in saved XML"
    xml = xml[:idx] + injection + xml[idx:]

    return mujoco.MjModel.from_xml_string(xml)


def body_center_and_extent(model, data):
    mins = np.full(3, np.inf)
    maxs = np.full(3, -np.inf)
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        if name and "mbs" in name:
            p = data.geom_xpos[i]
            mins = np.minimum(mins, p)
            maxs = np.maximum(maxs, p)
    return (mins + maxs) / 2.0, float(np.max(maxs - mins))


def render(model, data, lookat, distance, azimuth, elevation,
           png_path, pdf_path, mirror=False):
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation

    r = mujoco.Renderer(model, height=H, width=W)
    try:
        r.update_scene(data, camera=cam)
        img = r.render()
    finally:
        if hasattr(r, "close"):
            r.close()

    pil = Image.fromarray(img)
    if mirror:
        pil = pil.transpose(Image.FLIP_LEFT_RIGHT)

    # PNG: lossless, compressed. Good for LaTeX (pdflatex / xelatex).
    pil.save(png_path, optimize=True, compress_level=9)
    # PDF: same raster image wrapped as a single-page PDF. LaTeX-native.
    pil.convert("RGB").save(pdf_path, "PDF", resolution=300.0)
    print(f"Saved: {png_path} and {pdf_path}  ({pil.size[0]}x{pil.size[1]})")


def main():
    base_model = build_compiled_model()
    model = inject_sky_and_offscreen(base_model)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    center, extent = body_center_and_extent(model, data)
    dist = max(0.55, 2.0 * extent)
    print(f"robot bbox center={center}  extent={extent:.3f}  cam distance={dist:.3f}")

    render(model, data,
           lookat=tuple(center), distance=dist,
           azimuth=-55.0, elevation=-30.0,
           png_path=OUT_ANGLE_PNG, pdf_path=OUT_ANGLE_PDF, mirror=False)

    render(model, data,
           lookat=tuple(center), distance=dist,
           azimuth=0.0, elevation=-89.9,
           png_path=OUT_TOP_PNG, pdf_path=OUT_TOP_PDF, mirror=True)


if __name__ == "__main__":
    main()
