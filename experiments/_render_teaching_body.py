"""Render the teaching body in MuJoCo for Chapter 3 figures.

Uses the same aesthetic as Figure 3.2: gradient sky, high-contrast
checker floor with cross marks, warm directional sun + cool fill. Top-down
angle so all modules are visible.
"""
import math
import os

import numpy as np
import mujoco
from PIL import Image

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2

from teaching_body import teaching_body_v1


OUT_ANGLE = "/home/abdullah/masteroppgave/master_thesis/figures/teaching_body.png"
OUT_TOP = "/home/abdullah/masteroppgave/master_thesis/figures/teaching_body_top.png"
W, H = 640, 480  # MuJoCo's default offscreen framebuffer max


def build_model():
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
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def render_with_camera(model, data, lookat, distance, azimuth, elevation, out_path):
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

    Image.fromarray(img).save(out_path)
    print(f"Saved: {out_path}  ({W}x{H})")


def main():
    model, data = build_model()

    # Find the body's bounding box center for a well-centered lookat
    # The robot's geoms are in data.geom_xpos after mj_forward
    mins = np.full(3, np.inf)
    maxs = np.full(3, -np.inf)
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        if name and "mbs" in name:
            p = data.geom_xpos[i]
            mins = np.minimum(mins, p)
            maxs = np.maximum(maxs, p)
    center = (mins + maxs) / 2.0
    extent = float(np.max(maxs - mins))
    print(f"robot bbox center={center}  extent={extent:.3f}")

    # Angled (perspective) view
    render_with_camera(
        model, data,
        lookat=tuple(center),
        distance=max(0.55, 2.0 * extent),
        azimuth=-55.0, elevation=-30.0,
        out_path=OUT_ANGLE,
    )

    # Top-down (flat) view
    render_with_camera(
        model, data,
        lookat=tuple(center),
        distance=max(0.55, 2.0 * extent),
        azimuth=0.0, elevation=-89.9,
        out_path=OUT_TOP,
    )


if __name__ == "__main__":
    main()
