"""Render an evolved controller to mp4 with a tracking camera.

Usage:
    python _render_evolved_video.py <npy_path> <out_mp4>
        [--robot ROBOT] [--sim-time T] [--fps F]
        [--width W] [--height H]
        [--distance D] [--cam-height H] [--elev E] [--azim A]
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import tempfile

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import (
    MapType, Texture, TextureReference,
)
from revolve2.simulation.scene.vector2 import Vector2

from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    get_robot_core_body_id,
)


def flat_terrain():
    return Terrain(static_geometry=[GeometryPlane(
        pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
        texture=Texture(
            base_color=Color(200, 200, 200, 255),
            primary_color=Color(220, 220, 220, 255),
            secondary_color=Color(80, 80, 80, 255),
            map_type=MapType.MAP2D,
            reference=TextureReference(builtin="checker"),
            repeat=(50, 50),
        ),
    )], friction=1.0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("npy")
    p.add_argument("out_mp4")
    p.add_argument("--robot", default="spider")
    p.add_argument("--sim-time", type=float, default=10.0)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--width", type=int, default=960)
    p.add_argument("--height", type=int, default=540)
    p.add_argument("--distance", type=float, default=2.5)
    p.add_argument("--cam-height", type=float, default=0.3)
    p.add_argument("--elev", type=float, default=-20.0)
    p.add_argument("--azim", type=float, default=135.0)
    args = p.parse_args()

    body = modular_robots_v1.get(args.robot)
    hinges = body.find_modules_of_type(ActiveHinge)
    cpg, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)
    params = np.load(args.npy)
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=list(params), cpg_network_structure=cpg,
        initial_state_uniform=0.5, output_mapping=mapping,
    )
    robot = ModularRobot(body=body, brain=brain)

    scene = ModularRobotScene(terrain=flat_terrain())
    scene.add_robot(robot)
    sim_scene, _ = scene.to_simulation_scene()

    batch = make_standard_batch_parameters()
    batch.simulation_time = args.sim_time
    model, mj_mapping = scene_to_model(
        sim_scene, simulation_timestep=batch.simulation_timestep,
        cast_shadows=True, fast_sim=False,
    )
    data = mujoco.MjData(model)
    core = get_robot_core_body_id(model)
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
    cstep = 1.0 / batch.control_frequency
    last_ctrl = 0.0
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, width=args.width, height=args.height)
    cam = mujoco.MjvCamera()
    cam.distance = args.distance
    cam.elevation = args.elev
    cam.azimuth = args.azim

    frame_dt = 1.0 / args.fps
    next_frame_at = 0.0
    n_frames = 0

    print(f"rendering {args.sim_time}s × {args.fps}fps = {int(args.sim_time*args.fps)} frames at {args.width}x{args.height}")

    tmpdir = tempfile.mkdtemp(prefix="render_")
    try:
        while data.time < args.sim_time:
            if data.time >= last_ctrl + cstep:
                last_ctrl = math.floor(data.time / cstep) * cstep
                ss = SimulationStateImpl(
                    data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={},
                )
                sim_scene.handler.handle(ss, ctrl, cstep)
            mujoco.mj_step(model, data)
            if data.time >= next_frame_at:
                cpos = data.xpos[core].copy()
                cam.lookat[:] = [cpos[0], cpos[1], cpos[2] + args.cam_height * 0.5]
                renderer.update_scene(data, camera=cam)
                pixels = renderer.render()
                # write frame as PNG via ffmpeg-friendly path
                from PIL import Image
                Image.fromarray(pixels).save(os.path.join(tmpdir, f"frame_{n_frames:05d}.png"))
                n_frames += 1
                next_frame_at += frame_dt
                if n_frames % 30 == 0:
                    print(f"  {n_frames} frames  t={data.time:.2f}s")

        print(f"encoding {n_frames} frames -> {args.out_mp4}")
        cmd = [
            "ffmpeg", "-y", "-framerate", str(args.fps),
            "-i", os.path.join(tmpdir, "frame_%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-crf", "18",
            args.out_mp4,
        ]
        subprocess.run(cmd, check=True)
        print(f"wrote {args.out_mp4}")
    finally:
        for f in os.listdir(tmpdir):
            os.unlink(os.path.join(tmpdir, f))
        os.rmdir(tmpdir)


if __name__ == "__main__":
    main()
