"""Preview an evolved controller in the mujoco viewer with a tracking camera.

Usage:
    python _view_evolved_tracking.py <npy_path> [--robot ROBOT] [--sim-time T]
                                                [--distance D] [--height H] [--elev E]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
from pyrr import Vector3

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
    p.add_argument("--robot", default="spider")
    p.add_argument("--sim-time", type=float, default=30.0)
    p.add_argument("--distance", type=float, default=2.5,
                   help="camera horizontal distance from robot (m)")
    p.add_argument("--height", type=float, default=1.0,
                   help="camera height above robot (m)")
    p.add_argument("--elev", type=float, default=-20.0,
                   help="camera elevation angle (degrees, negative looks down)")
    p.add_argument("--azim", type=float, default=135.0,
                   help="camera azimuth (degrees)")
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
        cast_shadows=False, fast_sim=True,
    )
    data = mujoco.MjData(model)
    core = get_robot_core_body_id(model)
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
    cstep = 1.0 / batch.control_frequency
    last_ctrl = 0.0
    mujoco.mj_forward(model, data)

    print(f"opening viewer: {args.robot}  npy={args.npy}")
    print(f"camera: dist={args.distance}m height={args.height}m elev={args.elev}° azim={args.azim}°")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = args.distance
        viewer.cam.elevation = args.elev
        viewer.cam.azimuth = args.azim
        wall_start = time.time()
        sim_start = data.time
        while viewer.is_running() and data.time < args.sim_time:
            step_start = time.time()
            if data.time >= last_ctrl + cstep:
                last_ctrl = math.floor(data.time / cstep) * cstep
                ss = SimulationStateImpl(
                    data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={},
                )
                sim_scene.handler.handle(ss, ctrl, cstep)
            mujoco.mj_step(model, data)
            # camera tracks robot core
            cpos = data.xpos[core].copy()
            viewer.cam.lookat[:] = [cpos[0], cpos[1], cpos[2] + args.height * 0.3]
            viewer.sync()
            # pace to real-time
            elapsed = time.time() - step_start
            sleep = batch.simulation_timestep - elapsed
            if sleep > 0:
                time.sleep(sleep)


if __name__ == "__main__":
    main()
