"""Open the mujoco viewer with an evolved revolve2 BLF controller.

Usage:
    python _view_evolved.py <npy_path> [--terrain flat|rugged|rugged_hard] [--robot spider]
"""
from __future__ import annotations

import argparse
import os
import sys
import numpy as np
from pyrr import Vector3

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
from revolve2.standards import modular_robots_v1
from revolve2.standards.terrains import rugged_heightmap
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryHeightmap, GeometryPlane
from revolve2.simulation.scene.geometry.textures import (
    MapType, Texture, TextureReference,
)
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.simulator import BatchParameters

from contact_detection import active_hinges_to_cpg_network_structure_blf


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


def rugged_terrain():
    heights = rugged_heightmap(size=(20.0, 20.0), num_edges=(500, 500), density=1.5)
    return Terrain(static_geometry=[GeometryHeightmap(
        pose=Pose(), mass=0.0,
        size=Vector3([20.0, 20.0, 0.05]),
        base_thickness=0.2, heights=heights,
    )], friction=1.0)


def rugged_hard_terrain():
    heights = rugged_heightmap(size=(20.0, 20.0), num_edges=(500, 500), density=2.5)
    return Terrain(static_geometry=[GeometryHeightmap(
        pose=Pose(), mass=0.0,
        size=Vector3([20.0, 20.0, 0.10]),
        base_thickness=0.2, heights=heights,
    )], friction=1.0)


TERRAIN_BUILDERS = {
    "flat": flat_terrain,
    "rugged": rugged_terrain,
    "rugged_hard": rugged_hard_terrain,
}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("npy", help="path to best_params_run_*.npy")
    p.add_argument("--robot", default="spider")
    p.add_argument("--terrain", default="flat", choices=list(TERRAIN_BUILDERS.keys()))
    p.add_argument("--sim-time", type=float, default=60.0)
    args = p.parse_args()

    body = modular_robots_v1.get(args.robot)
    hinges = body.find_modules_of_type(ActiveHinge)
    cpg, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)

    params = np.load(args.npy)
    print(f"params shape: {params.shape}, expected {cpg.num_connections}")

    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=list(params), cpg_network_structure=cpg,
        initial_state_uniform=0.5, output_mapping=mapping,
    )
    robot = ModularRobot(body=body, brain=brain)

    terrain = TERRAIN_BUILDERS[args.terrain]()
    scene = ModularRobotScene(terrain=terrain)
    spawn_z = 0.3 if args.terrain in ("rugged", "rugged_hard") else 0.0
    scene.add_robot(robot, pose=Pose(position=Vector3([0.0, 0.0, spawn_z])))

    sim = LocalSimulator(headless=False, num_simulators=1)
    bp = BatchParameters(
        simulation_time=args.sim_time, sampling_frequency=5,
        simulation_timestep=0.004, control_frequency=20,
    )
    print(f"Opening viewer: robot={args.robot}, terrain={args.terrain}")
    print(f"Loaded from: {args.npy}")
    simulate_scenes(simulator=sim, batch_parameters=bp, scenes=scene)


if __name__ == "__main__":
    main()
