"""Open the mujoco viewer for a single revolve2 terrain.

Usage:
    python _view_terrain.py flat
    python _view_terrain.py rugged
    python _view_terrain.py crater
    python _view_terrain.py bowl

Adds a small static cube at origin so you have a sense of scale (1 m side).
"""
from __future__ import annotations

import argparse
import sys

from pyrr import Vector3

from revolve2.standards.terrains import rugged_heightmap, bowl_heightmap
from revolve2.modular_robot_simulation import Terrain
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryHeightmap, GeometryPlane
from revolve2.simulation.scene.geometry.textures import (
    MapType, Texture, TextureReference,
)
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.simulator import BatchParameters


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


def crater_terrain():
    NE = (200, 200)
    rugged = rugged_heightmap(size=(20.0, 20.0), num_edges=NE, density=1.5)
    bowl = bowl_heightmap(num_edges=NE)
    ruggedness, curviness = 0.05, 0.5
    heights = (ruggedness * rugged + curviness * bowl) / (ruggedness + curviness)
    return Terrain(static_geometry=[GeometryHeightmap(
        pose=Pose(), mass=0.0,
        size=Vector3([20.0, 20.0, ruggedness + curviness]),
        base_thickness=0.1 + ruggedness, heights=heights,
    )], friction=1.0)


def bowl_terrain():
    NE = (200, 200)
    heights = bowl_heightmap(num_edges=NE)
    return Terrain(static_geometry=[GeometryHeightmap(
        pose=Pose(), mass=0.0,
        size=Vector3([20.0, 20.0, 1.0]),
        base_thickness=0.2, heights=heights,
    )], friction=1.0)


TERRAINS = {
    "flat": flat_terrain,
    "rugged": rugged_terrain,
    "crater": crater_terrain,
    "bowl": bowl_terrain,
}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("terrain", choices=list(TERRAINS.keys()))
    args = p.parse_args()

    from revolve2.modular_robot_simulation import ModularRobotScene
    from revolve2.modular_robot import ModularRobot
    from revolve2.modular_robot.body.base import ActiveHinge
    from revolve2.modular_robot.brain.cpg import (
        BrainCpgNetworkStatic,
        active_hinges_to_cpg_network_structure_neighbor,
    )
    from revolve2.standards import modular_robots_v1
    import numpy as np

    body = modular_robots_v1.get("spider")
    hinges = body.find_modules_of_type(ActiveHinge)
    cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=np.zeros(cpg.num_connections),
        cpg_network_structure=cpg,
        initial_state_uniform=0.0,
        output_mapping=mp,
    )
    robot = ModularRobot(body=body, brain=brain)

    terrain = TERRAINS[args.terrain]()
    scene = ModularRobotScene(terrain=terrain)
    spawn_z = 0.3 if args.terrain in ("rugged", "crater", "bowl") else 0.0
    scene.add_robot(robot, pose=Pose(position=Vector3([0.0, 0.0, spawn_z])))

    sim = LocalSimulator(headless=False, num_simulators=1)
    bp = BatchParameters(
        simulation_time=300.0, sampling_frequency=5,
        simulation_timestep=0.004, control_frequency=20,
    )
    print(f"Opening viewer for terrain: {args.terrain}")
    print("Spider robot at origin (limp) for scale.")

    from revolve2.modular_robot_simulation import simulate_scenes
    simulate_scenes(simulator=sim, batch_parameters=bp, scenes=scene)


if __name__ == "__main__":
    main()
