"""Visualize spider on rugged terrain (Perlin noise heightmap)."""
import numpy as np
from pyrr import Vector3
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1
from revolve2.standards.terrains import rugged_heightmap
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, Terrain
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.scene import Pose
from revolve2.simulation.scene.geometry import GeometryHeightmap
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.simulator import BatchParameters


body = modular_robots_v1.get("spider")
hinges = body.find_modules_of_type(ActiveHinge)
cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
brain = BrainCpgNetworkStatic.uniform_from_params(
    params=np.zeros(cpg.num_connections),
    cpg_network_structure=cpg,
    initial_state_uniform=0.0, output_mapping=mp,
)
robot = ModularRobot(body=body, brain=brain)

# Match the slurm run config exactly
size = (20.0, 20.0)
num_edges = (500, 500)
heights = rugged_heightmap(size=size, num_edges=num_edges, density=1.5)

terrain = Terrain(static_geometry=[GeometryHeightmap(
    pose=Pose(),
    mass=0.0,
    size=Vector3([size[0], size[1], 0.05]),  # max bump height 5 cm
    base_thickness=0.2,
    heights=heights,
)])

scene = ModularRobotScene(terrain=terrain)
# Lift robot a bit so it doesn't spawn inside the ground
scene.add_robot(robot, pose=Pose(position=Vector3([0.0, 0.0, 0.3])))

simulator = LocalSimulator(headless=False, num_simulators=1)
batch = BatchParameters(simulation_time=30.0, sampling_frequency=5,
                        simulation_timestep=0.004, control_frequency=20)
print("Opening viewer: SPIDER on rugged terrain (max height 0.3 m)")
simulate_scenes(simulator=simulator, batch_parameters=batch, scenes=scene)
