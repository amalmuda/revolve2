"""Viz a controller on the rugged terrain (matching the slurm-evolved one)."""
import os, sys, argparse
import numpy as np
from pyrr import Vector3
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1
from revolve2.standards.terrains import rugged_heightmap
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, Terrain
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.scene import Pose
from revolve2.simulation.scene.geometry import GeometryHeightmap
from revolve2.simulation.simulator import BatchParameters

from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
)
from bonardi_brain import BrainBonardi, bonardi_structure_from_cpg_structure


parser = argparse.ArgumentParser()
parser.add_argument("--robot", default="spider")
parser.add_argument("--coupling", choices=["uncoupled","neighbor","blf"], required=True)
parser.add_argument("--evolve-phi0", action="store_true")
parser.add_argument("--params", required=True)
parser.add_argument("--time", type=float, default=30.0)
args = parser.parse_args()

body = modular_robots_v1.get(args.robot)
hinges = body.find_modules_of_type(ActiveHinge)
if args.coupling == "uncoupled":
    cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
elif args.coupling == "neighbor":
    cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
else:
    cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
ks = bonardi_structure_from_cpg_structure(cpg, evolve_phi0=args.evolve_phi0)
params = np.load(args.params)
brain = BrainBonardi.from_params(
    params=params, network_structure=ks, output_mapping=mp, nu_hz=0.5, w=1.0,
)
robot = ModularRobot(body=body, brain=brain)

heights = rugged_heightmap(size=(20.0, 20.0), num_edges=(500, 500), density=1.5)
terrain = Terrain(static_geometry=[GeometryHeightmap(
    pose=Pose(), mass=0.0,
    size=Vector3([20.0, 20.0, 0.05]),
    base_thickness=0.2,
    heights=heights,
)], friction=1.0)

scene = ModularRobotScene(terrain=terrain)
scene.add_robot(robot, pose=Pose(position=Vector3([0.0, 0.0, 0.3])))

simulator = LocalSimulator(headless=False, num_simulators=1)
batch = BatchParameters(simulation_time=args.time, sampling_frequency=5,
                        simulation_timestep=0.004, control_frequency=20)
print(f"Opening viewer: {args.robot} | {args.coupling} on RUGGED")
simulate_scenes(simulator=simulator, batch_parameters=batch, scenes=scene)
