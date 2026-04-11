"""
Visualize a saved evolved body from morphological evolution.

Usage:
    python morpho_visualize.py results/morpho_pilot/blf_seed42/gen_005/best_body.pkl --coupling blf
    python morpho_visualize.py results/morpho_pilot/blf_seed42/best_ever_body.pkl --coupling blf
"""
import argparse
import math
import os
import pickle
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    BrainCpgNetworkNeighborRandom,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, Terrain
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.simulator import BatchParameters

from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
)
from blf import analyze_robot


def get_cpg_structure(coupling, body, hinges):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    if coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)
    return active_hinges_to_cpg_network_structure_neighbor(hinges)


def main():
    parser = argparse.ArgumentParser(description="Visualize evolved body")
    parser.add_argument("body_pkl", type=str, help="Path to best_body.pkl")
    parser.add_argument("--coupling", type=str, default="blf",
                        choices=["uncoupled", "neighbor", "blf"])
    parser.add_argument("--params", type=str, default=None,
                        help="Path to best_params.npy (if not given, uses random brain)")
    parser.add_argument("--time", type=float, default=30.0)
    parser.add_argument("--flat", action="store_true", help="Zero brain (flat, no movement)")
    parser.add_argument("--blf-info", action="store_true", help="Print BLF classification")
    args = parser.parse_args()

    # Load body
    with open(args.body_pkl, "rb") as f:
        body = pickle.load(f)

    hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(hinges)
    print("Body: %d hinges" % n_hinges)

    # BLF info
    if args.blf_info:
        result = analyze_robot(body)
        print("\nBLF classification:")
        for node in result.nodes:
            if node.module in hinges:
                h_idx = hinges.index(node.module)
                jt = node.joint_type.name if node.joint_type else "NONE"
                print("  Hinge %d: %s (limb %d)" % (h_idx, jt, node.limb_id))

    # Auto-detect params if not specified
    if args.params is None:
        params_dir = os.path.dirname(args.body_pkl)
        for candidate in ["best_params.npy", "best_ever_params.npy"]:
            auto_params = os.path.join(params_dir, candidate)
            if os.path.exists(auto_params):
                args.params = auto_params
                print("Auto-detected params: %s" % auto_params)
                break

    # Build brain
    if args.flat:
        cpg_struct, output_mapping = get_cpg_structure(args.coupling, body, hinges)
        params = np.zeros(cpg_struct.num_connections)
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=params, cpg_network_structure=cpg_struct,
            initial_state_uniform=0.0, output_mapping=output_mapping,
        )
    elif args.params:
        params = np.load(args.params)
        cpg_struct, output_mapping = get_cpg_structure(args.coupling, body, hinges)
        print("Params: %d values, CPG expects %d" % (len(params), cpg_struct.num_connections))
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=params, cpg_network_structure=cpg_struct,
            initial_state_uniform=math.sqrt(2) * 0.5, output_mapping=output_mapping,
        )
    else:
        print("No params — using random brain")
        brain = BrainCpgNetworkNeighborRandom(body=body, rng=np.random.default_rng(42))

    robot = ModularRobot(body=body, brain=brain)

    terrain = Terrain(
        static_geometry=[GeometryPlane(pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
            texture=Texture(base_color=Color(200,200,200,255), primary_color=Color(220,220,220,255),
                secondary_color=Color(80,80,80,255), map_type=MapType.MAP2D,
                reference=TextureReference(builtin="checker"), repeat=(50, 50)))],
        friction=1.0)
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot, pose=Pose())

    simulator = LocalSimulator(headless=False, num_simulators=1)
    bp = BatchParameters(simulation_time=args.time, sampling_frequency=5,
                         simulation_timestep=0.004, control_frequency=20)

    print("Opening viewer...")
    simulate_scenes(simulator=simulator, batch_parameters=bp, scenes=scene)


if __name__ == "__main__":
    main()
