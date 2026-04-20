"""Visualize a robot in the MuJoCo viewer.

Usage:
    # Random brain (default):
    python visualize_robot.py spider

    # From saved evolved parameters:
    python visualize_robot.py spider --params results/comparison_v3/spider_ode_cpg_uncoupled_lambda0_dragging/best_params_run_1.npy --controller ode_cpg --coupling uncoupled
"""
import argparse
import math
import numpy as np
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkNeighborRandom,
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, Terrain
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.simulator import BatchParameters

from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_fully_connected,
)


def _build_cpg_structure(coupling, active_hinges, body):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(active_hinges)
    if coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(active_hinges, body)
    if coupling == "fully_connected":
        return active_hinges_to_cpg_network_structure_fully_connected(active_hinges)
    return active_hinges_to_cpg_network_structure_neighbor(active_hinges)


def build_robot(robot_name, controller=None, coupling=None, params_path=None, hz=0.2):
    """Build a robot with the specified brain."""
    body = modular_robots_v1.get(robot_name)

    if params_path is None or controller is None:
        # Random brain
        brain = BrainCpgNetworkNeighborRandom(body=body, rng=np.random.default_rng(42))
        return ModularRobot(body=body, brain=brain)

    params = np.load(params_path)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    cpg_structure, output_mapping = _build_cpg_structure(coupling, active_hinges, body)

    if controller == "kuramoto":
        # Params are in NATIVE scale (A, phi0, K, Delta) with length 2n + 2nc.
        from kuramoto_brain import (
            BrainKuramoto,
            kuramoto_structure_from_cpg_structure,
        )
        ks = kuramoto_structure_from_cpg_structure(cpg_structure)
        brain = BrainKuramoto.from_params(
            params=params,
            network_structure=ks,
            output_mapping=output_mapping,
            omega_hz=hz,
        )
    else:
        # ode_cpg (Hopf): params are just coupling weights, length = num_connections.
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=params,
            cpg_network_structure=cpg_structure,
            initial_state_uniform=math.sqrt(2) * 0.5,
            output_mapping=output_mapping,
        )

    return ModularRobot(body=body, brain=brain)


def main():
    parser = argparse.ArgumentParser(description="Visualize a robot in MuJoCo viewer")
    parser.add_argument("robot", type=str, choices=["spider", "big_spider", "gecko", "big_gecko", "hexapod", "xbot", "gecko_spider", "salamander", "ant", "snake", "pentapod", "turtle", "babya", "squarish", "longleg", "stingray", "mantis", "hydra", "queen", "zappa", "blokky", "insect", "babyb", "garrix", "linkin", "longleg", "park", "penguin", "stingray", "tinlicker", "turtle", "ww", "arachnid", "tripod"])
    parser.add_argument("--params", type=str, default=None, help="Path to saved .npy params")
    parser.add_argument("--controller", type=str, default=None,
                        choices=["ode_cpg", "kuramoto"])
    parser.add_argument("--coupling", type=str, default="uncoupled",
                        choices=["uncoupled", "neighbor", "blf", "fully_connected"])
    parser.add_argument("--hz", type=float, default=0.2,
                        help="Natural frequency (Kuramoto only). Default: 0.2")
    parser.add_argument("--time", type=float, default=30.0, help="Simulation time")
    args = parser.parse_args()

    robot = build_robot(args.robot, args.controller, args.coupling, args.params, hz=args.hz)

    terrain = Terrain(
        static_geometry=[
            GeometryPlane(
                pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
                texture=Texture(
                    base_color=Color(200, 200, 200, 255),
                    primary_color=Color(220, 220, 220, 255),
                    secondary_color=Color(80, 80, 80, 255),
                    map_type=MapType.MAP2D,
                    reference=TextureReference(builtin="checker"),
                    repeat=(50, 50),
                ),
            )
        ],
        friction=1.0,
    )
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot, pose=Pose())

    simulator = LocalSimulator(headless=False, num_simulators=1)
    batch_params = BatchParameters(
        simulation_time=args.time,
        sampling_frequency=5,
        simulation_timestep=0.004,
        control_frequency=20,
    )

    label = f"{args.robot.upper()}"
    if args.controller:
        label += f" | {args.controller} | {args.coupling}"
    if args.params:
        label += f" | {args.params}"
    print(f"Opening viewer: {label}")

    simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)


if __name__ == "__main__":
    main()
