"""
Visualize coupled sine robot with specified coupling strength.

Usage:
    python visualize_coupling.py --coupling 0    # No coupling
    python visualize_coupling.py --coupling 1    # Full coupling
    python visualize_coupling.py --coupling 1 --load fixed_coupling_1_results_seed42.npz
"""

import argparse
import math
import numpy as np

from brain_sine_coupled import BrainCoupledSine, CoupledSineParameters
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.standards.terrains import flat


def visualize_robot(coupling_strength: float, params: np.ndarray = None, frequency: float = 2.0):
    """
    Visualize robot with given coupling strength.

    If params is None, uses reasonable default parameters.
    """
    body = modular_robots_v1.get("spider")
    active_hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(active_hinges)

    if params is None:
        # Default parameters - alternating pattern with some asymmetry
        # These produce reasonable locomotion
        params = np.array([
            0.8, 0.0, 0.1,    # Hinge 0: amp, phase, offset
            0.7, 1.57, -0.1,  # Hinge 1
            0.8, 0.0, 0.1,    # Hinge 2
            0.7, 1.57, -0.1,  # Hinge 3
            0.8, 3.14, 0.1,   # Hinge 4
            0.7, -1.57, -0.1, # Hinge 5
            0.8, 3.14, 0.1,   # Hinge 6
            0.7, -1.57, -0.1, # Hinge 7
        ])

    # Create parameters list
    parameters = []
    for i in range(n_hinges):
        amp = float(params[3 * i])
        phase = float(params[3 * i + 1])
        offset = float(params[3 * i + 2])
        parameters.append(CoupledSineParameters(
            amplitude=amp,
            frequency=frequency,
            phase_offset=phase,
            position_offset=offset,
        ))

    brain = BrainCoupledSine(
        active_hinges=active_hinges,
        parameters=parameters,
        coupling_strength=coupling_strength,
    )
    robot = ModularRobot(body=body, brain=brain)

    # Create scene
    scene = ModularRobotScene(terrain=flat())
    scene.add_robot(robot)

    # Run simulation with visualization
    simulator = LocalSimulator(headless=False, num_simulators=1)
    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = 10

    print(f"\n{'='*60}")
    print(f"VISUALIZING: Coupling = {coupling_strength}")
    print(f"{'='*60}")
    print(f"Frequency: {frequency:.2f} Hz")
    print(f"\nJoint parameters:")
    for i in range(n_hinges):
        amp = params[3 * i]
        phase = params[3 * i + 1]
        offset = params[3 * i + 2]
        print(f"  Hinge {i}: amp={amp:.3f}, phase={phase:+.3f}, offset={offset:+.3f}")
    print(f"\nStarting visualization...")

    states = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)

    # Get final distance
    final_state = states[-1]
    pose = final_state.get_modular_robot_simulation_state(robot).get_pose()
    distance = math.sqrt(pose.position.x**2 + pose.position.y**2)

    print(f"\nFinal distance: {distance:.3f} m")
    print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(description="Visualize coupled sine robot")
    parser.add_argument("--coupling", type=float, default=1.0, help="Coupling strength (0-1)")
    parser.add_argument("--load", type=str, default=None, help="Load params from .npz file")
    parser.add_argument("--freq", type=float, default=2.5, help="Frequency (Hz)")
    args = parser.parse_args()

    params = None
    frequency = args.freq

    if args.load:
        try:
            data = np.load(args.load)
            params = data["best_params"]
            # Extract frequency from last param
            frequency = float(params[-1])
            params = params[:-1]  # Remove frequency from params array
            print(f"Loaded parameters from {args.load}")
            print(f"  Best distance: {data['best_distance']:.3f} m")
            print(f"  Body contact: {data['best_contact_m1']*100:.1f}%")
        except Exception as e:
            print(f"Could not load {args.load}: {e}")
            print("Using default parameters instead")

    visualize_robot(args.coupling, params, frequency)


if __name__ == "__main__":
    main()
