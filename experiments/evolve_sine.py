"""
Simple CMA-ES evolution for sine brain parameters.

This is a minimal example showing how to evolve sine controller parameters.
Much simpler than the full CPG evolution pipeline.

Usage:
    python evolve_sine.py                     # Default settings
    python evolve_sine.py --robot gecko       # Different robot
    python evolve_sine.py --generations 50    # More generations
    python evolve_sine.py --visualize         # Show best result
"""

import argparse
import math
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass

import cma
import numpy as np

from brain_sine import BrainSine, get_num_sine_params

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.standards.terrains import flat


@dataclass
class SimulationResult:
    """Result from a single simulation."""
    distance: float
    final_x: float
    final_y: float


def evaluate_single(
    params: np.ndarray,
    robot_name: str,
    frequency: float,
    simulation_time: int,
) -> SimulationResult:
    """
    Evaluate a single set of sine parameters.

    :param params: Parameter array [amp_0, phase_0, amp_1, phase_1, ...]
    :param robot_name: Name of the robot.
    :param frequency: Oscillation frequency (Hz).
    :param simulation_time: Simulation duration (seconds).
    :returns: Simulation result with distance.
    """
    try:
        body = modular_robots_v1.get(robot_name)

        # Create brain from parameters
        brain = BrainSine.from_parameters(body, params, frequency=frequency)
        robot = ModularRobot(body=body, brain=brain)

        # Setup simulation
        scene = ModularRobotScene(terrain=flat())
        scene.add_robot(robot)

        simulator = LocalSimulator(headless=True, num_simulators=1)
        batch_params = make_standard_batch_parameters()
        batch_params.simulation_time = simulation_time

        # Run simulation
        states = simulate_scenes(
            simulator=simulator,
            batch_parameters=batch_params,
            scenes=scene,
        )

        # Extract final pose (single scene returns flat list of states)
        final_state = states[-1]
        pose = final_state.get_modular_robot_simulation_state(robot).get_pose()

        distance = math.sqrt(pose.position.x**2 + pose.position.y**2)

        return SimulationResult(
            distance=distance,
            final_x=pose.position.x,
            final_y=pose.position.y,
        )

    except Exception as e:
        print(f"Evaluation error: {e}")
        return SimulationResult(distance=0.0, final_x=0.0, final_y=0.0)


def _eval_wrapper(args):
    """Wrapper for parallel evaluation."""
    idx, params, robot_name, frequency, sim_time = args
    result = evaluate_single(params, robot_name, frequency, sim_time)
    return idx, result


def run_evolution(
    robot_name: str = "spider",
    frequency: float = 1.0,
    simulation_time: int = 10,
    num_generations: int = 50,
    num_workers: int = 4,
    seed: int = None,
    visualize_best: bool = False,
):
    """
    Run CMA-ES evolution to optimize sine brain parameters.

    :param robot_name: Name of the robot to evolve.
    :param frequency: Oscillation frequency (Hz).
    :param simulation_time: Simulation time per evaluation (seconds).
    :param num_generations: Number of generations.
    :param num_workers: Number of parallel workers.
    :param seed: Random seed.
    :param visualize_best: Show best robot after evolution.
    """
    if seed is None:
        seed = int(time.time()) % 2**32

    print("\n" + "=" * 60)
    print("SINE BRAIN EVOLUTION")
    print("=" * 60)

    # Get robot info
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(active_hinges)
    n_params = get_num_sine_params(body)

    print(f"\nConfiguration:")
    print(f"  Robot:          {robot_name}")
    print(f"  Hinges:         {n_hinges}")
    print(f"  Parameters:     {n_params} (amp + phase per hinge)")
    print(f"  Frequency:      {frequency} Hz (fixed)")
    print(f"  Sim time:       {simulation_time}s")
    print(f"  Generations:    {num_generations}")
    print(f"  Workers:        {num_workers}")
    print(f"  Seed:           {seed}")

    # Parameter bounds
    # Amplitudes: [0, 1] (will be scaled to joint range)
    # Phases: [-π, π]
    # We use bounds implicitly through initialization and let CMA-ES explore

    # Initialize CMA-ES
    # Initial mean: small amplitudes, zero phases
    initial_mean = []
    for _ in range(n_hinges):
        initial_mean.append(0.5)  # amplitude
        initial_mean.append(0.0)  # phase

    initial_std = 0.3

    options = cma.CMAOptions()
    options.set("seed", seed)
    options.set("bounds", [
        [0.0, -math.pi] * n_hinges,  # Lower bounds
        [1.0, math.pi] * n_hinges,   # Upper bounds
    ])

    opt = cma.CMAEvolutionStrategy(initial_mean, initial_std, options)

    # Track best
    best_fitness = float("-inf")
    best_params = None
    best_result = None

    print("\nStarting evolution...")
    start_time = time.time()

    while opt.countiter < num_generations:
        gen_start = time.time()

        # Get population from CMA-ES
        solutions = opt.ask()

        # Evaluate population
        if num_workers <= 1:
            # Sequential
            results = []
            for params in solutions:
                result = evaluate_single(
                    np.array(params), robot_name, frequency, simulation_time
                )
                results.append(result)
        else:
            # Parallel
            results = [None] * len(solutions)
            args_list = [
                (i, np.array(params), robot_name, frequency, simulation_time)
                for i, params in enumerate(solutions)
            ]

            with ProcessPoolExecutor(max_workers=num_workers) as executor:
                futures = [executor.submit(_eval_wrapper, args) for args in args_list]
                for future in as_completed(futures):
                    idx, result = future.result()
                    results[idx] = result

        # Extract fitness (distance)
        fitnesses = [r.distance for r in results]

        # Tell CMA-ES (negate because it minimizes)
        opt.tell(solutions, [-f for f in fitnesses])

        # Track best
        gen_best_idx = np.argmax(fitnesses)
        if fitnesses[gen_best_idx] > best_fitness:
            best_fitness = fitnesses[gen_best_idx]
            best_params = np.array(solutions[gen_best_idx])
            best_result = results[gen_best_idx]

        # Log progress
        gen_time = time.time() - gen_start
        mean_fit = np.mean(fitnesses)
        max_fit = np.max(fitnesses)

        if opt.countiter % 10 == 0 or opt.countiter == num_generations:
            print(f"Gen {opt.countiter:3d}: mean={mean_fit:.4f}, max={max_fit:.4f}, "
                  f"best_ever={best_fitness:.4f} ({gen_time:.1f}s)")

    # Final summary
    total_time = time.time() - start_time
    print("\n" + "=" * 60)
    print("EVOLUTION COMPLETE")
    print("=" * 60)
    print(f"  Total time:     {total_time:.1f}s ({total_time/60:.1f} min)")
    print(f"  Best distance:  {best_fitness:.4f} m")
    print(f"  Best final pos: ({best_result.final_x:.3f}, {best_result.final_y:.3f})")

    print(f"\nBest parameters (amp, phase per hinge):")
    for i in range(n_hinges):
        amp = best_params[2*i]
        phase = best_params[2*i + 1]
        print(f"  Hinge {i}: amp={amp:.3f} rad ({math.degrees(amp):.1f}°), "
              f"phase={phase:.3f} rad ({math.degrees(phase):.1f}°)")

    # Visualize best
    if visualize_best and best_params is not None:
        print("\nVisualizing best robot...")
        visualize_robot(robot_name, best_params, frequency, simulation_time)

    return best_params, best_fitness


def visualize_robot(
    robot_name: str,
    params: np.ndarray,
    frequency: float,
    simulation_time: int,
):
    """Visualize a robot with given sine parameters."""
    body = modular_robots_v1.get(robot_name)
    brain = BrainSine.from_parameters(body, params, frequency=frequency)
    robot = ModularRobot(body=body, brain=brain)

    scene = ModularRobotScene(terrain=flat())
    scene.add_robot(robot)

    simulator = LocalSimulator(headless=False, num_simulators=1)
    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = simulation_time

    simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_params,
        scenes=scene,
    )


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Evolve sine brain parameters for modular robots"
    )
    parser.add_argument("--robot", type=str, default="spider",
                        help="Robot name (default: spider)")
    parser.add_argument("--frequency", type=float, default=1.0,
                        help="Oscillation frequency in Hz (default: 1.0)")
    parser.add_argument("--sim-time", type=int, default=10,
                        help="Simulation time in seconds (default: 10)")
    parser.add_argument("--generations", type=int, default=50,
                        help="Number of generations (default: 50)")
    parser.add_argument("--workers", type=int, default=4,
                        help="Number of parallel workers (default: 4)")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed")
    parser.add_argument("--visualize", action="store_true",
                        help="Visualize best robot after evolution")

    args = parser.parse_args()

    run_evolution(
        robot_name=args.robot,
        frequency=args.frequency,
        simulation_time=args.sim_time,
        num_generations=args.generations,
        num_workers=args.workers,
        seed=args.seed,
        visualize_best=args.visualize,
    )


if __name__ == "__main__":
    main()
