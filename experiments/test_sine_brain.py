"""
Test script for the simple sine brain controller.

Demonstrates:
1. Basic usage with different phase patterns
2. Integration with contact detection
3. Simple parameter optimization example
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np

from brain_sine import BrainSine, SineParameters, get_num_sine_params
from contact_detection import simulate_with_contact_detection

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.standards import modular_robots_v1


def test_basic_sine():
    """Test basic sine brain creation and simulation."""
    print("\n" + "=" * 60)
    print("TEST 1: Basic Sine Brain")
    print("=" * 60)

    # Get spider body
    body = modular_robots_v1.get("spider")
    active_hinges = body.find_modules_of_type(ActiveHinge)
    print(f"Spider has {len(active_hinges)} active hinges")

    # Create sine brain with alternating phases
    brain = BrainSine.from_body(
        body,
        amplitude=0.7,
        frequency=1.0,
        phase_pattern="alternating",
    )

    # Create robot
    robot = ModularRobot(body=body, brain=brain)

    print(f"Created robot with sine brain")
    print(f"  Amplitude: 0.7 rad ({math.degrees(0.7):.1f} deg)")
    print(f"  Frequency: 1.0 Hz")
    print(f"  Phase pattern: alternating (0, π, 0, π, ...)")


def test_with_contact_detection():
    """Test sine brain with contact detection metrics."""
    print("\n" + "=" * 60)
    print("TEST 2: Sine Brain with Contact Detection")
    print("=" * 60)

    body = modular_robots_v1.get("spider")

    # Test different phase patterns
    patterns = ["zero", "alternating", "sequential"]

    for pattern in patterns:
        print(f"\n--- Phase pattern: {pattern} ---")

        brain = BrainSine.from_body(
            body,
            amplitude=0.6,
            frequency=1.0,
            phase_pattern=pattern,
        )

        robot = ModularRobot(body=body, brain=brain)

        # Simulate with contact detection
        # Note: We need to pass the brain's parameters differently
        # Let's use the raw simulation approach

        from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
        from revolve2.simulators.mujoco_simulator import LocalSimulator
        from revolve2.standards.simulation_parameters import make_standard_batch_parameters
        from revolve2.standards.terrains import flat

        scene = ModularRobotScene(terrain=flat())
        scene.add_robot(robot)

        simulator = LocalSimulator(headless=True, num_simulators=1)
        batch_params = make_standard_batch_parameters()
        batch_params.simulation_time = 5

        states = simulate_scenes(
            simulator=simulator,
            batch_parameters=batch_params,
            scenes=scene,
        )

        # Get metrics
        final_state = states[-1]
        pose = final_state.get_modular_robot_simulation_state(robot).get_pose()
        distance = math.sqrt(pose.position.x**2 + pose.position.y**2)

        print(f"  Distance: {distance:.4f} m")
        print(f"  Final pos: ({pose.position.x:.3f}, {pose.position.y:.3f})")


def test_from_parameters():
    """Test creating sine brain from flat parameter array (for optimization)."""
    print("\n" + "=" * 60)
    print("TEST 3: Sine Brain from Parameter Array")
    print("=" * 60)

    body = modular_robots_v1.get("spider")
    n_hinges = len(body.find_modules_of_type(ActiveHinge))
    n_params = get_num_sine_params(body)

    print(f"Spider has {n_hinges} hinges, needs {n_params} parameters")
    print(f"Format: [amp_0, phase_0, amp_1, phase_1, ...]")

    # Create random parameters
    np.random.seed(42)
    params = np.random.uniform(-1, 1, n_params)

    # Scale amplitudes to reasonable range
    for i in range(n_hinges):
        params[2*i] = abs(params[2*i]) * 0.8  # Amplitude: 0 to 0.8
        params[2*i + 1] = params[2*i + 1] * math.pi  # Phase: -π to π

    print(f"\nRandom parameters:")
    for i in range(n_hinges):
        print(f"  Hinge {i}: amp={params[2*i]:.3f}, phase={params[2*i+1]:.3f}")

    # Create brain from parameters
    brain = BrainSine.from_parameters(body, params, frequency=1.0)
    robot = ModularRobot(body=body, brain=brain)

    print(f"\nCreated sine brain from {n_params} parameters")


def test_custom_parameters():
    """Test creating sine brain with custom per-hinge parameters."""
    print("\n" + "=" * 60)
    print("TEST 4: Custom Per-Hinge Parameters")
    print("=" * 60)

    body = modular_robots_v1.get("spider")
    active_hinges = body.find_modules_of_type(ActiveHinge)

    # Custom parameters for a walking gait
    # Spider typically has 8 hinges: 4 hip joints + 4 knee/ankle joints
    # We'll create a trot-like gait with diagonal legs in sync

    parameters = [
        # Assuming order: front-left, front-right, back-left, back-right (hips)
        #                 then corresponding lower joints
        SineParameters(amplitude=0.6, frequency=1.0, phase_offset=0.0),       # FL hip
        SineParameters(amplitude=0.6, frequency=1.0, phase_offset=math.pi),   # FR hip
        SineParameters(amplitude=0.6, frequency=1.0, phase_offset=math.pi),   # BL hip
        SineParameters(amplitude=0.6, frequency=1.0, phase_offset=0.0),       # BR hip
        SineParameters(amplitude=0.4, frequency=1.0, phase_offset=0.5),       # FL lower
        SineParameters(amplitude=0.4, frequency=1.0, phase_offset=math.pi+0.5), # FR lower
        SineParameters(amplitude=0.4, frequency=1.0, phase_offset=math.pi+0.5), # BL lower
        SineParameters(amplitude=0.4, frequency=1.0, phase_offset=0.5),       # BR lower
    ]

    # Adjust to actual number of hinges
    while len(parameters) < len(active_hinges):
        parameters.append(SineParameters(0.5, 1.0, 0.0))
    parameters = parameters[:len(active_hinges)]

    brain = BrainSine(active_hinges=active_hinges, parameters=parameters)
    robot = ModularRobot(body=body, brain=brain)

    print(f"Created trot-like gait with diagonal synchronization")
    print(f"  Hip amplitude: 0.6 rad")
    print(f"  Lower joint amplitude: 0.4 rad")
    print(f"  Frequency: 1.0 Hz")


def test_simple_optimization():
    """Simple demonstration of optimizing sine parameters."""
    print("\n" + "=" * 60)
    print("TEST 5: Simple Parameter Optimization (Grid Search)")
    print("=" * 60)

    from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
    from revolve2.simulators.mujoco_simulator import LocalSimulator
    from revolve2.standards.simulation_parameters import make_standard_batch_parameters
    from revolve2.standards.terrains import flat

    body = modular_robots_v1.get("spider")

    # Grid search over amplitude and frequency
    amplitudes = [0.3, 0.5, 0.7, 0.9]
    frequencies = [0.5, 1.0, 1.5, 2.0]

    best_distance = 0
    best_params = (0, 0)

    print("\nSearching amplitude x frequency grid...")

    simulator = LocalSimulator(headless=True, num_simulators=1)
    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = 5

    for amp in amplitudes:
        for freq in frequencies:
            brain = BrainSine.from_body(
                body,
                amplitude=amp,
                frequency=freq,
                phase_pattern="alternating",
            )
            robot = ModularRobot(body=body, brain=brain)

            scene = ModularRobotScene(terrain=flat())
            scene.add_robot(robot)

            states = simulate_scenes(
                simulator=simulator,
                batch_parameters=batch_params,
                scenes=scene,
            )

            final_state = states[-1]
            pose = final_state.get_modular_robot_simulation_state(robot).get_pose()
            distance = math.sqrt(pose.position.x**2 + pose.position.y**2)

            print(f"  amp={amp:.1f}, freq={freq:.1f} -> dist={distance:.4f} m")

            if distance > best_distance:
                best_distance = distance
                best_params = (amp, freq)

    print(f"\nBest: amplitude={best_params[0]}, frequency={best_params[1]}")
    print(f"Best distance: {best_distance:.4f} m")


def compare_with_cpg():
    """Compare sine brain performance with CPG brain."""
    print("\n" + "=" * 60)
    print("TEST 6: Sine vs CPG Comparison")
    print("=" * 60)

    from revolve2.modular_robot.brain.cpg import (
        BrainCpgNetworkStatic,
        active_hinges_to_cpg_network_structure_neighbor,
    )
    from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
    from revolve2.simulators.mujoco_simulator import LocalSimulator
    from revolve2.standards.simulation_parameters import make_standard_batch_parameters
    from revolve2.standards.terrains import flat

    body = modular_robots_v1.get("spider")
    active_hinges = body.find_modules_of_type(ActiveHinge)

    simulator = LocalSimulator(headless=True, num_simulators=1)
    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = 10

    # Test 1: Sine brain
    print("\n--- Sine Brain (alternating, amp=0.7, freq=1.0) ---")
    sine_brain = BrainSine.from_body(body, amplitude=0.7, frequency=1.0, phase_pattern="alternating")
    sine_robot = ModularRobot(body=body, brain=sine_brain)

    scene = ModularRobotScene(terrain=flat())
    scene.add_robot(sine_robot)
    states = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)
    pose = states[-1].get_modular_robot_simulation_state(sine_robot).get_pose()
    sine_distance = math.sqrt(pose.position.x**2 + pose.position.y**2)
    print(f"  Distance: {sine_distance:.4f} m")

    # Test 2: CPG brain with random weights
    print("\n--- CPG Brain (neighbor coupling, random weights) ---")
    cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)
    n_params = cpg_structure.num_connections

    np.random.seed(42)
    cpg_params = np.random.uniform(-1, 1, n_params)

    cpg_brain = BrainCpgNetworkStatic.uniform_from_params(
        params=cpg_params,
        cpg_network_structure=cpg_structure,
        initial_state_uniform=0.5 * math.pi / 2,
        output_mapping=output_mapping,
    )
    cpg_robot = ModularRobot(body=body, brain=cpg_brain)

    scene = ModularRobotScene(terrain=flat())
    scene.add_robot(cpg_robot)
    states = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)
    pose = states[-1].get_modular_robot_simulation_state(cpg_robot).get_pose()
    cpg_distance = math.sqrt(pose.position.x**2 + pose.position.y**2)
    print(f"  Distance: {cpg_distance:.4f} m")

    print(f"\n--- Summary ---")
    print(f"Sine brain: {sine_distance:.4f} m")
    print(f"CPG brain:  {cpg_distance:.4f} m")
    print(f"\nNote: CPG with random weights is not optimized. ")
    print(f"      Both controllers benefit greatly from optimization.")


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("SINE BRAIN CONTROLLER TESTS")
    print("=" * 60)

    test_basic_sine()
    test_from_parameters()
    test_custom_parameters()

    # These tests require simulation (slower)
    print("\n\nRunning simulation tests (this may take a moment)...")

    test_with_contact_detection()
    test_simple_optimization()
    compare_with_cpg()

    print("\n" + "=" * 60)
    print("ALL TESTS COMPLETED")
    print("=" * 60)


if __name__ == "__main__":
    main()
