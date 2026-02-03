"""
BLF-Coupled Sine Brain Controller.

Same as BrainCoupledSine but uses BLF coupling rules instead of neighbor coupling.
This is just a convenience wrapper - internally uses BrainCoupledSine with a
BLF-generated coupling matrix.
"""

import numpy as np
import numpy.typing as npt

from revolve2.modular_robot.body.base import ActiveHinge, Body

from brain_sine_coupled import BrainCoupledSine, CoupledSineParameters
from blf_analyzer import analyze_body, BLFResult


def create_blf_coupling_matrix(active_hinges: list[ActiveHinge], blf_result: BLFResult) -> np.ndarray:
    """
    Create a coupling matrix based on BLF rules.

    :param active_hinges: List of active hinges.
    :param blf_result: BLF analysis result.
    :returns: NxN coupling weight matrix.
    """
    n = len(active_hinges)
    weights = blf_result.coupling_matrix.copy().astype(float)

    # Normalize rows (same as neighbor version)
    row_sums = weights.sum(axis=1, keepdims=True)
    row_sums[row_sums == 0] = 1  # Avoid division by zero
    weights = weights / row_sums

    return weights


def brain_coupled_sine_blf(
    body: Body,
    amplitude: float = 0.5,
    frequency: float = 1.0,
    offset: float = 0.0,
    coupling_strength: float = 0.5,
    phase_pattern: str = "alternating",
) -> tuple[BrainCoupledSine, BLFResult]:
    """
    Create a BrainCoupledSine with BLF coupling instead of neighbor coupling.

    Identical to BrainCoupledSine.from_body() but uses BLF coupling rules.

    :param body: The robot body.
    :param amplitude: Oscillation amplitude.
    :param frequency: Oscillation frequency (Hz).
    :param offset: Position offset / bias.
    :param coupling_strength: Coupling strength (0-1).
    :param phase_pattern: Phase pattern for joints ("zero", "alternating", "sequential").
    :returns: Tuple of (brain, blf_result).
    """
    import math

    # Analyze body with BLF
    blf_result = analyze_body(body)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    n = len(active_hinges)

    # Create BLF coupling matrix
    coupling_weights = create_blf_coupling_matrix(active_hinges, blf_result)

    # Create phase pattern (same as BrainCoupledSine.from_body)
    if phase_pattern == "zero":
        phases = [0.0] * n
    elif phase_pattern == "alternating":
        phases = [0.0 if i % 2 == 0 else math.pi for i in range(n)]
    elif phase_pattern == "sequential":
        phases = [2.0 * math.pi * i / n for i in range(n)]
    else:
        phases = [0.0] * n

    # Create parameters (same as BrainCoupledSine.from_body)
    parameters = [
        CoupledSineParameters(
            amplitude=amplitude,
            frequency=frequency,
            phase_offset=phase,
            position_offset=offset,
        )
        for phase in phases
    ]

    # Create BrainCoupledSine with BLF coupling
    brain = BrainCoupledSine(
        active_hinges=active_hinges,
        parameters=parameters,
        coupling_weights=coupling_weights,
        coupling_strength=coupling_strength,
    )

    return brain, blf_result


def brain_coupled_sine_blf_from_parameters(
    body: Body,
    params: npt.NDArray[np.float64] | list[float],
    frequency: float = 1.0,
    coupling_strength: float = 0.5,
) -> tuple[BrainCoupledSine, BLFResult]:
    """
    Create BLF-coupled sine brain from flat parameter array.

    Same parameter format as BrainCoupledSine.from_parameters:
    [amp_0, phase_0, offset_0, amp_1, phase_1, offset_1, ...]
    Total params = 3 * num_hinges

    :param body: The robot body.
    :param params: Flat array of parameters.
    :param frequency: Oscillation frequency.
    :param coupling_strength: Coupling strength.
    :returns: Tuple of (brain, blf_result).
    """
    # Analyze body with BLF
    blf_result = analyze_body(body)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    n = len(active_hinges)

    # Validate params
    expected = 3 * n
    if len(params) != expected:
        raise ValueError(f"Expected {expected} params for {n} hinges, got {len(params)}")

    # Create BLF coupling matrix
    coupling_weights = create_blf_coupling_matrix(active_hinges, blf_result)

    # Parse parameters (same as BrainCoupledSine.from_parameters)
    parameters = []
    for i in range(n):
        amp = float(params[3 * i])
        phase = float(params[3 * i + 1])
        offset = float(params[3 * i + 2])
        parameters.append(CoupledSineParameters(
            amplitude=amp,
            frequency=frequency,
            phase_offset=phase,
            position_offset=offset,
        ))

    # Create BrainCoupledSine with BLF coupling
    brain = BrainCoupledSine(
        active_hinges=active_hinges,
        parameters=parameters,
        coupling_weights=coupling_weights,
        coupling_strength=coupling_strength,
    )

    return brain, blf_result


# =============================================================================
# Demo
# =============================================================================

if __name__ == "__main__":
    import math
    from revolve2.modular_robot import ModularRobot
    from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
    from revolve2.simulators.mujoco_simulator import LocalSimulator
    from revolve2.standards import modular_robots_v1, modular_robots_v2
    from revolve2.standards.simulation_parameters import make_standard_batch_parameters
    from revolve2.standards.terrains import flat

    print("=" * 60)
    print("BLF vs Neighbor Coupling Comparison")
    print("=" * 60)

    simulator = LocalSimulator(headless=True, num_simulators=1)
    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = 10

    # Test with spider
    print("\n--- SPIDER ---")
    spider_body = modular_robots_v1.get("spider")

    # Neighbor coupling
    print("\n1. Neighbor coupling:")
    brain_neighbor = BrainCoupledSine.from_body(
        spider_body, amplitude=0.5, frequency=1.0, coupling_strength=0.3
    )
    robot_neighbor = ModularRobot(body=spider_body, brain=brain_neighbor)

    scene1 = ModularRobotScene(terrain=flat())
    scene1.add_robot(robot_neighbor)
    states1 = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene1)
    pose1 = states1[-1].get_modular_robot_simulation_state(robot_neighbor).get_pose()
    dist1 = math.sqrt(pose1.position.x**2 + pose1.position.y**2)
    print(f"   Distance: {dist1:.3f} m")

    # BLF coupling
    print("\n2. BLF coupling:")
    spider_body2 = modular_robots_v1.get("spider")
    brain_blf, blf_result = brain_coupled_sine_blf(
        spider_body2, amplitude=0.5, frequency=1.0, coupling_strength=0.3
    )
    robot_blf = ModularRobot(body=spider_body2, brain=brain_blf)

    scene2 = ModularRobotScene(terrain=flat())
    scene2.add_robot(robot_blf)
    states2 = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene2)
    pose2 = states2[-1].get_modular_robot_simulation_state(robot_blf).get_pose()
    dist2 = math.sqrt(pose2.position.x**2 + pose2.position.y**2)
    print(f"   Distance: {dist2:.3f} m")

    blf_result.print_summary()

    print("\n" + "=" * 60)
    print(f"Spider - Neighbor: {dist1:.3f} m, BLF: {dist2:.3f} m")
    print("=" * 60)

    # Test with gecko
    print("\n--- GECKO ---")
    gecko_body = modular_robots_v2.get("gecko")

    # Neighbor coupling
    print("\n1. Neighbor coupling:")
    brain_neighbor_g = BrainCoupledSine.from_body(
        gecko_body, amplitude=0.5, frequency=1.0, coupling_strength=0.3
    )
    robot_neighbor_g = ModularRobot(body=gecko_body, brain=brain_neighbor_g)

    scene3 = ModularRobotScene(terrain=flat())
    scene3.add_robot(robot_neighbor_g)
    states3 = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene3)
    pose3 = states3[-1].get_modular_robot_simulation_state(robot_neighbor_g).get_pose()
    dist3 = math.sqrt(pose3.position.x**2 + pose3.position.y**2)
    print(f"   Distance: {dist3:.3f} m")

    # BLF coupling
    print("\n2. BLF coupling:")
    gecko_body2 = modular_robots_v2.get("gecko")
    brain_blf_g, blf_result_g = brain_coupled_sine_blf(
        gecko_body2, amplitude=0.5, frequency=1.0, coupling_strength=0.3
    )
    robot_blf_g = ModularRobot(body=gecko_body2, brain=brain_blf_g)

    scene4 = ModularRobotScene(terrain=flat())
    scene4.add_robot(robot_blf_g)
    states4 = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene4)
    pose4 = states4[-1].get_modular_robot_simulation_state(robot_blf_g).get_pose()
    dist4 = math.sqrt(pose4.position.x**2 + pose4.position.y**2)
    print(f"   Distance: {dist4:.3f} m")

    blf_result_g.print_summary()

    print("\n" + "=" * 60)
    print(f"Gecko - Neighbor: {dist3:.3f} m, BLF: {dist4:.3f} m")
    print("=" * 60)
