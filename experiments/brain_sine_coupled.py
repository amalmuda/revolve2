"""
Coupled Sinusoidal Brain Controller with Offset.

Enhanced sine controller with:
1. Position offset (bias) - center position for oscillation
2. Coupling - joints influence each other's phase
3. Amplitude, frequency, phase as before

Equation for each joint i:
    position_i(t) = offset_i + amplitude_i × sin(2π × freq × t + phase_i + coupling_term)

Where coupling_term = Σ(w_ij × sin(θ_j)) for neighbor joints j

This is a middle ground between pure sine and full CPG.
"""

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
import numpy.typing as npt

from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain import Brain, BrainInstance
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot._modular_robot_control_interface import (
    ModularRobotControlInterface,
)


@dataclass
class CoupledSineParameters:
    """Parameters for a coupled sine oscillator."""

    amplitude: float      # Oscillation amplitude (radians)
    frequency: float      # Oscillation frequency (Hz)
    phase_offset: float   # Phase offset (radians)
    position_offset: float = 0.0  # Center position / bias (radians)


class BrainCoupledSineInstance(BrainInstance):
    """
    Stateful instance of the coupled sine brain.

    Implements coupling between neighboring joints.
    """

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        parameters: list[CoupledSineParameters],
        coupling_weights: np.ndarray | None = None,
        coupling_strength: float = 0.5,
    ) -> None:
        """
        Initialize the coupled sine brain instance.

        :param active_hinges: List of active hinges to control.
        :param parameters: Sine parameters for each hinge.
        :param coupling_weights: NxN coupling weight matrix (None = auto from topology).
        :param coupling_strength: Global coupling strength multiplier.
        """
        assert len(active_hinges) == len(parameters)

        self._active_hinges = active_hinges
        self._parameters = parameters
        self._coupling_strength = coupling_strength
        self._time = 0.0
        self._n = len(active_hinges)

        # Previous phase values for coupling
        self._phases = np.zeros(self._n)

        # Build coupling matrix from robot topology if not provided
        if coupling_weights is None:
            self._coupling_weights = self._build_neighbor_coupling(active_hinges)
        else:
            self._coupling_weights = coupling_weights

    def _build_neighbor_coupling(self, active_hinges: list[ActiveHinge]) -> np.ndarray:
        """
        Build coupling matrix based on physical neighbor relationships.

        Joints that are neighbors in the robot tree are coupled.
        """
        n = len(active_hinges)
        weights = np.zeros((n, n))

        for i, hinge_i in enumerate(active_hinges):
            # Get neighbors within 2 hops
            neighbors = hinge_i.neighbours(within_range=2)

            for j, hinge_j in enumerate(active_hinges):
                if i != j and hinge_j in neighbors:
                    weights[i, j] = 1.0

        # Normalize rows
        row_sums = weights.sum(axis=1, keepdims=True)
        row_sums[row_sums == 0] = 1  # Avoid division by zero
        weights = weights / row_sums

        return weights

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the robot for one timestep with coupled oscillations.
        """
        self._time += dt

        # Calculate base phases for all oscillators
        base_phases = np.zeros(self._n)
        for i, params in enumerate(self._parameters):
            base_phases[i] = 2.0 * math.pi * params.frequency * self._time + params.phase_offset

        # Calculate coupling influence from previous state
        coupling_influence = self._coupling_strength * self._coupling_weights @ np.sin(self._phases)

        # Update phases with coupling
        self._phases = base_phases + coupling_influence

        # Calculate and apply joint positions
        for i, (hinge, params) in enumerate(zip(self._active_hinges, self._parameters)):
            # position = offset + amplitude * sin(phase + coupling)
            target = params.position_offset + params.amplitude * math.sin(self._phases[i])

            # Clamp to joint range
            target = max(-hinge.range, min(hinge.range, target))

            control_interface.set_active_hinge_target(hinge, target)


class BrainCoupledSine(Brain):
    """
    Coupled sinusoidal brain with offset.

    Each joint oscillates with:
    - Individual amplitude, phase, and offset
    - Coupling to neighboring joints
    """

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        parameters: list[CoupledSineParameters] | None = None,
        coupling_weights: np.ndarray | None = None,
        coupling_strength: float = 0.5,
        default_amplitude: float = 0.5,
        default_frequency: float = 1.0,
        default_offset: float = 0.0,
    ) -> None:
        """
        Initialize the coupled sine brain.

        :param active_hinges: List of active hinges in the robot.
        :param parameters: Optional list of parameters for each hinge.
        :param coupling_weights: Optional NxN coupling matrix.
        :param coupling_strength: Global coupling strength (0 = no coupling, 1 = strong).
        :param default_amplitude: Default amplitude for all hinges.
        :param default_frequency: Default frequency for all hinges.
        :param default_offset: Default position offset for all hinges.
        """
        self._active_hinges = active_hinges
        self._coupling_weights = coupling_weights
        self._coupling_strength = coupling_strength

        if parameters is not None:
            self._parameters = parameters
        else:
            self._parameters = [
                CoupledSineParameters(
                    amplitude=default_amplitude,
                    frequency=default_frequency,
                    phase_offset=0.0,
                    position_offset=default_offset,
                )
                for _ in active_hinges
            ]

    def make_instance(self) -> BrainInstance:
        """Create a stateful brain instance for simulation."""
        return BrainCoupledSineInstance(
            self._active_hinges,
            self._parameters,
            self._coupling_weights,
            self._coupling_strength,
        )

    @classmethod
    def from_body(
        cls,
        body: Body,
        amplitude: float = 0.5,
        frequency: float = 1.0,
        offset: float = 0.0,
        coupling_strength: float = 0.5,
        phase_pattern: str = "alternating",
    ) -> "BrainCoupledSine":
        """
        Create a coupled sine brain from a robot body.

        :param body: The robot body.
        :param amplitude: Oscillation amplitude.
        :param frequency: Oscillation frequency (Hz).
        :param offset: Position offset / bias.
        :param coupling_strength: Coupling strength (0-1).
        :param phase_pattern: Phase pattern for joints.
        """
        active_hinges = body.find_modules_of_type(ActiveHinge)
        n = len(active_hinges)

        if phase_pattern == "zero":
            phases = [0.0] * n
        elif phase_pattern == "alternating":
            phases = [0.0 if i % 2 == 0 else math.pi for i in range(n)]
        elif phase_pattern == "sequential":
            phases = [2.0 * math.pi * i / n for i in range(n)]
        else:
            phases = [0.0] * n

        parameters = [
            CoupledSineParameters(
                amplitude=amplitude,
                frequency=frequency,
                phase_offset=phase,
                position_offset=offset,
            )
            for phase in phases
        ]

        return cls(
            active_hinges=active_hinges,
            parameters=parameters,
            coupling_strength=coupling_strength,
        )

    @classmethod
    def from_parameters(
        cls,
        body: Body,
        params: npt.NDArray[np.float64] | list[float],
        frequency: float = 1.0,
        coupling_strength: float = 0.5,
    ) -> "BrainCoupledSine":
        """
        Create from flat parameter array (for optimization).

        Parameter format: [amp_0, phase_0, offset_0, amp_1, phase_1, offset_1, ...]
        Total params = 3 * num_hinges

        :param body: The robot body.
        :param params: Flat array of parameters.
        :param frequency: Oscillation frequency.
        :param coupling_strength: Coupling strength.
        """
        active_hinges = body.find_modules_of_type(ActiveHinge)
        n = len(active_hinges)

        expected = 3 * n
        if len(params) != expected:
            raise ValueError(f"Expected {expected} params for {n} hinges, got {len(params)}")

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

        return cls(
            active_hinges=active_hinges,
            parameters=parameters,
            coupling_strength=coupling_strength,
        )


def get_num_coupled_sine_params(body: Body) -> int:
    """
    Get number of parameters for coupled sine optimization.

    Returns 3 * num_hinges (amplitude + phase + offset per hinge).
    """
    active_hinges = body.find_modules_of_type(ActiveHinge)
    return 3 * len(active_hinges)


# =============================================================================
# Quick comparison demo
# =============================================================================

def demo_comparison():
    """Compare uncoupled vs coupled sine controllers.

    Uses evolved parameters from optimization for a fair comparison.
    Simple alternating patterns (0, π, 0, π...) produce symmetric motion
    that doesn't result in locomotion.
    """
    from brain_sine import BrainSine
    from revolve2.modular_robot import ModularRobot
    from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
    from revolve2.simulators.mujoco_simulator import LocalSimulator
    from revolve2.standards import modular_robots_v1
    from revolve2.standards.simulation_parameters import make_standard_batch_parameters
    from revolve2.standards.terrains import flat

    print("=" * 60)
    print("COMPARISON: Uncoupled vs Coupled Sine")
    print("=" * 60)

    # Best parameters from 10-generation evolution (achieved 2.48m in 10s)
    # These have asymmetric amplitudes and phases that produce locomotion
    evolved_params = np.array([
        0.912, 0.585,   # Hinge 0: amp, phase
        0.600, -1.002,  # Hinge 1
        0.929, -0.736,  # Hinge 2
        0.809, 0.065,   # Hinge 3
        0.448, -0.391,  # Hinge 4
        0.122, -0.081,  # Hinge 5
        0.258, -0.203,  # Hinge 6
        0.059, 0.526,   # Hinge 7
    ])

    simulator = LocalSimulator(headless=True, num_simulators=1)
    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = 10

    # Uncoupled sine with evolved parameters
    print("\n1. Running UNCOUPLED sine (evolved parameters)...")
    body1 = modular_robots_v1.get("spider")
    brain1 = BrainSine.from_parameters(body1, evolved_params, frequency=1.0)
    robot1 = ModularRobot(body=body1, brain=brain1)

    scene1 = ModularRobotScene(terrain=flat())
    scene1.add_robot(robot1)

    states1 = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene1)
    final_state1 = states1[-1]
    pose1 = final_state1.get_modular_robot_simulation_state(robot1).get_pose()
    dist1 = math.sqrt(pose1.position.x**2 + pose1.position.y**2)
    print(f"   Distance: {dist1:.3f} m")

    # Coupled sine with same base parameters + offset and coupling
    print("\n2. Running COUPLED sine (evolved params + offset + coupling)...")
    body2 = modular_robots_v1.get("spider")
    active_hinges2 = body2.find_modules_of_type(ActiveHinge)

    # Convert evolved params to CoupledSineParameters
    n = len(active_hinges2)
    coupled_params = []
    for i in range(n):
        amp = float(evolved_params[2 * i])
        phase = float(evolved_params[2 * i + 1])
        coupled_params.append(CoupledSineParameters(
            amplitude=amp,
            frequency=1.0,
            phase_offset=phase,
            position_offset=0.1,  # Add slight bias
        ))

    brain2 = BrainCoupledSine(
        active_hinges=active_hinges2,
        parameters=coupled_params,
        coupling_strength=0.3,
    )
    robot2 = ModularRobot(body=body2, brain=brain2)

    scene2 = ModularRobotScene(terrain=flat())
    scene2.add_robot(robot2)

    states2 = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene2)
    final_state2 = states2[-1]
    pose2 = final_state2.get_modular_robot_simulation_state(robot2).get_pose()
    dist2 = math.sqrt(pose2.position.x**2 + pose2.position.y**2)
    print(f"   Distance: {dist2:.3f} m")

    print("\n" + "=" * 60)
    print(f"Uncoupled: {dist1:.3f} m")
    print(f"Coupled:   {dist2:.3f} m")
    if dist1 > 0:
        print(f"Difference: {dist2 - dist1:+.3f} m ({(dist2/dist1 - 1)*100:+.1f}%)")
    else:
        print(f"Difference: {dist2 - dist1:+.3f} m")
    print("=" * 60)


if __name__ == "__main__":
    demo_comparison()
