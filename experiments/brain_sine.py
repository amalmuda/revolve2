"""
Simple Sinusoidal Brain Controller for Modular Robots.

Each active hinge oscillates according to:
    position(t) = position_offset + amplitude * sin(2π * frequency * t + phase_offset)

This is a simpler alternative to CPG networks, useful for:
- Baseline comparisons
- Quick testing
- Understanding basic locomotion patterns

Parameters per hinge:
- amplitude: How far the joint moves (radians, typically 0 to joint.range)
- frequency: How fast the joint oscillates (Hz)
- phase_offset: Phase difference between joints (radians)
- position_offset: Center position / bias (radians)
"""

import math
from dataclasses import dataclass
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
class SineParameters:
    """Parameters for a single sine oscillator."""

    amplitude: float  # Oscillation amplitude (radians)
    frequency: float  # Oscillation frequency (Hz)
    phase_offset: float  # Phase offset (radians)
    position_offset: float = 0.0  # Center position / bias (radians)

    @classmethod
    def default(cls, amplitude: float = 0.5, frequency: float = 1.0, phase_offset: float = 0.0, position_offset: float = 0.0) -> "SineParameters":
        """Create default parameters."""
        return cls(amplitude=amplitude, frequency=frequency, phase_offset=phase_offset, position_offset=position_offset)


class BrainSineInstance(BrainInstance):
    """
    Stateful instance of the sine brain that performs actual control.

    Called every control timestep to update joint positions.
    """

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        parameters: list[SineParameters],
    ) -> None:
        """
        Initialize the sine brain instance.

        :param active_hinges: List of active hinges to control.
        :param parameters: Sine parameters for each hinge.
        """
        assert len(active_hinges) == len(parameters), (
            f"Number of hinges ({len(active_hinges)}) must match "
            f"number of parameter sets ({len(parameters)})"
        )
        self._active_hinges = active_hinges
        self._parameters = parameters
        self._time = 0.0

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the robot for one timestep.

        :param dt: Time since last call (seconds).
        :param sensor_state: Current sensor readings (unused for open-loop).
        :param control_interface: Interface to set joint targets.
        """
        self._time += dt

        for hinge, params in zip(self._active_hinges, self._parameters):
            # Calculate sine wave: offset + amplitude * sin(2π * frequency * time + phase)
            target = params.position_offset + params.amplitude * math.sin(
                2.0 * math.pi * params.frequency * self._time + params.phase_offset
            )

            # Clamp to joint range
            target = max(-hinge.range, min(hinge.range, target))

            control_interface.set_active_hinge_target(hinge, target)


class BrainSine(Brain):
    """
    Simple sinusoidal brain for modular robots.

    Each joint oscillates independently with configurable amplitude,
    frequency, and phase offset.
    """

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        parameters: list[SineParameters] | None = None,
        default_amplitude: float = 0.5,
        default_frequency: float = 1.0,
        phase_offsets: list[float] | None = None,
    ) -> None:
        """
        Initialize the sine brain.

        :param active_hinges: List of active hinges in the robot.
        :param parameters: Optional list of SineParameters for each hinge.
                          If None, uses defaults with optional phase_offsets.
        :param default_amplitude: Default amplitude for all hinges (radians).
        :param default_frequency: Default frequency for all hinges (Hz).
        :param phase_offsets: Optional list of phase offsets for each hinge.
        """
        self._active_hinges = active_hinges

        if parameters is not None:
            self._parameters = parameters
        else:
            # Create default parameters
            if phase_offsets is None:
                phase_offsets = [0.0] * len(active_hinges)

            self._parameters = [
                SineParameters(
                    amplitude=default_amplitude,
                    frequency=default_frequency,
                    phase_offset=offset,
                )
                for offset in phase_offsets
            ]

    def make_instance(self) -> BrainInstance:
        """Create a stateful brain instance for simulation."""
        return BrainSineInstance(self._active_hinges, self._parameters)

    @classmethod
    def from_body(
        cls,
        body: Body,
        amplitude: float = 0.5,
        frequency: float = 1.0,
        phase_pattern: str = "alternating",
    ) -> "BrainSine":
        """
        Create a sine brain from a robot body with automatic phase patterns.

        :param body: The robot body.
        :param amplitude: Oscillation amplitude (radians).
        :param frequency: Oscillation frequency (Hz).
        :param phase_pattern: Phase pattern for joints:
            - "zero": All joints in phase (0)
            - "alternating": Alternating 0 and π
            - "sequential": Evenly spaced phases
            - "random": Random phases
        :returns: Configured BrainSine instance.
        """
        active_hinges = body.find_modules_of_type(ActiveHinge)
        n_hinges = len(active_hinges)

        if phase_pattern == "zero":
            phase_offsets = [0.0] * n_hinges
        elif phase_pattern == "alternating":
            phase_offsets = [0.0 if i % 2 == 0 else math.pi for i in range(n_hinges)]
        elif phase_pattern == "sequential":
            phase_offsets = [2.0 * math.pi * i / n_hinges for i in range(n_hinges)]
        elif phase_pattern == "random":
            phase_offsets = [np.random.uniform(0, 2 * math.pi) for _ in range(n_hinges)]
        else:
            raise ValueError(f"Unknown phase pattern: {phase_pattern}")

        return cls(
            active_hinges=active_hinges,
            default_amplitude=amplitude,
            default_frequency=frequency,
            phase_offsets=phase_offsets,
        )

    @classmethod
    def from_parameters(
        cls,
        body: Body,
        params: npt.NDArray[np.float64] | list[float],
        frequency: float = 1.0,
    ) -> "BrainSine":
        """
        Create a sine brain from a flat parameter array.

        Useful for evolutionary optimization (CMA-ES, etc.)

        Parameter format: [amp_0, phase_0, offset_0, amp_1, phase_1, offset_1, ...]
        Total params = 3 * num_hinges

        :param body: The robot body.
        :param params: Flat array of [amplitude, phase, offset] triplets.
        :param frequency: Oscillation frequency (same for all).
        :returns: Configured BrainSine instance.
        """
        active_hinges = body.find_modules_of_type(ActiveHinge)
        n_hinges = len(active_hinges)

        expected_params = 3 * n_hinges
        if len(params) != expected_params:
            raise ValueError(
                f"Expected {expected_params} parameters for {n_hinges} hinges, "
                f"got {len(params)}"
            )

        parameters = []
        for i in range(n_hinges):
            amp = float(params[3 * i])
            phase = float(params[3 * i + 1])
            offset = float(params[3 * i + 2])
            parameters.append(SineParameters(
                amplitude=amp,
                frequency=frequency,
                phase_offset=phase,
                position_offset=offset,
            ))

        return cls(active_hinges=active_hinges, parameters=parameters)

    @classmethod
    def from_amplitude_phase_arrays(
        cls,
        body: Body,
        amplitudes: Sequence[float],
        phases: Sequence[float],
        frequency: float = 1.0,
    ) -> "BrainSine":
        """
        Create a sine brain from separate amplitude and phase arrays.

        :param body: The robot body.
        :param amplitudes: Amplitude for each hinge.
        :param phases: Phase offset for each hinge.
        :param frequency: Oscillation frequency (same for all).
        :returns: Configured BrainSine instance.
        """
        active_hinges = body.find_modules_of_type(ActiveHinge)
        n_hinges = len(active_hinges)

        if len(amplitudes) != n_hinges or len(phases) != n_hinges:
            raise ValueError(
                f"Expected {n_hinges} amplitudes and phases, "
                f"got {len(amplitudes)} and {len(phases)}"
            )

        parameters = [
            SineParameters(amplitude=amp, frequency=frequency, phase_offset=phase)
            for amp, phase in zip(amplitudes, phases)
        ]

        return cls(active_hinges=active_hinges, parameters=parameters)


def get_num_sine_params(body: Body) -> int:
    """
    Get the number of parameters needed for sine brain optimization.

    Returns 3 * num_hinges (amplitude + phase + offset per hinge).

    :param body: The robot body.
    :returns: Number of parameters.
    """
    active_hinges = body.find_modules_of_type(ActiveHinge)
    return 3 * len(active_hinges)


# =============================================================================
# Example usage and testing
# =============================================================================

def demo_sine_brain():
    """Demonstrate the sine brain with a spider robot."""
    from revolve2.modular_robot import ModularRobot
    from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
    from revolve2.simulators.mujoco_simulator import LocalSimulator
    from revolve2.standards import modular_robots_v1
    from revolve2.standards.simulation_parameters import make_standard_batch_parameters
    from revolve2.standards.terrains import flat

    print("=" * 60)
    print("Sine Brain Demo")
    print("=" * 60)

    # Get a spider robot
    body = modular_robots_v1.get("spider")
    active_hinges = body.find_modules_of_type(ActiveHinge)
    print(f"\nRobot: spider with {len(active_hinges)} active hinges")

    # Best parameters from 10-generation evolution (achieved 2.48m in 10s)
    # Note: Simple alternating patterns (0, π, 0, π...) produce symmetric motion
    # that doesn't result in locomotion. Optimized parameters are needed.
    # Format: [amp, phase, offset] per hinge
    evolved_params = np.array([
        0.912, 0.585, 0.0,   # Hinge 0: amp, phase, offset
        0.600, -1.002, 0.0,  # Hinge 1
        0.929, -0.736, 0.0,  # Hinge 2
        0.809, 0.065, 0.0,   # Hinge 3
        0.448, -0.391, 0.0,  # Hinge 4
        0.122, -0.081, 0.0,  # Hinge 5
        0.258, -0.203, 0.0,  # Hinge 6
        0.059, 0.526, 0.0,   # Hinge 7
    ])

    brain = BrainSine.from_parameters(body, evolved_params, frequency=1.0)
    print("Using evolved parameters from optimization")

    # Create robot
    robot = ModularRobot(body=body, brain=brain)

    # Create scene
    scene = ModularRobotScene(terrain=flat())
    scene.add_robot(robot)

    # Simulate
    print("\nRunning simulation (10 seconds)...")
    simulator = LocalSimulator(headless=False, num_simulators=1)

    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = 10

    states = simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_params,
        scenes=scene,
    )

    # Get final position (single scene returns flat list of states)
    final_state = states[-1]
    pose = final_state.get_modular_robot_simulation_state(robot).get_pose()

    print(f"\nFinal position: x={pose.position.x:.3f}, y={pose.position.y:.3f}")
    distance = math.sqrt(pose.position.x**2 + pose.position.y**2)
    print(f"Distance traveled: {distance:.3f} m")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    demo_sine_brain()
