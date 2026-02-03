"""
Uncoupled Sinusoidal Brain Controller with Offset.

Enhanced sine controller with:
1. Position offset (bias) - center position for oscillation
2. Evolved frequency
3. NO coupling between joints (independent oscillators)

Equation for each joint i:
    position_i(t) = offset_i + amplitude_i * sin(2*pi*freq*t + phase_i)

This is for fair comparison with coupled sine (same params, minus coupling).
"""

import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain import Brain, BrainInstance
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot._modular_robot_control_interface import (
    ModularRobotControlInterface,
)


@dataclass
class SineOffsetParameters:
    """Parameters for a sine oscillator with offset."""

    amplitude: float      # Oscillation amplitude (radians)
    frequency: float      # Oscillation frequency (Hz)
    phase_offset: float   # Phase offset (radians)
    position_offset: float = 0.0  # Center position / bias (radians)


class BrainSineOffsetInstance(BrainInstance):
    """
    Stateful instance of the uncoupled sine brain with offset.

    Each joint oscillates independently (no coupling).
    """

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        parameters: list[SineOffsetParameters],
    ) -> None:
        """
        Initialize the sine offset brain instance.

        :param active_hinges: List of active hinges to control.
        :param parameters: Sine parameters for each hinge.
        """
        assert len(active_hinges) == len(parameters)

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
        Control the robot for one timestep with independent oscillations.
        """
        self._time += dt

        for hinge, params in zip(self._active_hinges, self._parameters):
            # Independent sine: position = offset + amplitude * sin(2*pi*f*t + phase)
            phase = 2.0 * math.pi * params.frequency * self._time + params.phase_offset
            target = params.position_offset + params.amplitude * math.sin(phase)

            # Clamp to joint range
            target = max(-hinge.range, min(hinge.range, target))

            control_interface.set_active_hinge_target(hinge, target)


class BrainSineOffset(Brain):
    """
    Uncoupled sinusoidal brain with offset.

    Each joint oscillates with:
    - Individual amplitude, phase, and offset
    - Shared frequency
    - NO coupling (independent oscillators)
    """

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        parameters: list[SineOffsetParameters] | None = None,
        default_amplitude: float = 0.5,
        default_frequency: float = 1.0,
        default_offset: float = 0.0,
    ) -> None:
        """
        Initialize the uncoupled sine brain with offset.

        :param active_hinges: List of active hinges in the robot.
        :param parameters: Optional list of parameters for each hinge.
        :param default_amplitude: Default amplitude for all hinges.
        :param default_frequency: Default frequency for all hinges.
        :param default_offset: Default position offset for all hinges.
        """
        self._active_hinges = active_hinges

        if parameters is not None:
            self._parameters = parameters
        else:
            self._parameters = [
                SineOffsetParameters(
                    amplitude=default_amplitude,
                    frequency=default_frequency,
                    phase_offset=0.0,
                    position_offset=default_offset,
                )
                for _ in active_hinges
            ]

    def make_instance(self) -> BrainInstance:
        """Create a stateful brain instance for simulation."""
        return BrainSineOffsetInstance(
            self._active_hinges,
            self._parameters,
        )

    @classmethod
    def from_parameters(
        cls,
        body: Body,
        params: npt.NDArray[np.float64] | list[float],
        frequency: float = 1.0,
    ) -> "BrainSineOffset":
        """
        Create from flat parameter array (for optimization).

        Parameter format: [amp_0, phase_0, offset_0, amp_1, phase_1, offset_1, ...]
        Total params = 3 * num_hinges

        :param body: The robot body.
        :param params: Flat array of parameters.
        :param frequency: Oscillation frequency (shared by all joints).
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
            parameters.append(SineOffsetParameters(
                amplitude=amp,
                frequency=frequency,
                phase_offset=phase,
                position_offset=offset,
            ))

        return cls(
            active_hinges=active_hinges,
            parameters=parameters,
        )


def get_num_sine_offset_params(body: Body) -> int:
    """
    Get number of parameters for sine offset optimization.

    Returns 3 * num_hinges (amplitude + phase + offset per hinge).
    """
    active_hinges = body.find_modules_of_type(ActiveHinge)
    return 3 * len(active_hinges)
