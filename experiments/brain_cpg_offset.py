"""
CPG Brain with Offset Support.

This module extends Revolve2's CPG brain to add per-joint offset parameters.
The offset allows joints to oscillate around non-zero positions, which may
help reduce dragging by keeping legs lifted.

Output = (cpg_state + offset) * hinge_range
"""

import math
import numpy as np
import numpy.typing as npt

from revolve2.modular_robot._modular_robot_control_interface import ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot.brain._brain import Brain
from revolve2.modular_robot.brain._brain_instance import BrainInstance
from revolve2.modular_robot.brain.cpg import CpgNetworkStructure


class BrainCpgInstanceWithOffset(BrainInstance):
    """
    CPG network brain with offset support.

    Output = (cpg_state + offset) * hinge_range
    """

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        weight_matrix: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
        offsets: npt.NDArray[np.float_],
    ) -> None:
        assert initial_state.ndim == 1
        assert weight_matrix.ndim == 2
        assert weight_matrix.shape[0] == weight_matrix.shape[1]
        assert initial_state.shape[0] == weight_matrix.shape[0]
        assert all([i >= 0 and i < len(initial_state) for i, _ in output_mapping])
        assert len(offsets) == len(output_mapping)

        self._state = initial_state.copy()
        self._weight_matrix = weight_matrix
        self._output_mapping = output_mapping
        self._offsets = offsets

    @staticmethod
    def _rk45(
        state: npt.NDArray[np.float_], A: npt.NDArray[np.float_], dt: float
    ) -> npt.NDArray[np.float_]:
        """RK4 integration."""
        A1 = np.matmul(A, state)
        A2 = np.matmul(A, (state + dt / 2 * A1))
        A3 = np.matmul(A, (state + dt / 2 * A2))
        A4 = np.matmul(A, (state + dt * A3))
        state = state + dt / 6 * (A1 + 2 * (A2 + A3) + A4)
        return np.clip(state, a_min=-1, a_max=1)

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        # Integrate ODE
        self._state = self._rk45(self._state, self._weight_matrix, dt)

        # Apply output with offset
        for idx, (state_index, active_hinge) in enumerate(self._output_mapping):
            cpg_output = float(self._state[state_index])
            offset = float(self._offsets[idx])
            # Clip the combined output to [-1, 1] before scaling
            combined = np.clip(cpg_output + offset, -1, 1)
            control_interface.set_active_hinge_target(
                active_hinge, combined * active_hinge.range
            )


class BrainCpgNetworkStaticWithOffset(Brain):
    """
    CPG brain with per-joint offset parameters.

    Parameters: [internal_weights..., external_weights..., offsets...]
    """

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        weight_matrix: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
        offsets: npt.NDArray[np.float_],
    ) -> None:
        self._initial_state = initial_state
        self._weight_matrix = weight_matrix
        self._output_mapping = output_mapping
        self._offsets = offsets

    @classmethod
    def uniform_from_params(
        cls,
        params: npt.NDArray[np.float_],
        cpg_network_structure: CpgNetworkStructure,
        initial_state_uniform: float,
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> "BrainCpgNetworkStaticWithOffset":
        """
        Create brain from parameters.

        params layout: [cpg_weights..., offsets...]
        - cpg_weights: num_connections parameters for the weight matrix
        - offsets: num_outputs parameters for per-joint offsets
        """
        num_cpg_params = cpg_network_structure.num_connections
        num_outputs = len(output_mapping)

        assert len(params) == num_cpg_params + num_outputs, \
            f"Expected {num_cpg_params + num_outputs} params, got {len(params)}"

        cpg_params = params[:num_cpg_params]
        offsets = params[num_cpg_params:]

        initial_state = cpg_network_structure.make_uniform_state(initial_state_uniform)
        weight_matrix = cpg_network_structure.make_connection_weights_matrix_from_params(
            list(cpg_params)
        )

        return cls(
            initial_state=initial_state,
            weight_matrix=weight_matrix,
            output_mapping=output_mapping,
            offsets=np.array(offsets),
        )

    def make_instance(self) -> BrainInstance:
        return BrainCpgInstanceWithOffset(
            initial_state=self._initial_state,
            weight_matrix=self._weight_matrix,
            output_mapping=self._output_mapping,
            offsets=self._offsets,
        )


def get_num_params_with_offset(cpg_network_structure: CpgNetworkStructure, num_hinges: int) -> int:
    """Get total number of parameters (CPG weights + offsets)."""
    return cpg_network_structure.num_connections + num_hinges


def get_bounds_with_offset(cpg_network_structure: CpgNetworkStructure, num_hinges: int):
    """Get parameter bounds: weights in [-1, 1], offsets in [-0.5, 0.5]."""
    num_cpg = cpg_network_structure.num_connections

    # CPG weights: [-1, 1]
    lower = [-1.0] * num_cpg
    upper = [1.0] * num_cpg

    # Offsets: [-0.5, 0.5] (moderate range to avoid extreme biases)
    lower.extend([-0.5] * num_hinges)
    upper.extend([0.5] * num_hinges)

    return lower, upper


# =============================================================================
# CPG with Amplitude Control
# =============================================================================

class BrainCpgInstanceWithAmplitude(BrainInstance):
    """
    CPG network brain with per-joint amplitude control.

    Output = cpg_state × amplitude × hinge_range

    This allows evolution to control oscillation magnitude per joint,
    similar to what parametric sine controllers can do.
    """

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        weight_matrix: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
        amplitudes: npt.NDArray[np.float_],
    ) -> None:
        assert initial_state.ndim == 1
        assert weight_matrix.ndim == 2
        assert weight_matrix.shape[0] == weight_matrix.shape[1]
        assert initial_state.shape[0] == weight_matrix.shape[0]
        assert all([i >= 0 and i < len(initial_state) for i, _ in output_mapping])
        assert len(amplitudes) == len(output_mapping)

        self._state = initial_state.copy()
        self._weight_matrix = weight_matrix
        self._output_mapping = output_mapping
        self._amplitudes = amplitudes

    @staticmethod
    def _rk45(
        state: npt.NDArray[np.float_], A: npt.NDArray[np.float_], dt: float
    ) -> npt.NDArray[np.float_]:
        """RK4 integration."""
        A1 = np.matmul(A, state)
        A2 = np.matmul(A, (state + dt / 2 * A1))
        A3 = np.matmul(A, (state + dt / 2 * A2))
        A4 = np.matmul(A, (state + dt * A3))
        state = state + dt / 6 * (A1 + 2 * (A2 + A3) + A4)
        return np.clip(state, a_min=-1, a_max=1)

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        # Integrate ODE
        self._state = self._rk45(self._state, self._weight_matrix, dt)

        # Apply output with amplitude scaling
        for idx, (state_index, active_hinge) in enumerate(self._output_mapping):
            cpg_output = float(self._state[state_index])
            amplitude = float(self._amplitudes[idx])
            # Scale by amplitude, then by hinge range
            output = cpg_output * amplitude * active_hinge.range
            control_interface.set_active_hinge_target(active_hinge, output)


class BrainCpgNetworkStaticWithAmplitude(Brain):
    """
    CPG brain with per-joint amplitude parameters.

    Parameters: [internal_weights..., external_weights..., amplitudes...]

    This gives Revolve2 CPG the same per-joint amplitude control that
    parametric sine controllers have, which may help reduce dragging.
    """

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        weight_matrix: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
        amplitudes: npt.NDArray[np.float_],
    ) -> None:
        self._initial_state = initial_state
        self._weight_matrix = weight_matrix
        self._output_mapping = output_mapping
        self._amplitudes = amplitudes

    @classmethod
    def uniform_from_params(
        cls,
        params: npt.NDArray[np.float_],
        cpg_network_structure: CpgNetworkStructure,
        initial_state_uniform: float,
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> "BrainCpgNetworkStaticWithAmplitude":
        """
        Create brain from parameters.

        params layout: [cpg_weights..., amplitudes...]
        - cpg_weights: num_connections parameters for the weight matrix
        - amplitudes: num_outputs parameters for per-joint amplitudes [0, 1]
        """
        num_cpg_params = cpg_network_structure.num_connections
        num_outputs = len(output_mapping)

        assert len(params) == num_cpg_params + num_outputs, \
            f"Expected {num_cpg_params + num_outputs} params, got {len(params)}"

        cpg_params = params[:num_cpg_params]
        amplitudes = params[num_cpg_params:]

        initial_state = cpg_network_structure.make_uniform_state(initial_state_uniform)
        weight_matrix = cpg_network_structure.make_connection_weights_matrix_from_params(
            list(cpg_params)
        )

        return cls(
            initial_state=initial_state,
            weight_matrix=weight_matrix,
            output_mapping=output_mapping,
            amplitudes=np.array(amplitudes),
        )

    def make_instance(self) -> BrainInstance:
        return BrainCpgInstanceWithAmplitude(
            initial_state=self._initial_state,
            weight_matrix=self._weight_matrix,
            output_mapping=self._output_mapping,
            amplitudes=self._amplitudes,
        )
