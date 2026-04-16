"""
Hopf oscillator brain for Revolve2.

Mirrors the structure of BrainCpgNetworkStatic / BrainCpgInstance but uses
Hopf oscillators where amplitude and frequency are explicit parameters
(no frequency-coupling confound).

Per-oscillator dynamics (polar form written in cartesian):
    x' = alpha * (mu - x^2 - y^2) * x - omega * y + sum_j k_ij * (x_j - x_i)
    y' = alpha * (mu - x^2 - y^2) * y + omega * x + sum_j k_ij * (y_j - y_i)

where:
    mu    = amplitude^2 (the radius of the limit cycle is sqrt(mu))
    omega = angular frequency (rad/s)
    alpha = convergence rate onto the limit cycle (fixed, ~5)
    k_ij  = coupling weight between oscillators i and j (diffusive)

Output mapping: x_i is the "primary" coordinate of oscillator i. The output
sent to the corresponding hinge is x_i (clipped to [-1, 1]) times the hinge
range, same convention as the ODE-CPG.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np
import numpy.typing as npt

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain import Brain, BrainInstance
from revolve2.modular_robot._modular_robot_control_interface import (
    ModularRobotControlInterface,
)
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot.brain.cpg._cpg_network_structure import (
    Cpg,
    CpgNetworkStructure,
    CpgPair,
)


# Convergence rate onto the limit cycle. Higher values make the oscillator
# recover from perturbations faster; 5.0 is a common choice in the literature.
DEFAULT_ALPHA = 5.0


@dataclass
class HopfNetworkStructure:
    """
    Topology of a Hopf oscillator network.

    This is analogous to CpgNetworkStructure but for Hopf oscillators.
    Each Cpg in `oscillators` is one Hopf oscillator (2D state).
    `connections` is the set of oscillator pairs that are diffusively coupled.
    """

    oscillators: list[Cpg]
    connections: set[CpgPair]

    @property
    def num_oscillators(self) -> int:
        return len(self.oscillators)

    @property
    def num_connections(self) -> int:
        return len(self.connections)

    @property
    def num_params(self) -> int:
        """
        Total evolved parameters = internal (one mu per oscillator) + coupling.

        Frequency (omega) is fixed by default so it does not count here.
        If you later decide to evolve omega too, add num_oscillators to this.
        """
        return self.num_oscillators + self.num_connections

    def build_coupling_matrix(
        self, coupling_weights: dict[CpgPair, float]
    ) -> npt.NDArray[np.float_]:
        """
        Build an (n x n) symmetric matrix K where K[i, j] = K[j, i] is the
        coupling weight between oscillators i and j, and K[i, i] = 0.
        """
        n = self.num_oscillators
        K = np.zeros((n, n), dtype=float)
        for pair, w in coupling_weights.items():
            i = pair.cpg_index_lowest.index
            j = pair.cpg_index_highest.index
            K[i, j] = w
            K[j, i] = w
        return K

    def split_params(
        self, params: Iterable[float]
    ) -> tuple[dict[Cpg, float], dict[CpgPair, float]]:
        """
        Split a flat parameter vector into (internal mu per oscillator,
        coupling weight per pair). Ordering matches num_oscillators first,
        then connections in sorted order.
        """
        plist = list(params)
        assert len(plist) == self.num_params, (
            f"Expected {self.num_params} params, got {len(plist)}"
        )
        mu_per_osc: dict[Cpg, float] = {}
        for i, cpg in enumerate(self.oscillators):
            mu_per_osc[cpg] = float(plist[i])
        weight_per_pair: dict[CpgPair, float] = {}
        sorted_pairs = sorted(
            self.connections,
            key=lambda p: (p.cpg_index_lowest.index, p.cpg_index_highest.index),
        )
        for j, pair in enumerate(sorted_pairs):
            weight_per_pair[pair] = float(plist[self.num_oscillators + j])
        return mu_per_osc, weight_per_pair


class BrainHopfStatic(Brain):
    """
    Static Hopf brain: amplitudes (mu) and coupling weights are fixed at
    construction time. Frequency (omega) is a fixed scalar or per-oscillator
    vector. Output is x_i (clipped to [-1, 1]) times hinge range.
    """

    def __init__(
        self,
        mu: npt.NDArray[np.float_],
        omega: npt.NDArray[np.float_] | float,
        coupling_matrix: npt.NDArray[np.float_],
        initial_state: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
        alpha: float = DEFAULT_ALPHA,
    ) -> None:
        """
        :param mu: shape (n,). Amplitude^2 per oscillator. mu >= 0.
        :param omega: either a scalar applied to all oscillators, or shape (n,) per oscillator.
        :param coupling_matrix: shape (n, n). Symmetric diffusive coupling.
        :param initial_state: shape (2n,) stored as [x0, ..., x_{n-1}, y0, ..., y_{n-1}].
        :param output_mapping: maps oscillator index to ActiveHinge.
        :param alpha: convergence rate onto the limit cycle.
        """
        n = len(mu)
        assert coupling_matrix.shape == (n, n)
        assert initial_state.shape == (2 * n,)
        if np.isscalar(omega):
            omega_arr = np.full(n, float(omega))
        else:
            omega_arr = np.asarray(omega, dtype=float)
            assert omega_arr.shape == (n,)
        assert all(0 <= i < n for i, _ in output_mapping)

        self._mu = np.asarray(mu, dtype=float)
        self._omega = omega_arr
        self._K = np.asarray(coupling_matrix, dtype=float)
        self._initial_state = np.asarray(initial_state, dtype=float)
        self._output_mapping = output_mapping
        self._alpha = float(alpha)

    @staticmethod
    def from_params(
        params: Iterable[float],
        network_structure: HopfNetworkStructure,
        output_mapping: list[tuple[int, ActiveHinge]],
        omega: float = 2 * np.pi,  # 1 Hz by default
        alpha: float = DEFAULT_ALPHA,
        initial_phase: float = 0.0,
    ) -> "BrainHopfStatic":
        """
        Build a BrainHopfStatic from a flat CMA-ES parameter vector.

        Parameters:
            params: length num_oscillators + num_connections.
                    First num_oscillators entries are mu per oscillator.
                    Remaining are coupling weights per pair (sorted by pair index).
            network_structure: topology.
            omega: fixed angular frequency (rad/s) applied to all oscillators.
            alpha: limit-cycle convergence rate.
            initial_phase: starting phase for all oscillators (radians).
        """
        mu_per_osc, weight_per_pair = network_structure.split_params(params)

        n = network_structure.num_oscillators
        mu_vec = np.array(
            [mu_per_osc[cpg] for cpg in network_structure.oscillators], dtype=float
        )
        # Clamp mu to [0, +inf). Negative mu would invert the stability of
        # the limit cycle (origin becomes stable), which is usually not what
        # you want. CMA-ES can still try negative values; we just clip here.
        mu_vec = np.clip(mu_vec, 0.0, None)

        K = network_structure.build_coupling_matrix(weight_per_pair)

        # Initial state: each oscillator starts on its limit cycle at
        # phase `initial_phase`. radius r = sqrt(mu).
        r = np.sqrt(mu_vec)
        x0 = r * np.cos(initial_phase)
        y0 = r * np.sin(initial_phase)
        # If mu is 0, x0 = y0 = 0 and the oscillator will stay dead. Seed it
        # with a tiny perturbation so it can be "woken up" if mu > 0 later.
        small = 1e-3
        x0 = np.where(mu_vec > 0, x0, small)
        y0 = np.where(mu_vec > 0, y0, 0.0)
        initial_state = np.concatenate([x0, y0])

        return BrainHopfStatic(
            mu=mu_vec,
            omega=omega,
            coupling_matrix=K,
            initial_state=initial_state,
            output_mapping=output_mapping,
            alpha=alpha,
        )

    def make_instance(self) -> "BrainHopfInstance":
        return BrainHopfInstance(
            mu=self._mu.copy(),
            omega=self._omega.copy(),
            coupling_matrix=self._K.copy(),
            initial_state=self._initial_state.copy(),
            output_mapping=list(self._output_mapping),
            alpha=self._alpha,
        )


class BrainHopfInstance(BrainInstance):
    """
    Stateful Hopf oscillator network.

    State layout: flat vector of length 2n, with [x0, ..., x_{n-1}, y0, ..., y_{n-1}].
    """

    def __init__(
        self,
        mu: npt.NDArray[np.float_],
        omega: npt.NDArray[np.float_],
        coupling_matrix: npt.NDArray[np.float_],
        initial_state: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
        alpha: float,
    ) -> None:
        n = len(mu)
        assert coupling_matrix.shape == (n, n)
        assert initial_state.shape == (2 * n,)
        assert omega.shape == (n,)
        self._n = n
        self._mu = mu
        self._omega = omega
        self._K = coupling_matrix
        self._state = initial_state
        self._output_mapping = output_mapping
        self._alpha = alpha

    def _dynamics(self, state: npt.NDArray[np.float_]) -> npt.NDArray[np.float_]:
        """Compute d(state)/dt for the full 2n-dim Hopf network."""
        n = self._n
        x = state[:n]
        y = state[n:]

        r2 = x * x + y * y
        relax = self._alpha * (self._mu - r2)

        # Diffusive coupling: K @ x - (row_sum(K)) * x
        # Using K as symmetric with zero diagonal: coupling term = K @ x - diag_sum * x
        row_sums = self._K.sum(axis=1)
        cx = self._K @ x - row_sums * x
        cy = self._K @ y - row_sums * y

        dx = relax * x - self._omega * y + cx
        dy = relax * y + self._omega * x + cy

        return np.concatenate([dx, dy])

    def _rk4(
        self, state: npt.NDArray[np.float_], dt: float
    ) -> npt.NDArray[np.float_]:
        """Classical 4th-order Runge-Kutta step."""
        k1 = self._dynamics(state)
        k2 = self._dynamics(state + dt / 2 * k1)
        k3 = self._dynamics(state + dt / 2 * k2)
        k4 = self._dynamics(state + dt * k3)
        return state + dt / 6 * (k1 + 2 * (k2 + k3) + k4)

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """Integrate one step and push hinge targets."""
        self._state = self._rk4(self._state, dt)

        # Send x_i (clipped) scaled to hinge range as the target.
        x = self._state[: self._n]
        for osc_index, active_hinge in self._output_mapping:
            target = float(np.clip(x[osc_index], -1.0, 1.0)) * active_hinge.range
            control_interface.set_active_hinge_target(active_hinge, target)


# ---------------------------------------------------------------------------
# Helpers: build a HopfNetworkStructure from an existing BLF / neighbor /
# uncoupled CpgNetworkStructure. The topology is identical; only the
# dynamics differ, so we can reuse the existing network builders.
# ---------------------------------------------------------------------------


def hopf_structure_from_cpg_structure(
    cpg_structure: CpgNetworkStructure,
) -> HopfNetworkStructure:
    """
    Convert a CpgNetworkStructure (used by the ODE-CPG) into a
    HopfNetworkStructure. The oscillator list and connection set are reused
    unchanged -- the only difference is how dynamics are computed.
    """
    return HopfNetworkStructure(
        oscillators=list(cpg_structure.cpgs),
        connections=set(cpg_structure.connections),
    )


# ===========================================================================
# Polar Hopf (Ijspeert/Bonardi style)
# ---------------------------------------------------------------------------
# Per-oscillator polar dynamics:
#   r_i prime     = alpha * (mu_i - r_i^2) * r_i
#   theta_i prime = omega_i + sum_j w_ij * r_j * sin(theta_j - theta_i - phi_ij)
#
# State is still stored as (x, y) per oscillator so the simulator interface
# is identical to the cartesian variant. Parameters evolved by CMA-ES:
#   mu per oscillator, w per pair, phi per pair.
# Phase-only coupling, amplitude independent. Matches Bonardi 2014.
# ===========================================================================


class BrainHopfPolarStatic(Brain):
    """Polar Hopf brain. Coupling is phase-only with explicit phase offsets."""

    def __init__(
        self,
        mu,
        omega,
        weight_matrix,
        phase_matrix,
        initial_state,
        output_mapping,
        alpha=DEFAULT_ALPHA,
    ):
        n = len(mu)
        assert weight_matrix.shape == (n, n)
        assert phase_matrix.shape == (n, n)
        assert initial_state.shape == (2 * n,)
        if np.isscalar(omega):
            omega_arr = np.full(n, float(omega))
        else:
            omega_arr = np.asarray(omega, dtype=float)
            assert omega_arr.shape == (n,)
        self._mu = np.asarray(mu, dtype=float)
        self._omega = omega_arr
        self._W = np.asarray(weight_matrix, dtype=float)
        self._Phi = np.asarray(phase_matrix, dtype=float)
        self._initial_state = np.asarray(initial_state, dtype=float)
        self._output_mapping = output_mapping
        self._alpha = float(alpha)

    @staticmethod
    def from_params(
        params,
        network_structure,
        output_mapping,
        omega=2 * np.pi,
        alpha=DEFAULT_ALPHA,
    ):
        """
        Build from a flat parameter vector.
        Layout: [mu_0..mu_{n-1}, w_0..w_{nc-1}, phi_0..phi_{nc-1}]
        """
        n = network_structure.num_oscillators
        nc = network_structure.num_connections
        plist = list(params)
        expected = n + 2 * nc
        assert len(plist) == expected, "Expected %d params, got %d" % (expected, len(plist))
        mu_vec = np.clip(np.asarray(plist[:n], dtype=float), 0.0, None)
        w_flat = np.asarray(plist[n:n + nc], dtype=float)
        phi_flat = np.asarray(plist[n + nc:], dtype=float)

        sorted_pairs = sorted(
            network_structure.connections,
            key=lambda p: (p.cpg_index_lowest.index, p.cpg_index_highest.index),
        )
        W = np.zeros((n, n), dtype=float)
        Phi = np.zeros((n, n), dtype=float)
        for k, pair in enumerate(sorted_pairs):
            i = pair.cpg_index_lowest.index
            j = pair.cpg_index_highest.index
            W[i, j] = w_flat[k]
            W[j, i] = w_flat[k]         # symmetric strength
            Phi[i, j] = phi_flat[k]
            Phi[j, i] = -phi_flat[k]    # antisymmetric phase offset

        r = np.sqrt(mu_vec)
        small = 1e-3
        x0 = np.where(mu_vec > 0, r, small)
        y0 = np.where(mu_vec > 0, 0.0, 0.0)
        initial_state = np.concatenate([x0, y0])

        return BrainHopfPolarStatic(
            mu=mu_vec,
            omega=omega,
            weight_matrix=W,
            phase_matrix=Phi,
            initial_state=initial_state,
            output_mapping=output_mapping,
            alpha=alpha,
        )

    def make_instance(self):
        return BrainHopfPolarInstance(
            mu=self._mu.copy(),
            omega=self._omega.copy(),
            weight_matrix=self._W.copy(),
            phase_matrix=self._Phi.copy(),
            initial_state=self._initial_state.copy(),
            output_mapping=list(self._output_mapping),
            alpha=self._alpha,
        )


class BrainHopfPolarInstance(BrainInstance):
    """Stateful polar Hopf oscillator network."""

    def __init__(
        self,
        mu,
        omega,
        weight_matrix,
        phase_matrix,
        initial_state,
        output_mapping,
        alpha,
    ):
        n = len(mu)
        self._n = n
        self._mu = mu
        self._omega = omega
        self._W = weight_matrix
        self._Phi = phase_matrix
        self._state = initial_state
        self._output_mapping = output_mapping
        self._alpha = alpha

    def _dynamics(self, state):
        n = self._n
        x = state[:n]
        y = state[n:]
        r = np.sqrt(x * x + y * y)
        theta = np.arctan2(y, x)
        relax = self._alpha * (self._mu - r * r)

        # Phase coupling vectorized: theta_diff[i, j] = theta_j - theta_i - Phi[i, j]
        theta_diff = theta[None, :] - theta[:, None] - self._Phi
        coupling_term = (self._W * r[None, :] * np.sin(theta_diff)).sum(axis=1)
        Omega = self._omega + coupling_term

        dx = relax * x - Omega * y
        dy = relax * y + Omega * x
        return np.concatenate([dx, dy])

    def _rk4(self, state, dt):
        k1 = self._dynamics(state)
        k2 = self._dynamics(state + dt / 2 * k1)
        k3 = self._dynamics(state + dt / 2 * k2)
        k4 = self._dynamics(state + dt * k3)
        return state + dt / 6 * (k1 + 2 * (k2 + k3) + k4)

    def control(self, dt, sensor_state, control_interface):
        self._state = self._rk4(self._state, dt)
        x = self._state[: self._n]
        for osc_index, active_hinge in self._output_mapping:
            target = float(np.clip(x[osc_index], -1.0, 1.0)) * active_hinge.range
            control_interface.set_active_hinge_target(active_hinge, target)
