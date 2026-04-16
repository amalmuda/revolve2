"""
Kuramoto oscillator brain for Revolve2.

Pure phase model: each active hinge controlled by one phase variable theta.
    dtheta_i/dt = omega + sum_j K_ij * sin(theta_j - theta_i - phi_ij)

Joint target = sin(theta_i) * hinge_range.

Evolved parameters (per pair):
    K_ij   coupling weight (symmetric: K_ij = K_ji)
    phi_ij phase offset    (antisymmetric: phi_ij = -phi_ji)

omega is fixed across all oscillators (not evolved). No amplitude state, no
limit-cycle dynamics. The simplest possible coupled-oscillator CPG.
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


@dataclass
class KuramotoNetworkStructure:
    """Topology of a Kuramoto oscillator network. Mirrors HopfNetworkStructure."""

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
        """Total evolved params = 2 per connection (weight + phase offset)."""
        return 2 * self.num_connections

    def split_params(
        self, params: Iterable[float]
    ) -> tuple[npt.NDArray[np.float_], npt.NDArray[np.float_]]:
        """
        Split a flat parameter vector into (weights, phase offsets).

        Layout: [w_0, ..., w_{nc-1}, phi_0, ..., phi_{nc-1}]
        Pair ordering: sorted by (lowest_index, highest_index).
        """
        plist = np.asarray(list(params), dtype=float)
        nc = self.num_connections
        assert plist.size == 2 * nc, f"Expected {2*nc} params, got {plist.size}"
        return plist[:nc], plist[nc:]

    def build_matrices(
        self,
        weights: npt.NDArray[np.float_],
        phase_offsets: npt.NDArray[np.float_],
    ) -> tuple[npt.NDArray[np.float_], npt.NDArray[np.float_]]:
        """
        Build (n x n) coupling matrix K (symmetric) and phase offset matrix
        Phi (antisymmetric). K[i, i] = 0; Phi[i, i] = 0.
        """
        n = self.num_oscillators
        K = np.zeros((n, n), dtype=float)
        Phi = np.zeros((n, n), dtype=float)
        sorted_pairs = sorted(
            self.connections,
            key=lambda p: (p.cpg_index_lowest.index, p.cpg_index_highest.index),
        )
        for k, pair in enumerate(sorted_pairs):
            i = pair.cpg_index_lowest.index
            j = pair.cpg_index_highest.index
            K[i, j] = weights[k]
            K[j, i] = weights[k]                # symmetric
            Phi[i, j] = phase_offsets[k]
            Phi[j, i] = -phase_offsets[k]       # antisymmetric
        return K, Phi


class BrainKuramoto(Brain):
    """Static Kuramoto brain. Coupling weights and phase offsets fixed at
    construction time. omega is a fixed scalar shared by all oscillators."""

    def __init__(
        self,
        K: npt.NDArray[np.float_],
        Phi: npt.NDArray[np.float_],
        omega: float,
        initial_phases: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        n = K.shape[0]
        assert K.shape == (n, n)
        assert Phi.shape == (n, n)
        assert initial_phases.shape == (n,)
        self._K = np.asarray(K, dtype=float)
        self._Phi = np.asarray(Phi, dtype=float)
        self._omega = float(omega)
        self._initial_phases = np.asarray(initial_phases, dtype=float)
        self._output_mapping = output_mapping

    @staticmethod
    def from_params(
        params: Iterable[float],
        network_structure: KuramotoNetworkStructure,
        output_mapping: list[tuple[int, ActiveHinge]],
        omega: float = 2 * np.pi,           # 1 Hz default
        initial_phases: npt.NDArray[np.float_] | None = None,
    ) -> "BrainKuramoto":
        weights, phase_offsets = network_structure.split_params(params)
        K, Phi = network_structure.build_matrices(weights, phase_offsets)
        n = network_structure.num_oscillators
        if initial_phases is None:
            initial_phases = np.zeros(n, dtype=float)
        else:
            initial_phases = np.asarray(initial_phases, dtype=float)
            assert initial_phases.shape == (n,)
        return BrainKuramoto(
            K=K, Phi=Phi, omega=omega,
            initial_phases=initial_phases,
            output_mapping=output_mapping,
        )

    def make_instance(self) -> "BrainKuramotoInstance":
        return BrainKuramotoInstance(
            K=self._K.copy(), Phi=self._Phi.copy(),
            omega=self._omega,
            phases=self._initial_phases.copy(),
            output_mapping=list(self._output_mapping),
        )


class BrainKuramotoInstance(BrainInstance):
    """Stateful Kuramoto network. State = (n,) phase vector."""

    def __init__(
        self,
        K: npt.NDArray[np.float_],
        Phi: npt.NDArray[np.float_],
        omega: float,
        phases: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        self._K = K
        self._Phi = Phi
        self._omega = omega
        self._theta = phases
        self._output_mapping = output_mapping
        self._n = K.shape[0]

    def _dynamics(self, theta: npt.NDArray[np.float_]) -> npt.NDArray[np.float_]:
        """dtheta/dt = omega + sum_j K_ij * sin(theta_j - theta_i - Phi_ij)."""
        # theta_diff[i, j] = theta[j] - theta[i] - Phi[i, j]
        theta_diff = theta[None, :] - theta[:, None] - self._Phi
        coupling = (self._K * np.sin(theta_diff)).sum(axis=1)
        return self._omega + coupling

    def _rk4(
        self, theta: npt.NDArray[np.float_], dt: float
    ) -> npt.NDArray[np.float_]:
        k1 = self._dynamics(theta)
        k2 = self._dynamics(theta + dt / 2 * k1)
        k3 = self._dynamics(theta + dt / 2 * k2)
        k4 = self._dynamics(theta + dt * k3)
        return theta + dt / 6 * (k1 + 2 * (k2 + k3) + k4)

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        self._theta = self._rk4(self._theta, dt)
        # Wrap phases to [-pi, pi] periodically to prevent unbounded growth.
        self._theta = (self._theta + np.pi) % (2 * np.pi) - np.pi

        for osc_index, hinge in self._output_mapping:
            target = float(np.sin(self._theta[osc_index])) * hinge.range
            control_interface.set_active_hinge_target(hinge, target)


# ---------------------------------------------------------------------------
# Helper: build KuramotoNetworkStructure from existing CpgNetworkStructure
# (so we can reuse the BLF / neighbor / uncoupled topology builders).
# ---------------------------------------------------------------------------


def kuramoto_structure_from_cpg_structure(
    cpg_structure: CpgNetworkStructure,
) -> KuramotoNetworkStructure:
    return KuramotoNetworkStructure(
        oscillators=list(cpg_structure.cpgs),
        connections=set(cpg_structure.connections),
    )
