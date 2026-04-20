"""Kuramoto phase-oscillator CPG for Revolve2.

One phase variable per active hinge. Shared fixed natural frequency.
Per-joint amplitude and initial phase are evolved. Per-link coupling
weight and phase offset are evolved. Frequency is decoupled from
coupling strength by construction.

Dynamics
--------
    dphi_i/dt = 2*pi*omega + sum_j K_ij * sin(phi_j - phi_i - Delta_ij)

Output
------
    theta_i = A_i * sin(phi_i)

Evolved parameters
------------------
    Per joint:      A_i     in [0, pi/3]       (output amplitude)
                    phi0_i  in [0, 2*pi]       (initial phase)
    Per link (i,j): K_ij    in [0, 2]          (coupling strength)
                    Delta_ij in [0, 2*pi]      (target phase offset)

K is symmetric (K_ji = K_ij). Delta is antisymmetric (Delta_ji = -Delta_ij).

Fixed
-----
    omega: shared natural frequency in Hz (default 0.2).

Parameter layout (flat vector of length 2n + 2*nc)
--------------------------------------------------
    [A_0, ..., A_{n-1}, phi0_0, ..., phi0_{n-1},
     K_0, ..., K_{nc-1}, Delta_0, ..., Delta_{nc-1}]

Link ordering is sorted by (lowest_index, highest_index) of the
CpgPair, matching the conventions used by the Hopf brain.
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


# Defaults matching the thesis spec.
DEFAULT_OMEGA_HZ = 0.2
A_MAX = np.pi / 3  # 1.047 rad ~= V1 hinge range
K_MAX = 2.0


@dataclass
class KuramotoNetworkStructure:
    """Topology container for a Kuramoto CPG."""

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
        return 2 * self.num_oscillators + 2 * self.num_connections

    def _sorted_pairs(self) -> list[CpgPair]:
        return sorted(
            self.connections,
            key=lambda p: (p.cpg_index_lowest.index, p.cpg_index_highest.index),
        )

    def split_params(
        self, params: Iterable[float]
    ) -> tuple[
        npt.NDArray[np.float_],
        npt.NDArray[np.float_],
        npt.NDArray[np.float_],
        npt.NDArray[np.float_],
    ]:
        """Split a flat parameter vector into (A, phi0, K, Delta)."""
        plist = np.asarray(list(params), dtype=float)
        n, nc = self.num_oscillators, self.num_connections
        expected = 2 * n + 2 * nc
        assert plist.size == expected, f"Expected {expected} params, got {plist.size}"
        A = plist[:n]
        phi0 = plist[n : 2 * n]
        K_flat = plist[2 * n : 2 * n + nc]
        Delta_flat = plist[2 * n + nc :]
        return A, phi0, K_flat, Delta_flat

    def build_matrices(
        self,
        K_flat: npt.NDArray[np.float_],
        Delta_flat: npt.NDArray[np.float_],
    ) -> tuple[npt.NDArray[np.float_], npt.NDArray[np.float_]]:
        """Build (n,n) K (symmetric) and Delta (antisymmetric) matrices."""
        n = self.num_oscillators
        K = np.zeros((n, n), dtype=float)
        D = np.zeros((n, n), dtype=float)
        for k, pair in enumerate(self._sorted_pairs()):
            i = pair.cpg_index_lowest.index
            j = pair.cpg_index_highest.index
            K[i, j] = K_flat[k]
            K[j, i] = K_flat[k]
            D[i, j] = Delta_flat[k]
            D[j, i] = -Delta_flat[k]
        return K, D


class BrainKuramoto(Brain):
    """Static Kuramoto brain. All parameters frozen at construction."""

    def __init__(
        self,
        A: npt.NDArray[np.float_],
        phi0: npt.NDArray[np.float_],
        K: npt.NDArray[np.float_],
        Delta: npt.NDArray[np.float_],
        omega_hz: float,
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        n = A.size
        assert phi0.shape == (n,)
        assert K.shape == (n, n)
        assert Delta.shape == (n, n)
        self._A = np.asarray(A, dtype=float)
        self._phi0 = np.asarray(phi0, dtype=float)
        self._K = np.asarray(K, dtype=float)
        self._Delta = np.asarray(Delta, dtype=float)
        self._omega_hz = float(omega_hz)
        self._output_mapping = output_mapping

    @staticmethod
    def from_params(
        params: Iterable[float],
        network_structure: KuramotoNetworkStructure,
        output_mapping: list[tuple[int, ActiveHinge]],
        omega_hz: float = DEFAULT_OMEGA_HZ,
    ) -> "BrainKuramoto":
        A, phi0, K_flat, Delta_flat = network_structure.split_params(params)
        K, D = network_structure.build_matrices(K_flat, Delta_flat)
        return BrainKuramoto(
            A=A, phi0=phi0, K=K, Delta=D,
            omega_hz=omega_hz, output_mapping=output_mapping,
        )

    def make_instance(self) -> "BrainKuramotoInstance":
        return BrainKuramotoInstance(
            A=self._A.copy(),
            K=self._K.copy(),
            Delta=self._Delta.copy(),
            omega_hz=self._omega_hz,
            phases=self._phi0.copy(),
            output_mapping=list(self._output_mapping),
        )


class BrainKuramotoInstance(BrainInstance):
    """Runtime instance. Integrates phases with RK4."""

    def __init__(
        self,
        A: npt.NDArray[np.float_],
        K: npt.NDArray[np.float_],
        Delta: npt.NDArray[np.float_],
        omega_hz: float,
        phases: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        self._A = A
        self._K = K
        self._Delta = Delta
        self._omega_rad = 2.0 * np.pi * omega_hz
        self._phi = phases
        self._output_mapping = output_mapping

    def _dphi(self, phi: npt.NDArray[np.float_]) -> npt.NDArray[np.float_]:
        # diff[i, j] = phi[j] - phi[i] - Delta[i, j]
        diff = phi[None, :] - phi[:, None] - self._Delta
        coupling = (self._K * np.sin(diff)).sum(axis=1)
        return self._omega_rad + coupling

    def _rk4_step(
        self, phi: npt.NDArray[np.float_], dt: float
    ) -> npt.NDArray[np.float_]:
        k1 = self._dphi(phi)
        k2 = self._dphi(phi + 0.5 * dt * k1)
        k3 = self._dphi(phi + 0.5 * dt * k2)
        k4 = self._dphi(phi + dt * k3)
        return phi + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        self._phi = self._rk4_step(self._phi, dt)
        # Wrap to [-pi, pi) to keep the numeric range bounded.
        self._phi = (self._phi + np.pi) % (2.0 * np.pi) - np.pi

        for osc_index, hinge in self._output_mapping:
            target = float(self._A[osc_index] * np.sin(self._phi[osc_index]))
            control_interface.set_active_hinge_target(hinge, target)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def kuramoto_structure_from_cpg_structure(
    cpg_structure: CpgNetworkStructure,
) -> KuramotoNetworkStructure:
    """Reuse existing BLF / neighbour / uncoupled topology builders."""
    return KuramotoNetworkStructure(
        oscillators=list(cpg_structure.cpgs),
        connections=set(cpg_structure.connections),
    )


def param_bounds(
    network_structure: KuramotoNetworkStructure,
) -> tuple[list[float], list[float]]:
    """CMA-ES bounds aligned with the parameter layout."""
    n, nc = network_structure.num_oscillators, network_structure.num_connections
    lower = [0.0] * n + [0.0] * n + [0.0] * nc + [0.0] * nc
    upper = [A_MAX] * n + [2 * np.pi] * n + [K_MAX] * nc + [2 * np.pi] * nc
    return lower, upper
