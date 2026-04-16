"""
Ijspeert-style Amplitude-Controlled Phase Oscillator (ACPO) brain.

Same family as Bonardi 2014 (eqs 1-3), but with coupling weights w_ij
evolved instead of fixed at 2.

Per oscillator dynamics:
    phi_i' = omega + sum_j w_ij * r_j * sin(phi_j - phi_i - psi_ij)
    r_i'   = a * (A_i - r_i)
    theta_i = clip(r_i * sin(phi_i) + X_i, -hinge_range, +hinge_range)

Fixed (not evolved):
    omega   - common angular frequency for all oscillators
    a       - amplitude convergence rate (linear)

Evolved per oscillator:
    A_i     - amplitude target (limit cycle radius for r_i)
    X_i     - offset (joint angle bias)

Evolved per coupling pair:
    w_ij    - coupling weight (symmetric)
    psi_ij  - phase lag (antisymmetric)
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


DEFAULT_AMP_CONVERGENCE = 5.0  # a_i; how fast r_i tracks A_i


@dataclass
class AcpoNetworkStructure:
    """Topology for an ACPO network. Mirrors Kuramoto/Hopf network structures."""

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
        """2 per oscillator (A, X) + 2 per coupling (w, psi)."""
        return 2 * self.num_oscillators + 2 * self.num_connections

    def split_params(
        self, params: Iterable[float]
    ) -> tuple[
        npt.NDArray[np.float_],   # A
        npt.NDArray[np.float_],   # X
        npt.NDArray[np.float_],   # w
        npt.NDArray[np.float_],   # psi
    ]:
        """Layout: [A_0..A_{n-1}, X_0..X_{n-1}, w_0..w_{nc-1}, psi_0..psi_{nc-1}]."""
        plist = np.asarray(list(params), dtype=float)
        n = self.num_oscillators
        nc = self.num_connections
        assert plist.size == 2 * n + 2 * nc, (
            f"Expected {2*n + 2*nc} params, got {plist.size}"
        )
        A = plist[:n]
        X = plist[n:2 * n]
        w = plist[2 * n:2 * n + nc]
        psi = plist[2 * n + nc:]
        return A, X, w, psi

    def build_matrices(
        self,
        weights: npt.NDArray[np.float_],
        phase_lags: npt.NDArray[np.float_],
    ) -> tuple[npt.NDArray[np.float_], npt.NDArray[np.float_]]:
        n = self.num_oscillators
        W = np.zeros((n, n), dtype=float)
        Psi = np.zeros((n, n), dtype=float)
        sorted_pairs = sorted(
            self.connections,
            key=lambda p: (p.cpg_index_lowest.index, p.cpg_index_highest.index),
        )
        for k, pair in enumerate(sorted_pairs):
            i = pair.cpg_index_lowest.index
            j = pair.cpg_index_highest.index
            W[i, j] = weights[k]
            W[j, i] = weights[k]                # symmetric
            Psi[i, j] = phase_lags[k]
            Psi[j, i] = -phase_lags[k]          # antisymmetric
        return W, Psi


class BrainAcpo(Brain):
    """Static ACPO brain. All evolved parameters fixed at construction."""

    def __init__(
        self,
        A: npt.NDArray[np.float_],
        X: npt.NDArray[np.float_],
        W: npt.NDArray[np.float_],
        Psi: npt.NDArray[np.float_],
        omega: float,
        a_conv: float,
        initial_phases: npt.NDArray[np.float_],
        initial_radii: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        n = A.shape[0]
        assert X.shape == (n,)
        assert W.shape == (n, n)
        assert Psi.shape == (n, n)
        assert initial_phases.shape == (n,)
        assert initial_radii.shape == (n,)
        self._A = np.asarray(A, dtype=float)
        self._X = np.asarray(X, dtype=float)
        self._W = np.asarray(W, dtype=float)
        self._Psi = np.asarray(Psi, dtype=float)
        self._omega = float(omega)
        self._a_conv = float(a_conv)
        self._init_phi = np.asarray(initial_phases, dtype=float)
        self._init_r = np.asarray(initial_radii, dtype=float)
        self._output_mapping = output_mapping

    @staticmethod
    def from_params(
        params: Iterable[float],
        network_structure: AcpoNetworkStructure,
        output_mapping: list[tuple[int, ActiveHinge]],
        omega: float = 2 * np.pi,             # 1 Hz default
        a_conv: float = DEFAULT_AMP_CONVERGENCE,
        initial_phases: npt.NDArray[np.float_] | None = None,
    ) -> "BrainAcpo":
        A, X, w, psi = network_structure.split_params(params)
        # Clip A to non-negative; negative amplitude is unphysical (the linear
        # relaxation eq would still work with A<0 but the output meaning breaks).
        A = np.clip(A, 0.0, None)
        W_mat, Psi_mat = network_structure.build_matrices(w, psi)
        n = network_structure.num_oscillators
        if initial_phases is None:
            initial_phases = np.zeros(n, dtype=float)
        else:
            initial_phases = np.asarray(initial_phases, dtype=float)
            assert initial_phases.shape == (n,)
        # Start radii on the limit cycle so amplitude doesn't ramp up from 0.
        initial_radii = A.copy()
        return BrainAcpo(
            A=A, X=X, W=W_mat, Psi=Psi_mat,
            omega=omega, a_conv=a_conv,
            initial_phases=initial_phases,
            initial_radii=initial_radii,
            output_mapping=output_mapping,
        )

    def make_instance(self) -> "BrainAcpoInstance":
        return BrainAcpoInstance(
            A=self._A.copy(), X=self._X.copy(),
            W=self._W.copy(), Psi=self._Psi.copy(),
            omega=self._omega, a_conv=self._a_conv,
            phi=self._init_phi.copy(), r=self._init_r.copy(),
            output_mapping=list(self._output_mapping),
        )


class BrainAcpoInstance(BrainInstance):
    """Stateful ACPO. State = (phi, r) concatenated, length 2n."""

    def __init__(
        self,
        A: npt.NDArray[np.float_],
        X: npt.NDArray[np.float_],
        W: npt.NDArray[np.float_],
        Psi: npt.NDArray[np.float_],
        omega: float,
        a_conv: float,
        phi: npt.NDArray[np.float_],
        r: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        n = A.shape[0]
        self._n = n
        self._A = A
        self._X = X
        self._W = W
        self._Psi = Psi
        self._omega = omega
        self._a = a_conv
        self._phi = phi
        self._r = r
        self._output_mapping = output_mapping

    def _dynamics(
        self, phi: npt.NDArray[np.float_], r: npt.NDArray[np.float_]
    ) -> tuple[npt.NDArray[np.float_], npt.NDArray[np.float_]]:
        # Phase: dphi/dt = omega + sum_j W_ij * r_j * sin(phi_j - phi_i - Psi_ij)
        phi_diff = phi[None, :] - phi[:, None] - self._Psi
        coupling = (self._W * r[None, :] * np.sin(phi_diff)).sum(axis=1)
        dphi = self._omega + coupling
        # Amplitude: dr/dt = a * (A - r)
        dr = self._a * (self._A - r)
        return dphi, dr

    def _rk4(
        self,
        phi: npt.NDArray[np.float_],
        r: npt.NDArray[np.float_],
        dt: float,
    ) -> tuple[npt.NDArray[np.float_], npt.NDArray[np.float_]]:
        k1_phi, k1_r = self._dynamics(phi, r)
        k2_phi, k2_r = self._dynamics(phi + dt / 2 * k1_phi, r + dt / 2 * k1_r)
        k3_phi, k3_r = self._dynamics(phi + dt / 2 * k2_phi, r + dt / 2 * k2_r)
        k4_phi, k4_r = self._dynamics(phi + dt * k3_phi, r + dt * k3_r)
        new_phi = phi + dt / 6 * (k1_phi + 2 * (k2_phi + k3_phi) + k4_phi)
        new_r = r + dt / 6 * (k1_r + 2 * (k2_r + k3_r) + k4_r)
        return new_phi, new_r

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        self._phi, self._r = self._rk4(self._phi, self._r, dt)
        # Wrap phases to [-pi, pi] to avoid unbounded growth.
        self._phi = (self._phi + np.pi) % (2 * np.pi) - np.pi

        # Output: theta_i = r_i * sin(phi_i) + X_i, clipped to hinge range.
        for osc_index, hinge in self._output_mapping:
            target = float(self._r[osc_index]) * float(np.sin(self._phi[osc_index])) + float(self._X[osc_index])
            target = max(-hinge.range, min(hinge.range, target))
            control_interface.set_active_hinge_target(hinge, target)


def acpo_structure_from_cpg_structure(
    cpg_structure: CpgNetworkStructure,
) -> AcpoNetworkStructure:
    return AcpoNetworkStructure(
        oscillators=list(cpg_structure.cpgs),
        connections=set(cpg_structure.connections),
    )
