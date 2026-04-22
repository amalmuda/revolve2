"""Bonardi-style phase-amplitude CPG for Revolve2.

Adapted from Bonardi et al. (2014), "Automatic generation of reduced CPG
control networks for locomotion of arbitrary modular robot structures".

Dynamics (standard form, with frequency ADDED — the paper's eq. 1 has
a typo that multiplies ν with the coupling sum, which breaks the
uncoupled case):

    dphi_i/dt = 2*pi*nu + sum_j w_ij * A_j * sin(phi_j - phi_i - psi_ij)
    theta_i = A_i * sin(phi_i) + X_i

Simplifications vs. the paper:
  - No dynamic amplitude (skip the ṙ = a(A - r) equation). r_j is
    replaced by A_j directly, which matches the steady state of the
    original formulation.
  - Uniform joint-type bounds (Revolve2 V1 hinges all have the same
    physical range, so the paper's joint-type-specific amplitude
    bounds are not needed).

Fixed (not evolved):
    nu         shared natural frequency (Hz)
    w_ij       all-edge uniform coupling strength
    phi_i(0)   initial phases, set to 0 for all oscillators
    psi_ji = -psi_ij  (antisymmetric constraint)

Evolved parameters (flat vector of length 2*n + nc):
    A_i      in [0, pi/3]       amplitude per joint            (n values)
    X_i      in [-pi/3, pi/3]   output offset per joint        (n values)
    psi_ij   in [0, 2*pi]       phase lag per edge             (nc values)

Link ordering is sorted by (lowest_index, highest_index) of the CpgPair.
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


DEFAULT_NU_HZ = 1.0
DEFAULT_W = 1.0
A_MAX = np.pi / 3        # 1.047 rad, matches V1 hinge range
X_MAX = np.pi / 3        # symmetric offset range
PSI_MAX = 2 * np.pi      # full circle


@dataclass
class BonardiNetworkStructure:
    """Topology container for a Bonardi-style CPG.

    Supports four variants via flags:
      - evolve_phi0: True = evolve initial phases per oscillator (+n params)
      - evolve_w:    True = evolve per-edge coupling weights (+nc params)

    Base evolved params (always): A (n), X (n), psi (nc).
    Full layout: [A, X, psi, phi0?, w?]
    """

    oscillators: list[Cpg]
    connections: set[CpgPair]
    evolve_phi0: bool = False
    evolve_w: bool = False

    @property
    def num_oscillators(self) -> int:
        return len(self.oscillators)

    @property
    def num_connections(self) -> int:
        return len(self.connections)

    @property
    def num_params(self) -> int:
        n, nc = self.num_oscillators, self.num_connections
        p = 2 * n + nc  # A, X, psi always evolved
        if self.evolve_phi0:
            p += n
        if self.evolve_w:
            p += nc
        return p

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
        npt.NDArray[np.float_] | None,
        npt.NDArray[np.float_] | None,
    ]:
        """Split a flat parameter vector into (A, X, psi, phi0, w).

        phi0 is None if not evolved (brain will use zeros).
        w is None if not evolved (brain will use DEFAULT_W).
        """
        plist = np.asarray(list(params), dtype=float)
        n, nc = self.num_oscillators, self.num_connections
        expected = self.num_params
        assert plist.size == expected, f"Expected {expected} params, got {plist.size}"
        offset = 0
        A = plist[offset:offset + n]; offset += n
        X = plist[offset:offset + n]; offset += n
        psi_flat = plist[offset:offset + nc]; offset += nc
        phi0 = None
        if self.evolve_phi0:
            phi0 = plist[offset:offset + n]
            offset += n
        w_flat = None
        if self.evolve_w:
            w_flat = plist[offset:offset + nc]
            offset += nc
        return A, X, psi_flat, phi0, w_flat

    def build_psi_matrix(
        self, psi_flat: npt.NDArray[np.float_]
    ) -> npt.NDArray[np.float_]:
        """Build antisymmetric (n, n) psi matrix from flat per-edge values."""
        n = self.num_oscillators
        psi = np.zeros((n, n), dtype=float)
        for k, pair in enumerate(self._sorted_pairs()):
            i = pair.cpg_index_lowest.index
            j = pair.cpg_index_highest.index
            psi[i, j] = psi_flat[k]
            psi[j, i] = -psi_flat[k]
        return psi

    def build_w_matrix(
        self, w_default: float, w_flat: npt.NDArray[np.float_] | None = None
    ) -> npt.NDArray[np.float_]:
        """Build coupling-strength matrix. Uses w_flat if provided, else uniform w_default."""
        n = self.num_oscillators
        W = np.zeros((n, n), dtype=float)
        for k, pair in enumerate(self._sorted_pairs()):
            i = pair.cpg_index_lowest.index
            j = pair.cpg_index_highest.index
            w_ij = float(w_flat[k]) if w_flat is not None else w_default
            W[i, j] = w_ij
            W[j, i] = w_ij
        return W


class BrainBonardi(Brain):
    """Static Bonardi-style brain. All parameters frozen at construction."""

    def __init__(
        self,
        A: npt.NDArray[np.float_],
        X: npt.NDArray[np.float_],
        phi0: npt.NDArray[np.float_],
        psi: npt.NDArray[np.float_],
        W: npt.NDArray[np.float_],
        nu_hz: float,
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        n = A.size
        assert X.shape == (n,)
        assert phi0.shape == (n,)
        assert psi.shape == (n, n)
        assert W.shape == (n, n)
        self._A = np.asarray(A, dtype=float)
        self._X = np.asarray(X, dtype=float)
        self._phi0 = np.asarray(phi0, dtype=float)
        self._psi = np.asarray(psi, dtype=float)
        self._W = np.asarray(W, dtype=float)
        self._nu_hz = float(nu_hz)
        self._output_mapping = output_mapping

    @staticmethod
    def from_params(
        params: Iterable[float],
        network_structure: BonardiNetworkStructure,
        output_mapping: list[tuple[int, ActiveHinge]],
        nu_hz: float = DEFAULT_NU_HZ,
        w: float = DEFAULT_W,
    ) -> "BrainBonardi":
        A, X, psi_flat, phi0_opt, w_flat = network_structure.split_params(params)
        psi = network_structure.build_psi_matrix(psi_flat)
        W = network_structure.build_w_matrix(w, w_flat)
        n = network_structure.num_oscillators
        phi0 = phi0_opt if phi0_opt is not None else np.zeros(n, dtype=float)
        return BrainBonardi(
            A=A, X=X, phi0=phi0, psi=psi, W=W,
            nu_hz=nu_hz, output_mapping=output_mapping,
        )

    def make_instance(self) -> "BrainBonardiInstance":
        return BrainBonardiInstance(
            A=self._A.copy(),
            X=self._X.copy(),
            psi=self._psi.copy(),
            W=self._W.copy(),
            nu_hz=self._nu_hz,
            phases=self._phi0.copy(),
            output_mapping=list(self._output_mapping),
        )


class BrainBonardiInstance(BrainInstance):
    """Runtime instance. Integrates phases with RK4. Amplitudes/offsets static."""

    def __init__(
        self,
        A: npt.NDArray[np.float_],
        X: npt.NDArray[np.float_],
        psi: npt.NDArray[np.float_],
        W: npt.NDArray[np.float_],
        nu_hz: float,
        phases: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        self._A = A
        self._X = X
        self._psi = psi
        self._W = W
        self._nu_rad = 2.0 * np.pi * nu_hz
        self._phi = phases
        self._output_mapping = output_mapping

    def _dphi(self, phi: npt.NDArray[np.float_]) -> npt.NDArray[np.float_]:
        # diff[i, j] = phi[j] - phi[i] - psi[i, j]
        diff = phi[None, :] - phi[:, None] - self._psi
        # Coupling term weights by A_j so larger-amplitude oscillators pull
        # stronger, matching the paper's r_j factor at steady state.
        coupling = (self._W * self._A[None, :] * np.sin(diff)).sum(axis=1)
        return self._nu_rad + coupling

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
        # Keep phases bounded for numeric stability.
        self._phi = (self._phi + np.pi) % (2.0 * np.pi) - np.pi

        for osc_index, hinge in self._output_mapping:
            target = float(
                self._A[osc_index] * np.sin(self._phi[osc_index])
                + self._X[osc_index]
            )
            control_interface.set_active_hinge_target(hinge, target)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def bonardi_structure_from_cpg_structure(
    cpg_structure: CpgNetworkStructure,
    evolve_phi0: bool = False,
    evolve_w: bool = False,
) -> BonardiNetworkStructure:
    """Reuse existing BLF / neighbour / uncoupled topology builders."""
    return BonardiNetworkStructure(
        oscillators=list(cpg_structure.cpgs),
        connections=set(cpg_structure.connections),
        evolve_phi0=evolve_phi0,
        evolve_w=evolve_w,
    )


W_MAX = 2.0  # upper bound on per-edge coupling strength when evolved


def param_bounds(
    network_structure: BonardiNetworkStructure,
) -> tuple[list[float], list[float]]:
    """Native parameter bounds aligned with the parameter layout.

    Order: [A (n), X (n), psi (nc), phi0 (n)?, w (nc)?]
    """
    n, nc = network_structure.num_oscillators, network_structure.num_connections
    lower = [0.0] * n + [-X_MAX] * n + [0.0] * nc
    upper = [A_MAX] * n + [X_MAX] * n + [PSI_MAX] * nc
    if network_structure.evolve_phi0:
        lower += [0.0] * n
        upper += [PSI_MAX] * n
    if network_structure.evolve_w:
        lower += [0.0] * nc
        upper += [W_MAX] * nc
    return lower, upper
