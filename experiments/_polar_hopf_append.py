"""Append polar Hopf classes to hopf_brain.py."""
import os

appendix = '''

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
'''

path = os.path.expanduser("~/masteroppgave/revolve2/experiments/hopf_brain.py")
with open(path, "a") as f:
    f.write(appendix)
print("Appended polar Hopf classes to hopf_brain.py")
