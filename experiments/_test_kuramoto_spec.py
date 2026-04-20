"""Spec tests for BrainKuramoto, matching the original task description.

Test 1: Uncoupled produces a clean sine at omega.
Test 2: Coupled pair converges to target Delta.
"""
import math
import numpy as np
from scipy.optimize import curve_fit

from revolve2.modular_robot.brain.cpg._cpg_network_structure import Cpg, CpgPair
from kuramoto_brain import KuramotoNetworkStructure, BrainKuramoto


def _structure(n, connections):
    cpgs = [Cpg(index=i) for i in range(n)]
    pairs = {CpgPair(cpg_1=cpgs[i], cpg_2=cpgs[j]) for i, j in connections}
    return KuramotoNetworkStructure(oscillators=cpgs, connections=pairs)


def _run(brain, T, dt, wrap=True):
    """Step the brain instance for T seconds at dt, return (t, phi_history)."""
    inst = brain.make_instance()
    steps = int(round(T / dt))
    phi_hist = np.zeros((steps + 1, inst._phi.size))
    phi_hist[0] = inst._phi.copy()
    for k in range(steps):
        inst._phi = inst._rk4_step(inst._phi, dt)
        if wrap:
            inst._phi = (inst._phi + np.pi) % (2 * np.pi) - np.pi
        phi_hist[k + 1] = inst._phi.copy()
    t = np.arange(steps + 1) * dt
    return t, phi_hist, inst


def test_1_uncoupled_clean_sine():
    """Spec: A = pi/4, phi0 = 0, no couplings, omega = 0.2 Hz, dt = 0.05, T = 10s.
    Fit a sine to the output; frequency within 1% of 0.2 Hz, amplitude within 1% of pi/4.
    """
    print("Test 1: uncoupled produces clean sine at omega")
    struct = _structure(n=1, connections=[])
    A_target = math.pi / 4
    omega_hz = 0.2
    params = [A_target, 0.0]  # [A_0, phi0_0]
    brain = BrainKuramoto.from_params(params, struct, [], omega_hz=omega_hz)

    T, dt = 10.0, 0.05
    t, phi_hist, _ = _run(brain, T, dt)
    output = A_target * np.sin(phi_hist[:, 0])

    # Fit y(t) = A * sin(2*pi*f*t + phi) to the output.
    def sine_model(tt, A, f, p):
        return A * np.sin(2 * np.pi * f * tt + p)

    popt, _ = curve_fit(sine_model, t, output, p0=[A_target, omega_hz, 0.0])
    A_fit, f_fit, _ = popt
    A_fit = abs(A_fit)  # sign absorbed into phase if needed

    freq_err_pct = abs(f_fit - omega_hz) / omega_hz * 100
    amp_err_pct = abs(A_fit - A_target) / A_target * 100
    print(f"  fitted frequency: {f_fit:.6f} Hz  (target {omega_hz}, err {freq_err_pct:.4f}%)")
    print(f"  fitted amplitude: {A_fit:.6f}    (target {A_target:.6f}, err {amp_err_pct:.4f}%)")

    assert freq_err_pct < 1.0, f"frequency off by {freq_err_pct:.2f}%"
    assert amp_err_pct < 1.0, f"amplitude off by {amp_err_pct:.2f}%"
    print("  PASS")


def test_2_coupled_pair_converges_to_delta():
    """Spec: A = pi/4, phi0_0 = 0, phi0_1 = 0, K = 1, Delta = pi, T = 10s.
    Verify final phase difference is within 0.1 rad of pi.
    """
    print("Test 2: coupled pair converges to target Delta")
    struct = _structure(n=2, connections=[(0, 1)])
    A = math.pi / 4
    # params layout: [A_0, A_1, phi0_0, phi0_1, K_01, Delta_01]
    params = [A, A, 0.0, 0.0, 1.0, math.pi]
    brain = BrainKuramoto.from_params(params, struct, [], omega_hz=0.2)

    T, dt = 10.0, 0.05
    t, phi_hist, _ = _run(brain, T, dt)

    diff = phi_hist[:, 1] - phi_hist[:, 0]
    final_diff = (diff[-1] + np.pi) % (2 * np.pi) - np.pi
    err = abs((final_diff - math.pi + np.pi) % (2 * np.pi) - np.pi)
    print(f"  initial phi_1 - phi_0 = {diff[0]:.6e}")
    print(f"  final   phi_1 - phi_0 = {final_diff:.6f}  (target {math.pi:.6f})")
    print(f"  error                 = {err:.6e} rad")

    assert err < 0.1, f"did not converge: err = {err:.3e} rad"
    print("  PASS")


def main():
    print("=" * 60)
    test_1_uncoupled_clean_sine()
    print()
    test_2_coupled_pair_converges_to_delta()
    print("=" * 60)
    print("Spec tests passed.")


if __name__ == "__main__":
    main()
