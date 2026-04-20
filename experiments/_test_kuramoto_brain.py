"""Sanity tests for the Kuramoto CPG.

Test 1: Uncoupled oscillator produces a clean sine at omega.
Test 2: Coupled pair converges to the target phase offset Delta.
"""
import math
import numpy as np

from kuramoto_brain import (
    KuramotoNetworkStructure,
    BrainKuramoto,
    DEFAULT_OMEGA_HZ,
    A_MAX,
)
from revolve2.modular_robot.brain.cpg._cpg_network_structure import (
    Cpg,
    CpgPair,
    CpgNetworkStructure,
)


def _make_structure(n: int, connections: list[tuple[int, int]]) -> KuramotoNetworkStructure:
    cpgs = [Cpg(index=i) for i in range(n)]
    pairs = set()
    for i, j in connections:
        lo, hi = sorted((i, j))
        pairs.add(CpgPair(cpg_1=cpgs[lo], cpg_2=cpgs[hi]))
    return KuramotoNetworkStructure(oscillators=cpgs, connections=pairs)


def _integrate(brain: BrainKuramoto, T: float, dt: float):
    """Run the RK4 loop without the Revolve2 control interface; record phi."""
    inst = brain.make_instance()
    steps = int(round(T / dt))
    phi_history = np.zeros((steps + 1, inst._phi.size))
    phi_history[0] = inst._phi.copy()
    for k in range(steps):
        inst._phi = inst._rk4_step(inst._phi, dt)
        # Don't wrap here — we want raw phase so we can check the drift rate.
        phi_history[k + 1] = inst._phi.copy()
    t = np.arange(steps + 1) * dt
    return t, phi_history


def test1_uncoupled_clean_sine():
    print("Test 1: uncoupled produces a clean sine at omega")
    # Two uncoupled oscillators, same natural frequency.
    struct = _make_structure(n=2, connections=[])
    omega = DEFAULT_OMEGA_HZ  # 0.2 Hz
    # params: [A_0, A_1, phi0_0, phi0_1]  (no coupling params)
    params = [A_MAX, A_MAX, 0.0, math.pi / 4]
    brain = BrainKuramoto.from_params(
        params=params, network_structure=struct,
        output_mapping=[], omega_hz=omega,
    )

    T, dt = 20.0, 0.01  # 20 s simulated, 100 Hz integration
    t, phi = _integrate(brain, T, dt)

    # With no coupling, phi_i(t) = 2*pi*omega*t + phi0_i exactly.
    expected = 2 * np.pi * omega * t
    drift_0 = phi[:, 0] - (expected + 0.0)
    drift_1 = phi[:, 1] - (expected + math.pi / 4)

    max_drift_0 = float(np.max(np.abs(drift_0)))
    max_drift_1 = float(np.max(np.abs(drift_1)))
    print(f"  max |phi_0(t) - 2pi*omega*t| = {max_drift_0:.2e}")
    print(f"  max |phi_1(t) - 2pi*omega*t - pi/4| = {max_drift_1:.2e}")
    assert max_drift_0 < 1e-6, f"osc 0 not pure sine at omega: drift={max_drift_0}"
    assert max_drift_1 < 1e-6, f"osc 1 not pure sine at omega: drift={max_drift_1}"

    # Also verify: sin output has the right period (1/omega = 5 s).
    theta = A_MAX * np.sin(phi[:, 0])
    # Count zero-crossings on the way up over [0, T].
    up_crossings = np.where(np.diff(np.sign(theta)) > 0)[0]
    # Expected: T * omega ascending crossings.
    expected_count = int(round(T * omega))
    assert abs(len(up_crossings) - expected_count) <= 1, \
        f"expected ~{expected_count} up-crossings, got {len(up_crossings)}"
    print(f"  up-crossings in {T}s: {len(up_crossings)} (expect ~{expected_count})")
    print("  PASS")


def test2_coupled_pair_converges_to_delta():
    print("Test 2: coupled pair converges to target Delta")
    # Two oscillators coupled with strong K, target offset Delta.
    struct = _make_structure(n=2, connections=[(0, 1)])
    omega = DEFAULT_OMEGA_HZ
    delta_target = math.pi / 2  # want phi_1 - phi_0 -> pi/2

    # params: [A_0, A_1, phi0_0, phi0_1, K_01, Delta_01]
    K_coupling = 2.0  # max coupling for fast convergence
    params = [
        A_MAX, A_MAX,        # amplitudes
        0.0, 2.3,             # start out of phase (far from delta_target)
        K_coupling,           # K
        delta_target,         # Delta
    ]
    brain = BrainKuramoto.from_params(
        params=params, network_structure=struct,
        output_mapping=[], omega_hz=omega,
    )

    T, dt = 50.0, 0.01
    t, phi = _integrate(brain, T, dt)

    # phi difference over time, wrapped to [-pi, pi]
    diff = phi[:, 1] - phi[:, 0]
    diff_wrapped = (diff + np.pi) % (2 * np.pi) - np.pi
    final = float(diff_wrapped[-1])

    # Distance to target (accounting for circular wrap).
    err = abs((final - delta_target + np.pi) % (2 * np.pi) - np.pi)
    print(f"  initial phi_1 - phi_0 = {diff_wrapped[0]:.3f}")
    print(f"  final   phi_1 - phi_0 = {final:.3f}  (target {delta_target:.3f})")
    print(f"  |final - target|      = {err:.2e}")
    assert err < 1e-2, f"did not converge to Delta: err={err}"
    print("  PASS")


def main():
    print("=" * 60)
    test1_uncoupled_clean_sine()
    print()
    test2_coupled_pair_converges_to_delta()
    print("=" * 60)
    print("All Kuramoto brain sanity tests passed.")


if __name__ == "__main__":
    main()
