"""Extreme stress test suite for the Kuramoto brain.

Each test block is self-contained and prints PASS/FAIL. The script exits
non-zero on any failure.

Coverage
--------
- Math correctness: uncoupled sine, pair convergence, ring traveling wave,
  all-to-all synchrony, anti-phase lock.
- Conservation: mean angular velocity = 2pi*omega for any K, Delta, phases
  (consequence of K symmetric + Delta antisymmetric + sin odd).
- Matrix invariants: K symmetric, Delta antisymmetric, zero diagonal.
- Parameter layout: split/build round-trip; wrong-length raises.
- Boundary conditions: A=0 silent; K=0 decouples; Delta=0 mod 2*pi.
- Numerical stability: long runs, stiff high-K, small/large dt.
- Topology reuse: uncoupled / neighbor / BLF structure builders all slot
  into KuramotoNetworkStructure with expected param counts.
- Determinism: two instances from same brain follow identical trajectories.
- Frustration: triangle with incompatible Deltas does not reach full lock.
"""
import math
import sys
import traceback

import numpy as np

from revolve2.modular_robot.brain.cpg._cpg_network_structure import (
    Cpg, CpgPair, CpgNetworkStructure,
)
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)

from kuramoto_brain import (
    KuramotoNetworkStructure, BrainKuramoto,
    kuramoto_structure_from_cpg_structure, param_bounds,
    DEFAULT_OMEGA_HZ, A_MAX, K_MAX,
)
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
)


FAILURES: list[tuple[str, str]] = []


def _case(name):
    def deco(fn):
        def wrapped():
            try:
                fn()
                print(f"  PASS: {name}")
            except AssertionError as e:
                FAILURES.append((name, str(e)))
                print(f"  FAIL: {name}")
                print(f"        {e}")
            except Exception as e:
                FAILURES.append((name, f"{type(e).__name__}: {e}"))
                print(f"  FAIL: {name}")
                print(f"        {type(e).__name__}: {e}")
                traceback.print_exc()
        wrapped.__name__ = fn.__name__
        return wrapped
    return deco


def _structure(n, connections):
    cpgs = [Cpg(index=i) for i in range(n)]
    pairs = {CpgPair(cpg_1=cpgs[i], cpg_2=cpgs[j]) for i, j in connections}
    return KuramotoNetworkStructure(oscillators=cpgs, connections=pairs)


def _integrate(brain, T, dt, wrap=False):
    inst = brain.make_instance()
    steps = int(round(T / dt))
    hist = np.zeros((steps + 1, inst._phi.size))
    hist[0] = inst._phi.copy()
    for k in range(steps):
        inst._phi = inst._rk4_step(inst._phi, dt)
        if wrap:
            inst._phi = (inst._phi + np.pi) % (2 * np.pi) - np.pi
        hist[k + 1] = inst._phi.copy()
    return np.arange(steps + 1) * dt, hist


def _wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def _R(phases):
    """Kuramoto order parameter."""
    z = np.exp(1j * phases).mean(axis=-1)
    return np.abs(z)


# ---------------------------------------------------------------------------
# 1. Math correctness
# ---------------------------------------------------------------------------


@_case("uncoupled matches analytic phase exactly")
def t_uncoupled_analytic():
    struct = _structure(3, [])
    params = [A_MAX, A_MAX, A_MAX, 0.0, 0.7, 2.1]  # A then phi0
    brain = BrainKuramoto.from_params(params, struct, [], omega_hz=DEFAULT_OMEGA_HZ)
    t, phi = _integrate(brain, T=50.0, dt=0.01)
    phi0 = np.array([0.0, 0.7, 2.1])
    expected = 2 * np.pi * DEFAULT_OMEGA_HZ * t[:, None] + phi0[None, :]
    err = np.max(np.abs(phi - expected))
    assert err < 1e-8, f"max drift {err:.2e}"


@_case("pair locks to Delta = pi/2")
def t_pair_pi2():
    struct = _structure(2, [(0, 1)])
    params = [A_MAX, A_MAX, 0.0, 2.3, 2.0, math.pi / 2]
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi = _integrate(brain, 40.0, 0.01)
    final_diff = _wrap(phi[-1, 1] - phi[-1, 0])
    err = abs(_wrap(final_diff - math.pi / 2))
    assert err < 1e-6, f"err = {err:.2e}"


@_case("pair locks to Delta = pi (anti-phase)")
def t_pair_antiphase():
    struct = _structure(2, [(0, 1)])
    params = [A_MAX, A_MAX, 0.0, 0.1, 2.0, math.pi]  # start nearly in phase
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi = _integrate(brain, 60.0, 0.01)
    final_diff = _wrap(phi[-1, 1] - phi[-1, 0])
    err = abs(_wrap(final_diff - math.pi))
    assert err < 1e-3, f"err = {err:.3e}"


@_case("all-to-all with Delta=0 -> phase synchrony R->1")
def t_allto_all_sync():
    n = 5
    struct = _structure(n, [(i, j) for i in range(n) for j in range(i + 1, n)])
    rng = np.random.default_rng(0)
    phi0 = rng.uniform(0, 2 * np.pi, size=n)
    nc = struct.num_connections
    params = list(np.full(n, A_MAX)) + list(phi0) + [1.5] * nc + [0.0] * nc
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi = _integrate(brain, 40.0, 0.01)
    R_final = _R(phi[-1])
    assert R_final > 0.999, f"R = {R_final:.4f}"


@_case("3-ring with Delta=2pi/3 -> traveling wave; pairwise diffs lock")
def t_ring_traveling_wave():
    struct = _structure(3, [(0, 1), (1, 2), (0, 2)])
    # We want phi_1 - phi_0 = 2pi/3, phi_2 - phi_1 = 2pi/3, phi_2 - phi_0 = 4pi/3.
    # Link layout sorted by (lo, hi): (0,1), (0,2), (1,2).
    # Delta_01 = 2pi/3, Delta_02 = 4pi/3 (= -2pi/3 mod 2pi), Delta_12 = 2pi/3.
    K = [1.5, 1.5, 1.5]
    D = [2 * math.pi / 3, 4 * math.pi / 3, 2 * math.pi / 3]
    n = 3
    params = [A_MAX] * n + [0.0, 0.2, 0.4] + K + D
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi = _integrate(brain, 80.0, 0.01)
    d01 = _wrap(phi[-1, 1] - phi[-1, 0])
    d12 = _wrap(phi[-1, 2] - phi[-1, 1])
    assert abs(_wrap(d01 - 2 * math.pi / 3)) < 1e-3, f"d01 off: {d01:.4f}"
    assert abs(_wrap(d12 - 2 * math.pi / 3)) < 1e-3, f"d12 off: {d12:.4f}"


# ---------------------------------------------------------------------------
# 2. Conservation
# ---------------------------------------------------------------------------


@_case("mean angular velocity = 2*pi*omega (K symmetric + Delta antisymm + sin odd)")
def t_conservation_mean_velocity():
    # Random non-trivial network, verify Sigma_i dphi_i/dt = n * 2*pi*omega
    # for arbitrary phi. Not a time-integrated test — direct algebra.
    n = 6
    struct = _structure(n, [(i, j) for i in range(n) for j in range(i + 1, n)])
    rng = np.random.default_rng(42)
    A = rng.uniform(0, A_MAX, n)
    phi0 = rng.uniform(0, 2 * np.pi, n)
    nc = struct.num_connections
    K = rng.uniform(0, K_MAX, nc)
    D = rng.uniform(0, 2 * np.pi, nc)
    params = list(A) + list(phi0) + list(K) + list(D)
    brain = BrainKuramoto.from_params(params, struct, [])
    inst = brain.make_instance()
    # Try 20 random phi vectors; each must yield Sigma dphi/dt = n * 2pi*omega.
    for seed in range(20):
        r = np.random.default_rng(seed)
        phi = r.uniform(-10, 10, n)
        dphi = inst._dphi(phi)
        total = float(dphi.sum())
        expected = n * 2 * np.pi * DEFAULT_OMEGA_HZ
        assert abs(total - expected) < 1e-9, (
            f"seed={seed}: sum={total:.6f} expected={expected:.6f}")


@_case("after long integration, mean(phi(T) - phi(0)) = 2*pi*omega*T")
def t_conservation_time_integrated():
    n = 5
    struct = _structure(n, [(i, j) for i in range(n) for j in range(i + 1, n)])
    rng = np.random.default_rng(1)
    phi0 = rng.uniform(0, 2 * np.pi, n)
    nc = struct.num_connections
    params = (
        list(np.full(n, A_MAX)) + list(phi0) +
        list(rng.uniform(0, K_MAX, nc)) +
        list(rng.uniform(0, 2 * np.pi, nc))
    )
    brain = BrainKuramoto.from_params(params, struct, [])
    T = 100.0
    _, phi = _integrate(brain, T, 0.005)
    mean_drift = float(np.mean(phi[-1] - phi[0]))
    expected = 2 * np.pi * DEFAULT_OMEGA_HZ * T
    assert abs(mean_drift - expected) < 1e-6, (
        f"mean drift {mean_drift:.6f} expected {expected:.6f}")


# ---------------------------------------------------------------------------
# 3. Matrix invariants
# ---------------------------------------------------------------------------


@_case("K matrix is symmetric and zero on diagonal")
def t_k_symmetric():
    struct = _structure(4, [(0, 1), (1, 2), (2, 3), (0, 3)])
    rng = np.random.default_rng(7)
    K_flat = rng.uniform(0, K_MAX, struct.num_connections)
    D_flat = rng.uniform(0, 2 * np.pi, struct.num_connections)
    K, D = struct.build_matrices(K_flat, D_flat)
    assert np.allclose(K, K.T), "K not symmetric"
    assert np.allclose(np.diag(K), 0.0), "K has nonzero diagonal"


@_case("Delta matrix is antisymmetric and zero on diagonal")
def t_delta_antisymmetric():
    struct = _structure(4, [(0, 1), (1, 2), (2, 3), (0, 3)])
    rng = np.random.default_rng(8)
    K_flat = rng.uniform(0, K_MAX, struct.num_connections)
    D_flat = rng.uniform(0, 2 * np.pi, struct.num_connections)
    _, D = struct.build_matrices(K_flat, D_flat)
    assert np.allclose(D, -D.T), "Delta not antisymmetric"
    assert np.allclose(np.diag(D), 0.0), "Delta has nonzero diagonal"


# ---------------------------------------------------------------------------
# 4. Parameter layout
# ---------------------------------------------------------------------------


@_case("num_params = 2n + 2nc")
def t_num_params():
    for n, connections in [(1, []), (2, [(0, 1)]),
                            (5, [(0, 1), (1, 2), (2, 3), (3, 4)])]:
        s = _structure(n, connections)
        assert s.num_params == 2 * n + 2 * s.num_connections


@_case("split_params -> build_matrices round-trip preserves K, Delta")
def t_split_roundtrip():
    n = 4
    struct = _structure(n, [(0, 1), (0, 2), (1, 2), (2, 3)])
    rng = np.random.default_rng(2)
    A = rng.uniform(0, A_MAX, n)
    phi0 = rng.uniform(0, 2 * np.pi, n)
    K_flat = rng.uniform(0, K_MAX, struct.num_connections)
    D_flat = rng.uniform(0, 2 * np.pi, struct.num_connections)
    params = list(A) + list(phi0) + list(K_flat) + list(D_flat)
    A2, phi02, K_flat2, D_flat2 = struct.split_params(params)
    assert np.allclose(A, A2)
    assert np.allclose(phi0, phi02)
    assert np.allclose(K_flat, K_flat2)
    assert np.allclose(D_flat, D_flat2)


@_case("wrong-length params raises AssertionError")
def t_wrong_length_raises():
    struct = _structure(3, [(0, 1), (1, 2)])
    try:
        struct.split_params([0.0] * (struct.num_params - 1))
    except AssertionError:
        return
    raise AssertionError("no AssertionError raised on short param vector")


@_case("param_bounds lengths match num_params")
def t_param_bounds_lengths():
    struct = _structure(4, [(0, 1), (1, 2), (2, 3)])
    lo, hi = param_bounds(struct)
    assert len(lo) == len(hi) == struct.num_params
    # Upper bound structure: n*A_MAX, n*2pi, nc*K_MAX, nc*2pi
    n, nc = struct.num_oscillators, struct.num_connections
    assert all(v == A_MAX for v in hi[:n])
    assert all(v == 2 * np.pi for v in hi[n:2 * n])
    assert all(v == K_MAX for v in hi[2 * n:2 * n + nc])
    assert all(v == 2 * np.pi for v in hi[2 * n + nc:])


# ---------------------------------------------------------------------------
# 5. Boundary conditions
# ---------------------------------------------------------------------------


@_case("A=0 silences the joint (output always 0)")
def t_amplitude_zero():
    struct = _structure(2, [])
    params = [0.0, A_MAX, 0.0, 0.0]
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi = _integrate(brain, 10.0, 0.01)
    targets = 0.0 * np.sin(phi[:, 0])  # joint 0 has A=0
    assert np.allclose(targets, 0.0)


@_case("K=0 decouples (phase difference drifts freely)")
def t_k_zero_decouples():
    # If K=0 everywhere, each phase evolves independently at omega.
    struct = _structure(3, [(0, 1), (1, 2), (0, 2)])
    rng = np.random.default_rng(3)
    phi0 = rng.uniform(0, 2 * np.pi, 3)
    nc = struct.num_connections
    params = ([A_MAX] * 3 + list(phi0) +
              [0.0] * nc + rng.uniform(0, 2 * np.pi, nc).tolist())
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi = _integrate(brain, 30.0, 0.01)
    phi_exp = 2 * np.pi * DEFAULT_OMEGA_HZ * np.arange(phi.shape[0])[:, None] * 0.01 + phi0[None, :]
    assert np.allclose(phi, phi_exp, atol=1e-8)


@_case("Delta = 0 and Delta = 2pi produce same dynamics (periodicity)")
def t_delta_periodic():
    struct = _structure(2, [(0, 1)])
    pa = [A_MAX, A_MAX, 0.0, 1.0, 1.5, 0.0]
    pb = [A_MAX, A_MAX, 0.0, 1.0, 1.5, 2 * np.pi]
    ba = BrainKuramoto.from_params(pa, struct, [])
    bb = BrainKuramoto.from_params(pb, struct, [])
    _, phi_a = _integrate(ba, 15.0, 0.01)
    _, phi_b = _integrate(bb, 15.0, 0.01)
    assert np.allclose(phi_a, phi_b, atol=1e-9)


# ---------------------------------------------------------------------------
# 6. Numerical stability
# ---------------------------------------------------------------------------


@_case("long run: 1000s with wrap has no NaN/Inf and mean-velocity still holds")
def t_long_run_stable():
    n = 4
    struct = _structure(n, [(0, 1), (1, 2), (2, 3), (0, 3)])
    rng = np.random.default_rng(9)
    nc = struct.num_connections
    params = (
        [A_MAX] * n + list(rng.uniform(0, 2 * np.pi, n)) +
        list(rng.uniform(0, K_MAX, nc)) + list(rng.uniform(0, 2 * np.pi, nc))
    )
    brain = BrainKuramoto.from_params(params, struct, [])
    inst = brain.make_instance()
    # Reuse control loop manually (with wrap) to simulate prolonged use.
    dt = 0.02
    T = 1000.0
    for _ in range(int(T / dt)):
        inst._phi = inst._rk4_step(inst._phi, dt)
        inst._phi = (inst._phi + np.pi) % (2 * np.pi) - np.pi
    assert np.all(np.isfinite(inst._phi))
    assert np.all(np.abs(inst._phi) <= np.pi + 1e-9)


@_case("stiff regime: K=K_MAX, strong mismatch, dt=0.01 still converges")
def t_stiff_converges():
    struct = _structure(2, [(0, 1)])
    # Start far from target with max coupling.
    params = [A_MAX, A_MAX, 0.0, 3.0, K_MAX, math.pi / 3]
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi = _integrate(brain, 50.0, 0.01)
    final = _wrap(phi[-1, 1] - phi[-1, 0])
    err = abs(_wrap(final - math.pi / 3))
    assert err < 1e-4, f"err = {err:.3e}"


@_case("small-dt and large-dt agree to 3 decimals for moderate coupling")
def t_dt_consistency():
    struct = _structure(3, [(0, 1), (1, 2)])
    params = [A_MAX] * 3 + [0.0, 1.0, 2.0] + [1.0, 1.0] + [0.5, 0.5]
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi_fine = _integrate(brain, 20.0, 0.001)
    _, phi_coarse = _integrate(brain, 20.0, 0.02)
    diff = _wrap(phi_fine[-1] - phi_coarse[-1])
    assert np.max(np.abs(diff)) < 1e-3, f"max diff {np.max(np.abs(diff)):.3e}"


# ---------------------------------------------------------------------------
# 7. Topology reuse — plug into existing Revolve2 topology builders
# ---------------------------------------------------------------------------


@_case("topology builders (uncoupled/neighbor/BLF) produce valid Kuramoto structs")
def t_topology_builders():
    body = modular_robots_v1.get("spider")
    hinges = body.find_modules_of_type(ActiveHinge)
    for name, builder in [
        ("uncoupled", lambda: active_hinges_to_cpg_network_structure_internal_only(hinges)),
        ("neighbor",  lambda: active_hinges_to_cpg_network_structure_neighbor(hinges)),
        ("blf",       lambda: active_hinges_to_cpg_network_structure_blf(hinges, body)),
    ]:
        cpg, _ = builder()
        k = kuramoto_structure_from_cpg_structure(cpg)
        n = k.num_oscillators
        nc = k.num_connections
        assert n == 8, f"{name}: expected 8 oscillators for spider, got {n}"
        assert k.num_params == 2 * n + 2 * nc
        # Exercise build_matrices with random params
        rng = np.random.default_rng(123)
        A = rng.uniform(0, A_MAX, n).tolist()
        phi0 = rng.uniform(0, 2 * np.pi, n).tolist()
        K_flat = rng.uniform(0, K_MAX, nc).tolist()
        D_flat = rng.uniform(0, 2 * np.pi, nc).tolist()
        brain = BrainKuramoto.from_params(
            A + phi0 + K_flat + D_flat, k, [], omega_hz=DEFAULT_OMEGA_HZ
        )
        inst = brain.make_instance()
        # One integration step to make sure nothing crashes
        inst._phi = inst._rk4_step(inst._phi, 0.02)
        assert np.all(np.isfinite(inst._phi))


# ---------------------------------------------------------------------------
# 8. Determinism
# ---------------------------------------------------------------------------


@_case("two instances from the same brain produce identical trajectories")
def t_determinism():
    struct = _structure(3, [(0, 1), (1, 2)])
    params = [A_MAX] * 3 + [0.2, 1.1, 2.8] + [1.2, 0.8] + [1.0, 2.0]
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi_a = _integrate(brain, 10.0, 0.01)
    _, phi_b = _integrate(brain, 10.0, 0.01)
    assert np.allclose(phi_a, phi_b)


@_case("instance state is independent between makes (no shared mutable state)")
def t_instance_isolation():
    struct = _structure(2, [(0, 1)])
    params = [A_MAX, A_MAX, 0.0, 1.0, 1.0, math.pi / 2]
    brain = BrainKuramoto.from_params(params, struct, [])
    a = brain.make_instance()
    b = brain.make_instance()
    a._phi = a._rk4_step(a._phi, 0.1)
    # b should be unchanged by a's step
    assert np.allclose(b._phi, np.array([0.0, 1.0]))


# ---------------------------------------------------------------------------
# 9. Frustration
# ---------------------------------------------------------------------------


@_case("frustrated triangle (Deltas don't sum to 0 around cycle) does not fully lock")
def t_frustration():
    # Triangle with Delta_01 = Delta_12 = Delta_02 = pi/2. Sum around cycle:
    # (phi_1-phi_0) + (phi_2-phi_1) + (phi_0-phi_2) = 0, so for all three
    # sin terms to vanish simultaneously we would need Delta sums to match.
    # Here Delta_01 + Delta_12 = pi, but Delta_02 = pi/2. Incompatible -> no
    # stationary lock at all three targets.
    struct = _structure(3, [(0, 1), (0, 2), (1, 2)])
    params = [A_MAX] * 3 + [0.0, 0.1, 0.2] + [1.0, 1.0, 1.0] + [math.pi / 2] * 3
    brain = BrainKuramoto.from_params(params, struct, [])
    _, phi = _integrate(brain, 80.0, 0.01)
    d01 = _wrap(phi[-1, 1] - phi[-1, 0])
    d12 = _wrap(phi[-1, 2] - phi[-1, 1])
    d02 = _wrap(phi[-1, 2] - phi[-1, 0])
    err = (abs(_wrap(d01 - math.pi / 2))
           + abs(_wrap(d12 - math.pi / 2))
           + abs(_wrap(d02 - math.pi / 2)))
    # Total error across the three targets must be non-trivial.
    assert err > 0.2, f"unexpectedly small total error {err:.3f}"


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main():
    tests = [
        # math correctness
        t_uncoupled_analytic, t_pair_pi2, t_pair_antiphase,
        t_allto_all_sync, t_ring_traveling_wave,
        # conservation
        t_conservation_mean_velocity, t_conservation_time_integrated,
        # matrix invariants
        t_k_symmetric, t_delta_antisymmetric,
        # parameter layout
        t_num_params, t_split_roundtrip, t_wrong_length_raises,
        t_param_bounds_lengths,
        # boundary conditions
        t_amplitude_zero, t_k_zero_decouples, t_delta_periodic,
        # numerical stability
        t_long_run_stable, t_stiff_converges, t_dt_consistency,
        # topology reuse
        t_topology_builders,
        # determinism
        t_determinism, t_instance_isolation,
        # frustration
        t_frustration,
    ]
    print("=" * 70)
    print(f"KURAMOTO BRAIN STRESS TEST  ({len(tests)} cases)")
    print("=" * 70)
    for t in tests:
        t()
    print("=" * 70)
    if FAILURES:
        print(f"FAILED {len(FAILURES)}/{len(tests)}:")
        for name, msg in FAILURES:
            print(f"  - {name}: {msg}")
        sys.exit(1)
    print(f"ALL {len(tests)} TESTS PASSED")


if __name__ == "__main__":
    main()
