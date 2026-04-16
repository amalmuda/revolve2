"""Sanity check the pairwise phase consistency (PLV) metric on synthetic signals."""
import math
import numpy as np

from compute_phase_consistency import (
    compute_phase_consistency_from_traces,
    pairwise_phase_consistency,
)
from compute_kuramoto import instantaneous_phase


def _make_sine(t, freq, phase=0.0, amp=1.0):
    return amp * np.sin(2 * math.pi * freq * t + phase)


def main():
    fs = 1000.0
    T = 10.0
    t = np.arange(int(T * fs)) / fs

    print("Sanity tests for pairwise phase consistency (PLV)")
    print("=" * 65)

    # Test 1: Same sine duplicated -> PLV = 1.
    s1 = _make_sine(t, 1.0)
    traces_self = np.stack([s1, s1], axis=1)
    R1 = compute_phase_consistency_from_traces(traces_self, fs)
    print(f"1. Same sine duplicated:                   R = {R1:.4f}  (expect ~1.0)")
    assert R1 > 0.99, f"FAIL: expected ~1.0, got {R1}"

    # Test 2: Two sines, constant 45 deg offset.
    # Phase difference is constant -> PLV = 1 (any stable offset).
    # This is THE key behavioural difference from Kuramoto R, which would
    # return cos(pi/8) ~= 0.924 for this input.
    s_a = _make_sine(t, 1.0, phase=0.0)
    s_b = _make_sine(t, 1.0, phase=math.pi / 4)
    traces_offset = np.stack([s_a, s_b], axis=1)
    R2 = compute_phase_consistency_from_traces(traces_offset, fs)
    print(f"2. Two sines, 45deg constant offset:       R = {R2:.4f}  (expect ~1.0)")
    assert R2 > 0.99, f"FAIL: expected ~1.0, got {R2}"

    # Test 3: Two sines, anti-phase (constant 180 deg offset).
    # Again constant offset -> PLV = 1.
    s_b_anti = _make_sine(t, 1.0, phase=math.pi)
    traces_anti = np.stack([s_a, s_b_anti], axis=1)
    R3 = compute_phase_consistency_from_traces(traces_anti, fs)
    print(f"3. Two sines, anti-phase (180deg offset):  R = {R3:.4f}  (expect ~1.0)")
    assert R3 > 0.99, f"FAIL: expected ~1.0, got {R3}"

    # Test 4: Two sines with slowly drifting phase difference (1.0 vs 1.05 Hz).
    # Offset drifts linearly -> PLV < 1 (but still notable because only
    # half a cycle of drift over the 9 s window).
    s_b_drift = _make_sine(t, 1.05)
    traces_drift = np.stack([s_a, s_b_drift], axis=1)
    R4 = compute_phase_consistency_from_traces(traces_drift, fs)
    print(f"4. Two sines, 1.0 vs 1.05 Hz (slow drift): R = {R4:.4f}  (expect <1)")
    assert R4 < 0.95, f"FAIL: expected <0.95, got {R4}"

    # Test 4b: Bigger frequency mismatch -> PLV much smaller.
    s_b_drift2 = _make_sine(t, 1.5)
    traces_drift2 = np.stack([s_a, s_b_drift2], axis=1)
    R4b = compute_phase_consistency_from_traces(traces_drift2, fs)
    print(f"4b. Two sines, 1.0 vs 1.5 Hz (big drift):  R = {R4b:.4f}  (expect <0.3)")
    assert R4b < 0.3, f"FAIL: expected <0.3, got {R4b}"

    # Test 5: Four sines uniformly spread (0, pi/2, pi, 3pi/2).
    # Kuramoto R ~= 0, but every pairwise offset is constant -> PLV = 1.
    s_phases = [_make_sine(t, 1.0, phase=p)
                for p in [0, math.pi / 2, math.pi, 3 * math.pi / 2]]
    traces_uniform = np.stack(s_phases, axis=1)
    R5 = compute_phase_consistency_from_traces(traces_uniform, fs)
    print(f"5. Four sines, uniform phases (90 apart):  R = {R5:.4f}  (expect ~1.0)")
    assert R5 > 0.99, f"FAIL: expected ~1.0, got {R5}"

    # Test 6: Four independent noise channels -> no stable pairwise offsets.
    np.random.seed(0)
    noise_traces = np.random.randn(len(t), 4)
    R6 = compute_phase_consistency_from_traces(noise_traces, fs)
    print(f"6. Four noise channels:                    R = {R6:.4f}  (expect small)")
    assert R6 < 0.3, f"FAIL: expected small, got {R6}"

    # Test 7: Mixed set -- two locked + two independent noise channels.
    # Pairs of locked channels give PLV=1, all other pairs ~0.
    # 6 pairs total, 1 locked pair + 5 unrelated -> mean ~ 1/6 plus noise.
    locked = _make_sine(t, 1.0)
    mixed = np.stack([
        locked,
        locked,
        np.random.randn(len(t)),
        np.random.randn(len(t)),
    ], axis=1)
    R7 = compute_phase_consistency_from_traces(mixed, fs)
    print(f"7. 2 locked + 2 noise (6 pairs, 1 locked): R = {R7:.4f}  (expect ~0.15-0.35)")
    assert 0.10 < R7 < 0.45, f"FAIL: expected moderate, got {R7}"

    # Test 8: Silent joint dropped, two locked sines remain.
    traces_silent = np.stack([s_a, np.zeros_like(s_a), s_a], axis=1)
    R8 = compute_phase_consistency_from_traces(traces_silent, fs, drop_silent=True)
    print(f"8. Silent joint dropped, 2 sync sines:     R = {R8:.4f}  (expect ~1.0)")
    assert R8 > 0.99, f"FAIL: expected ~1.0, got {R8}"

    # Test 9: Single surviving channel -> undefined pairwise metric.
    traces_one = np.stack([s_a, np.zeros_like(s_a)], axis=1)
    R9 = compute_phase_consistency_from_traces(traces_one, fs, drop_silent=True)
    print(f"9. Single surviving channel:               R = {R9}  (expect NaN)")
    assert math.isnan(R9), f"FAIL: expected NaN, got {R9}"

    # Test 10: Direct PLV on pre-computed phases (bypasses Hilbert).
    phi_a = 2 * math.pi * 1.0 * t
    phi_b = phi_a + math.pi / 3          # constant offset
    phase_matrix = np.stack([phi_a, phi_b], axis=1)
    R10 = pairwise_phase_consistency(phase_matrix)
    print(f"10. Direct PLV, constant 60deg offset:     R = {R10:.4f}  (expect ~1.0)")
    assert R10 > 0.99, f"FAIL: expected ~1.0, got {R10}"

    print()
    print("All sanity checks PASSED.")


if __name__ == "__main__":
    main()
