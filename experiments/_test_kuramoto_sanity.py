"""Sanity check the Kuramoto R metric on synthetic signals."""
import math
import numpy as np

from compute_kuramoto import compute_R_from_traces, kuramoto_R, instantaneous_phase


def _make_sine(t, freq, phase=0.0, amp=1.0):
    return amp * np.sin(2 * math.pi * freq * t + phase)


def main():
    fs = 1000.0
    T = 10.0
    t = np.arange(int(T * fs)) / fs

    print("Sanity tests for Kuramoto R")
    print("=" * 60)

    # Test 1: Single sine vs itself (just one signal, then duplicate)
    s1 = _make_sine(t, 1.0)
    traces_self = np.stack([s1, s1], axis=1)
    R1 = compute_R_from_traces(traces_self, fs)
    print(f"1. Same sine duplicated:                  R = {R1:.4f}  (expect ~1.0)")
    assert R1 > 0.99, f"FAIL: expected ~1.0, got {R1}"

    # Test 2: Two sines with constant phase offset
    # R = |mean(e^{i*0} + e^{i*phi})| = |1 + e^{i*phi}| / 2 = cos(phi/2)
    # For phi = pi/4 (45 deg) -> R = cos(pi/8) ~= 0.924
    s_a = _make_sine(t, 1.0, phase=0.0)
    s_b = _make_sine(t, 1.0, phase=math.pi / 4)
    traces_offset = np.stack([s_a, s_b], axis=1)
    R2 = compute_R_from_traces(traces_offset, fs)
    expected = math.cos(math.pi / 8)
    print(f"2. Two sines, 45deg constant offset:      R = {R2:.4f}  (expect {expected:.4f}, constant)")
    assert abs(R2 - expected) < 0.01, f"FAIL: expected ~{expected:.3f}, got {R2}"

    # Test 3: Two sines with constant phase offset, anti-phase
    s_b_anti = _make_sine(t, 1.0, phase=math.pi)
    traces_anti = np.stack([s_a, s_b_anti], axis=1)
    R3 = compute_R_from_traces(traces_anti, fs)
    print(f"3. Two sines, anti-phase (180deg offset): R = {R3:.4f}  (expect ~0)")
    # R = |mean(e^{i*0} + e^{i*pi}) / 2| = |1 + (-1)| / 2 = 0
    assert R3 < 0.05, f"FAIL: expected ~0, got {R3}"

    # Test 4: Two sines with linearly drifting phase difference
    # 1.0 vs 1.05 Hz over 9s after skip -> ~0.45 cycle drift
    s_a = _make_sine(t, 1.0)
    s_b_drift = _make_sine(t, 1.05)
    traces_drift = np.stack([s_a, s_b_drift], axis=1)
    R4 = compute_R_from_traces(traces_drift, fs)
    print(f"4. Two sines, 1.0 vs 1.05 Hz (drift):     R = {R4:.4f}  (expect <1)")
    assert R4 < 0.9, f"FAIL: expected <0.9, got {R4}"

    # Test 4b: Bigger frequency mismatch -> R should be smaller
    s_b_drift2 = _make_sine(t, 1.5)
    traces_drift2 = np.stack([s_a, s_b_drift2], axis=1)
    R4b = compute_R_from_traces(traces_drift2, fs)
    print(f"4b. Two sines, 1.0 vs 1.5 Hz (big drift): R = {R4b:.4f}  (expect <0.7)")
    assert R4b < 0.7, f"FAIL: expected <0.7, got {R4b}"

    # Test 5: 4 sines all in phase
    traces_4_sync = np.stack([_make_sine(t, 1.0)] * 4, axis=1)
    R5 = compute_R_from_traces(traces_4_sync, fs)
    print(f"5. Four sines, perfectly synchronized:    R = {R5:.4f}  (expect ~1.0)")
    assert R5 > 0.99, f"FAIL: expected ~1.0, got {R5}"

    # Test 6: 4 sines uniformly distributed in phase: 0, pi/2, pi, 3pi/2
    s_phases = [_make_sine(t, 1.0, phase=p) for p in [0, math.pi / 2, math.pi, 3 * math.pi / 2]]
    traces_4_uniform = np.stack(s_phases, axis=1)
    R6 = compute_R_from_traces(traces_4_uniform, fs)
    # mean(e^{i*[0, pi/2, pi, 3pi/2]}) = (1 + i - 1 - i) / 4 = 0
    print(f"6. Four sines, uniform phases (90deg apt):R = {R6:.4f}  (expect ~0)")
    assert R6 < 0.05, f"FAIL: expected ~0, got {R6}"

    # Test 7: Pure noise -> R should be small
    np.random.seed(0)
    noise_traces = np.random.randn(len(t), 4)
    R7 = compute_R_from_traces(noise_traces, fs)
    print(f"7. White noise, 4 channels:               R = {R7:.4f}  (expect small)")
    assert R7 < 0.5, f"FAIL: expected small, got {R7}"

    # Test 8: Trace with one silent joint
    traces_silent = np.stack([s_a, np.zeros_like(s_a), s_a], axis=1)
    R8 = compute_R_from_traces(traces_silent, fs, drop_silent=True)
    print(f"8. Silent joint dropped, 2 sync sines:    R = {R8:.4f}  (expect ~1.0)")
    assert R8 > 0.99

    print()
    print("All sanity checks PASSED.")


if __name__ == "__main__":
    main()
