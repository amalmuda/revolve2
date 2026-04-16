"""
Kuramoto order parameter R as phase consistency metric.

For each evaluated robot, capture joint angle time series, extract instantaneous
phase via Hilbert transform, then compute:
  R(t) = | mean_k( exp(i * phi_k(t)) ) |
Average R over time -> single scalar in [0, 1].

R = 1: all joints perfectly phase-locked
R = 0: phases uniformly distributed (no coordination)

Two variants:
  R_global    : across all actuated joints
  R_interlimb : across one representative joint per limb (hip)
"""
from __future__ import annotations

import math
import numpy as np
import numpy.typing as npt
from scipy.signal import hilbert


def instantaneous_phase(signal: npt.NDArray[np.float_]) -> npt.NDArray[np.float_]:
    """
    Detrend a 1D signal and return its instantaneous phase via Hilbert transform.

    :param signal: shape (T,)
    :returns: shape (T,) array of phases in (-pi, pi].
    """
    if signal.ndim != 1:
        raise ValueError("instantaneous_phase expects a 1D signal")
    sig = signal - float(signal.mean())
    if np.allclose(sig, 0):
        # All zero - phase undefined. Return zeros, the caller should drop these.
        return np.zeros_like(sig)
    analytic = hilbert(sig)
    return np.angle(analytic)


def kuramoto_R(phase_matrix: npt.NDArray[np.float_]) -> float:
    """
    Compute mean Kuramoto order parameter over time.

    :param phase_matrix: shape (T, K) = T timesteps, K oscillators.
    :returns: scalar R in [0, 1] (mean R over time).
    """
    if phase_matrix.ndim != 2:
        raise ValueError("phase_matrix must be 2D (T, K)")
    if phase_matrix.shape[1] == 0:
        return 0.0
    if phase_matrix.shape[1] == 1:
        # Single oscillator is trivially phase-locked with itself.
        return 1.0
    # Per-timestep order: |mean(exp(i * phi))|
    z = np.exp(1j * phase_matrix)               # shape (T, K)
    mean_z = z.mean(axis=1)                     # shape (T,)
    R_t = np.abs(mean_z)                        # shape (T,)
    return float(np.mean(R_t))


def compute_R_from_traces(
    joint_traces: npt.NDArray[np.float_],
    fs: float,
    skip_seconds: float = 1.0,
    drop_silent: bool = True,
) -> float:
    """
    Compute mean Kuramoto R across joints from raw joint angle time series.

    :param joint_traces: shape (T, K) raw joint angles per timestep.
    :param fs: sampling rate in Hz (1.0 / simulation_timestep).
    :param skip_seconds: seconds to drop from the start (Hilbert edge + CPG
        settling). Default 1.0 s.
    :param drop_silent: if True, drop joints whose detrended signal is
        ~zero (would have undefined phase).
    :returns: mean Kuramoto R over time across the surviving joints.
    """
    if joint_traces.ndim != 2:
        raise ValueError("joint_traces must be 2D (T, K)")
    skip = int(round(skip_seconds * fs))
    if skip >= joint_traces.shape[0]:
        raise ValueError("skip_seconds longer than trace duration")
    traces = joint_traces[skip:]

    phases = []
    for k in range(traces.shape[1]):
        sig = traces[:, k]
        if drop_silent and np.allclose(sig - sig.mean(), 0):
            continue
        phases.append(instantaneous_phase(sig))
    if not phases:
        return 0.0
    phase_matrix = np.stack(phases, axis=1)  # (T, K_alive)
    return kuramoto_R(phase_matrix)


def compute_R_global_and_interlimb(
    joint_traces: npt.NDArray[np.float_],
    fs: float,
    hip_indices: list[int] | None = None,
    skip_seconds: float = 1.0,
) -> tuple[float, float]:
    """
    Compute both R_global (all joints) and R_interlimb (hips only).

    :param joint_traces: shape (T, n_hinges) raw joint angles.
    :param fs: sampling rate (Hz).
    :param hip_indices: column indices into joint_traces of hip joints
        (one per limb). If None or empty, R_interlimb = NaN.
    :param skip_seconds: seconds to drop from start.
    :returns: (R_global, R_interlimb).
    """
    R_global = compute_R_from_traces(joint_traces, fs, skip_seconds)
    if hip_indices is None or len(hip_indices) == 0:
        return R_global, float("nan")
    if len(hip_indices) == 1:
        # Only one limb -> trivial R = 1 (would be misleading). Return NaN.
        return R_global, float("nan")
    hip_traces = joint_traces[:, hip_indices]
    R_interlimb = compute_R_from_traces(hip_traces, fs, skip_seconds)
    return R_global, R_interlimb
