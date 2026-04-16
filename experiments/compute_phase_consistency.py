"""
Pairwise phase consistency (phase-locking value, PLV) as a coordination metric.

For each pair of hinges (j, k) with instantaneous phases phi_j(t), phi_k(t):
    R_jk = | (1/T) * sum_t exp(i * (phi_j(t) - phi_k(t))) |
Average R_jk over all pairs (j, k) with j < k -> single scalar in [0, 1].

R = 1: every pair maintains a constant phase offset for the whole window
       (offset itself may be anything -- 0, pi/3, anti-phase, ...).
R = 0: phase differences drift or fluctuate without stable structure.

This differs from Kuramoto R: Kuramoto measures instantaneous alignment across
oscillators and penalises constant non-zero offsets. Phase consistency (PLV)
rewards any stable offset, so it captures coordination regardless of the
specific gait pattern.

Reference: Pikovsky, Rosenblum & Kurths (2001), Synchronization: A Universal
Concept in Nonlinear Sciences.
"""
from __future__ import annotations

import numpy as np
import numpy.typing as npt

from compute_kuramoto import instantaneous_phase


def pairwise_phase_consistency(phase_matrix: npt.NDArray[np.float_]) -> float:
    """
    Compute mean pairwise PLV from a matrix of instantaneous phases.

    :param phase_matrix: shape (T, K), phases in radians.
    :returns: mean R over the K*(K-1)/2 unordered pairs. Returns NaN for
        K < 2 (no pairs defined).
    """
    if phase_matrix.ndim != 2:
        raise ValueError("phase_matrix must be 2D (T, K)")
    T, K = phase_matrix.shape
    if K < 2:
        return float("nan")

    z = np.exp(1j * phase_matrix)                # (T, K)
    # M[j, k] = (1/T) * sum_t exp(i * (phi_j(t) - phi_k(t)))
    #        = (1/T) * sum_t z[t, j] * conj(z[t, k])
    M = (z.T @ z.conj()) / T                     # (K, K), complex
    R_pair = np.abs(M)                           # (K, K), in [0, 1]

    iu = np.triu_indices(K, k=1)
    return float(R_pair[iu].mean())


def compute_phase_consistency_from_traces(
    joint_traces: npt.NDArray[np.float_],
    fs: float,
    skip_seconds: float = 1.0,
    drop_silent: bool = True,
) -> float:
    """
    Compute mean pairwise PLV from raw joint angle time series.

    :param joint_traces: shape (T, K) raw joint angles per timestep.
    :param fs: sampling rate in Hz (1.0 / simulation_timestep).
    :param skip_seconds: seconds to drop from the start (Hilbert edge +
        CPG settling). Default 1.0 s.
    :param drop_silent: if True, drop joints whose detrended signal is
        ~zero (phase undefined).
    :returns: mean pairwise R over surviving joints, or NaN if fewer than
        two surviving joints.
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
    if len(phases) < 2:
        return float("nan")
    phase_matrix = np.stack(phases, axis=1)      # (T, K_alive)
    return pairwise_phase_consistency(phase_matrix)


def compute_phase_consistency_global_and_interlimb(
    joint_traces: npt.NDArray[np.float_],
    fs: float,
    hip_indices: list[int] | None = None,
    skip_seconds: float = 1.0,
) -> tuple[float, float]:
    """
    Compute PLV across all joints and across hips only, mirroring the
    R_global / R_interlimb split used for Kuramoto R.

    :returns: (PLV_global, PLV_interlimb). PLV_interlimb is NaN when fewer
        than two hip indices are supplied.
    """
    plv_global = compute_phase_consistency_from_traces(
        joint_traces, fs, skip_seconds
    )
    if hip_indices is None or len(hip_indices) < 2:
        return plv_global, float("nan")
    hip_traces = joint_traces[:, hip_indices]
    plv_interlimb = compute_phase_consistency_from_traces(
        hip_traces, fs, skip_seconds
    )
    return plv_global, plv_interlimb
