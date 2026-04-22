"""
Compute frequency (Welch peak) and Kuramoto order parameter R per run
for spider and gecko across uncoupled/neighbour/structured couplings
and all lambda values.

Output: cpg_metrics.md with per-run table + summary tables.
"""
import os
import math
import sqlite3
import sys
import numpy as np
from scipy.signal import welch, hilbert

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1

from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
)


# ============== Setup ==============

SIM_TIME = 30.0  # seconds
CONTROL_FREQ = 20  # Hz
DT = 1.0 / CONTROL_FREQ
N_STEPS = int(SIM_TIME / DT)
TRANSIENT_SECONDS = 5.0
TRANSIENT_SAMPLES = int(TRANSIENT_SECONDS / DT)

RESULTS_BASE = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/results/final_results"
)
OUTPUT_MD = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/cpg_metrics.md"
)

# Inter-limb hinges for Kuramoto R per morphology
# Spider: 4 hips (Left, Right, Front, Back) -> hinge indices 0, 2, 4, 6
SPIDER_INTERLIMB = [0, 2, 4, 6]
# Gecko: front L hip, front R hip, rear L hip, rear R hip, spine 1, spine 2
# Hinge mapping: 0=FL hip, 1=FR hip, 2=spine1, 3=spine2, 4=RL hip, 5=RR hip
GECKO_INTERLIMB = [0, 1, 4, 5, 2, 3]


# ============== Helpers ==============

def get_cpg_structure(coupling, body, hinges):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    if coupling == "neighbor":
        return active_hinges_to_cpg_network_structure_neighbor(hinges)
    if coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)
    raise ValueError(coupling)


def integrate_cpg(weight_matrix, initial_state, dt, n_steps):
    """RK45 integration of X' = WX, clipped to [-1, 1]."""
    state = initial_state.copy()
    history = np.zeros((n_steps, len(state)))
    for i in range(n_steps):
        history[i] = state
        a1 = weight_matrix @ state
        a2 = weight_matrix @ (state + dt / 2 * a1)
        a3 = weight_matrix @ (state + dt / 2 * a2)
        a4 = weight_matrix @ (state + dt * a3)
        state = state + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
        state = np.clip(state, -1.0, 1.0)
    return history


def compute_frequency(neuron_trajectories, fs):
    """Mean peak frequency across hinges using Welch PSD."""
    freqs = []
    for col in range(neuron_trajectories.shape[1]):
        signal = neuron_trajectories[:, col]
        # Skip flat signals (would yield meaningless peak)
        if np.std(signal) < 1e-6:
            freqs.append(0.0)
            continue
        nperseg = min(256, len(signal))
        f, psd = welch(signal, fs=fs, nperseg=nperseg)
        # Ignore the DC bin (f=0)
        if len(f) > 1:
            peak_idx = np.argmax(psd[1:]) + 1
            freqs.append(f[peak_idx])
        else:
            freqs.append(0.0)
    return float(np.mean(freqs))


def compute_kuramoto_R(neuron_trajectories, interlimb_indices, transient_samples):
    """
    Mean Kuramoto order parameter R over time, skipping transient.
    Selected inter-limb neurons only.
    """
    selected = neuron_trajectories[:, interlimb_indices]
    # Hilbert transform per channel -> instantaneous phase
    phases = np.angle(hilbert(selected, axis=0))
    # Skip transient
    phases = phases[transient_samples:]
    # R(t) = |mean over neurons of exp(i*phase)|
    R_t = np.abs(np.mean(np.exp(1j * phases), axis=1))
    return float(np.mean(R_t))


def get_distance_from_db(db_path):
    """Get distance of best individual at final generation."""
    try:
        conn = sqlite3.connect(db_path)
        cur = conn.cursor()
        cur.execute("""
            SELECT i.distance FROM comparison_individual i
            JOIN comparison_population p ON i.population_id = p.id
            JOIN comparison_generation g ON g.population_id = p.id
            WHERE g.generation_index = (SELECT MAX(generation_index) FROM comparison_generation)
            ORDER BY i.fitness DESC LIMIT 1
        """)
        row = cur.fetchone()
        conn.close()
        return row[0] if row else None
    except Exception:
        return None


# ============== Main ==============

def main():
    morphologies = [
        ("spider", "Spider", SPIDER_INTERLIMB),
        ("gecko", "Gecko", GECKO_INTERLIMB),
    ]
    couplings = [
        ("uncoupled", "No coupling"),
        ("neighbor", "Neighbour"),
        ("blf", "Structured"),
    ]
    lambdas = [0, 1, 2, 3]

    rows = []  # list of dicts

    for robot_key, robot_label, interlimb in morphologies:
        body = modular_robots_v1.get(robot_key)
        hinges = body.find_modules_of_type(ActiveHinge)
        n_hinges = len(hinges)

        for cpl_key, cpl_label in couplings:
            cpg_struct, _ = get_cpg_structure(cpl_key, body, hinges)
            initial_state = cpg_struct.make_uniform_state(math.sqrt(2) * 0.5)

            for lam in lambdas:
                exp_dir = "%s_ode_cpg_%s_lambda%d_dragging" % (robot_key, cpl_key, lam)
                exp_path = os.path.join(RESULTS_BASE, exp_dir)
                if not os.path.isdir(exp_path):
                    print("WARN: missing dir %s" % exp_path)
                    continue

                for run_num in range(1, 31):
                    npy_path = os.path.join(exp_path, "best_params_run_%d.npy" % run_num)
                    db_path = os.path.join(exp_path, "run_%d.sqlite" % run_num)
                    if not os.path.exists(npy_path):
                        continue

                    params = np.load(npy_path)
                    weight_matrix = cpg_struct.make_connection_weights_matrix_from_params(list(params))
                    history = integrate_cpg(weight_matrix, initial_state, DT, N_STEPS)
                    neurons = history[:, :n_hinges]

                    freq_hz = compute_frequency(neurons, fs=CONTROL_FREQ)
                    R = compute_kuramoto_R(neurons, interlimb, TRANSIENT_SAMPLES)
                    distance = get_distance_from_db(db_path)

                    rows.append({
                        "morphology": robot_label,
                        "coupling": cpl_label,
                        "lambda": lam,
                        "run_id": run_num,
                        "distance": distance,
                        "frequency_hz": freq_hz,
                        "kuramoto_R": R,
                    })

                done_count = sum(1 for r in rows if r["morphology"] == robot_label and r["coupling"] == cpl_label and r["lambda"] == lam)
                print("  %s %s lambda=%d: %d runs done" % (robot_label, cpl_label, lam, done_count))

    # ===== Build markdown =====

    lines = []
    lines.append("# CPG Metrics Analysis")
    lines.append("")
    lines.append("Per-run frequency (Welch peak) and Kuramoto order parameter R, computed by")
    lines.append("offline RK45 integration of saved CMA-ES best parameters (no physics).")
    lines.append("")
    lines.append("- Simulation time: 30 s, control frequency 20 Hz (600 samples per neuron)")
    lines.append("- Transient skipped for Kuramoto R: first 5 seconds (100 samples)")
    lines.append("- Frequency: peak of Welch PSD per hinge, then mean across all hinges")
    lines.append("- Kuramoto R: mean of `|mean(exp(i*phase))|` over time, using Hilbert phases of inter-limb hinges")
    lines.append("- Inter-limb hinges: spider = 4 hips; gecko = front L/R hips + rear L/R hips + spine 1 + spine 2")
    lines.append("")

    # Summary tables
    lines.append("## Summary tables (mean ± std, 30 runs)")
    lines.append("")
    for robot_label in ["Spider", "Gecko"]:
        lines.append("### %s" % robot_label)
        lines.append("")
        lines.append("| Coupling | λ | Distance (m) | Frequency (Hz) | Kuramoto R |")
        lines.append("|---|---|---|---|---|")
        for cpl_label in ["No coupling", "Neighbour", "Structured"]:
            for lam in lambdas:
                cell = [r for r in rows if r["morphology"] == robot_label and r["coupling"] == cpl_label and r["lambda"] == lam]
                if not cell:
                    continue
                dists = [r["distance"] for r in cell if r["distance"] is not None]
                freqs = [r["frequency_hz"] for r in cell]
                Rs = [r["kuramoto_R"] for r in cell]
                d_str = "%.2f ± %.2f" % (np.mean(dists), np.std(dists)) if dists else "N/A"
                f_str = "%.3f ± %.3f" % (np.mean(freqs), np.std(freqs))
                r_str = "%.3f ± %.3f" % (np.mean(Rs), np.std(Rs))
                lines.append("| %s | %d | %s | %s | %s |" % (cpl_label, lam, d_str, f_str, r_str))
        lines.append("")

    # Per-run table
    lines.append("## Per-run data")
    lines.append("")
    lines.append("| morphology | coupling | lambda | run_id | distance | frequency_hz | kuramoto_R |")
    lines.append("|---|---|---|---|---|---|---|")
    for r in rows:
        d = "%.4f" % r["distance"] if r["distance"] is not None else "N/A"
        lines.append("| %s | %s | %d | %d | %s | %.4f | %.4f |" % (
            r["morphology"], r["coupling"], r["lambda"], r["run_id"],
            d, r["frequency_hz"], r["kuramoto_R"]
        ))

    with open(OUTPUT_MD, "w") as f:
        f.write("\n".join(lines) + "\n")

    print("\nWrote %d rows to %s" % (len(rows), OUTPUT_MD))


if __name__ == "__main__":
    main()
