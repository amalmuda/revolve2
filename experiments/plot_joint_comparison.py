"""
Plot CPG joint outputs for 4 spider coupling topologies side by side.

For each coupling (uncoupled, neighbour, structured, fully_connected) at lambda=0,
finds the median-fitness run and integrates the CPG ODE to get joint outputs over time.

Output: 4x4 grid PDF (4 couplings rows x 4 legs columns).
Each cell shows hip + knee for one leg.
"""
import os
import math
import sqlite3
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_fully_connected,
)


def get_cpg_structure(coupling, body, hinges):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "neighbor":
        return active_hinges_to_cpg_network_structure_neighbor(hinges)
    elif coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)
    elif coupling == "fully_connected":
        return active_hinges_to_cpg_network_structure_fully_connected(hinges)


def integrate_cpg(weight_matrix, initial_state, dt, n_steps):
    """RK45 integration of X' = WX, clipped to [-1, 1]."""
    state = initial_state.copy()
    history = np.zeros((n_steps, len(state)))
    for i in range(n_steps):
        history[i] = state
        A1 = weight_matrix @ state
        A2 = weight_matrix @ (state + dt / 2 * A1)
        A3 = weight_matrix @ (state + dt / 2 * A2)
        A4 = weight_matrix @ (state + dt * A3)
        state = state + dt / 6 * (A1 + 2 * A2 + 2 * A3 + A4)
        state = np.clip(state, -1.0, 1.0)
    return history


def find_median_run(exp_path):
    """Find the run number whose final-gen best fitness is closest to median."""
    fitnesses = []
    for run_num in range(1, 31):
        db = os.path.join(exp_path, "run_%d.sqlite" % run_num)
        if not os.path.exists(db):
            continue
        try:
            conn = sqlite3.connect(db)
            cur = conn.cursor()
            cur.execute("""
                SELECT i.fitness FROM comparison_individual i
                JOIN comparison_population p ON i.population_id = p.id
                JOIN comparison_generation g ON g.population_id = p.id
                WHERE g.generation_index = (SELECT MAX(generation_index) FROM comparison_generation)
                ORDER BY i.fitness DESC LIMIT 1
            """)
            row = cur.fetchone()
            if row:
                fitnesses.append((run_num, row[0]))
            conn.close()
        except:
            pass
    if not fitnesses:
        return None
    sorted_runs = sorted(fitnesses, key=lambda x: x[1])
    median_idx = len(sorted_runs) // 2
    return sorted_runs[median_idx][0]


def main():
    body = modular_robots_v1.get("spider")
    hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(hinges)

    # Hinge index -> (leg_name, joint_type)
    hinge_labels = [
        ("Left", "Hip"),
        ("Left", "Knee"),
        ("Right", "Hip"),
        ("Right", "Knee"),
        ("Front", "Hip"),
        ("Front", "Knee"),
        ("Back", "Hip"),
        ("Back", "Knee"),
    ]
    legs = ["Left", "Right", "Front", "Back"]

    # Coupling configs: (key, label, results_dir)
    couplings = [
        ("uncoupled", "Uncoupled", "results/final_results/spider_ode_cpg_uncoupled_lambda0_dragging"),
        ("neighbor", "Neighbour", "results/final_results/spider_ode_cpg_neighbor_lambda0_dragging"),
        ("blf", "Structured", "results/final_results/spider_ode_cpg_blf_lambda0_dragging"),
        ("fully_connected", "Fully connected", "results/fully_connected_local/spider_ode_cpg_fully_connected_lambda0_dragging"),
    ]

    # CPG integration parameters
    sim_time = 30.0
    control_freq = 20  # Hz
    dt = 1.0 / control_freq
    n_steps = int(sim_time / dt)
    time_axis = np.arange(n_steps) * dt

    # Set up plot: 4 rows (couplings) x 4 cols (legs)
    fig, axes = plt.subplots(4, 4, figsize=(14, 10), sharex=True, sharey=True)
    fig.suptitle("Spider CPG joint outputs by coupling topology (lambda=0, median run)", fontsize=14)

    for row, (cpl_key, cpl_label, results_dir) in enumerate(couplings):
        # Find median run
        if not os.path.isdir(results_dir):
            print("WARN: %s not found" % results_dir)
            for col in range(4):
                axes[row, col].text(0.5, 0.5, "no data", ha="center", va="center", transform=axes[row, col].transAxes)
            continue

        median_run = find_median_run(results_dir)
        if median_run is None:
            print("WARN: no runs in %s" % results_dir)
            continue

        params_path = os.path.join(results_dir, "best_params_run_%d.npy" % median_run)
        if not os.path.exists(params_path):
            print("WARN: %s not found" % params_path)
            continue

        params = np.load(params_path)

        # Build CPG structure and weight matrix
        cpg_structure, output_mapping = get_cpg_structure(cpl_key, body, hinges)
        weight_matrix = cpg_structure.make_connection_weights_matrix_from_params(list(params))
        initial_state = cpg_structure.make_uniform_state(math.sqrt(2) * 0.5)

        # Integrate
        history = integrate_cpg(weight_matrix, initial_state, dt, n_steps)
        # Output is first n_hinges elements of state
        outputs = history[:, :n_hinges]

        # Plot each leg in its own column
        for col, leg_name in enumerate(legs):
            ax = axes[row, col]
            hip_idx = next(i for i, (l, j) in enumerate(hinge_labels) if l == leg_name and j == "Hip")
            knee_idx = next(i for i, (l, j) in enumerate(hinge_labels) if l == leg_name and j == "Knee")

            ax.plot(time_axis, outputs[:, hip_idx], label="Hip", color="#2266bb", linewidth=1.4)
            ax.plot(time_axis, outputs[:, knee_idx], label="Knee", color="#dd6622", linewidth=1.4, linestyle="--")
            ax.axhline(0, color="gray", linewidth=0.4, alpha=0.5)
            ax.set_ylim(-1.1, 1.1)
            ax.grid(alpha=0.3)

            if row == 0:
                ax.set_title("%s leg" % leg_name, fontsize=11)
            if col == 0:
                ax.set_ylabel(cpl_label, fontsize=11)
            if row == 3:
                ax.set_xlabel("Time (s)")
            if row == 0 and col == 3:
                ax.legend(loc="upper right", fontsize=8)

        print("%s: median run %d" % (cpl_label, median_run))

    plt.tight_layout()
    out_path = os.path.expanduser("~/masteroppgave/revolve2/experiments/spider_joint_comparison.pdf")
    plt.savefig(out_path, bbox_inches="tight")
    print("Saved: %s" % out_path)


if __name__ == "__main__":
    main()
