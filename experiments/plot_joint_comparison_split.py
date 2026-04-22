"""
Plot CPG joint outputs for spider and gecko under split-bounds experiments.

3 couplings (uncoupled, neighbour, structured) at lambda=0.
Generates 2 PDFs: spider_joint_comparison_split.pdf and gecko_joint_comparison_split.pdf.
"""
import os
import math
import sqlite3
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
)


def get_cpg_structure(coupling, body, hinges):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "neighbor":
        return active_hinges_to_cpg_network_structure_neighbor(hinges)
    elif coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)


def integrate_cpg(weight_matrix, initial_state, dt, n_steps):
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
    return sorted_runs[len(sorted_runs) // 2][0]


def plot_spider():
    body = modular_robots_v1.get("spider")
    hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(hinges)

    legs = ["Left", "Right", "Front", "Back"]
    hinge_labels = [
        ("Left", "Hip"), ("Left", "Knee"),
        ("Right", "Hip"), ("Right", "Knee"),
        ("Front", "Hip"), ("Front", "Knee"),
        ("Back", "Hip"), ("Back", "Knee"),
    ]

    couplings = [
        ("uncoupled", "Uncoupled", "results/split_bounds_local/spider_ode_cpg_uncoupled_lambda0_dragging"),
        ("neighbor", "Neighbour", "results/split_bounds_local/spider_ode_cpg_neighbor_lambda0_dragging"),
        ("blf", "Structured", "results/split_bounds_local/spider_ode_cpg_blf_lambda0_dragging"),
    ]

    sim_time = 30.0
    dt = 1.0 / 20
    n_steps = int(sim_time / dt)
    time_axis = np.arange(n_steps) * dt

    fig, axes = plt.subplots(3, 4, figsize=(14, 8), sharex=True, sharey=True)
    fig.suptitle("Spider CPG joint outputs (split-bounds, lambda=0, median run)", fontsize=14)

    for row, (cpl_key, cpl_label, results_dir) in enumerate(couplings):
        median_run = find_median_run(results_dir)
        params_path = os.path.join(results_dir, "best_params_run_%d.npy" % median_run)
        params = np.load(params_path)

        cpg_structure, _ = get_cpg_structure(cpl_key, body, hinges)
        weight_matrix = cpg_structure.make_connection_weights_matrix_from_params(list(params))
        initial_state = cpg_structure.make_uniform_state(math.sqrt(2) * 0.5)
        outputs = integrate_cpg(weight_matrix, initial_state, dt, n_steps)[:, :n_hinges]

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
            if row == 2:
                ax.set_xlabel("Time (s)")
            if row == 0 and col == 3:
                ax.legend(loc="upper right", fontsize=8)

        print("Spider %s: median run %d" % (cpl_label, median_run))

    plt.tight_layout()
    out_path = os.path.expanduser("~/masteroppgave/revolve2/experiments/spider_joint_comparison_split.pdf")
    plt.savefig(out_path, bbox_inches="tight")
    print("Saved: %s" % out_path)
    plt.close()


def plot_gecko():
    body = modular_robots_v1.get("gecko")
    hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(hinges)

    column_groups = [
        ("Front legs", [(0, "Front L", "#2266bb", "-"), (1, "Front R", "#dd6622", "--")]),
        ("Spine",      [(2, "Spine 1", "#2266bb", "-"), (3, "Spine 2", "#dd6622", "--")]),
        ("Rear legs",  [(4, "Rear L",  "#2266bb", "-"), (5, "Rear R",  "#dd6622", "--")]),
    ]

    couplings = [
        ("uncoupled", "Uncoupled", "results/split_bounds_local/gecko_ode_cpg_uncoupled_lambda0_dragging"),
        ("neighbor", "Neighbour", "results/split_bounds_local/gecko_ode_cpg_neighbor_lambda0_dragging"),
        ("blf", "Structured", "results/split_bounds_local/gecko_ode_cpg_blf_lambda0_dragging"),
    ]

    sim_time = 30.0
    dt = 1.0 / 20
    n_steps = int(sim_time / dt)
    time_axis = np.arange(n_steps) * dt

    fig, axes = plt.subplots(3, 3, figsize=(12, 8), sharex=True, sharey=True)
    fig.suptitle("Gecko CPG joint outputs (split-bounds, lambda=0, median run)", fontsize=14)

    for row, (cpl_key, cpl_label, results_dir) in enumerate(couplings):
        median_run = find_median_run(results_dir)
        params_path = os.path.join(results_dir, "best_params_run_%d.npy" % median_run)
        params = np.load(params_path)

        cpg_structure, _ = get_cpg_structure(cpl_key, body, hinges)
        weight_matrix = cpg_structure.make_connection_weights_matrix_from_params(list(params))
        initial_state = cpg_structure.make_uniform_state(math.sqrt(2) * 0.5)
        outputs = integrate_cpg(weight_matrix, initial_state, dt, n_steps)[:, :n_hinges]

        for col, (group_name, lines) in enumerate(column_groups):
            ax = axes[row, col]
            for hinge_idx, label, color, style in lines:
                ax.plot(time_axis, outputs[:, hinge_idx], label=label, color=color, linewidth=1.4, linestyle=style)
            ax.axhline(0, color="gray", linewidth=0.4, alpha=0.5)
            ax.set_ylim(-1.1, 1.1)
            ax.grid(alpha=0.3)
            if row == 0:
                ax.set_title(group_name, fontsize=11)
            if col == 0:
                ax.set_ylabel(cpl_label, fontsize=11)
            if row == 2:
                ax.set_xlabel("Time (s)")
            ax.legend(loc="upper right", fontsize=7, ncol=2, frameon=True)

        print("Gecko %s: median run %d" % (cpl_label, median_run))

    plt.tight_layout()
    out_path = os.path.expanduser("~/masteroppgave/revolve2/experiments/gecko_joint_comparison_split.pdf")
    plt.savefig(out_path, bbox_inches="tight")
    print("Saved: %s" % out_path)
    plt.close()


if __name__ == "__main__":
    plot_spider()
    plot_gecko()
