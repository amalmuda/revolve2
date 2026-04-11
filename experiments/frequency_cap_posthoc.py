"""
Post-hoc frequency-capping experiment.

For each evolved genome (spider/gecko × uncoupled/neighbour/structured × λ 0-3 × 30 runs):
  1. Load saved params, reconstruct weight matrix W
  2. Compute f_nat = largest |Im| eigenvalue of W (in Hz)
  3. Pick f_cap = min of median f_nat across all conditions
  4. k = min(1, f_cap / f_nat); W_scaled = k * W
  5. If k == 1: copy original distance/dragging (no re-sim needed)
     Else: re-simulate with W_scaled using full MuJoCo physics
  6. Output CSV + boxplots + printed summary

Notes:
- CPG clipping to [-1, 1] makes scaling only approximately frequency-preserving.
- Physics is unchanged, only the controller's W is scaled.
"""
import os
import math
import sqlite3
import sys
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
    get_robot_core_body_id,
    identify_geometry_types,
    get_contacts_with_ground,
)


FINAL_RESULTS = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/results/final_results"
)
OUTPUT_CSV = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/frequency_cap_results.csv"
)
OUTPUT_PLOT_SPIDER = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/frequency_cap_spider.pdf"
)
OUTPUT_PLOT_GECKO = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/frequency_cap_gecko.pdf"
)


def get_cpg_structure(coupling, body, hinges):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    if coupling == "neighbor":
        return active_hinges_to_cpg_network_structure_neighbor(hinges)
    if coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)


def compute_natural_frequency(W):
    """Largest |Im(eigenvalue)| of W, converted to Hz."""
    eigs = np.linalg.eigvals(W)
    max_imag = np.max(np.abs(eigs.imag))
    return max_imag / (2.0 * math.pi)  # rad/s -> Hz


def get_original_distance_dragging(db_path):
    try:
        conn = sqlite3.connect(db_path)
        cur = conn.cursor()
        cur.execute("""
            SELECT i.distance, i.dragging FROM comparison_individual i
            JOIN comparison_population p ON i.population_id = p.id
            JOIN comparison_generation g ON g.population_id = p.id
            WHERE g.generation_index = (SELECT MAX(generation_index) FROM comparison_generation)
            ORDER BY i.fitness DESC LIMIT 1
        """)
        row = cur.fetchone()
        conn.close()
        if row and row[0] is not None:
            return float(row[0]), float(row[1])
    except Exception:
        pass
    return None, None


def simulate_with_custom_W(body, cpg_structure, output_mapping, W_scaled, simulation_time=30.0):
    """Run full MuJoCo simulation with a custom weight matrix. Returns (distance, dragging)."""
    initial_state = cpg_structure.make_uniform_state(math.sqrt(2) * 0.5)
    brain = BrainCpgNetworkStatic(
        initial_state=initial_state,
        weight_matrix=W_scaled,
        output_mapping=output_mapping,
    )
    robot = ModularRobot(body=body, brain=brain)

    terrain = Terrain(
        static_geometry=[
            GeometryPlane(
                pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
                texture=Texture(
                    base_color=Color(200, 200, 200, 255),
                    primary_color=Color(220, 220, 220, 255),
                    secondary_color=Color(80, 80, 80, 255),
                    map_type=MapType.MAP2D,
                    reference=TextureReference(builtin="checker"),
                    repeat=(50, 50),
                ),
            )
        ],
        friction=1.0,
    )
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)
    simulation_scene, _ = scene.to_simulation_scene()

    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = simulation_time

    model, mujoco_mapping = scene_to_model(
        simulation_scene,
        simulation_timestep=batch_params.simulation_timestep,
        cast_shadows=False,
        fast_sim=True,
    )
    data = mujoco.MjData(model)

    core_body_id = get_robot_core_body_id(model)
    if core_body_id is None:
        return 0.0, 1.0

    control_interface = ControlInterfaceImpl(
        data=data, abstraction_to_mujoco_mapping=mujoco_mapping
    )

    ground_geom_ids, robot_geom_ids, foot_geom_ids = identify_geometry_types(model)
    non_foot_geom_ids = robot_geom_ids - foot_geom_ids

    control_step = 1.0 / batch_params.control_frequency
    last_control_time = 0.0

    mujoco.mj_forward(model, data)
    initial_pos = data.xpos[core_body_id].copy()

    total_timesteps = 0
    timesteps_with_dragging = 0

    while data.time < simulation_time:
        total_timesteps += 1

        contacts = get_contacts_with_ground(model, data, ground_geom_ids, robot_geom_ids)
        for rg, _, _, _ in contacts:
            if rg in non_foot_geom_ids:
                timesteps_with_dragging += 1
                break

        if data.time >= last_control_time + control_step:
            last_control_time = math.floor(data.time / control_step) * control_step
            state = SimulationStateImpl(
                data=data,
                abstraction_to_mujoco_mapping=mujoco_mapping,
                camera_views={},
            )
            simulation_scene.handler.handle(state, control_interface, control_step)

        mujoco.mj_step(model, data)

    final_pos = data.xpos[core_body_id].copy()
    dx = final_pos[0] - initial_pos[0]
    dy = final_pos[1] - initial_pos[1]
    distance = math.sqrt(dx * dx + dy * dy)
    dragging = timesteps_with_dragging / total_timesteps if total_timesteps > 0 else 1.0
    return distance, dragging


def main():
    morphologies = [
        ("spider", "Spider"),
        ("gecko", "Gecko"),
    ]
    couplings = [
        ("uncoupled", "No coupling"),
        ("neighbor", "Neighbour"),
        ("blf", "Structured"),
    ]
    lambdas = [0, 1, 2, 3]

    # ========= Step 1-2: Load genomes, compute f_nat =========
    print("=" * 70)
    print("STEP 1-2: Loading genomes and computing natural frequencies")
    print("=" * 70)

    # entries[i] = {morphology, coupling, lam, run, body, cpg, mapping, W, f_nat, dist_orig, drag_orig}
    entries = []

    for robot_key, robot_label in morphologies:
        body = modular_robots_v1.get(robot_key)
        hinges = body.find_modules_of_type(ActiveHinge)
        for cpl_key, cpl_label in couplings:
            cpg_struct, mapping = get_cpg_structure(cpl_key, body, hinges)
            for lam in lambdas:
                exp_dir = "%s_ode_cpg_%s_lambda%d_dragging" % (robot_key, cpl_key, lam)
                exp_path = os.path.join(FINAL_RESULTS, exp_dir)
                if not os.path.isdir(exp_path):
                    continue
                for run in range(1, 31):
                    npy = os.path.join(exp_path, "best_params_run_%d.npy" % run)
                    db = os.path.join(exp_path, "run_%d.sqlite" % run)
                    if not os.path.exists(npy):
                        continue
                    params = np.load(npy)
                    W = cpg_struct.make_connection_weights_matrix_from_params(list(params))
                    f_nat = compute_natural_frequency(W)
                    d_orig, dr_orig = get_original_distance_dragging(db)
                    entries.append({
                        "robot_key": robot_key,
                        "morphology": robot_label,
                        "coupling": cpl_label,
                        "coupling_key": cpl_key,
                        "lam": lam,
                        "run": run,
                        "body": body,
                        "cpg_struct": cpg_struct,
                        "mapping": mapping,
                        "W": W,
                        "f_nat": f_nat,
                        "dist_orig": d_orig,
                        "drag_orig": dr_orig,
                    })

    print("Loaded %d genomes" % len(entries))

    # ========= Step 3: Frequency distributions =========
    print("\n" + "=" * 70)
    print("STEP 3: Natural frequency distributions")
    print("=" * 70)
    print("%-12s %-14s %-8s %-16s %-16s" % (
        "Morphology", "Coupling", "λ", "f_nat mean±std", "f_nat median"
    ))
    print("-" * 70)

    all_median_fnat = []

    for robot_label in ["Spider", "Gecko"]:
        for cpl_label in ["No coupling", "Neighbour", "Structured"]:
            for lam in lambdas:
                cell = [e["f_nat"] for e in entries
                        if e["morphology"] == robot_label
                        and e["coupling"] == cpl_label
                        and e["lam"] == lam]
                if cell:
                    mean_f = np.mean(cell)
                    std_f = np.std(cell)
                    med_f = np.median(cell)
                    all_median_fnat.append(med_f)
                    print("%-12s %-14s %-8d %-16s %-16s" % (
                        robot_label, cpl_label, lam,
                        "%.3f ± %.3f" % (mean_f, std_f),
                        "%.3f" % med_f,
                    ))

    # ========= Step 4: Pick f_cap =========
    f_cap = min(all_median_fnat)
    print("\n" + "=" * 70)
    print("STEP 4: Frequency cap")
    print("=" * 70)
    print("f_cap = min(median f_nat across all conditions) = %.4f Hz" % f_cap)
    print("This is the lowest median frequency observed (= uncoupled-ish).")
    print("Every condition can at least reach this frequency.")

    # ========= Step 5-6: Scale W and re-simulate if needed =========
    print("\n" + "=" * 70)
    print("STEP 5-6: Scale and re-simulate (only where k < 1)")
    print("=" * 70)

    n_total = len(entries)
    n_capped = sum(1 for e in entries if e["f_nat"] > f_cap)
    print("Will re-simulate %d / %d entries (rest are already below cap)" % (n_capped, n_total))

    sim_count = 0
    for i, e in enumerate(entries):
        if e["f_nat"] <= f_cap or e["f_nat"] == 0:
            e["k"] = 1.0
            e["dist_capped"] = e["dist_orig"]
            e["drag_capped"] = e["drag_orig"]
            continue
        k = f_cap / e["f_nat"]
        e["k"] = k
        W_scaled = k * e["W"]
        sim_count += 1
        try:
            d, dr = simulate_with_custom_W(e["body"], e["cpg_struct"], e["mapping"], W_scaled)
            e["dist_capped"] = d
            e["drag_capped"] = dr
        except Exception as ex:
            print("ERROR sim %d/%d (%s %s λ=%d run=%d): %s" % (
                sim_count, n_capped, e["morphology"], e["coupling"], e["lam"], e["run"], ex
            ))
            e["dist_capped"] = None
            e["drag_capped"] = None
        if sim_count % 20 == 0:
            print("  [%d/%d] sims done" % (sim_count, n_capped))

    # ========= Step 7: CSV output =========
    print("\n" + "=" * 70)
    print("STEP 7: Writing CSV and plots")
    print("=" * 70)

    with open(OUTPUT_CSV, "w") as f:
        f.write("morphology,coupling,lambda,run_id,f_nat,k,distance_original,distance_capped,dragging_original,dragging_capped\n")
        for e in entries:
            f.write("%s,%s,%d,%d,%.6f,%.6f,%.4f,%s,%.4f,%s\n" % (
                e["morphology"], e["coupling"], e["lam"], e["run"],
                e["f_nat"], e["k"],
                e["dist_orig"] if e["dist_orig"] is not None else 0.0,
                ("%.4f" % e["dist_capped"]) if e["dist_capped"] is not None else "",
                e["drag_orig"] if e["drag_orig"] is not None else 0.0,
                ("%.4f" % e["drag_capped"]) if e["drag_capped"] is not None else "",
            ))
    print("Wrote CSV: %s" % OUTPUT_CSV)

    # ========= Summary print =========
    print("\n" + "=" * 70)
    print("SUMMARY: mean distance before vs after capping")
    print("=" * 70)
    print("%-12s %-14s %-4s %-14s %-14s %-10s" % (
        "Morphology", "Coupling", "λ", "Dist orig", "Dist capped", "Δ (%)"
    ))
    print("-" * 72)

    for robot_label in ["Spider", "Gecko"]:
        for cpl_label in ["No coupling", "Neighbour", "Structured"]:
            for lam in lambdas:
                cell = [e for e in entries
                        if e["morphology"] == robot_label
                        and e["coupling"] == cpl_label
                        and e["lam"] == lam
                        and e["dist_orig"] is not None and e["dist_capped"] is not None]
                if not cell:
                    continue
                d_o = np.mean([e["dist_orig"] for e in cell])
                d_c = np.mean([e["dist_capped"] for e in cell])
                s_o = np.std([e["dist_orig"] for e in cell])
                s_c = np.std([e["dist_capped"] for e in cell])
                rel = ((d_c - d_o) / d_o) * 100 if d_o > 0 else 0
                print("%-12s %-14s %-4d %5.2f ± %4.2f  %5.2f ± %4.2f  %+6.1f%%" % (
                    robot_label, cpl_label, lam, d_o, s_o, d_c, s_c, rel
                ))

    # ========= Plots: boxplots per morphology =========
    def make_plot(robot_label, out_path):
        fig, axes = plt.subplots(1, 4, figsize=(16, 5), sharey=True)
        fig.suptitle("%s — Distance before vs after frequency cap (f_cap = %.3f Hz)" % (robot_label, f_cap),
                     fontsize=13, weight="bold")
        for col, lam in enumerate(lambdas):
            ax = axes[col]
            data_orig = []
            data_capped = []
            labels = []
            colors = []
            for cpl_label in ["No coupling", "Neighbour", "Structured"]:
                cell = [e for e in entries
                        if e["morphology"] == robot_label
                        and e["coupling"] == cpl_label
                        and e["lam"] == lam
                        and e["dist_orig"] is not None and e["dist_capped"] is not None]
                orig = [e["dist_orig"] for e in cell]
                capped = [e["dist_capped"] for e in cell]
                data_orig.append(orig)
                data_capped.append(capped)
                labels.append(cpl_label)
            positions_orig = [1, 3, 5]
            positions_capped = [1.8, 3.8, 5.8]
            bp1 = ax.boxplot(data_orig, positions=positions_orig, widths=0.7,
                             patch_artist=True, showfliers=True)
            bp2 = ax.boxplot(data_capped, positions=positions_capped, widths=0.7,
                             patch_artist=True, showfliers=True)
            for patch in bp1["boxes"]:
                patch.set_facecolor("#4a7abc")
                patch.set_alpha(0.7)
            for patch in bp2["boxes"]:
                patch.set_facecolor("#dd6622")
                patch.set_alpha(0.7)
            ax.set_xticks([1.4, 3.4, 5.4])
            ax.set_xticklabels(labels, rotation=20, fontsize=9)
            ax.set_title("λ = %d" % lam, fontsize=11)
            ax.grid(alpha=0.3, axis="y")
            if col == 0:
                ax.set_ylabel("Distance (m)")
            if col == 3:
                ax.legend([bp1["boxes"][0], bp2["boxes"][0]],
                          ["Original", "Capped"], loc="upper right", fontsize=9)
        plt.tight_layout()
        plt.savefig(out_path, bbox_inches="tight")
        plt.close()
        print("Saved plot: %s" % out_path)

    make_plot("Spider", OUTPUT_PLOT_SPIDER)
    make_plot("Gecko", OUTPUT_PLOT_GECKO)


if __name__ == "__main__":
    main()
