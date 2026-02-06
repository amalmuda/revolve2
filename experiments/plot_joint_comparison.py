"""
Joint angle comparison across all ODE-CPG coupling modes vs Parametric Sine.
Re-simulates from saved .npy parameters and plots joint angles over time.

Grid layout: rows = lambda (0, 1, 3), columns = controllers
Spider has 4 legs x 2 joints = 8 hinges (proximal + distal per leg).
"""

import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Color, Pose, UUIDKey
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.standards import modular_robots_v1

# CPG structure builders
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
)

# Sine brain
from brain_sine import BrainSine

# Joint classification
from core_centric import analyze_robot, JointType


# 4 leg colors: same color for proximal + distal of the same leg
LEG_COLORS = [
    "#1f77b4",  # blue  - Leg 1
    "#ff7f0e",  # orange - Leg 2
    "#2ca02c",  # green  - Leg 3
    "#d62728",  # red    - Leg 4
]


def simulate_and_record(robot, simulation_time=30.0, sample_interval=0.01):
    """
    Run simulation and record target angles per hinge at each timestep.
    Uses the official revolve2 mapping (ActiveHinge -> JointHinge -> MuJoCo)
    to guarantee correct hinge-to-actuator correspondence.

    Returns (timestamps, target_angles_dict).
    """
    body = robot.body
    active_hinges = body.find_modules_of_type(ActiveHinge)

    # Create scene
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

    batch_params = make_standard_batch_parameters()
    simulation_scene, _ = scene.to_simulation_scene()
    model, mujoco_mapping = scene_to_model(
        simulation_scene,
        simulation_timestep=batch_params.simulation_timestep,
        cast_shadows=False, fast_sim=True,
    )
    data = mujoco.MjData(model)
    control_interface = ControlInterfaceImpl(
        data=data, abstraction_to_mujoco_mapping=mujoco_mapping
    )

    # === Proper mapping via official revolve2 chain ===
    # ActiveHinge -> JointHinge (via BodyToMultiBodySystemMapping)
    # JointHinge -> JointHingeMujoco (via AbstractionToMujocoMapping)
    body_mapping = simulation_scene.handler._brains[0][1]

    hinge_to_ctrl_idx = {}
    for hinge in active_hinges:
        joint_hinge = body_mapping.active_hinge_to_joint_hinge[UUIDKey(hinge)]
        joint_mujoco = mujoco_mapping.hinge_joint[UUIDKey(joint_hinge)]
        hinge_to_ctrl_idx[hinge] = joint_mujoco.ctrl_index_position

    # Verify all hinges are mapped
    assert len(hinge_to_ctrl_idx) == len(active_hinges), \
        f"Mapping incomplete: {len(hinge_to_ctrl_idx)}/{len(active_hinges)} hinges mapped"

    # Storage
    timestamps = []
    target_angles = {i: [] for i in range(len(active_hinges))}
    current_targets = {h: 0.0 for h in active_hinges}

    control_step = 1.0 / batch_params.control_frequency
    last_control_time = 0.0
    last_sample_time = -1.0

    mujoco.mj_forward(model, data)

    while (time := data.time) < simulation_time:
        # Control step
        if time >= last_control_time + control_step:
            last_control_time = math.floor(time / control_step) * control_step
            simulation_state = SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mujoco_mapping, camera_views={},
            )
            simulation_scene.handler.handle(simulation_state, control_interface, control_step)
            for hinge in active_hinges:
                current_targets[hinge] = data.ctrl[hinge_to_ctrl_idx[hinge]]

        # Sample
        if time >= last_sample_time + sample_interval:
            timestamps.append(time)
            for idx, hinge in enumerate(active_hinges):
                target_angles[idx].append(current_targets[hinge])
            last_sample_time = time

        mujoco.mj_step(model, data)

    return timestamps, target_angles


def build_cpg_robot(robot_name, params_path, coupling, initial_state=math.sqrt(2) * 0.5):
    """Build an ODE-CPG robot from saved parameters."""
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    params = np.load(params_path)

    if coupling == "uncoupled":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_internal_only(active_hinges)
    elif coupling == "neighbor":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)
    elif coupling == "blf":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_blf(active_hinges, body)
    else:
        raise ValueError(f"Unknown coupling: {coupling}")

    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=params,
        cpg_network_structure=cpg_structure,
        initial_state_uniform=initial_state,
        output_mapping=output_mapping,
    )
    return ModularRobot(body=body, brain=brain), params


def build_sine_robot(robot_name, params_path, frequency=0.2):
    """Build a Sine robot from saved parameters."""
    body = modular_robots_v1.get(robot_name)
    params = np.load(params_path)
    brain = BrainSine.from_parameters(body, params, frequency=frequency)
    return ModularRobot(body=body, brain=brain), params


def main():
    print("=" * 60)
    print("Joint Angle Comparison: All Couplings x All Lambdas")
    print("=" * 60)

    robot_name = "spider"

    # Get joint classifications (same for all spider instances)
    ref_body = modular_robots_v1.get(robot_name)
    active_hinges = ref_body.find_modules_of_type(ActiveHinge)
    n_hinges = len(active_hinges)
    cc_result = analyze_robot(ref_body)

    # Spider: 4 legs x 2 joints each = 8 hinges (hip + knee per leg)
    # Tree traversal order: [L1_hip, L1_knee, L2_hip, L2_knee, ...]
    hinge_labels = []
    hinge_colors = []
    hinge_styles = []
    for i in range(n_hinges):
        leg = i // 2 + 1
        is_hip = (i % 2 == 0)
        hinge_labels.append(f"Leg {leg} {'hip' if is_hip else 'knee'}")
        hinge_colors.append(LEG_COLORS[(leg - 1) % len(LEG_COLORS)])
        hinge_styles.append("-" if is_hip else "--")

    print(f"  Hinge labels: {hinge_labels}")

    v3_initial_state = math.sqrt(2) * 0.5  # 0.707

    # Controller columns
    controllers = [
        {"label": "No Coupling", "type": "cpg", "coupling": "uncoupled",
         "dir": "spider_ode_cpg_uncoupled"},
        {"label": "Neighbour", "type": "cpg", "coupling": "neighbor",
         "dir": "spider_ode_cpg_neighbor"},
        {"label": "Structured", "type": "cpg", "coupling": "blf",
         "dir": "spider_ode_cpg_blf"},
        {"label": "Sine (Uncoupled)", "type": "sine", "coupling": "uncoupled",
         "dir": "spider_sine_uncoupled"},
    ]

    lambdas = [0, 1, 3]
    run_num = 1  # Use Run 1 consistently for all configs

    # V3 results for Run 1 of each config (from V3_VS_V4_RESULTS.txt)
    run1_stats = {
        # (dir_prefix, lambda): (distance, dragging)
        ("spider_ode_cpg_uncoupled", 0): (1.72, 68.0),
        ("spider_ode_cpg_uncoupled", 1): (1.81, 32.5),
        ("spider_ode_cpg_uncoupled", 3): (1.15, 10.4),
        ("spider_ode_cpg_neighbor", 0): (2.68, 59.4),
        ("spider_ode_cpg_neighbor", 1): (2.19, 20.0),
        ("spider_ode_cpg_neighbor", 3): (1.41, 0.5),
        ("spider_ode_cpg_blf", 0): (2.95, 48.7),
        ("spider_ode_cpg_blf", 1): (3.04, 6.6),
        ("spider_ode_cpg_blf", 3): (2.58, 0.2),
        ("spider_sine_uncoupled", 0): (3.82, 12.8),
        ("spider_sine_uncoupled", 1): (2.91, 0.2),
        ("spider_sine_uncoupled", 3): (3.53, 0.2),
    }

    n_rows = len(lambdas)
    n_cols = len(controllers)
    total_panels = n_rows * n_cols

    # Simulate all panels
    sim_results = {}  # (row, col) -> (timestamps, targets)
    panel_titles = {}

    panel_num = 0
    for row, lam in enumerate(lambdas):
        for col, ctrl in enumerate(controllers):
            panel_num += 1
            params_path = f"results/comparison_v3/{ctrl['dir']}_lambda{lam}_dragging/best_params_run_{run_num}.npy"

            stats = run1_stats.get((ctrl['dir'], lam), (0, 0))
            dist_str, drag_str = f"{stats[0]:.2f}", f"{stats[1]:.1f}"

            title = f"{ctrl['label']}\n{dist_str}m, {drag_str}% drag"
            panel_titles[(row, col)] = title

            print(f"\n[{panel_num}/{total_panels}] λ={lam}, {ctrl['label']}")
            if ctrl["type"] == "cpg":
                robot, params = build_cpg_robot(robot_name, params_path, ctrl["coupling"],
                                                initial_state=v3_initial_state)
            else:
                robot, params = build_sine_robot(robot_name, params_path)
            print(f"  Params: {params.shape}, Simulating...")
            ts, targets = simulate_and_record(robot, simulation_time=30.0)
            print(f"  {len(ts)} samples")
            sim_results[(row, col)] = (ts, targets)

    # ===== Plot: 3x4 grid =====
    print("\n--- Generating plot ---")

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(24, 14), sharey=True, sharex=True)

    t_start, t_end = 0.0, 30.0

    for row in range(n_rows):
        for col in range(n_cols):
            ax = axes[row, col]
            ts, target_data = sim_results[(row, col)]

            ts_arr = np.array(ts)
            mask = (ts_arr >= t_start) & (ts_arr <= t_end)
            t_plot = ts_arr[mask]

            for idx in range(n_hinges):
                target_arr = np.array(target_data[idx])[mask]
                ax.plot(t_plot, target_arr,
                        color=hinge_colors[idx],
                        linestyle=hinge_styles[idx],
                        linewidth=1.5 if hinge_styles[idx] == "-" else 1.0,
                        alpha=0.85,
                        label=hinge_labels[idx] if row == 0 and col == 0 else None)

            ax.set_title(panel_titles[(row, col)], fontsize=10, fontweight='bold')
            ax.grid(True, linestyle='--', alpha=0.4)
            ax.axhline(y=0, color='black', linewidth=0.5, alpha=0.3)
            ax.set_xlim(t_start, t_end)

            if col == 0:
                ax.set_ylabel(f"λ={lambdas[row]}\nTarget angle (rad)", fontsize=10)
            if row == n_rows - 1:
                ax.set_xlabel("Time (s)", fontsize=10)

    # Single shared legend at top
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=4,
               fontsize=9, bbox_to_anchor=(0.5, 1.0))

    fig.suptitle("Spider Joint Angles: ODE-CPG Coupling Modes vs Parametric Sine (V3, Run 1)",
                 fontsize=14, fontweight='bold', y=1.03)
    plt.tight_layout(rect=[0, 0, 1, 0.97])

    output_path = "joint_comparison_all_couplings.png"
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nSaved to: {output_path}")
    print("Done!")


if __name__ == "__main__":
    main()
