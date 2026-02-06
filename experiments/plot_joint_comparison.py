"""
Side-by-side joint angle comparison: high-dragging ODE-CPG vs low-dragging Sine.
Re-simulates from saved .npy parameters and plots joint angles over time.
"""

import math
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for headless servers
import matplotlib.pyplot as plt

import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.standards import modular_robots_v1

# Import CPG structure builders
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)
from blf_cpg import active_hinges_to_cpg_network_structure_blf

# Import Sine brain
from brain_sine import BrainSine

# Import joint classification
from core_centric import analyze_robot, JointType


JOINT_COLORS = {
    JointType.SPINE: "#1f77b4",    # blue
    JointType.HIP: "#2ca02c",      # green
    JointType.KNEE: "#ff7f0e",     # orange
    JointType.ANKLE: "#d62728",    # red
    JointType.LOCKED: "#7f7f7f",   # gray
    JointType.UNCLASSIFIED: "#9467bd",  # purple
}


def simulate_and_record(robot, simulation_time=30.0, sample_interval=0.01):
    """
    Run simulation and record joint angles at each timestep.
    Works with ANY brain type (ODE-CPG, Sine, etc.).
    Returns (timestamps, joint_angles_dict, target_angles_dict).
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
    control_interface = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mujoco_mapping)

    # Map hinges to MuJoCo joint IDs
    hinge_to_joint_id = {}
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name and "mbs" in joint_name:
            for hinge in active_hinges:
                if hinge not in hinge_to_joint_id:
                    if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
                        hinge_to_joint_id[hinge] = i
                        break

    # Map hinges to MuJoCo actuator IDs
    hinge_to_actuator_id = {}
    actuator_idx = 0
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if actuator_name and "mbs" in actuator_name:
            if actuator_idx < len(active_hinges):
                hinge_to_actuator_id[active_hinges[actuator_idx]] = i
                actuator_idx += 1

    # Storage
    timestamps = []
    joint_angles = {i: [] for i in range(len(active_hinges))}
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
                if hinge in hinge_to_actuator_id:
                    current_targets[hinge] = data.ctrl[hinge_to_actuator_id[hinge]]

        # Sample
        if time >= last_sample_time + sample_interval:
            timestamps.append(time)
            for idx, hinge in enumerate(active_hinges):
                if hinge in hinge_to_joint_id:
                    qpos_idx = model.jnt_qposadr[hinge_to_joint_id[hinge]]
                    joint_angles[idx].append(data.qpos[qpos_idx])
                else:
                    joint_angles[idx].append(0.0)
                target_angles[idx].append(current_targets.get(hinge, 0.0))
            last_sample_time = time

        mujoco.mj_step(model, data)

    return timestamps, joint_angles, target_angles


def main():
    print("=" * 60)
    print("Joint Angle Comparison: ODE-CPG vs Sine")
    print("=" * 60)

    robot_name = "spider"
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(active_hinges)

    # Get joint classifications
    cc_result = analyze_robot(body)
    joint_types = []
    for hinge in active_hinges:
        jt = cc_result.joint_types.get(hinge, JointType.UNCLASSIFIED)
        joint_types.append(jt)

    # ===== Run 1: ODE-CPG Structured (BLF) lambda=0, Run 4 (62.59% drag) =====
    print("\n--- Loading ODE-CPG Structured Run 4 ---")
    cpg_params = np.load("results/comparison_v3/spider_ode_cpg_blf_lambda0_dragging/best_params_run_4.npy")
    print(f"  Params shape: {cpg_params.shape}")

    # Create BLF CPG structure
    cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_blf(active_hinges, body)
    brain_cpg = BrainCpgNetworkStatic.uniform_from_params(
        params=cpg_params,
        cpg_network_structure=cpg_structure,
        initial_state_uniform=math.sqrt(2) * 0.5,  # v3 used 0.707
        output_mapping=output_mapping,
    )
    robot_cpg = ModularRobot(body=body, brain=brain_cpg)

    print("  Simulating (30s)...")
    ts_cpg, angles_cpg, targets_cpg = simulate_and_record(robot_cpg, simulation_time=30.0)
    print(f"  Recorded {len(ts_cpg)} samples")

    # ===== Run 2: Sine Uncoupled lambda=0, Run 3 (0.26% drag) =====
    print("\n--- Loading Sine Uncoupled Run 3 ---")
    # Need fresh body for each robot
    body2 = modular_robots_v1.get(robot_name)
    sine_params = np.load("results/comparison_v3/spider_sine_uncoupled_lambda0_dragging/best_params_run_3.npy")
    print(f"  Params shape: {sine_params.shape}")

    brain_sine = BrainSine.from_parameters(body2, sine_params, frequency=0.2)
    robot_sine = ModularRobot(body=body2, brain=brain_sine)

    print("  Simulating (30s)...")
    ts_sine, angles_sine, targets_sine = simulate_and_record(robot_sine, simulation_time=30.0)
    print(f"  Recorded {len(ts_sine)} samples")

    # ===== Plot: side-by-side, last 10 seconds =====
    print("\n--- Generating plot ---")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 7), sharey=True)

    # Time window: last 10 seconds (20-30s) when gait is settled
    t_start, t_end = 20.0, 30.0

    for ax, ts, angles, targets, title_text in [
        (ax1, ts_cpg, targets_cpg, angles_cpg,
         "Revolve2 CPG (Structured, λ=0)\nRun 4 — Distance: 2.79m, Dragging: 62.6%"),
        (ax2, ts_sine, targets_sine, angles_sine,
         "Parametric Sine CPG (Uncoupled, λ=0)\nRun 3 — Distance: 3.65m, Dragging: 0.3%"),
    ]:
        ts_arr = np.array(ts)
        mask = (ts_arr >= t_start) & (ts_arr <= t_end)
        t_plot = ts_arr[mask]

        # Track which joint types we've already added to legend
        legend_added = set()

        for idx in range(n_hinges):
            jt = joint_types[idx]
            color = JOINT_COLORS.get(jt, "black")
            label = f"{jt.value}" if jt not in legend_added else None
            legend_added.add(jt)

            # Plot target angles (what the controller commands)
            target_arr = np.array(targets[idx])[mask]
            ax.plot(t_plot, target_arr, color=color, linewidth=1.0, alpha=0.7, label=label)

        ax.set_xlabel("Time (s)", fontsize=12)
        ax.set_title(title_text, fontsize=12, fontweight='bold')
        ax.grid(True, linestyle='--', alpha=0.4)
        ax.axhline(y=0, color='black', linewidth=0.5, alpha=0.3)
        ax.set_xlim(t_start, t_end)
        ax.legend(loc='upper right', fontsize=9)

    ax1.set_ylabel("Joint Angle (radians)", fontsize=12)

    fig.suptitle("Spider Joint Angle Comparison: High-Dragging CPG vs Low-Dragging Sine",
                 fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()

    output_path = "joint_comparison_cpg_vs_sine.png"
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nSaved to: {output_path}")
    print("Done!")


if __name__ == "__main__":
    main()
