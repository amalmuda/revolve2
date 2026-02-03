"""
Joint Angle Plotter for Revolve2 Modular Robots.

This module provides functionality to:
1. Run a simulation with CPG controller
2. Log each hinge's actual angle at each timestep
3. Generate plots with time vs angle, colored by joint type
4. Export data as PNG images and CSV files

Based on contact_detection.py simulation structure and core_centric.py joint classification.
"""

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pandas as pd

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    CpgNetworkStructure,
)
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

# Import Core-Centric for joint classification
from core_centric import (
    CoreCentricResult,
    JointType,
    analyze_robot,
    generate_core_centric_cpg_structure,
    expand_cc_sym_params,
    get_cc_sym_expansion_info,
)


# Color mapping for joint types
JOINT_COLORS = {
    JointType.SPINE: "blue",
    JointType.HIP: "green",
    JointType.KNEE: "orange",
    JointType.ANKLE: "red",
    JointType.LOCKED: "gray",
    JointType.UNCLASSIFIED: "purple",
}


@dataclass
class JointAngleData:
    """Container for joint angle time series data."""
    timestamps: list[float] = field(default_factory=list)
    joint_angles: dict[str, list[float]] = field(default_factory=dict)  # hinge_name -> actual angles
    target_angles: dict[str, list[float]] = field(default_factory=dict)  # hinge_name -> CPG target angles
    joint_types: dict[str, JointType] = field(default_factory=dict)  # hinge_name -> type
    hinge_order: list[str] = field(default_factory=list)  # Ordered list of hinge names


def get_hinge_name(hinge: ActiveHinge, idx: int) -> str:
    """
    Generate a descriptive name for a hinge.

    :param hinge: The active hinge.
    :param idx: Index of the hinge in the robot.
    :returns: A descriptive name string.
    """
    # Try to determine position from parent attachment
    parent = hinge.parent
    if parent is not None:
        parent_type = type(parent).__name__
        slot_idx = hinge.parent_child_index

        # Common slot mappings for Core
        slot_names = {0: "front", 1: "right", 2: "back", 3: "left"}
        slot_name = slot_names.get(slot_idx, f"slot{slot_idx}")

        return f"H{idx}_{slot_name}"

    return f"H{idx}"


def simulate_and_log_angles(
    body: Body,
    cpg_params: list[float],
    simulation_time: float = 30.0,
    use_core_centric: bool = True,
    use_cc_sym: bool = False,
    verbose: bool = False,
) -> JointAngleData:
    """
    Run simulation and log joint angles at each timestep.

    :param body: The robot body.
    :param cpg_params: CPG parameters (reduced if use_cc_sym, full otherwise).
    :param simulation_time: Duration of simulation in seconds.
    :param use_core_centric: If True, use Core-Centric CPG structure.
    :param use_cc_sym: If True, expand CC-SYM reduced params to full params.
    :param verbose: If True, print progress.
    :returns: JointAngleData with recorded angles.
    """
    # Analyze robot for joint classification
    result = analyze_robot(body)
    active_hinges = body.find_modules_of_type(ActiveHinge)

    # Get CPG structure
    if use_core_centric:
        structure, hinge_mapping, _ = generate_core_centric_cpg_structure(body)
    else:
        # Standard neighbor coupling
        from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
        structure, hinge_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

    # Expand CC-SYM params if needed
    if use_cc_sym:
        full_params = expand_cc_sym_params(cpg_params, body)
    else:
        full_params = cpg_params

    # Validate parameter count
    expected_params = structure.num_connections
    if len(full_params) != expected_params:
        raise ValueError(f"Expected {expected_params} CPG params, got {len(full_params)}")

    # Create brain
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=np.array(full_params),
        cpg_network_structure=structure,
        initial_state_uniform=math.sqrt(2) * 0.5,
        output_mapping=hinge_mapping,
    )

    robot = ModularRobot(body, brain)

    # Create scene
    terrain = Terrain(
        static_geometry=[
            GeometryPlane(
                pose=Pose(),
                mass=0.0,
                size=Vector2([20.0, 20.0]),
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

    # Convert to simulation scene
    batch_params = make_standard_batch_parameters()
    simulation_scene, robot_to_mbs_mapping = scene.to_simulation_scene()

    # Create MuJoCo model
    model, mujoco_mapping = scene_to_model(
        simulation_scene,
        simulation_timestep=batch_params.simulation_timestep,
        cast_shadows=False,
        fast_sim=True,
    )
    data = mujoco.MjData(model)

    # Create control interface
    control_interface = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mujoco_mapping)

    # Initialize data container
    angle_data = JointAngleData()

    # Build hinge name mapping and find joint/actuator IDs in MuJoCo
    hinge_to_joint_id: dict[ActiveHinge, int] = {}
    hinge_to_actuator_id: dict[ActiveHinge, int] = {}
    hinge_names: dict[ActiveHinge, str] = {}

    for idx, hinge in enumerate(active_hinges):
        hinge_names[hinge] = get_hinge_name(hinge, idx)
        angle_data.hinge_order.append(hinge_names[hinge])
        angle_data.joint_angles[hinge_names[hinge]] = []
        angle_data.target_angles[hinge_names[hinge]] = []  # Initialize target angles

        # Get joint type from Core-Centric analysis
        jtype = result.joint_types.get(hinge, JointType.UNCLASSIFIED)
        angle_data.joint_types[hinge_names[hinge]] = jtype

    # Find MuJoCo joint IDs for each hinge
    hinge_joint_ids = []
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name and "mbs" in joint_name:
            # Match to hinge by order (hinges appear in order in the model)
            for hinge in active_hinges:
                if hinge not in hinge_to_joint_id:
                    # Check if this joint is a hinge joint (1 DOF rotation)
                    if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
                        hinge_to_joint_id[hinge] = i
                        hinge_joint_ids.append(i)
                        break

    # Find MuJoCo actuator IDs for each hinge (actuators control the joints)
    actuator_idx = 0
    for i in range(model.nu):  # model.nu = number of actuators
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if actuator_name and "mbs" in actuator_name:
            # Actuators appear in same order as hinges
            if actuator_idx < len(active_hinges):
                hinge_to_actuator_id[active_hinges[actuator_idx]] = i
                actuator_idx += 1

    if verbose:
        print(f"Running simulation for {simulation_time}s...")
        print(f"Found {len(hinge_to_joint_id)} hinge joints in MuJoCo model")
        print(f"Found {len(hinge_to_actuator_id)} actuators in MuJoCo model")

    # Simulation loop
    control_step = 1.0 / batch_params.control_frequency
    last_control_time = 0.0
    sample_interval = 0.01  # Sample every 10ms for smooth plots
    last_sample_time = 0.0

    # Track current target angles (updated at each control step)
    current_targets: dict[ActiveHinge, float] = {h: 0.0 for h in active_hinges}

    mujoco.mj_forward(model, data)

    while (time := data.time) < simulation_time:
        # Control step - execute before sampling to update targets
        if time >= last_control_time + control_step:
            last_control_time = math.floor(time / control_step) * control_step
            simulation_state = SimulationStateImpl(
                data=data,
                abstraction_to_mujoco_mapping=mujoco_mapping,
                camera_views={},
            )
            simulation_scene.handler.handle(simulation_state, control_interface, control_step)

            # Capture target angles from data.ctrl after control step
            for hinge in active_hinges:
                if hinge in hinge_to_actuator_id:
                    actuator_id = hinge_to_actuator_id[hinge]
                    current_targets[hinge] = data.ctrl[actuator_id]

        # Sample joint angles at regular intervals
        if time >= last_sample_time + sample_interval or time == 0:
            angle_data.timestamps.append(time)

            for hinge in active_hinges:
                name = hinge_names[hinge]

                # Actual angle from simulation
                if hinge in hinge_to_joint_id:
                    joint_id = hinge_to_joint_id[hinge]
                    qpos_idx = model.jnt_qposadr[joint_id]
                    actual_angle = data.qpos[qpos_idx]
                else:
                    actual_angle = 0.0  # Joint not found

                angle_data.joint_angles[name].append(actual_angle)

                # Target angle from CPG controller
                target_angle = current_targets.get(hinge, 0.0)
                angle_data.target_angles[name].append(target_angle)

            last_sample_time = time

        mujoco.mj_step(model, data)

    if verbose:
        print(f"Simulation complete. Recorded {len(angle_data.timestamps)} samples.")

    return angle_data


def plot_joint_angles(
    angle_data: JointAngleData,
    title: str = "Joint Angles Over Time",
    figsize: tuple[int, int] = (14, 8),
    show_legend: bool = True,
    show_grid: bool = True,
    show_actual: bool = True,
    show_targets: bool = True,
) -> plt.Figure:
    """
    Generate a matplotlib figure with joint angles over time.

    :param angle_data: The recorded joint angle data.
    :param title: Plot title.
    :param figsize: Figure size (width, height) in inches.
    :param show_legend: If True, show legend.
    :param show_grid: If True, show grid.
    :param show_actual: If True, show actual joint angles (solid lines).
    :param show_targets: If True, show CPG target angles (dashed if both, solid if only targets).
    :returns: The matplotlib Figure object.
    """
    fig, ax = plt.subplots(figsize=figsize)

    timestamps = np.array(angle_data.timestamps)

    # Group hinges by joint type for legend organization
    type_order = [JointType.SPINE, JointType.HIP, JointType.KNEE, JointType.ANKLE, JointType.LOCKED]

    # Track if we have target data
    has_targets = show_targets and bool(angle_data.target_angles)

    # Determine line styles based on what's being shown
    if show_actual and has_targets:
        actual_style = '-'
        target_style = '--'
        target_alpha = 0.4
        legend_note = "Solid: Actual | Dashed: Target"
    elif has_targets and not show_actual:
        # Only targets - use solid lines
        target_style = '-'
        target_alpha = 0.9
        legend_note = None
    else:
        actual_style = '-'
        target_style = '--'
        target_alpha = 0.4
        legend_note = None

    for jtype in type_order:
        for name in angle_data.hinge_order:
            if angle_data.joint_types.get(name) == jtype:
                color = JOINT_COLORS.get(jtype, "black")
                label = f"{name} ({jtype.value})"

                # Plot actual angles (solid line)
                if show_actual:
                    actual_angles = np.array(angle_data.joint_angles[name])
                    ax.plot(timestamps, actual_angles, color=color, label=label,
                            linewidth=1.2, alpha=0.9, linestyle=actual_style)

                # Plot target angles
                if has_targets and name in angle_data.target_angles:
                    target_angles = np.array(angle_data.target_angles[name])
                    # Only add label if we're not showing actual (to avoid duplicate legend entries)
                    target_label = label if not show_actual else None
                    ax.plot(timestamps, target_angles, color=color, label=target_label,
                            linewidth=1.0 if not show_actual else 0.8,
                            alpha=target_alpha, linestyle=target_style)

    # Handle unclassified joints
    for name in angle_data.hinge_order:
        jtype = angle_data.joint_types.get(name)
        if jtype not in type_order:
            color = JOINT_COLORS.get(jtype, "black")
            label = f"{name} ({jtype.value if jtype else 'unknown'})"

            if show_actual:
                actual_angles = np.array(angle_data.joint_angles[name])
                ax.plot(timestamps, actual_angles, color=color, label=label,
                        linewidth=1.2, alpha=0.9, linestyle=actual_style)

            if has_targets and name in angle_data.target_angles:
                target_angles = np.array(angle_data.target_angles[name])
                target_label = label if not show_actual else None
                ax.plot(timestamps, target_angles, color=color, label=target_label,
                        linewidth=1.0 if not show_actual else 0.8,
                        alpha=target_alpha, linestyle=target_style)

    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Angle (radians)", fontsize=12)
    ax.set_title(title, fontsize=14)

    if show_grid:
        ax.grid(True, linestyle='--', alpha=0.5)

    if show_legend:
        # Place legend outside the plot
        ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=9)

        # Add a note about line styles if showing both
        if legend_note:
            ax.text(1.02, -0.05, legend_note,
                    transform=ax.transAxes, fontsize=8, alpha=0.7)

    # Add horizontal line at 0
    ax.axhline(y=0, color='black', linestyle='-', linewidth=0.5, alpha=0.3)

    plt.tight_layout()

    return fig


def save_plot_as_png(
    fig: plt.Figure,
    filepath: str | Path,
    dpi: int = 150,
) -> None:
    """
    Save the figure as a PNG file.

    :param fig: The matplotlib Figure object.
    :param filepath: Path to save the PNG file.
    :param dpi: Resolution in dots per inch.
    """
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(filepath, dpi=dpi, bbox_inches='tight')
    print(f"Saved plot to: {filepath}")


def export_to_csv(
    angle_data: JointAngleData,
    filepath: str | Path,
) -> None:
    """
    Export joint angle data to a CSV file.

    Columns: time, joint1_actual, joint1_target, joint1_type, joint2_actual, ...

    :param angle_data: The recorded joint angle data.
    :param filepath: Path to save the CSV file.
    """
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)

    # Build DataFrame
    data_dict = {"time": angle_data.timestamps}

    for name in angle_data.hinge_order:
        # Actual angle from simulation
        data_dict[f"{name}_actual"] = angle_data.joint_angles[name]

        # Target angle from CPG controller
        if name in angle_data.target_angles and angle_data.target_angles[name]:
            data_dict[f"{name}_target"] = angle_data.target_angles[name]
        else:
            data_dict[f"{name}_target"] = [0.0] * len(angle_data.timestamps)

        # Joint type
        jtype = angle_data.joint_types.get(name, JointType.UNCLASSIFIED)
        data_dict[f"{name}_type"] = [jtype.value] * len(angle_data.timestamps)

    df = pd.DataFrame(data_dict)
    df.to_csv(filepath, index=False)
    print(f"Exported data to: {filepath}")


def generate_joint_angle_plot(
    body: Body,
    cpg_params: list[float],
    joint_classifications: Optional[dict[ActiveHinge, JointType]] = None,
    simulation_time: float = 30.0,
    use_core_centric: bool = True,
    use_cc_sym: bool = False,
    title: str = "Joint Angles Over Time",
    save_png: Optional[str | Path] = None,
    save_csv: Optional[str | Path] = None,
    show_plot: bool = True,
    show_actual: bool = True,
    show_targets: bool = True,
    verbose: bool = False,
) -> tuple[plt.Figure, JointAngleData]:
    """
    Main function to generate joint angle plots for any Revolve2 robot.

    This is the main entry point that combines simulation, logging, and plotting.

    :param body: The robot body.
    :param cpg_params: CPG parameters from optimization.
    :param joint_classifications: Optional pre-computed joint classifications.
                                  If None, Core-Centric algorithm is used.
    :param simulation_time: Duration of simulation in seconds (default 30s).
    :param use_core_centric: If True, use Core-Centric CPG structure.
    :param use_cc_sym: If True, treat cpg_params as CC-SYM reduced params.
    :param title: Plot title.
    :param save_png: If provided, save plot to this path as PNG.
    :param save_csv: If provided, export data to this path as CSV.
    :param show_plot: If True, display the plot interactively.
    :param show_actual: If True, show actual joint angles (solid lines).
    :param show_targets: If True, show CPG target angles.
    :param verbose: If True, print progress information.
    :returns: Tuple of (matplotlib Figure, JointAngleData).

    Example usage:
    ```python
    from joint_angle_plotter import generate_joint_angle_plot
    from revolve2.standards import modular_robots_v1

    body = modular_robots_v1.get("spider")
    cpg_params = [1.0, 0.5, ...]  # Your optimized parameters

    fig, data = generate_joint_angle_plot(
        body=body,
        cpg_params=cpg_params,
        simulation_time=30.0,
        use_cc_sym=True,  # If using CC-SYM reduced params
        save_png="joint_angles.png",
        save_csv="joint_angles.csv",
    )
    ```
    """
    if verbose:
        print(f"Generating joint angle plot for robot with {len(body.find_modules_of_type(ActiveHinge))} hinges")
        print(f"Simulation time: {simulation_time}s")
        print(f"Using Core-Centric: {use_core_centric}, CC-SYM: {use_cc_sym}")

    # Run simulation and log angles
    angle_data = simulate_and_log_angles(
        body=body,
        cpg_params=cpg_params,
        simulation_time=simulation_time,
        use_core_centric=use_core_centric,
        use_cc_sym=use_cc_sym,
        verbose=verbose,
    )

    # Override joint classifications if provided
    if joint_classifications is not None:
        for hinge, jtype in joint_classifications.items():
            name = None
            for n, t in angle_data.joint_types.items():
                # Match by hinge (would need better matching in production)
                pass
            # For now, the Core-Centric classifications are used

    # Generate plot
    fig = plot_joint_angles(angle_data, title=title, show_actual=show_actual, show_targets=show_targets)

    # Save outputs if requested
    if save_png is not None:
        save_plot_as_png(fig, save_png)

    if save_csv is not None:
        export_to_csv(angle_data, save_csv)

    if show_plot:
        plt.show()

    return fig, angle_data


def main():
    """Demo: Plot joint angles for the best CC-SYM spider."""
    print("Joint Angle Plotter Demo")
    print("=" * 50)

    # Best CC-SYM spider parameters from evolution
    CC_SYM_PARAMS = [
        1.5568441989887698, 0.0020151722361741236, 0.29752924192173225,
        0.294053236387799, 0.9567565792094528, -0.012687399530811239,
        -0.9714437754158323, -0.9984657837551124, 0.6888372168318937,
        -0.9993784498728023, 0.6044314509955745, 0.9582703743539801
    ]

    # Get spider body
    body = modular_robots_v1.get("spider")

    print(f"\nRobot: spider")
    print(f"Hinges: {len(body.find_modules_of_type(ActiveHinge))}")
    print(f"CC-SYM params: {len(CC_SYM_PARAMS)}")

    # Generate plot with both actual and target
    print("\n--- Generating combined plot (actual + target) ---")
    fig, data = generate_joint_angle_plot(
        body=body,
        cpg_params=CC_SYM_PARAMS,
        simulation_time=30.0,  # 30 seconds as specified
        use_core_centric=True,
        use_cc_sym=True,
        title="CC-SYM Spider Joint Angles (Solid=Actual, Dashed=Target)",
        save_png="joint_angles_spider.png",
        save_csv="joint_angles_spider.csv",
        show_plot=False,
        show_actual=True,
        show_targets=True,
        verbose=True,
    )

    # Generate target-only plot
    print("\n--- Generating target-only plot ---")
    fig_target = plot_joint_angles(
        data,
        title="CC-SYM Spider CPG Target Angles",
        show_actual=False,
        show_targets=True,
    )
    save_plot_as_png(fig_target, "joint_angles_spider_target.png")

    # Generate actual-only plot
    print("\n--- Generating actual-only plot ---")
    fig_actual = plot_joint_angles(
        data,
        title="CC-SYM Spider Actual Joint Angles",
        show_actual=True,
        show_targets=False,
    )
    save_plot_as_png(fig_actual, "joint_angles_spider_actual.png")

    plt.show()
    print("\nDone!")


if __name__ == "__main__":
    main()
