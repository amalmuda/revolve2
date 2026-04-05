"""
Compute Kargar et al. (2021) Balance metric for evolved robots.

B = 1 - (r + p) / (t * 180 * 2)

Where:
  r = accumulated sum of absolute roll angles (degrees, 0-180)
  p = accumulated sum of absolute pitch angles (degrees, 0-180)
  t = total number of timesteps
  B is 0-1 (1 = perfectly level, 0 = maximum tilting)

Re-simulates best individual from each run using saved .npy params.
"""
import math
import os
import sys
import numpy as np
import mujoco

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

from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
    get_robot_core_body_id,
)


def quaternion_to_roll_pitch_degrees(quat):
    """
    Convert MuJoCo quaternion [w, x, y, z] to roll and pitch in degrees.
    Returns absolute values clamped to [0, 180].
    """
    w, x, y, z = quat

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll_rad = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch_rad = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch_rad = math.asin(sinp)

    # Convert to degrees and take absolute value, clamp to [0, 180]
    roll_deg = min(abs(math.degrees(roll_rad)), 180.0)
    pitch_deg = min(abs(math.degrees(pitch_rad)), 180.0)

    return roll_deg, pitch_deg


def compute_balance(robot_name, coupling, params_path, simulation_time=30.0):
    """
    Re-simulate a robot and compute Kargar balance metric.

    Returns: (balance, distance, dragging)
    """
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)

    # Get CPG structure
    if coupling == "uncoupled":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_internal_only(active_hinges)
    elif coupling == "blf":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_blf(active_hinges, body)
    else:  # neighbor
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

    # Load params and create brain
    params = np.load(params_path)
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=params,
        cpg_network_structure=cpg_structure,
        initial_state_uniform=math.sqrt(2) * 0.5,
        output_mapping=output_mapping,
    )

    robot = ModularRobot(body=body, brain=brain)

    # Create terrain
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

    # Build scene and MuJoCo model
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
        return 0.0, 0.0, 1.0

    # Control interface
    control_interface = ControlInterfaceImpl(
        data=data,
        abstraction_to_mujoco_mapping=mujoco_mapping
    )

    # Identify foot vs non-foot for dragging
    from contact_detection import identify_geometry_types, get_contacts_with_ground
    ground_geom_ids, robot_geom_ids, foot_geom_ids = identify_geometry_types(model)
    non_foot_geom_ids = robot_geom_ids - foot_geom_ids

    # Simulation loop
    control_step = 1.0 / batch_params.control_frequency
    last_control_time = 0.0

    mujoco.mj_forward(model, data)
    initial_pos = data.xpos[core_body_id].copy()

    accumulated_roll = 0.0
    accumulated_pitch = 0.0
    total_timesteps = 0
    timesteps_with_dragging = 0

    while data.time < simulation_time:
        total_timesteps += 1

        # Get core quaternion and accumulate roll/pitch
        core_quat = data.xquat[core_body_id].copy()
        roll_deg, pitch_deg = quaternion_to_roll_pitch_degrees(core_quat)
        accumulated_roll += roll_deg
        accumulated_pitch += pitch_deg

        # Track dragging
        contacts = get_contacts_with_ground(model, data, ground_geom_ids, robot_geom_ids)
        for robot_geom, ground_geom, force, position in contacts:
            if robot_geom in non_foot_geom_ids:
                timesteps_with_dragging += 1
                break

        # Control step
        if data.time >= last_control_time + control_step:
            last_control_time = math.floor(data.time / control_step) * control_step
            simulation_state = SimulationStateImpl(
                data=data,
                abstraction_to_mujoco_mapping=mujoco_mapping,
                camera_views={},
            )
            simulation_scene.handler.handle(simulation_state, control_interface, control_step)

        mujoco.mj_step(model, data)

    # Compute balance
    if total_timesteps > 0:
        balance = 1.0 - (accumulated_roll + accumulated_pitch) / (total_timesteps * 180.0 * 2.0)
        balance = max(0.0, min(1.0, balance))  # clamp to [0, 1]
        dragging = timesteps_with_dragging / total_timesteps
    else:
        balance = 0.0
        dragging = 1.0

    # Compute distance
    final_pos = data.xpos[core_body_id].copy()
    dx = final_pos[0] - initial_pos[0]
    dy = final_pos[1] - initial_pos[1]
    distance = math.sqrt(dx ** 2 + dy ** 2)

    return balance, distance, dragging


def main():
    base = os.path.expanduser("~/revolve2/experiments/results/final_experiments")
    output_lines = []
    output_lines.append("morphology\tcoupling\tlambda\trun\tbalance\tdistance\tdragging")

    robots = ["spider", "gecko"]
    couplings = [("uncoupled", "No coupling"), ("neighbor", "Neighbour"), ("blf", "Structured")]
    lambdas = [0, 1, 2, 3]

    total = len(robots) * len(couplings) * len(lambdas) * 30
    count = 0

    for robot in robots:
        for coupling_key, coupling_label in couplings:
            for lam in lambdas:
                exp_dir = "%s_ode_cpg_%s_lambda%d_dragging" % (robot, coupling_key, lam)
                exp_path = os.path.join(base, exp_dir)

                for run_num in range(1, 31):
                    count += 1
                    params_path = os.path.join(exp_path, "best_params_run_%d.npy" % run_num)

                    if not os.path.exists(params_path):
                        print("[%d/%d] SKIP %s run %d (no params)" % (count, total, exp_dir, run_num))
                        continue

                    try:
                        balance, distance, dragging = compute_balance(
                            robot, coupling_key, params_path
                        )
                        output_lines.append("%s\t%s\t%d\t%d\t%.6f\t%.4f\t%.4f" % (
                            robot, coupling_label, lam, run_num, balance, distance, dragging
                        ))
                        print("[%d/%d] %s %s lambda=%d run=%d: balance=%.4f dist=%.2f drag=%.1f%%" % (
                            count, total, robot, coupling_label, lam, run_num, balance, distance, dragging * 100
                        ))
                    except Exception as e:
                        print("[%d/%d] ERROR %s run %d: %s" % (count, total, exp_dir, run_num, e))

    # Write output
    output_path = os.path.expanduser("~/revolve2/experiments/balance_results.txt")
    with open(output_path, "w") as f:
        f.write("\n".join(output_lines) + "\n")
    print("\nResults saved to: %s" % output_path)


if __name__ == "__main__":
    main()
