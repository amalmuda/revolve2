"""
Compute balance, HHS, and EM in a SINGLE simulation pass per fully-connected run.

Avoids running 3 separate sims per run (which is what compute_balance.py +
compute_hhs.py + compute_em.py would do separately).

Output: fc_quality_metrics.txt with columns:
  morphology  coupling  lambda  run  balance  hhs  em  distance  dragging
"""
import math
import os
import sys
import numpy as np
import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
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
    active_hinges_to_cpg_network_structure_fully_connected,
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
    get_robot_core_body_id,
    identify_geometry_types,
    get_contacts_with_ground,
)
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor


FC_LOCAL = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/results/fully_connected_local"
)
OUTPUT = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/_fc_quality_metrics.txt"
)


def quaternion_to_roll_pitch_degrees(quat):
    w, x, y, z = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll_rad = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch_rad = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch_rad = math.asin(sinp)
    roll_deg = min(abs(math.degrees(roll_rad)), 180.0)
    pitch_deg = min(abs(math.degrees(pitch_rad)), 180.0)
    return roll_deg, pitch_deg


def compute_all(robot_name, params_path, simulation_time=30.0, coupling="fully_connected"):
    """Returns (balance, hhs, em, distance, dragging)."""
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "fully_connected":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_fully_connected(active_hinges)
    elif coupling == "blf":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_blf(active_hinges, body)
    elif coupling == "uncoupled":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_internal_only(active_hinges)
    elif coupling == "neighbor":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)
    else:
        raise ValueError(f"Unknown coupling: {coupling}")

    params = np.load(params_path)
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=params,
        cpg_network_structure=cpg_structure,
        initial_state_uniform=math.sqrt(2) * 0.5,
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
        return 0.0, 0.0, 0.0, 0.0, 1.0

    control_interface = ControlInterfaceImpl(
        data=data, abstraction_to_mujoco_mapping=mujoco_mapping
    )

    ground_geom_ids, robot_geom_ids, foot_geom_ids = identify_geometry_types(model)
    non_foot_geom_ids = robot_geom_ids - foot_geom_ids

    control_step = 1.0 / batch_params.control_frequency
    last_control_time = 0.0

    mujoco.mj_forward(model, data)
    initial_pos = data.xpos[core_body_id].copy()

    prev_z = data.xpos[core_body_id][2]
    prev_x = initial_pos[0]
    prev_y = initial_pos[1]

    accumulated_roll = 0.0
    accumulated_pitch = 0.0
    hhs = 0.0
    total_traveled = 0.0
    total_timesteps = 0
    timesteps_with_dragging = 0

    while data.time < simulation_time:
        total_timesteps += 1

        # Balance
        core_quat = data.xquat[core_body_id].copy()
        roll_deg, pitch_deg = quaternion_to_roll_pitch_degrees(core_quat)
        accumulated_roll += roll_deg
        accumulated_pitch += pitch_deg

        # HHS
        current_z = data.xpos[core_body_id][2]
        hhs += abs(current_z - prev_z)
        prev_z = current_z

        # EM (path length)
        current_x = data.xpos[core_body_id][0]
        current_y = data.xpos[core_body_id][1]
        total_traveled += math.sqrt((current_x - prev_x) ** 2 + (current_y - prev_y) ** 2)
        prev_x = current_x
        prev_y = current_y

        # Dragging
        contacts = get_contacts_with_ground(model, data, ground_geom_ids, robot_geom_ids)
        for robot_geom, _, _, _ in contacts:
            if robot_geom in non_foot_geom_ids:
                timesteps_with_dragging += 1
                break

        if data.time >= last_control_time + control_step:
            last_control_time = math.floor(data.time / control_step) * control_step
            simulation_state = SimulationStateImpl(
                data=data,
                abstraction_to_mujoco_mapping=mujoco_mapping,
                camera_views={},
            )
            simulation_scene.handler.handle(simulation_state, control_interface, control_step)

        mujoco.mj_step(model, data)

    if total_timesteps > 0:
        balance = 1.0 - (accumulated_roll + accumulated_pitch) / (total_timesteps * 180.0 * 2.0)
        balance = max(0.0, min(1.0, balance))
        dragging = timesteps_with_dragging / total_timesteps
    else:
        balance = 0.0
        dragging = 1.0

    final_pos = data.xpos[core_body_id].copy()
    dx = final_pos[0] - initial_pos[0]
    dy = final_pos[1] - initial_pos[1]
    displacement = math.sqrt(dx ** 2 + dy ** 2)

    if total_traveled > 0.001:
        em = displacement / total_traveled
    else:
        em = 0.0
    em = max(0.0, min(1.0, em))

    return balance, hhs, em, displacement, dragging


def main():
    lines = ["morphology\tcoupling\tlambda\trun\tbalance\thhs\tem\tdistance\tdragging"]
    total = 240
    count = 0

    for robot in ["spider", "gecko"]:
        for lam in [0, 1, 2, 3]:
            exp_dir = "%s_ode_cpg_fully_connected_lambda%d_dragging" % (robot, lam)
            exp_path = os.path.join(FC_LOCAL, exp_dir)
            if not os.path.isdir(exp_path):
                continue
            for run_num in range(1, 31):
                npy = os.path.join(exp_path, "best_params_run_%d.npy" % run_num)
                if not os.path.exists(npy):
                    continue
                count += 1
                try:
                    b, h, e, d, dr = compute_all(robot, npy)
                    lines.append("%s\tFully_connected\t%d\t%d\t%.6f\t%.6f\t%.6f\t%.4f\t%.4f" % (
                        robot, lam, run_num, b, h, e, d, dr
                    ))
                    print("[%d/%d] %s lambda=%d run=%d: B=%.3f HHS=%.3f EM=%.3f" % (
                        count, total, robot, lam, run_num, b, h, e
                    ))
                except Exception as ex:
                    print("[%d/%d] ERROR %s lambda=%d run=%d: %s" % (count, total, robot, lam, run_num, ex))

    with open(OUTPUT, "w") as f:
        f.write("\n".join(lines) + "\n")

    print("\nWrote %d rows to %s" % (len(lines) - 1, OUTPUT))


if __name__ == "__main__":
    main()
