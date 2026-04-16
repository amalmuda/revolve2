"""Compute all locomotion metrics for evolved Hopf robots: balance, HHS, EM, CoT, dragging."""
import math
import sys
import numpy as np
import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
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

from hopf_brain import BrainHopfStatic, hopf_structure_from_cpg_structure
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
    get_robot_core_body_id, get_contacts_with_ground, identify_geometry_types,
)
from compute_balance import quaternion_to_roll_pitch_degrees

SIM_TIME = 20.0


def evaluate(robot_name, params_path, coupling, hz=1.0):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "blf":
        cpg_struct, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    elif coupling == "uncoupled":
        cpg_struct, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
    else:
        cpg_struct, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    hopf_struct = hopf_structure_from_cpg_structure(cpg_struct)
    params = np.load(params_path)
    brain = BrainHopfStatic.from_params(
        params=params, network_structure=hopf_struct, output_mapping=mp,
        omega=2 * math.pi * hz,
    )
    robot = ModularRobot(body=body, brain=brain)
    terrain = Terrain(static_geometry=[GeometryPlane(pose=Pose(), mass=0.0, size=Vector2([20.0,20.0]),
        texture=Texture(base_color=Color(200,200,200,255), primary_color=Color(220,220,220,255),
        secondary_color=Color(80,80,80,255), map_type=MapType.MAP2D,
        reference=TextureReference(builtin="checker"), repeat=(50,50)))], friction=1.0)
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)
    sim_scene, _ = scene.to_simulation_scene()
    batch = make_standard_batch_parameters()
    batch.simulation_time = SIM_TIME
    model, mj_mapping = scene_to_model(sim_scene, simulation_timestep=batch.simulation_timestep,
                                        cast_shadows=False, fast_sim=True)
    data = mujoco.MjData(model)
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)

    ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
    non_foot = robot_ids - foot_ids
    core_id = get_robot_core_body_id(model)

    # Get robot mass
    robot_mass = sum(model.body_mass[i] for i in range(model.nbody)
                     if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
                     and "mbs" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""))

    mujoco.mj_forward(model, data)
    initial_pos = data.xpos[core_id].copy()
    initial_z = float(data.xpos[core_id][2])

    control_step = 1.0 / batch.control_frequency
    last_ctrl = 0.0
    total = 0
    drag = 0
    accum_roll = 0.0
    accum_pitch = 0.0
    z_history = []
    pos_history = []
    energy_total = 0.0

    while data.time < SIM_TIME:
        total += 1
        # Roll/pitch from core quaternion
        quat = data.xquat[core_id].copy()
        r, p = quaternion_to_roll_pitch_degrees(quat)
        accum_roll += r
        accum_pitch += p

        # Z position for HHS
        z_history.append(float(data.xpos[core_id][2]))
        pos_history.append((float(data.xpos[core_id][0]), float(data.xpos[core_id][1])))

        # Dragging
        contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
        if any(rg in non_foot for rg, *_ in contacts):
            drag += 1

        # Energy (sum of |torque * angular_velocity| * dt for actuators)
        for ai in range(model.nu):
            joint_id = model.actuator_trnid[ai, 0]
            torque = float(data.actuator_force[ai])
            qvel_idx = model.jnt_dofadr[joint_id]
            angvel = float(data.qvel[qvel_idx])
            energy_total += abs(torque * angvel) * batch.simulation_timestep

        if data.time >= last_ctrl + control_step:
            last_ctrl = math.floor(data.time/control_step)*control_step
            sim_state = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
            sim_scene.handler.handle(sim_state, ctrl, control_step)
        mujoco.mj_step(model, data)

    final_pos = data.xpos[core_id].copy()
    dx = final_pos[0] - initial_pos[0]
    dy = final_pos[1] - initial_pos[1]
    distance = math.sqrt(dx*dx + dy*dy)

    # Metrics
    dragging = drag / total
    avg_roll = accum_roll / total
    avg_pitch = accum_pitch / total
    balance = max(0.0, 1.0 - (accum_roll + accum_pitch) / (total * 180.0 * 2.0))
    z_arr = np.array(z_history)
    hhs = float(np.sum(np.abs(np.diff(z_arr))))  # cumulative absolute z changes
    pos_arr = np.array(pos_history)
    path_lens = np.sqrt(np.diff(pos_arr[:,0])**2 + np.diff(pos_arr[:,1])**2)
    total_path = float(np.sum(path_lens))
    em = distance / total_path if total_path > 0 else 0.0
    cot = energy_total / (robot_mass * 9.81 * distance) if distance > 0.001 and robot_mass > 0 else float("nan")

    return dict(
        distance=distance, dragging=dragging,
        balance=balance, avg_roll=avg_roll, avg_pitch=avg_pitch,
        hhs=hhs, em=em, cot=cot, mass=robot_mass,
    )


configs = [
    ("spider", "hopf_spider_1hz_100gen.npy", "neighbor"),
    ("spider", "hopf_spider_blf_best.npy", "blf"),
    ("gecko", "hopf_gecko_best.npy", "neighbor"),
    ("gecko", "hopf_gecko_blf_best.npy", "blf"),
]

print(f"{'Robot':<8}{'Coupling':<10}{'Distance':<10}{'Drag':<7}{'Balance':<9}{'Roll°':<8}{'Pitch°':<8}{'HHS':<8}{'EM':<7}{'CoT':<8}")
print("-" * 90)
for robot, params, coupling in configs:
    r = evaluate(robot, params, coupling)
    cot_str = f"{r['cot']:.2f}" if not math.isnan(r['cot']) else "n/a"
    print(f"{robot:<8}{coupling:<10}{r['distance']:<10.3f}{r['dragging']:<7.2%}{r['balance']:<9.4f}{r['avg_roll']:<8.2f}{r['avg_pitch']:<8.2f}{r['hhs']:<8.3f}{r['em']:<7.3f}{cot_str:<8}")
