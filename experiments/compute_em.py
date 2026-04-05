"""
Compute Effective Movement (EM) from Kargar et al. (2021).

EM = D / T

Where D = displacement (straight-line XY distance start to end)
      T = total traveled distance (sum of step-to-step XY distances)
Result is 0-1. 1 = perfectly straight. Lower = more wandering.

Re-simulates best individual from each run using saved .npy params.
"""
import math
import os
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


def compute_em(robot_name, coupling, params_path, simulation_time=30.0):
    """
    Re-simulate a robot and compute EM metric.

    Returns: (em, distance, dragging)
    """
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)

    if coupling == "uncoupled":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_internal_only(active_hinges)
    elif coupling == "blf":
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_blf(active_hinges, body)
    else:
        cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

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
        return 0.0, 0.0, 1.0

    control_interface = ControlInterfaceImpl(
        data=data,
        abstraction_to_mujoco_mapping=mujoco_mapping
    )

    from contact_detection import identify_geometry_types, get_contacts_with_ground
    ground_geom_ids, robot_geom_ids, foot_geom_ids = identify_geometry_types(model)
    non_foot_geom_ids = robot_geom_ids - foot_geom_ids

    control_step = 1.0 / batch_params.control_frequency
    last_control_time = 0.0

    mujoco.mj_forward(model, data)
    initial_pos = data.xpos[core_body_id].copy()

    prev_x = initial_pos[0]
    prev_y = initial_pos[1]
    total_traveled = 0.0
    total_timesteps = 0
    timesteps_with_dragging = 0

    while data.time < simulation_time:
        total_timesteps += 1

        current_x = data.xpos[core_body_id][0]
        current_y = data.xpos[core_body_id][1]
        step_dist = math.sqrt((current_x - prev_x) ** 2 + (current_y - prev_y) ** 2)
        total_traveled += step_dist
        prev_x = current_x
        prev_y = current_y

        # Track dragging
        contacts = get_contacts_with_ground(model, data, ground_geom_ids, robot_geom_ids)
        for robot_geom, ground_geom, force, position in contacts:
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
        dragging = timesteps_with_dragging / total_timesteps
    else:
        dragging = 1.0

    final_pos = data.xpos[core_body_id].copy()
    dx = final_pos[0] - initial_pos[0]
    dy = final_pos[1] - initial_pos[1]
    displacement = math.sqrt(dx ** 2 + dy ** 2)

    # EM = displacement / total_traveled
    if total_traveled > 0.001:
        em = displacement / total_traveled
    else:
        em = 0.0

    # Clamp to [0, 1]
    em = max(0.0, min(1.0, em))

    return em, displacement, dragging


if __name__ == "__main__":
    import sys
    robot = sys.argv[1]
    coupling = sys.argv[2]
    params_path = sys.argv[3]
    output_file = sys.argv[4]
    coupling_label = sys.argv[5]
    lam = sys.argv[6]
    run_num = sys.argv[7]

    em, dist, drag = compute_em(robot, coupling, params_path)
    with open(output_file, "w") as f:
        f.write("%s\t%s\t%s\t%s\t%.6f\t%.4f\t%.4f\n" % (
            robot, coupling_label, lam, run_num, em, dist, drag
        ))
    print("EM=%.4f Distance=%.2f Dragging=%.1f%%" % (em, dist, drag * 100))
