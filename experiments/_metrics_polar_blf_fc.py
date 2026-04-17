"""Evaluate EM (effective movement) and HHS (head height stability) on the
polar Hopf spider runs for BLF and FC at λ=0 and λ=1."""
import math
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

from hopf_brain import BrainHopfPolarStatic, hopf_structure_from_cpg_structure
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_fully_connected,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)

OMEGA = 2 * math.pi * 1.0
SIM_TIME = 20.0


def evaluate(params_path, coupling):
    body = modular_robots_v1.get("spider")
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "blf":
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    elif coupling == "fc":
        cpg, mp = active_hinges_to_cpg_network_structure_fully_connected(hinges)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    hopf_struct = hopf_structure_from_cpg_structure(cpg)
    params = np.load(params_path)
    brain = BrainHopfPolarStatic.from_params(
        params=params, network_structure=hopf_struct,
        output_mapping=mp, omega=OMEGA,
    )
    robot = ModularRobot(body=body, brain=brain)
    terrain = Terrain(static_geometry=[GeometryPlane(
        pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
        texture=Texture(
            base_color=Color(200, 200, 200, 255),
            primary_color=Color(220, 220, 220, 255),
            secondary_color=Color(80, 80, 80, 255),
            map_type=MapType.MAP2D,
            reference=TextureReference(builtin="checker"),
            repeat=(50, 50),
        ),
    )], friction=1.0)
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)
    sim_scene, _ = scene.to_simulation_scene()
    batch = make_standard_batch_parameters()
    batch.simulation_time = SIM_TIME
    model, mj_mapping = scene_to_model(
        sim_scene, simulation_timestep=batch.simulation_timestep,
        cast_shadows=False, fast_sim=True,
    )
    data = mujoco.MjData(model)
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
    core = get_robot_core_body_id(model)
    ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
    non_foot = robot_ids - foot_ids
    cstep = 1.0 / batch.control_frequency
    last_ctrl = 0.0
    mujoco.mj_forward(model, data)
    init_pos = data.xpos[core].copy()
    prev_pos = init_pos.copy()

    total_steps = 0
    drag_steps = 0
    path_length = 0.0
    hhs_sum = 0.0

    while data.time < SIM_TIME:
        total_steps += 1
        cur = data.xpos[core].copy()
        # Path length: xy distance step-to-step
        path_length += math.sqrt((cur[0]-prev_pos[0])**2 + (cur[1]-prev_pos[1])**2)
        # HHS: z step-to-step
        hhs_sum += abs(cur[2] - prev_pos[2])
        prev_pos = cur

        if any(rg in non_foot for rg, *_ in get_contacts_with_ground(model, data, ground_ids, robot_ids)):
            drag_steps += 1
        if data.time >= last_ctrl + cstep:
            last_ctrl = math.floor(data.time / cstep) * cstep
            ss = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
            sim_scene.handler.handle(ss, ctrl, cstep)
        mujoco.mj_step(model, data)

    fp = data.xpos[core].copy()
    dx = fp[0] - init_pos[0]
    dy = fp[1] - init_pos[1]
    displacement = math.sqrt(dx*dx + dy*dy)
    em = displacement / path_length if path_length > 0.001 else 0.0
    drag = drag_steps / total_steps
    return displacement, drag, em, hhs_sum


def main():
    runs = [
        ("BLF λ=0", "polar_spider_blf_xy_best.npy", "blf"),
        ("BLF λ=1", "polar_spider_blf_xy_lam1_best.npy", "blf"),
        ("FC  λ=0", "polar_spider_fc_xy_lam0_1p00hz_best.npy", "fc"),
        ("FC  λ=1", "polar_spider_fc_xy_lam1_1p00hz_best.npy", "fc"),
    ]
    print(f"{'Config':<12}{'Dist':<10}{'Drag%':<10}{'EM':<10}{'HHS':<10}")
    print("-" * 55)
    for name, path, coup in runs:
        d, dr, em, hhs = evaluate(path, coup)
        print(f"{name:<12}{d:<10.3f}{dr*100:<10.1f}{em:<10.3f}{hhs:<10.3f}")


if __name__ == "__main__":
    main()
