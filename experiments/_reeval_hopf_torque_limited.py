"""Re-evaluate the evolved Hopf babyb params with torque limits patched in."""
import math
import numpy as np
import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
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
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)

TORQUE_LIMIT = 0.948013269
OMEGA = 2 * math.pi * 1.0  # 1 Hz


def run(params_path, torque_limit=None, sim_time=20.0):
    body = modular_robots_v1.get("babyb")
    hinges = body.find_modules_of_type(ActiveHinge)
    cpg_struct, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)
    hopf_struct = hopf_structure_from_cpg_structure(cpg_struct)
    params = np.load(params_path)
    brain = BrainHopfStatic.from_params(
        params=params, network_structure=hopf_struct,
        output_mapping=mapping, omega=OMEGA,
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
    batch.simulation_time = sim_time
    model, mj_mapping = scene_to_model(
        sim_scene, simulation_timestep=batch.simulation_timestep,
        cast_shadows=False, fast_sim=True,
    )
    if torque_limit is not None:
        model.actuator_forcerange[:, 0] = -torque_limit
        model.actuator_forcerange[:, 1] = torque_limit
        model.actuator_forcelimited[:] = 1

    data = mujoco.MjData(model)
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
    core = get_robot_core_body_id(model)
    ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
    non_foot = robot_ids - foot_ids
    ctrl_step = 1.0 / batch.control_frequency
    last_ctrl = 0.0

    mujoco.mj_forward(model, data)
    init_pos = data.xpos[core].copy()
    total, drag = 0, 0
    forces_over = 0
    total_force_samples = 0
    while data.time < sim_time:
        total += 1
        contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
        if any(rg in non_foot for rg, *_ in contacts):
            drag += 1
        if data.time >= last_ctrl + ctrl_step:
            last_ctrl = math.floor(data.time / ctrl_step) * ctrl_step
            sim_state = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
            sim_scene.handler.handle(sim_state, ctrl, ctrl_step)
        mujoco.mj_step(model, data)
        # Track how often torque is at / above nominal limit
        f = np.abs(data.actuator_force)
        forces_over += int((f > 0.948).sum())
        total_force_samples += len(f)

    fp = data.xpos[core].copy()
    dx = fp[0] - init_pos[0]
    dy = fp[1] - init_pos[1]
    distance = math.sqrt(dx * dx + dy * dy)
    dragging = drag / total if total > 0 else 0
    pct_over = 100.0 * forces_over / total_force_samples if total_force_samples > 0 else 0
    return distance, dragging, pct_over


if __name__ == "__main__":
    print("Re-evaluating Hopf babyb BLF (xy form) params...\n")
    params_path = "hopf_babyb_blf_xy_best.npy"

    d_unc, drag_unc, over_unc = run(params_path, torque_limit=None)
    print(f"{'UNCONSTRAINED:':<25} distance={d_unc:.3f} m  drag={drag_unc*100:.1f}%  forces>0.948Nm: {over_unc:.2f}%")

    d_lim, drag_lim, over_lim = run(params_path, torque_limit=TORQUE_LIMIT)
    print(f"{'TORQUE LIMITED (0.948):':<25} distance={d_lim:.3f} m  drag={drag_lim*100:.1f}%  forces>0.948Nm: {over_lim:.2f}%")

    drop = (d_unc - d_lim) / d_unc * 100 if d_unc > 0 else 0
    print(f"\nDrop under torque limits: {drop:.1f}%")
