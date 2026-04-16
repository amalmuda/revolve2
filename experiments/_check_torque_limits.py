"""
Verify whether the nominal 0.948 Nm torque limit on Revolve2 V1 hinges is
enforced during MuJoCo simulation. Logs actuator_force values across one
30 s run of the best babyb BLF lambda=2 solution.
"""
import math
import sys
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

from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
from contact_detection import active_hinges_to_cpg_network_structure_blf


NOMINAL_LIMIT = 0.948013269  # Nm, from ActiveHingeV1 effort field
ROBOT = "babyb"
PARAMS = "/tmp/babyb_blf_l2_run8.npy"  # best BLF lambda=2 seed
SIM_TIME = 30.0


def main():
    body = modular_robots_v1.get(ROBOT)
    hinges = body.find_modules_of_type(ActiveHinge)
    cpg_struct, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)
    params = np.load(PARAMS)
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=params,
        cpg_network_structure=cpg_struct,
        initial_state_uniform=math.sqrt(2) * 0.5,
        output_mapping=mapping,
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
        sim_scene,
        simulation_timestep=batch.simulation_timestep,
        cast_shadows=False,
        fast_sim=True,
    )
    data = mujoco.MjData(model)
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)

    print(f"Model: {model.nu} actuators total")
    print(f"Nominal torque limit: {NOMINAL_LIMIT:.3f} Nm per hinge")
    print(f"actuator_forcerange[:5] = {model.actuator_forcerange[:5]}")
    print(f"actuator_forcelimited   = {model.actuator_forcelimited}")
    print()

    mujoco.mj_forward(model, data)
    control_step = 1.0 / batch.control_frequency
    last_ctrl = 0.0

    # Actuators alternate position / velocity per hinge. Track both.
    all_forces = []  # (T, nu) snapshots

    while data.time < SIM_TIME:
        if data.time >= last_ctrl + control_step:
            last_ctrl = math.floor(data.time / control_step) * control_step
            sim_state = SimulationStateImpl(
                data=data,
                abstraction_to_mujoco_mapping=mj_mapping,
                camera_views={},
            )
            sim_scene.handler.handle(sim_state, ctrl, control_step)
        mujoco.mj_step(model, data)
        all_forces.append(data.actuator_force.copy())

    F = np.stack(all_forces, axis=0)
    print(f"Recorded {F.shape[0]} timesteps x {F.shape[1]} actuators")
    print()

    # Position + velocity actuators alternate: name prefixes tell them apart.
    pos_idx = []
    vel_idx = []
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if "actuator_position_" in name:
            pos_idx.append(i)
        elif "actuator_velocity_" in name:
            vel_idx.append(i)

    print(f"Position actuators: {len(pos_idx)}  |  Velocity actuators: {len(vel_idx)}")
    print()

    def summarise(label, idxs):
        if not idxs:
            return
        F_sub = np.abs(F[:, idxs])
        print(f"=== {label} (|force|, Nm) ===")
        print(f"  max across all samples:     {F_sub.max():.4f}")
        print(f"  99th percentile:            {np.percentile(F_sub, 99):.4f}")
        print(f"  mean across all samples:    {F_sub.mean():.4f}")
        print(f"  % samples > {NOMINAL_LIMIT:.3f} Nm:  {100*(F_sub > NOMINAL_LIMIT).mean():.2f} %")
        print(f"  max per actuator (Nm):")
        per_act = F_sub.max(axis=0)
        for k, idx in enumerate(idxs):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"idx{idx}"
            over = "  <-- OVER LIMIT" if per_act[k] > NOMINAL_LIMIT else ""
            print(f"    [{k:2d}] {name}: {per_act[k]:.4f}{over}")
        print()

    # Total torque at each joint is position + velocity actuator sum.
    # Here we compare each channel individually first, then the sum per hinge.
    summarise("Position actuator force", pos_idx)
    summarise("Velocity actuator force", vel_idx)

    if pos_idx and vel_idx and len(pos_idx) == len(vel_idx):
        total = F[:, pos_idx] + F[:, vel_idx]
        total_abs = np.abs(total)
        print(f"=== Total joint torque (position + velocity) ===")
        print(f"  max across all samples:     {total_abs.max():.4f} Nm")
        print(f"  99th percentile:            {np.percentile(total_abs, 99):.4f} Nm")
        print(f"  mean across all samples:    {total_abs.mean():.4f} Nm")
        print(f"  % samples > {NOMINAL_LIMIT:.3f} Nm:  {100*(total_abs > NOMINAL_LIMIT).mean():.2f} %")


if __name__ == "__main__":
    main()
