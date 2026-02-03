"""Measure effective amplitude of each hinge during simulation."""

import math
import numpy as np
import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
from revolve2.modular_robot.brain.cpg._make_cpg_network_structure_neighbor import (
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.modular_robot_simulation import ModularRobotScene
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.simulation.scene import Pose, Color
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2

from contact_detection import Terrain


def measure_effective_amplitudes(
    cpg_params: np.ndarray,
    robot_name: str = "spider",
    simulation_time: float = 10.0,
) -> tuple[np.ndarray, list[str]]:
    """
    Measure effective amplitude of each hinge during simulation.

    Returns:
        effective_amplitudes: array of (max - min) / 2 for each hinge
        hinge_roles: list of "HIP" or "KNEE" for each hinge
    """
    # Create robot with CPG brain
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(active_hinges)

    # Determine hip vs knee for each hinge
    hinge_roles = []
    for h in active_hinges:
        parent_type = type(h.parent).__name__
        if "Core" in parent_type:
            hinge_roles.append("HIP")
        else:
            hinge_roles.append("KNEE")

    # Create CPG brain
    cpg_network_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(
        active_hinges
    )

    initial_state = 0.5 * math.sqrt(2)
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=cpg_params,
        cpg_network_structure=cpg_network_structure,
        initial_state_uniform=initial_state,
        output_mapping=output_mapping,
    )

    robot = ModularRobot(body=body, brain=brain)

    # Setup scene
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

    batch_params = make_standard_batch_parameters()
    simulation_scene, _ = scene.to_simulation_scene()

    model, mujoco_mapping = scene_to_model(
        simulation_scene,
        simulation_timestep=batch_params.simulation_timestep,
        cast_shadows=False,
        fast_sim=True,
    )
    data = mujoco.MjData(model)

    control_interface = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mujoco_mapping)
    mujoco.mj_forward(model, data)

    # Find all hinge joints and their qpos addresses
    hinge_joints = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            # Get joint type - hinge joints are type 3
            jnt_type = model.jnt_type[i]
            if jnt_type == 3:  # mjJNT_HINGE
                qpos_addr = model.jnt_qposadr[i]
                hinge_joints.append((i, name, qpos_addr))

    # Storage for joint angles - use qpos addresses
    joint_angles = {addr: [] for _, _, addr in hinge_joints}

    control_freq = batch_params.control_frequency
    control_step = 1.0 / control_freq
    last_control_time = 0.0
    handler = simulation_scene.handler

    # Run simulation and record joint angles
    while data.time < simulation_time:
        if data.time >= last_control_time + control_step:
            last_control_time = int(data.time / control_step) * control_step

            from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
            state = SimulationStateImpl(data, mujoco_mapping, {})
            handler.handle(state, control_interface, control_step)

        mujoco.mj_step(model, data)

        # Record joint angles
        for _, _, qpos_addr in hinge_joints:
            joint_angles[qpos_addr].append(data.qpos[qpos_addr])

    # Calculate effective amplitudes (only for robot hinges, not all joints)
    # Sort by qpos address to maintain consistent ordering
    sorted_addrs = sorted(joint_angles.keys())
    effective_amplitudes = []

    for addr in sorted_addrs:
        angles = np.array(joint_angles[addr])
        if len(angles) > 0:
            eff_amp = (np.max(angles) - np.min(angles)) / 2
            effective_amplitudes.append(eff_amp)

    # Take only the first n_hinges (in case there are extra joints)
    effective_amplitudes = np.array(effective_amplitudes[:n_hinges])

    return effective_amplitudes, hinge_roles, hinge_joints[:n_hinges]


if __name__ == "__main__":
    # High-drag CPG params (Lambda=0, Run 25)
    high_drag_params = np.array([
        0.99639885, -0.99872902, -0.91727893, 0.86271357,
        0.96403001, -0.52641671, -0.31360675, 0.07630563,  # Internal (8)
        0.3005678, 0.96621355, 0.98988844, -0.96832737     # External (4)
    ])

    print("=" * 60)
    print("Measuring Effective Amplitudes - HIGH DRAG CPG")
    print("=" * 60)
    print("\nRunning 10s simulation...")

    eff_amps, roles, joints = measure_effective_amplitudes(high_drag_params)

    print(f"\nFound {len(joints)} hinge joints:")
    for jid, name, addr in joints:
        print(f"  Joint {jid}: {name} (qpos[{addr}])")

    print("\nResults:")
    print("-" * 60)
    print(f"{'Hinge':<8} {'Role':<6} {'Effective Amplitude (rad)':<25}")
    print("-" * 60)
    for i, (amp, role) in enumerate(zip(eff_amps, roles)):
        leg_num = i // 2 + 1
        print(f"[{i}]      {role:<6} {amp:.4f}  (Leg {leg_num})")

    print("-" * 60)
    print(f"\nEffective amplitudes: {list(np.round(eff_amps, 4))}")
    print(f"Hinge roles:          {roles}")
