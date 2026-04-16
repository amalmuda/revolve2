"""Sanity check: run a spider with a Hopf brain, plot joint trajectories."""
import math
import numpy as np
import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
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

from hopf_brain import (
    BrainHopfStatic,
    hopf_structure_from_cpg_structure,
)


def run_test(robot_name: str = "spider", sim_time: float = 10.0) -> None:
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)

    # Use neighbor-coupling topology for this sanity test (spider: 4 hips +
    # 4 knees, with neighbor couplings within each leg).
    cpg_struct, output_mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
    hopf_struct = hopf_structure_from_cpg_structure(cpg_struct)

    print(f"Robot: {robot_name}")
    print(f"Oscillators: {hopf_struct.num_oscillators}")
    print(f"Connections: {hopf_struct.num_connections}")
    print(f"Total params: {hopf_struct.num_params}")

    # Hand-crafted params: all oscillators mu=0.5 (amplitude ~0.71),
    # small coupling.
    num_params = hopf_struct.num_params
    n = hopf_struct.num_oscillators
    params = np.zeros(num_params)
    params[:n] = 0.5  # mu for all
    params[n:] = 0.2  # coupling for all pairs

    brain = BrainHopfStatic.from_params(
        params=params,
        network_structure=hopf_struct,
        output_mapping=output_mapping,
        omega=2 * np.pi * 1.0,  # 1 Hz
    )
    robot = ModularRobot(body=body, brain=brain)

    # Standard terrain and simulation boilerplate.
    terrain = Terrain(
        static_geometry=[GeometryPlane(
            pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
            texture=Texture(
                base_color=Color(200, 200, 200, 255),
                primary_color=Color(220, 220, 220, 255),
                secondary_color=Color(80, 80, 80, 255),
                map_type=MapType.MAP2D,
                reference=TextureReference(builtin="checker"),
                repeat=(50, 50),
            ),
        )],
        friction=1.0,
    )
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)
    sim_scene, _ = scene.to_simulation_scene()

    batch_params = make_standard_batch_parameters()
    batch_params.simulation_time = sim_time

    model, mj_mapping = scene_to_model(
        sim_scene,
        simulation_timestep=batch_params.simulation_timestep,
        cast_shadows=False,
        fast_sim=True,
    )
    data = mujoco.MjData(model)
    control_interface = ControlInterfaceImpl(
        data=data, abstraction_to_mujoco_mapping=mj_mapping
    )

    control_step = 1.0 / batch_params.control_frequency
    last_control_time = 0.0
    mujoco.mj_forward(model, data)
    core_body_id = 1  # first after worldbody usually; not critical for this test
    initial_pos = data.xpos[core_body_id].copy()

    # Record joint angles over time.
    joint_angles = []
    times = []

    while data.time < sim_time:
        # Read current hinge joint positions.
        qpos_sample = []
        for j in range(model.njnt):
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
                addr = model.jnt_qposadr[j]
                qpos_sample.append(float(data.qpos[addr]))
        joint_angles.append(qpos_sample)
        times.append(data.time)

        if data.time >= last_control_time + control_step:
            last_control_time = math.floor(data.time / control_step) * control_step
            sim_state = SimulationStateImpl(
                data=data,
                abstraction_to_mujoco_mapping=mj_mapping,
                camera_views={},
            )
            sim_scene.handler.handle(sim_state, control_interface, control_step)

        mujoco.mj_step(model, data)

    final_pos = data.xpos[core_body_id].copy()
    dx = final_pos[0] - initial_pos[0]
    dy = final_pos[1] - initial_pos[1]
    dist = math.sqrt(dx * dx + dy * dy)

    joint_angles = np.array(joint_angles)

    print()
    print(f"Simulation finished after {sim_time}s.")
    print(f"Distance moved: {dist:.3f} m")
    print(f"Joint angle trace shape: {joint_angles.shape}")
    print(f"Joint angle ranges (min, max) per joint:")
    for j in range(joint_angles.shape[1]):
        col = joint_angles[:, j]
        print(f"  joint {j}: [{col.min():.3f}, {col.max():.3f}] rad")

    # Estimate dominant frequency from joint 0 (zero crossings).
    if joint_angles.shape[0] > 10:
        col = joint_angles[:, 0] - joint_angles[:, 0].mean()
        zero_crossings = np.where(np.diff(np.signbit(col)))[0]
        if len(zero_crossings) > 1:
            period_steps = np.mean(np.diff(zero_crossings)) * 2
            fs = 1.0 / batch_params.simulation_timestep
            est_freq = fs / period_steps
            print(f"\nEstimated oscillation frequency (joint 0): {est_freq:.3f} Hz")
            print(f"(Target was 1.0 Hz)")


if __name__ == "__main__":
    run_test()
