"""Visualize sine controller with user-provided parameters."""

import math
import numpy as np
import mujoco
import mujoco.viewer

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
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
from brain_sine_offset import BrainSineOffset, SineOffsetParameters

# User parameters
amplitudes = np.array([0.91351724, 0.71214163, 0.73219062, 0.50504586,
                       0.89844802, 0.88646999, 0.781354, 0.15433051])
phases = np.array([0.99224094, 5.98598932, 4.29914234, 6.18809434,
                   0.81121247, 1.79327941, 3.71902453, 0.74360644])
offsets = np.array([-0.35346571, 0.23869804, -0.42190033, 0.125564,
                    0.17553313, -0.22235914, 0.01664249, -0.03592111])
frequency = 0.3  # Hz for all joints

print("Parameters:")
print(f"  Amplitudes: {amplitudes}")
print(f"  Phases: {phases}")
print(f"  Offsets: {offsets}")
print(f"  Frequency: {frequency} Hz")
print("  Coupling: 0 - uncoupled")

# Create robot
body = modular_robots_v1.get("spider")
active_hinges = body.find_modules_of_type(ActiveHinge)

# Create parameters for each hinge
parameters = [
    SineOffsetParameters(
        amplitude=amplitudes[i],
        frequency=frequency,
        phase_offset=phases[i],
        position_offset=offsets[i],
    )
    for i in range(len(active_hinges))
]

brain = BrainSineOffset(
    active_hinges=active_hinges,
    parameters=parameters,
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
    cast_shadows=True,
    fast_sim=False,
)
data = mujoco.MjData(model)

control_interface = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mujoco_mapping)
mujoco.mj_forward(model, data)

# Get initial position
core_body_id = None
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if name and "core" in name.lower():
        core_body_id = i
        break

start_pos = data.xpos[core_body_id].copy() if core_body_id else None

control_freq = batch_params.control_frequency
control_step = 1.0 / control_freq
last_control_time = 0.0
handler = simulation_scene.handler

print("\nStarting visualization - close window to stop...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        if data.time >= last_control_time + control_step:
            last_control_time = int(data.time / control_step) * control_step

            from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
            state = SimulationStateImpl(data, mujoco_mapping, {})
            handler.handle(state, control_interface, control_step)

        mujoco.mj_step(model, data)
        viewer.sync()

        # Print distance every 2 seconds
        if core_body_id and int(data.time * 10) % 20 == 0 and data.time > 0.1:
            curr_pos = data.xpos[core_body_id]
            dist = math.sqrt((curr_pos[0] - start_pos[0])**2 + (curr_pos[1] - start_pos[1])**2)
            print(f"Time: {data.time:.1f}s, Distance: {dist:.3f}m", end="\r")

print("\nVisualization ended.")
