"""Simulate the ODE-CPG spider with neighbor coupling lambda=0 from Fox."""
import math
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
from contact_detection import get_robot_core_body_id, get_contacts_with_ground, identify_geometry_types

SIM_TIME = 20.0

body = modular_robots_v1.get("spider")
hinges = body.find_modules_of_type(ActiveHinge)
cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
params = np.load("/tmp/fox_spider_n_l0_r1.npy")
print(f"Params shape: {params.shape}")
print(f"Param values: min={params.min():.3f}, max={params.max():.3f}")

brain = BrainCpgNetworkStatic.uniform_from_params(
    params=params, cpg_network_structure=cpg,
    initial_state_uniform=math.sqrt(2) * 0.5, output_mapping=mp,
)
robot = ModularRobot(body=body, brain=brain)

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
batch = make_standard_batch_parameters()
batch.simulation_time = SIM_TIME

model, mj_mapping = scene_to_model(
    sim_scene, simulation_timestep=batch.simulation_timestep,
    cast_shadows=False, fast_sim=True,
)
data = mujoco.MjData(model)
ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)

ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
non_foot = robot_ids - foot_ids

core_id = get_robot_core_body_id(model)
mujoco.mj_forward(model, data)
initial_pos = data.xpos[core_id].copy()

control_step = 1.0 / batch.control_frequency
last_ctrl = 0.0
total = 0
drag = 0

while data.time < SIM_TIME:
    total += 1
    contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
    if any(rg in non_foot for rg, *_ in contacts):
        drag += 1
    if data.time >= last_ctrl + control_step:
        last_ctrl = math.floor(data.time / control_step) * control_step
        sim_state = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
        sim_scene.handler.handle(sim_state, ctrl, control_step)
    mujoco.mj_step(model, data)

final_pos = data.xpos[core_id].copy()
dx = final_pos[0] - initial_pos[0]
dy = final_pos[1] - initial_pos[1]
dist = math.sqrt(dx*dx + dy*dy)
drag_pct = drag / total

print(f"\nDistance: {dist:.3f} m (in {SIM_TIME}s)")
print(f"Dragging: {drag_pct:.2%}")
