"""Debug script to see which bodies are classified as leaf/non-leaf and which are touching."""

import math
import config
import numpy as np
import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.modular_robot_simulation import ModularRobotScene
from revolve2.simulation.scene import UUIDKey
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.standards import terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters

PARAMS = np.array([0.97560332, -0.98052375, 0.70084728, -0.06435287, 0.69862624, -0.05989215, -0.99993381, 0.99979145, 0.99602341, 0.64765969, 0.72665882, -0.90461619])

# Find all active hinges in the body
active_hinges = config.BODY.find_modules_of_type(ActiveHinge)

# Create CPG network structure
cpg_network_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

# Create robot
robot = ModularRobot(
    body=config.BODY,
    brain=BrainCpgNetworkStatic.uniform_from_params(
        params=PARAMS,
        cpg_network_structure=cpg_network_structure,
        initial_state_uniform=math.sqrt(2) * 0.5,
        output_mapping=output_mapping,
    ),
)

# Create scene
scene = ModularRobotScene(terrain=terrains.flat())
scene.add_robot(robot)
sim_scene, robot_mapping = scene.to_simulation_scene()

# Get parameters
batch_params = make_standard_batch_parameters()

# Convert to MuJoCo model
model, abstraction_mapping = scene_to_model(
    scene=sim_scene,
    simulation_timestep=batch_params.simulation_timestep,
    cast_shadows=False,
    fast_sim=True,
)

# Initialize MuJoCo data
data = mujoco.MjData(model)

# Identify terrain geometries
terrain_geom_ids = set()
for i in range(model.ngeom):
    body_id = model.geom_bodyid[i]
    if body_id <= 1:
        terrain_geom_ids.add(i)

# Identify leaf brick bodies
children_count = {}
for i in range(model.nbody):
    children_count[i] = 0

for i in range(model.nbody):
    parent_id = model.body_parentid[i]
    if parent_id >= 0:
        children_count[parent_id] += 1

leaf_brick_body_ids = set()
for i in range(model.nbody):
    # Leaf = robot body (not world/terrain) with no children
    if i > 1 and children_count[i] == 0:
        leaf_brick_body_ids.add(i)

# Collect all robot body IDs
all_robot_body_ids = set()
for i in range(model.ngeom):
    body_id = model.geom_bodyid[i]
    geom_id = i
    if geom_id not in terrain_geom_ids and body_id > 1:
        all_robot_body_ids.add(body_id)

# Non-leaf bodies
non_leaf_bodies = all_robot_body_ids - leaf_brick_body_ids

print("\n" + "="*70)
print("BODY CLASSIFICATION")
print("="*70)
print(f"\nTotal robot bodies: {len(all_robot_body_ids)}")
print(f"Leaf brick bodies: {len(leaf_brick_body_ids)}")
print(f"Non-leaf bodies: {len(non_leaf_bodies)}")

print("\n--- LEAF BODIES (should be feet) ---")
for body_id in sorted(leaf_brick_body_ids):
    print(f"  Body {body_id}: {model.body(body_id).name}")

print("\n--- NON-LEAF BODIES (core, hinges, non-terminal bricks) ---")
for body_id in sorted(non_leaf_bodies):
    print(f"  Body {body_id}: {model.body(body_id).name}")

# Run simulation for a bit and track which non-leaf bodies touch
print("\n" + "="*70)
print("RUNNING SIMULATION TO TRACK CONTACTS")
print("="*70)

mujoco.mj_forward(model, data)
control_step = 1.0 / batch_params.control_frequency
last_control_time = 0.0
control_interface = ControlInterfaceImpl(data, abstraction_mapping)

non_leaf_touch_counts = {body_id: 0 for body_id in non_leaf_bodies}
steps = 0
max_steps = 1000  # Just run for a short time

while steps < max_steps:
    # Control
    if data.time >= last_control_time + control_step:
        last_control_time = math.floor(data.time / control_step) * control_step
        simulation_state = SimulationStateImpl(
            data=data,
            abstraction_to_mujoco_mapping=abstraction_mapping,
            camera_views={},
        )
        sim_scene.handler.handle(simulation_state, control_interface, control_step)

    # Step simulation
    mujoco.mj_step(model, data)
    steps += 1

    # Collect contacts
    touching_bodies = set()
    for i in range(data.ncon):
        contact = data.contact[i]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)

        robot_geom = None
        if geom1 in terrain_geom_ids and geom2 not in terrain_geom_ids:
            robot_geom = geom2
        elif geom2 in terrain_geom_ids and geom1 not in terrain_geom_ids:
            robot_geom = geom1

        if robot_geom is not None:
            body_id = int(model.geom_bodyid[robot_geom])
            touching_bodies.add(body_id)

    # Count non-leaf contacts
    for body_id in touching_bodies & non_leaf_bodies:
        non_leaf_touch_counts[body_id] += 1

print(f"\nSimulated {steps} timesteps ({data.time:.2f}s)")
print("\n--- NON-LEAF BODY CONTACT FREQUENCY ---")
for body_id in sorted(non_leaf_touch_counts.keys(), key=lambda x: non_leaf_touch_counts[x], reverse=True):
    count = non_leaf_touch_counts[body_id]
    percentage = (count / steps) * 100
    if count > 0:
        print(f"  Body {body_id:2d} ({model.body(body_id).name:30s}): {count:4d}/{steps} ({percentage:5.1f}%)")

print("\n" + "="*70)
