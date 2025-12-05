"""Analyze which body parts touch the ground most frequently."""

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

print("=" * 80)
print("COMPLETE BODY CONTACT ANALYSIS")
print("=" * 80)

# Run simulation and track all body contacts
mujoco.mj_forward(model, data)
control_step = 1.0 / batch_params.control_frequency
last_control_time = 0.0
control_interface = ControlInterfaceImpl(data, abstraction_mapping)

# Track contacts for all bodies
all_body_contacts = {body_id: 0 for body_id in all_robot_body_ids}
steps = 0
max_steps = 1000

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
            if body_id in all_robot_body_ids:
                touching_bodies.add(body_id)

    # Count contacts
    for body_id in touching_bodies:
        all_body_contacts[body_id] += 1

# Display results
print(f"\nSimulated {steps} timesteps ({data.time:.2f}s)")
print("\n" + "=" * 80)
print("ALL ROBOT BODY CONTACTS (sorted by frequency)")
print("=" * 80)
print(f"\n{'Body ID':<8} {'Type':<10} {'Name':<35} {'Contacts':<10} {'%':<8}")
print("-" * 80)

# Sort by contact frequency
sorted_bodies = sorted(all_body_contacts.items(), key=lambda x: x[1], reverse=True)

for body_id, count in sorted_bodies:
    body_type = "LEAF" if body_id in leaf_brick_body_ids else "NON-LEAF"
    name = model.body(body_id).name
    percentage = (count / steps) * 100

    # Highlight the most contacted body
    marker = " ← MOST CONTACT" if count == sorted_bodies[0][1] and count > 0 else ""

    print(f"{body_id:<8} {body_type:<10} {name:<35} {count:<10} {percentage:>6.1f}%{marker}")

# Summary statistics
print("\n" + "=" * 80)
print("SUMMARY")
print("=" * 80)

total_leaf_contacts = sum(count for body_id, count in all_body_contacts.items() if body_id in leaf_brick_body_ids)
total_non_leaf_contacts = sum(count for body_id, count in all_body_contacts.items() if body_id in non_leaf_bodies)

print(f"\nTotal leaf body contacts: {total_leaf_contacts} ({total_leaf_contacts/(steps*len(leaf_brick_body_ids))*100:.1f}% avg per leaf body)")
print(f"Total non-leaf body contacts: {total_non_leaf_contacts} ({total_non_leaf_contacts/(steps*len(non_leaf_bodies))*100:.1f}% avg per non-leaf body)")

print(f"\nLeaf bodies (feet): {len(leaf_brick_body_ids)}")
print(f"Non-leaf bodies (core/hinges/segments): {len(non_leaf_bodies)}")

# Find which body is Body 5 specifically
print("\n" + "=" * 80)
print("BODY 5 DETAILS (highest non-leaf contact)")
print("=" * 80)
body5_name = model.body(5).name
body5_parent = model.body_parentid[5]
body5_parent_name = model.body(body5_parent).name if body5_parent >= 0 else "None"
print(f"Body 5 name: {body5_name}")
print(f"Parent body: {body5_parent} ({body5_parent_name})")
print(f"Is leaf: {5 in leaf_brick_body_ids}")
print(f"Contacts: {all_body_contacts[5]}/{steps} ({all_body_contacts[5]/steps*100:.1f}%)")

print("=" * 80)
