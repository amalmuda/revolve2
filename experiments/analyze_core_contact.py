"""Analyze core contact patterns in detail."""

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

# Find the core body (should be body 2, the root of the robot)
core_body_id = 2  # Based on debug_contacts.py output

print("=" * 70)
print("CORE CONTACT ANALYSIS")
print("=" * 70)
print(f"\nCore body ID: {core_body_id}")
print(f"Core body name: {model.body(core_body_id).name}")

# Run simulation and track core contact patterns
mujoco.mj_forward(model, data)
control_step = 1.0 / batch_params.control_frequency
last_control_time = 0.0
control_interface = ControlInterfaceImpl(data, abstraction_mapping)

steps = 0
max_steps = 1000
core_contact_steps = []  # Track which timesteps have core contact
contact_durations = []  # Track consecutive contact periods
current_duration = 0

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

    # Check for core contact
    core_touching = False
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
            if body_id == core_body_id:
                core_touching = True
                break

    if core_touching:
        core_contact_steps.append(steps)
        current_duration += 1
    else:
        if current_duration > 0:
            contact_durations.append(current_duration)
            current_duration = 0

# Don't forget the last duration if simulation ended during contact
if current_duration > 0:
    contact_durations.append(current_duration)

# Analysis
total_core_contacts = len(core_contact_steps)
core_contact_ratio = total_core_contacts / steps

print(f"\n--- OVERALL STATISTICS ---")
print(f"Total timesteps: {steps}")
print(f"Core contact timesteps: {total_core_contacts}")
print(f"Core contact ratio: {core_contact_ratio:.1%}")

if contact_durations:
    print(f"\n--- CONTACT DURATION ANALYSIS ---")
    print(f"Number of contact events: {len(contact_durations)}")
    print(f"Average contact duration: {np.mean(contact_durations):.1f} timesteps")
    print(f"Median contact duration: {np.median(contact_durations):.1f} timesteps")
    print(f"Max contact duration: {max(contact_durations)} timesteps")
    print(f"Min contact duration: {min(contact_durations)} timesteps")

    # Show distribution of contact durations
    print(f"\n--- CONTACT DURATION DISTRIBUTION ---")
    if len(contact_durations) > 0:
        bins = [1, 2, 5, 10, 20, float('inf')]
        labels = ['1', '2-4', '5-9', '10-19', '20+']
        for i in range(len(bins) - 1):
            count = sum(1 for d in contact_durations if bins[i] <= d < bins[i+1])
            if count > 0:
                print(f"  {labels[i]:>5s} timesteps: {count:3d} events ({count/len(contact_durations)*100:5.1f}%)")

    # Temporal distribution
    print(f"\n--- TEMPORAL DISTRIBUTION ---")
    simulation_time = steps  # in timesteps
    quarter = simulation_time // 4
    quarters = [0] * 4
    for step in core_contact_steps:
        quarter_idx = min(3, (step - 1) // quarter)
        quarters[quarter_idx] += 1

    for i, count in enumerate(quarters):
        start = i * quarter
        end = (i + 1) * quarter if i < 3 else simulation_time
        print(f"  Quarter {i+1} (steps {start:4d}-{end:4d}): {count:3d} contacts ({count/quarter*100 if quarter > 0 else 0:5.1f}%)")

    # Show a sample of when core contacts occur
    print(f"\n--- SAMPLE CORE CONTACT TIMELINE (first 100 steps) ---")
    timeline = ""
    for step in range(1, min(101, steps + 1)):
        if step in core_contact_steps:
            timeline += "█"
        else:
            timeline += "·"
        if step % 50 == 0:
            timeline += f" {step}\n"
    print(timeline)
    print("(█ = core contact, · = no core contact)")

print("\n" + "=" * 70)
