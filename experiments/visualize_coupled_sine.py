"""Visualize the coupled sine brain robot with evolved parameters."""

import math
import numpy as np

from brain_sine_coupled import BrainCoupledSine
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.standards.terrains import flat

# Best parameters from 10-generation evolution with evolved frequency
# Achieved 3.25m distance in 10s
best_params = np.array([
    0.997, -0.337, +0.226,  # Hinge 0: amp, phase, offset
    0.911, -0.163, +0.369,  # Hinge 1
    0.050, -0.788, +0.475,  # Hinge 2
    0.826, +0.362, +0.372,  # Hinge 3
    0.601, -0.358, +0.152,  # Hinge 4
    0.447, +0.033, -0.409,  # Hinge 5
    0.705, +0.425, -0.173,  # Hinge 6
    0.505, +0.027, -0.499,  # Hinge 7
])
coupling_strength = 0.127
frequency = 1.458

print("=" * 60)
print("VISUALIZING COUPLED SINE BRAIN ROBOT")
print("=" * 60)
print("\nEvolved parameters (10 generations, pure distance):")
print(f"  Distance achieved: 3.25 m in 10s")
print(f"  Body contact: 41.2%")
print(f"  Coupling strength: {coupling_strength}")
print(f"  Frequency: {frequency} Hz")
print("\nStarting visualization...")

# Create robot
body = modular_robots_v1.get("spider")
brain = BrainCoupledSine.from_parameters(
    body,
    best_params,
    frequency=frequency,
    coupling_strength=coupling_strength,
)
robot = ModularRobot(body=body, brain=brain)

# Create scene
scene = ModularRobotScene(terrain=flat())
scene.add_robot(robot)

# Run visualization
simulator = LocalSimulator(headless=False, num_simulators=1)
batch_params = make_standard_batch_parameters()
batch_params.simulation_time = 15  # Run for 15 seconds

states = simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)

# Get final position
final_state = states[-1]
pose = final_state.get_modular_robot_simulation_state(robot).get_pose()
distance = math.sqrt(pose.position.x**2 + pose.position.y**2)

print(f"\nFinal position: ({pose.position.x:.3f}, {pose.position.y:.3f})")
print(f"Distance traveled: {distance:.3f} m")
print("\nVisualization complete!")
