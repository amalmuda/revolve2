"""Visualize the best sine brain robot from the evolution."""

import math
import numpy as np

from brain_sine import BrainSine
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.standards.terrains import flat

# Best parameters from 10-generation evolution
best_params = np.array([
    0.912, 0.585,   # Hinge 0: amp=52.3°, phase=+33.5°
    0.600, -1.002,  # Hinge 1: amp=34.4°, phase=-57.4°
    0.929, -0.736,  # Hinge 2: amp=53.2°, phase=-42.2°
    0.809, 0.065,   # Hinge 3: amp=46.3°, phase=+3.7°
    0.448, -0.391,  # Hinge 4: amp=25.6°, phase=-22.4°
    0.122, -0.081,  # Hinge 5: amp=7.0°, phase=-4.6°
    0.258, -0.203,  # Hinge 6: amp=14.8°, phase=-11.6°
    0.059, 0.526,   # Hinge 7: amp=3.4°, phase=+30.1°
])

print("=" * 60)
print("VISUALIZING BEST SINE BRAIN ROBOT")
print("=" * 60)
print(f"\nDistance achieved: 2.48 m in 10s")
print(f"Body contact: 59.3%")
print(f"Fitness: 1.84")
print("\nStarting visualization...")

# Create robot
body = modular_robots_v1.get("spider")
brain = BrainSine.from_parameters(body, best_params, frequency=1.0)
robot = ModularRobot(body=body, brain=brain)

# Create scene
scene = ModularRobotScene(terrain=flat())
scene.add_robot(robot)

# Run visualization
simulator = LocalSimulator(headless=False, num_simulators=1)
batch_params = make_standard_batch_parameters()
batch_params.simulation_time = 15  # Run for 15 seconds

simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)

print("\nVisualization complete!")
