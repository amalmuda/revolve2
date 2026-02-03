"""Test sine controller with user-provided parameters (no visualization)."""

import math
import numpy as np

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.standards import modular_robots_v1

from contact_detection import calculate_locomotion_metrics
from brain_sine_offset import BrainSineOffset, SineOffsetParameters

# Import simulation function from existing script
from run_cpg_12params import simulate_robot

# User parameters
amplitudes = np.array([0.91351724, 0.71214163, 0.73219062, 0.50504586,
                       0.89844802, 0.88646999, 0.781354, 0.15433051])
phases = np.array([0.99224094, 5.98598932, 4.29914234, 6.18809434,
                   0.81121247, 1.79327941, 3.71902453, 0.74360644])
offsets = np.array([-0.35346571, 0.23869804, -0.42190033, 0.125564,
                    0.17553313, -0.22235914, 0.01664249, -0.03592111])
frequency = 0.3  # Hz for all joints

print("=" * 60)
print("Testing Sine Controller with User Parameters")
print("=" * 60)
print(f"\nAmplitudes: {amplitudes}")
print(f"Phases: {phases}")
print(f"Offsets: {offsets}")
print(f"Frequency: {frequency} Hz")
print("Coupling: uncoupled")

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

print("\nRunning 10s simulation...")
contact_tracker, loco_tracker = simulate_robot(robot, simulation_time=10.0)

metrics = calculate_locomotion_metrics(loco_tracker, contact_tracker, 10.0)

print("\n" + "=" * 60)
print("RESULTS")
print("=" * 60)
print(f"Distance traveled:    {metrics.distance:.4f} m")
print(f"Body contact (M1):    {metrics.contact_metric_1:.1%}")
print(f"Contact+slip (M2):    {metrics.contact_metric_2:.1%}")
print(f"Path length:          {loco_tracker.total_path_length:.4f} m")
print(f"Energy used:          {loco_tracker.total_energy:.4f} J")
