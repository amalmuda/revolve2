"""Visualize gecko with gen 50 parameters using LocalSimulator."""

import numpy as np

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import modular_robots_v2
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.standards.terrains import flat

from brain_sine_offset import BrainSineOffset, SineOffsetParameters

# Gen 50 parameters
amplitudes = np.array([0.99686259, 0.96958213, 0.34532825, 0.66528586, 0.96023932, 0.99998648])
phases = np.array([0.9106601, 1.63869106, -1.1872914, 2.73806191, 2.45517221, -0.27228478])
offsets = np.array([0.08160152, -0.40091293, 0.47257527, 0.42236993, 0.30910061, 0.41825748])
frequency = 0.5  # Hz

print("=" * 60)
print("VISUALIZING GECKO GEN 50")
print("=" * 60)
print(f"\nAmplitudes: {amplitudes}")
print(f"Phases: {phases}")
print(f"Offsets: {offsets}")
print(f"Frequency: {frequency} Hz")

# Create robot
body = modular_robots_v2.get("gecko")
active_hinges = body.find_modules_of_type(ActiveHinge)

print(f"\nGecko has {len(active_hinges)} active hinges")

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

# Create scene with flat terrain
scene = ModularRobotScene(terrain=flat())
scene.add_robot(robot)

# Run visualization
print("\nStarting visualization...")
print("Controls: Mouse drag=rotate, Scroll=zoom, Space=pause")

simulator = LocalSimulator(headless=False, num_simulators=1)
batch_params = make_standard_batch_parameters()
batch_params.simulation_time = 20  # Run for 20 seconds

simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)

print("\nVisualization complete!")
