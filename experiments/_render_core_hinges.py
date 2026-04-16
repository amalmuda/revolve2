"""Render core with one hinge on each side, showing orientation differences."""
import sys
import numpy as np
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.v1 import BodyV1, ActiveHingeV1, BrickV1
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, Terrain
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.simulator import BatchParameters

body = BodyV1()

# Front: 0 deg hinge (vertical swing), no brick
body.core_v1.front = ActiveHingeV1(0.0)

# Back: 90 deg hinge (horizontal swing), no brick
body.core_v1.back = ActiveHingeV1(np.pi / 2.0)

# Left: 0 deg hinge, no brick
body.core_v1.left = ActiveHingeV1(0.0)

# Right: 90 deg hinge, no brick
body.core_v1.right = ActiveHingeV1(np.pi / 2.0)

hinges = body.find_modules_of_type(ActiveHinge)
cpg, mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
params = np.zeros(cpg.num_connections)
brain = BrainCpgNetworkStatic.uniform_from_params(
    params=params, cpg_network_structure=cpg,
    initial_state_uniform=0.0, output_mapping=mapping,
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

print("Core with 4 hinges:")
print("  Front: 0 deg (vertical swing)")
print("  Back:  90 deg (horizontal swing)")
print("  Left:  0 deg (vertical swing)")
print("  Right: 90 deg (horizontal swing)")
print("")
print("Compare how the servo orientation differs between 0 and 90 deg hinges.")

simulator = LocalSimulator(headless=False, num_simulators=1)
batch_params = BatchParameters(
    simulation_time=120, sampling_frequency=1,
    simulation_timestep=0.001, control_frequency=50,
)
simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)
