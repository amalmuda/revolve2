"""Open insect body in MuJoCo viewer, flat, no movement."""
import numpy as np
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, Terrain
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.simulator import BatchParameters


body = modular_robots_v1.get("insect")
hinges = body.find_modules_of_type(ActiveHinge)
cpg, mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
brain = BrainCpgNetworkStatic.uniform_from_params(
    params=np.zeros(cpg.num_connections),
    cpg_network_structure=cpg,
    initial_state_uniform=0.0, output_mapping=mapping,
)
robot = ModularRobot(body=body, brain=brain)

terrain = Terrain(static_geometry=[GeometryPlane(
    pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
    texture=Texture(
        base_color=Color(200, 200, 200, 255),
        primary_color=Color(220, 220, 220, 255),
        secondary_color=Color(80, 80, 80, 255),
        map_type=MapType.MAP2D,
        reference=TextureReference(builtin="checker"),
        repeat=(50, 50),
    ),
)], friction=1.0)
scene = ModularRobotScene(terrain=terrain)
scene.add_robot(robot, pose=Pose())

simulator = LocalSimulator(headless=False, num_simulators=1)
batch_params = BatchParameters(
    simulation_time=30.0, sampling_frequency=5,
    simulation_timestep=0.004, control_frequency=20,
)
print("Opening viewer: INSECT (flat, no movement)")
print("Hinges: %d" % len(hinges))
simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)
