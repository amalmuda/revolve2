"""Spider on a hills environment (de Bruin et al. 2024 style):
parallel hills perpendicular to walking direction, 0.35 m tall, 2 m spaced."""
import numpy as np
from pyrr import Vector3
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
from revolve2.simulation.scene.geometry import GeometryBox, GeometryPlane
from revolve2.simulation.scene.geometry.textures import (
    MapType, Texture, TextureReference,
)
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.scene._aabb import AABB
from revolve2.simulation.simulator import BatchParameters


# Hills env (de Bruin 2024):
# 0.35 m tall hills every 2 m, perpendicular to walking direction.
# Robot walks toward -x; hills are long ridges along y-axis at fixed x positions.
HILL_HEIGHT = 0.35
HILL_DEPTH = 0.40   # along x (how thick each hill is)
HILL_WIDTH = 4.0    # along y
HILL_SPACING = 2.0  # 2 m between centres
N_HILLS = 6


body = modular_robots_v1.get("spider")
hinges = body.find_modules_of_type(ActiveHinge)
cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
brain = BrainCpgNetworkStatic.uniform_from_params(
    params=np.zeros(cpg.num_connections),
    cpg_network_structure=cpg,
    initial_state_uniform=0.0, output_mapping=mp,
)
robot = ModularRobot(body=body, brain=brain)


floor_tex = Texture(
    base_color=Color(245, 240, 230, 255),
    primary_color=Color(245, 240, 230, 255),
    secondary_color=Color(220, 215, 205, 255),
    map_type=MapType.MAP2D,
    reference=TextureReference(builtin="checker"),
    repeat=(50, 50),
)
hill_tex = Texture(
    base_color=Color(105, 179, 162, 255),
    primary_color=Color(105, 179, 162, 255),
    secondary_color=Color(60, 120, 100, 255),
    map_type=MapType.MAP2D,
    reference=TextureReference(builtin="flat"),
    repeat=(1, 1),
)

geoms = [
    GeometryPlane(
        pose=Pose(),
        mass=0.0,
        size=Vector2([20.0, 20.0]),
        texture=floor_tex,
    )
]

# First hill at x = -1.0 (in front of robot if it walks -x), then -3, -5, ...
for i in range(N_HILLS):
    cx = -1.0 - i * HILL_SPACING
    geoms.append(GeometryBox(
        pose=Pose(position=Vector3([cx, 0.0, HILL_HEIGHT/2])),
        mass=0.0,
        aabb=AABB(size=Vector3([HILL_DEPTH, HILL_WIDTH, HILL_HEIGHT])),
        texture=hill_tex,
    ))

print(f"Hills env: {N_HILLS} hills, height {HILL_HEIGHT*100:.0f} cm, "
      f"spacing {HILL_SPACING:.1f} m")

terrain = Terrain(static_geometry=geoms, friction=1.0)
scene = ModularRobotScene(terrain=terrain)
scene.add_robot(robot, pose=Pose(position=Vector3([0.0, 0.0, 0.15])))

simulator = LocalSimulator(headless=False, num_simulators=1)
batch = BatchParameters(simulation_time=10.0, sampling_frequency=5,
                        simulation_timestep=0.004, control_frequency=20)
print("Opening viewer: SPIDER on HILLS")
simulate_scenes(simulator=simulator, batch_parameters=batch, scenes=scene)
