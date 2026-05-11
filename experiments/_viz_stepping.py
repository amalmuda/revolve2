"""Open spider on stepping terrain in MuJoCo viewer (no movement, just look)."""
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
from revolve2.simulation.scene._aabb import AABB
from revolve2.simulation.scene.geometry.textures import (
    MapType, Texture, TextureReference,
)
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.simulator import BatchParameters


# Stepping terrain parameters - kept small to stay under MuJoCo's 1000-texture limit
TERRAIN_SIZE = (6.0, 6.0)
BOX_SIZE = (0.25, 0.25)
BOX_HEIGHT = 0.05
SPACING = 0.70
JITTER = 0.05


body = modular_robots_v1.get("spider")
hinges = body.find_modules_of_type(ActiveHinge)
cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
brain = BrainCpgNetworkStatic.uniform_from_params(
    params=np.zeros(cpg.num_connections),
    cpg_network_structure=cpg,
    initial_state_uniform=0.0, output_mapping=mp,
)
robot = ModularRobot(body=body, brain=brain)


# Build stepping terrain: ground plane + grid of boxes
geoms = [
    GeometryPlane(
        pose=Pose(),
        mass=0.0,
        size=Vector2([TERRAIN_SIZE[0], TERRAIN_SIZE[1]]),
        texture=Texture(
            base_color=Color(245, 240, 230, 255),
            primary_color=Color(245, 240, 230, 255),
            secondary_color=Color(220, 215, 205, 255),
            map_type=MapType.MAP2D,
            reference=TextureReference(builtin="checker"),
            repeat=(50, 50),
        ),
    )
]

box_texture = Texture(
    base_color=Color(105, 179, 162, 255),
    primary_color=Color(105, 179, 162, 255),
    secondary_color=Color(60, 120, 100, 255),
    map_type=MapType.MAP2D,
    reference=TextureReference(builtin="flat"),
    repeat=(1, 1),
)

# Use a smaller terrain region for the demo to keep texture count low.
rng = np.random.RandomState(0)
xs = np.arange(-TERRAIN_SIZE[0]/2, TERRAIN_SIZE[0]/2, SPACING)
ys = np.arange(-TERRAIN_SIZE[1]/2, TERRAIN_SIZE[1]/2, SPACING)
n_boxes = 0
for x in xs:
    for y in ys:
        dx = (rng.rand() - 0.5) * 2 * JITTER
        dy = (rng.rand() - 0.5) * 2 * JITTER
        # Skip a small area around origin so robot has space to spawn
        if abs(x + dx) < 0.4 and abs(y + dy) < 0.4:
            continue
        geoms.append(GeometryBox(
            pose=Pose(position=Vector3([x + dx, y + dy, BOX_HEIGHT/2])),
            mass=0.0,
            aabb=AABB(size=Vector3([BOX_SIZE[0], BOX_SIZE[1], BOX_HEIGHT])),
            texture=box_texture,
        ))
        n_boxes += 1
print(f"Built stepping terrain with {n_boxes} boxes")

terrain = Terrain(static_geometry=geoms, friction=1.0)
scene = ModularRobotScene(terrain=terrain)
scene.add_robot(robot, pose=Pose(position=Vector3([0.0, 0.0, 0.3])))

simulator = LocalSimulator(headless=False, num_simulators=1)
batch = BatchParameters(simulation_time=10.0, sampling_frequency=5,
                        simulation_timestep=0.004, control_frequency=20)
print("Opening viewer: SPIDER on STEPPING terrain (no movement)")
simulate_scenes(simulator=simulator, batch_parameters=batch, scenes=scene)
