"""Spider on stairs - ascending steps in robot's walking direction."""
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


# Stairs: 10 steps, each 30 cm deep (along walking direction), 4 m wide (perpendicular)
N_STEPS = 10
STEP_DEPTH = 0.30   # along x (walking direction)
STEP_WIDTH = 4.0    # along y (perpendicular)
STEP_RISE = 0.04    # 4 cm per step → 40 cm total climb
START_X = -0.8       # first step starts at x = -0.8 (robot walks toward -x)
ROBOT_SPAWN = Vector3([0.0, 0.0, 0.15])


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
step_tex = Texture(
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

# Each step is a box. Step i has top surface at height i * STEP_RISE.
# Box height grows with step index so the box reaches from floor (0) to its top.
for i in range(N_STEPS):
    top_z = (i + 1) * STEP_RISE
    box_h = top_z
    cx = START_X - (i + 0.5) * STEP_DEPTH
    cy = 0.0
    cz = box_h / 2
    geoms.append(GeometryBox(
        pose=Pose(position=Vector3([cx, cy, cz])),
        mass=0.0,
        aabb=AABB(size=Vector3([STEP_DEPTH, STEP_WIDTH, box_h])),
        texture=step_tex,
    ))

print(f"Built stairs: {N_STEPS} steps, "
      f"{STEP_RISE*100:.1f} cm rise each, "
      f"total climb {N_STEPS * STEP_RISE * 100:.0f} cm")

terrain = Terrain(static_geometry=geoms, friction=1.0)
scene = ModularRobotScene(terrain=terrain)
scene.add_robot(robot, pose=Pose(position=ROBOT_SPAWN))

simulator = LocalSimulator(headless=False, num_simulators=1)
batch = BatchParameters(simulation_time=10.0, sampling_frequency=5,
                        simulation_timestep=0.004, control_frequency=20)
print("Opening viewer: SPIDER on STAIRS (no movement)")
simulate_scenes(simulator=simulator, batch_parameters=batch, scenes=scene)
