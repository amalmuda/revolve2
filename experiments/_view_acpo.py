"""View an evolved ACPO robot."""
import math
import sys
import numpy as np

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, Terrain
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.simulator import BatchParameters

from acpo_brain import BrainAcpo, acpo_structure_from_cpg_structure
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
)


ROBOT = sys.argv[1] if len(sys.argv) > 1 else "spider"
PARAMS = sys.argv[2] if len(sys.argv) > 2 else "acpo_spider_blf_lam1_1p00hz_best.npy"
COUPLING = sys.argv[3] if len(sys.argv) > 3 else "blf"
HZ = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
SIM_TIME = float(sys.argv[5]) if len(sys.argv) > 5 else 20.0

body = modular_robots_v1.get(ROBOT)
hinges = body.find_modules_of_type(ActiveHinge)
if COUPLING == "blf":
    cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
elif COUPLING == "uncoupled":
    cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
else:
    cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
acpo_s = acpo_structure_from_cpg_structure(cpg)
params = np.load(PARAMS)

brain = BrainAcpo.from_params(
    params=params, network_structure=acpo_s,
    output_mapping=mp, omega=2 * math.pi * HZ,
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
scene.add_robot(robot)

print(f"Opening: {ROBOT.upper()} ACPO {COUPLING} | omega={HZ} Hz")
sim = LocalSimulator(headless=False, num_simulators=1)
batch = BatchParameters(
    simulation_time=int(SIM_TIME), sampling_frequency=1,
    simulation_timestep=0.001, control_frequency=50,
)
simulate_scenes(simulator=sim, batch_parameters=batch, scenes=scene)
