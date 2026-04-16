"""View the evolved Hopf spider in the MuJoCo viewer."""
import math
import sys
import numpy as np

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
)
from revolve2.standards import modular_robots_v1
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes, Terrain
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
from revolve2.simulation.simulator import BatchParameters

from hopf_brain import BrainHopfStatic, hopf_structure_from_cpg_structure


ROBOT = sys.argv[1] if len(sys.argv) > 1 else "spider"
PARAMS = sys.argv[2] if len(sys.argv) > 2 else "hopf_spider_best.npy"
HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
COUPLING = sys.argv[5] if len(sys.argv) > 5 else "neighbor"

body = modular_robots_v1.get(ROBOT)
hinges = body.find_modules_of_type(ActiveHinge)
if COUPLING == "blf":
    cpg_struct, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)
elif COUPLING == "uncoupled":
    cpg_struct, mapping = active_hinges_to_cpg_network_structure_internal_only(hinges)
else:
    cpg_struct, mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
hopf_struct = hopf_structure_from_cpg_structure(cpg_struct)

params = np.load(PARAMS)
brain = BrainHopfStatic.from_params(
    params=params,
    network_structure=hopf_struct,
    output_mapping=mapping,
    omega=2 * math.pi * HZ,
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

print(f"Opening viewer: {ROBOT.upper()} with Hopf brain ({PARAMS})")
simulator = LocalSimulator(headless=False, num_simulators=1)
SIM_TIME = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
batch = BatchParameters(
    simulation_time=int(SIM_TIME), sampling_frequency=1,
    simulation_timestep=0.001, control_frequency=50,
)
simulate_scenes(simulator=simulator, batch_parameters=batch, scenes=scene)
