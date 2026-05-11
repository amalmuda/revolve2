"""Visualize an evolved controller with per-oscillator nu jitter in MuJoCo viewer."""
import os, sys, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
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

from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
)
from bonardi_brain import (
    BrainBonardi, BrainBonardiInstance, bonardi_structure_from_cpg_structure,
)


parser = argparse.ArgumentParser()
parser.add_argument("--robot", default="spider")
parser.add_argument("--coupling", choices=["uncoupled", "neighbor", "blf"], required=True)
parser.add_argument("--evolve-phi0", action="store_true")
parser.add_argument("--params", required=True)
parser.add_argument("--jitter", type=float, default=0.02, help="nu jitter std (Hz)")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--time", type=float, default=30.0)
args = parser.parse_args()


body = modular_robots_v1.get(args.robot)
hinges = body.find_modules_of_type(ActiveHinge)
n = len(hinges)
rng = np.random.RandomState(args.seed)
nu_hz_vec = 0.5 + args.jitter * rng.randn(n)
print(f"per-oscillator nu (Hz): {nu_hz_vec.round(4)}")
NU_VEC_RAD = 2.0 * np.pi * nu_hz_vec


_orig_dphi = BrainBonardiInstance._dphi
def _dphi_jitter(self, phi):
    diff = phi[None, :] - phi[:, None] - self._psi
    coupling = (self._W * self._A[None, :] * np.sin(diff)).sum(axis=1)
    return NU_VEC_RAD + coupling
BrainBonardiInstance._dphi = _dphi_jitter


if args.coupling == "uncoupled":
    cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
elif args.coupling == "neighbor":
    cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
else:
    cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
ks = bonardi_structure_from_cpg_structure(cpg, evolve_phi0=args.evolve_phi0)
params = np.load(args.params)
brain = BrainBonardi.from_params(
    params=params, network_structure=ks, output_mapping=mp,
    nu_hz=0.5, w=1.0,
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
batch = BatchParameters(simulation_time=args.time, sampling_frequency=5,
                        simulation_timestep=0.004, control_frequency=20)
print(f"Opening viewer: {args.robot} | {args.coupling} | jitter={args.jitter} | seed={args.seed}")
simulate_scenes(simulator=simulator, batch_parameters=batch, scenes=scene)
