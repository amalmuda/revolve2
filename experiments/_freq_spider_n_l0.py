"""Check the evolved frequency of the Fox spider neighbor lambda0 run."""
import math
import numpy as np
import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2


SIM_TIME = 10.0

def zero_cross_freq(sig, fs):
    sig = sig - sig.mean()
    if np.allclose(sig, 0): return 0.0
    zc = np.where(np.diff(np.signbit(sig)))[0]
    if len(zc) < 2: return 0.0
    return fs / (np.mean(np.diff(zc)) * 2)

body = modular_robots_v1.get("spider")
hinges = body.find_modules_of_type(ActiveHinge)
cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
params = np.load("/tmp/fox_spider_n_l0_r1.npy")

brain = BrainCpgNetworkStatic.uniform_from_params(
    params=params, cpg_network_structure=cpg,
    initial_state_uniform=math.sqrt(2) * 0.5, output_mapping=mp,
)
robot = ModularRobot(body=body, brain=brain)
terrain = Terrain(static_geometry=[GeometryPlane(pose=Pose(), mass=0.0, size=Vector2([20.0,20.0]),
    texture=Texture(base_color=Color(200,200,200,255),primary_color=Color(220,220,220,255),secondary_color=Color(80,80,80,255),
    map_type=MapType.MAP2D, reference=TextureReference(builtin="checker"), repeat=(50,50)))], friction=1.0)
scene = ModularRobotScene(terrain=terrain)
scene.add_robot(robot)
sim_scene, _ = scene.to_simulation_scene()
batch = make_standard_batch_parameters()
batch.simulation_time = SIM_TIME
model, mj_mapping = scene_to_model(sim_scene, simulation_timestep=batch.simulation_timestep, cast_shadows=False, fast_sim=True)
data = mujoco.MjData(model)
ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)

hinge_addrs = [int(model.jnt_qposadr[j]) for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
mujoco.mj_forward(model, data)
control_step = 1.0 / batch.control_frequency
last_ctrl = 0.0
traces = []
while data.time < SIM_TIME:
    traces.append(np.array([data.qpos[a] for a in hinge_addrs]))
    if data.time >= last_ctrl + control_step:
        last_ctrl = math.floor(data.time/control_step)*control_step
        sim_state = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
        sim_scene.handler.handle(sim_state, ctrl, control_step)
    mujoco.mj_step(model, data)
traces = np.stack(traces, axis=0)
fs = 1.0 / batch.simulation_timestep
traces = traces[int(fs):]  # skip first 1s

print("Per-joint frequency (Hz):")
for j in range(traces.shape[1]):
    f = zero_cross_freq(traces[:, j], fs)
    print(f"  joint {j}: {f:.3f}")
freqs = [zero_cross_freq(traces[:, j], fs) for j in range(traces.shape[1])]
freqs = [f for f in freqs if f > 0.01]
if freqs:
    print(f"\nMean: {np.mean(freqs):.3f} Hz")
    print(f"Min: {np.min(freqs):.3f} Hz")
    print(f"Max: {np.max(freqs):.3f} Hz")
