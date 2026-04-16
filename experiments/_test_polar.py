"""Quick test that polar Hopf produces coordinated oscillation at target frequency."""
import math
import numpy as np
import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
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

from hopf_brain import (
    BrainHopfPolarStatic,
    hopf_structure_from_cpg_structure,
)

body = modular_robots_v1.get("spider")
hinges = body.find_modules_of_type(ActiveHinge)
cpg_struct, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
hopf_struct = hopf_structure_from_cpg_structure(cpg_struct)

n = hopf_struct.num_oscillators
nc = hopf_struct.num_connections
print(f"Oscillators: {n}")
print(f"Connections: {nc}")
print(f"Polar params expected: {n + 2 * nc}")

# Test params: all mu=0.5, all weights=0.5, various phase targets
params = np.concatenate([
    np.full(n, 0.5),            # mu
    np.full(nc, 0.5),            # weights
    np.full(nc, math.pi),        # phase offsets = pi (anti-phase target)
])
print(f"Total params: {len(params)}")

brain = BrainHopfPolarStatic.from_params(
    params=params, network_structure=hopf_struct, output_mapping=mp,
    omega=2 * math.pi * 1.0,  # 1 Hz target
)
robot = ModularRobot(body=body, brain=brain)
terrain = Terrain(static_geometry=[GeometryPlane(pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
    texture=Texture(base_color=Color(200,200,200,255),primary_color=Color(220,220,220,255),
    secondary_color=Color(80,80,80,255),map_type=MapType.MAP2D,
    reference=TextureReference(builtin="checker"), repeat=(50,50)))], friction=1.0)
scene = ModularRobotScene(terrain=terrain)
scene.add_robot(robot)
sim_scene, _ = scene.to_simulation_scene()
batch = make_standard_batch_parameters()
batch.simulation_time = 10.0
model, mj_mapping = scene_to_model(sim_scene, simulation_timestep=batch.simulation_timestep,
                                    cast_shadows=False, fast_sim=True)
data = mujoco.MjData(model)
ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
mujoco.mj_forward(model, data)

control_step = 1.0 / batch.control_frequency
last_ctrl = 0.0
hinge_addrs = [int(model.jnt_qposadr[j]) for j in range(model.njnt)
               if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
traces = []
while data.time < 10.0:
    traces.append(np.array([data.qpos[a] for a in hinge_addrs]))
    if data.time >= last_ctrl + control_step:
        last_ctrl = math.floor(data.time/control_step)*control_step
        sim_state = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
        sim_scene.handler.handle(sim_state, ctrl, control_step)
    mujoco.mj_step(model, data)
traces = np.stack(traces, axis=0)

fs = 1.0 / batch.simulation_timestep
traces = traces[int(fs):]  # skip first 1s

# Zero-crossing frequency per joint
print("\nJoint frequencies:")
for j in range(traces.shape[1]):
    sig = traces[:, j] - traces[:, j].mean()
    if np.allclose(sig, 0):
        print(f"  joint {j}: silent")
        continue
    zc = np.where(np.diff(np.signbit(sig)))[0]
    if len(zc) < 2:
        print(f"  joint {j}: no oscillation")
        continue
    period = np.mean(np.diff(zc)) * 2
    freq = fs / period
    print(f"  joint {j}: {freq:.3f} Hz, range [{traces[:,j].min():.2f}, {traces[:,j].max():.2f}]")

print("\nTarget frequency was 1.0 Hz.")
