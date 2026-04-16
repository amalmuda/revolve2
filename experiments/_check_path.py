"""Check if a robot walks straight or circles by tracking its core trajectory."""
import math
import sys
import numpy as np
import mujoco

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
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

from hopf_brain import BrainHopfStatic, hopf_structure_from_cpg_structure
from contact_detection import get_robot_core_body_id

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "spider"
PARAMS = sys.argv[2] if len(sys.argv) > 2 else f"hopf_{ROBOT}_best.npy"
HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
COUPLING = sys.argv[4] if len(sys.argv) > 4 else "neighbor"
SIM_TIME = 20.0

body = modular_robots_v1.get(ROBOT)
hinges = body.find_modules_of_type(ActiveHinge)
if COUPLING == "blf":
    cpg_struct, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
elif COUPLING == "uncoupled":
    cpg_struct, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
else:
    cpg_struct, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
hopf_struct = hopf_structure_from_cpg_structure(cpg_struct)
params = np.load(PARAMS)
brain = BrainHopfStatic.from_params(
    params=params, network_structure=hopf_struct, output_mapping=mp,
    omega=2 * math.pi * HZ,
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
core = get_robot_core_body_id(model)
mujoco.mj_forward(model, data)

control_step = 1.0 / batch.control_frequency
last_ctrl = 0.0
positions = []
sample_step = 0.1
last_sample = -1
while data.time < SIM_TIME:
    if data.time - last_sample >= sample_step:
        last_sample = data.time
        positions.append((data.time, data.xpos[core][0], data.xpos[core][1]))
    if data.time >= last_ctrl + control_step:
        last_ctrl = math.floor(data.time/control_step)*control_step
        sim_state = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
        sim_scene.handler.handle(sim_state, ctrl, control_step)
    mujoco.mj_step(model, data)

positions = np.array(positions)
times = positions[:, 0]
xs = positions[:, 1]
ys = positions[:, 2]

# Displacement and path length
dx = xs[-1] - xs[0]
dy = ys[-1] - ys[0]
displacement = math.sqrt(dx*dx + dy*dy)
path_lengths = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
total_path = float(np.sum(path_lengths))
em = displacement / total_path if total_path > 0 else 0.0

# Heading at start (first 3s) vs end (last 3s)
def avg_heading(start_t, end_t):
    mask = (times >= start_t) & (times <= end_t)
    if mask.sum() < 2:
        return None
    seg_x = xs[mask]
    seg_y = ys[mask]
    return math.degrees(math.atan2(seg_y[-1] - seg_y[0], seg_x[-1] - seg_x[0]))

h_start = avg_heading(0, 3)
h_mid = avg_heading(8, 12)
h_end = avg_heading(17, 20)

print(f"Robot: {ROBOT}")
print(f"Sim time: {SIM_TIME}s")
print()
print(f"Start position: ({xs[0]:.3f}, {ys[0]:.3f})")
print(f"End position:   ({xs[-1]:.3f}, {ys[-1]:.3f})")
print(f"Displacement:   {displacement:.3f} m")
print(f"Total path:     {total_path:.3f} m")
print(f"Effective Movement (EM): {em:.3f}  (1.0 = straight, 0 = back to start)")
print()
print(f"Heading first 3s:  {h_start:.1f} deg")
print(f"Heading 8-12s:     {h_mid:.1f} deg")
print(f"Heading last 3s:   {h_end:.1f} deg")
if h_start is not None and h_end is not None:
    drift = h_end - h_start
    while drift > 180: drift -= 360
    while drift < -180: drift += 360
    print(f"Heading drift:     {drift:+.1f} deg over 20s")
    if abs(drift) > 30:
        print("  -> CIRCLING (heading changes a lot)")
    elif abs(drift) > 10:
        print("  -> drifting (some heading change)")
    else:
        print("  -> walking straight (heading stable)")

if em < 0.4:
    print(f"\nEM={em:.2f}: very curved path (cycling/looping)")
elif em < 0.7:
    print(f"\nEM={em:.2f}: somewhat curved path (drifting)")
else:
    print(f"\nEM={em:.2f}: relatively straight path")
