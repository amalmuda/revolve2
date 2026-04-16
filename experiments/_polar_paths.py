"""Check straightness of all 4 polar spider experiments."""
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
from hopf_brain import BrainHopfPolarStatic, hopf_structure_from_cpg_structure
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf, get_robot_core_body_id,
)

SIM_TIME = 20.0


def evaluate_path(robot_name, params_path, coupling, hz=1.0):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "blf":
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    hopf_struct = hopf_structure_from_cpg_structure(cpg)
    params = np.load(params_path)
    brain = BrainHopfPolarStatic.from_params(
        params=params, network_structure=hopf_struct, output_mapping=mp,
        omega=2 * math.pi * hz,
    )
    robot = ModularRobot(body=body, brain=brain)
    terrain = Terrain(static_geometry=[GeometryPlane(pose=Pose(), mass=0.0, size=Vector2([20.0,20.0]),
        texture=Texture(base_color=Color(200,200,200,255),primary_color=Color(220,220,220,255),
        secondary_color=Color(80,80,80,255),map_type=MapType.MAP2D,
        reference=TextureReference(builtin="checker"), repeat=(50,50)))], friction=1.0)
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)
    sim_scene, _ = scene.to_simulation_scene()
    batch = make_standard_batch_parameters()
    batch.simulation_time = SIM_TIME
    model, mj_mapping = scene_to_model(sim_scene, simulation_timestep=batch.simulation_timestep,
                                        cast_shadows=False, fast_sim=True)
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
            positions.append((data.time, float(data.xpos[core][0]), float(data.xpos[core][1])))
        if data.time >= last_ctrl + control_step:
            last_ctrl = math.floor(data.time/control_step)*control_step
            sim_state = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
            sim_scene.handler.handle(sim_state, ctrl, control_step)
        mujoco.mj_step(model, data)

    positions = np.array(positions)
    ts, xs, ys = positions[:,0], positions[:,1], positions[:,2]
    dx = xs[-1] - xs[0]
    dy = ys[-1] - ys[0]
    disp = math.sqrt(dx*dx + dy*dy)
    path_lens = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    em = disp / float(np.sum(path_lens)) if np.sum(path_lens) > 0 else 0.0

    def heading(start_t, end_t):
        m = (ts >= start_t) & (ts <= end_t)
        if m.sum() < 2:
            return None
        sx = xs[m]; sy = ys[m]
        return math.degrees(math.atan2(sy[-1] - sy[0], sx[-1] - sx[0]))

    h0 = heading(0, 3)
    h1 = heading(17, 20)
    drift = h1 - h0
    while drift > 180: drift -= 360
    while drift < -180: drift += 360
    return disp, em, drift


configs = [
    ("polar_spider_neighbor_xy_best.npy", "neighbor"),
    ("polar_spider_neighbor_directed_best.npy", "neighbor"),
    ("polar_spider_blf_xy_best.npy", "blf"),
    ("polar_spider_blf_directed_best.npy", "blf"),
]
print(f"{'Run':<45}{'Disp':<7}{'EM':<7}{'Heading drift'}")
print("-" * 75)
for p, c in configs:
    disp, em, drift = evaluate_path("spider", p, c)
    print(f"{p:<45}{disp:<7.2f}{em:<7.2f}{drift:+.1f} deg")
