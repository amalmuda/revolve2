"""Re-simulate an evolved ODE-CPG run with all weights scaled by alpha.
Scaling W by alpha slows the oscillator (frequency *= alpha) and weakens
coupling proportionally. Reports distance + dragging at each alpha.
"""
import math, sys, numpy as np, mujoco
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
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
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    identify_geometry_types, get_contacts_with_ground, get_robot_core_body_id,
)


def run(robot_name, params, alpha, sim_time=30.0):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    scaled = params * alpha   # uniform scaling of W
    brain = BrainCpgNetworkStatic.uniform_from_params(
        params=scaled, cpg_network_structure=cpg,
        initial_state_uniform=math.sqrt(2)*0.5, output_mapping=mp,
    )
    robot = ModularRobot(body=body, brain=brain)
    terrain = Terrain(static_geometry=[GeometryPlane(
        pose=Pose(), mass=0.0, size=Vector2([20.0,20.0]),
        texture=Texture(
            base_color=Color(200,200,200,255),
            primary_color=Color(220,220,220,255),
            secondary_color=Color(80,80,80,255),
            map_type=MapType.MAP2D,
            reference=TextureReference(builtin='checker'), repeat=(50,50),
        ),
    )], friction=1.0)
    scene = ModularRobotScene(terrain=terrain); scene.add_robot(robot)
    sim_scene, _ = scene.to_simulation_scene()
    batch = make_standard_batch_parameters(); batch.simulation_time = sim_time
    model, mjmap = scene_to_model(sim_scene, simulation_timestep=batch.simulation_timestep,
                                  cast_shadows=False, fast_sim=True)
    data = mujoco.MjData(model)
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mjmap)
    core = get_robot_core_body_id(model)
    g_ids, r_ids, f_ids = identify_geometry_types(model)
    non_foot = r_ids - f_ids
    cstep = 1.0/batch.control_frequency; last = 0.0
    mujoco.mj_forward(model, data); init = data.xpos[core].copy()
    total = drag = 0
    while data.time < sim_time:
        total += 1
        if any(rg in non_foot for rg, *_ in get_contacts_with_ground(model, data, g_ids, r_ids)):
            drag += 1
        if data.time >= last + cstep:
            last = math.floor(data.time/cstep)*cstep
            ss = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mjmap, camera_views={})
            sim_scene.handler.handle(ss, ctrl, cstep)
        mujoco.mj_step(model, data)
    fp = data.xpos[core].copy()
    dist = math.sqrt((fp[0]-init[0])**2 + (fp[1]-init[1])**2)
    return dist, drag/total if total else 0


if __name__ == "__main__":
    robot = sys.argv[1] if len(sys.argv) > 1 else "babyb"
    params_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/babyb_blf_l0_run9.npy"
    params = np.load(params_path)
    print(f"Robot: {robot}  params: {params_path}")
    print(f"{'alpha':<8}{'distance':<12}{'drag%':<10}")
    print("-"*30)
    for a in [2.0, 1.5, 1.25, 1.0, 0.75, 0.5, 0.35, 0.25]:
        d, dr = run(robot, params, a)
        print(f"{a:<8.2f}{d:<12.3f}{dr*100:<10.1f}")
