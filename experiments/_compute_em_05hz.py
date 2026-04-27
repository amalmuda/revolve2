"""Compute Effective Movement for all 0.5 Hz Bonardi runs."""
import math, os, sys, sqlite3, glob, statistics
import mujoco, numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
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

from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
    get_robot_core_body_id,
)
from bonardi_brain import BrainBonardi, bonardi_structure_from_cpg_structure


def get_struct(robot_name, coupling, evolve_phi0=False):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "neighbor":
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    return body, bonardi_structure_from_cpg_structure(cpg, evolve_phi0=evolve_phi0), mp


def simulate_em(params, robot, coupling, evolve_phi0, sim_time=30.0, nu_hz=0.5, w=1.0):
    body, ks, mapping = get_struct(robot, coupling, evolve_phi0=evolve_phi0)
    brain = BrainBonardi.from_params(
        params=params, network_structure=ks,
        output_mapping=mapping, nu_hz=nu_hz, w=w,
    )
    robot_obj = ModularRobot(body=body, brain=brain)
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
    scene.add_robot(robot_obj)
    sim_scene, _ = scene.to_simulation_scene()

    batch = make_standard_batch_parameters()
    batch.simulation_time = sim_time
    model, mj_mapping = scene_to_model(
        sim_scene, simulation_timestep=batch.simulation_timestep,
        cast_shadows=False, fast_sim=True,
    )
    data = mujoco.MjData(model)

    core = get_robot_core_body_id(model)
    if core is None:
        return 0.0, 0.0, 0.0

    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
    cstep = 1.0 / batch.control_frequency
    last_ctrl = 0.0
    mujoco.mj_forward(model, data)
    init_pos = data.xpos[core].copy()
    prev_x, prev_y = init_pos[0], init_pos[1]
    path = 0.0

    while data.time < sim_time:
        if data.time >= last_ctrl + cstep:
            last_ctrl = math.floor(data.time / cstep) * cstep
            ss = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
            sim_scene.handler.handle(ss, ctrl, cstep)
            cx, cy = data.xpos[core][0], data.xpos[core][1]
            path += math.sqrt((cx - prev_x)**2 + (cy - prev_y)**2)
            prev_x, prev_y = cx, cy
        mujoco.mj_step(model, data)

    fp = data.xpos[core].copy()
    disp = math.sqrt((fp[0] - init_pos[0])**2 + (fp[1] - init_pos[1])**2)
    em = disp / path if path > 1e-6 else 0.0
    return disp, path, em


def best_params(db_path):
    con = sqlite3.connect(db_path)
    row = con.execute(
        "SELECT g.serialized_parameters FROM comparison_individual i "
        "JOIN comparison_genotype g ON g.id=i.genotype_id "
        "ORDER BY fitness DESC LIMIT 1"
    ).fetchone()
    con.close()
    return np.array([float(x) for x in row[0].split(";")])


for robot in ["spider", "gecko", "babya"]:
    print("\n=== %s ===" % robot)
    print("%-22s %3s %6s %6s %6s" % ("condition", "n", "disp", "path", "EM"))
    print("-" * 50)
    base = "results/%s_bonardi_nu0.5_w1" % robot
    configs = [
        ("base uncoupled", "base", "uncoupled", False),
        ("phi  uncoupled", "phi",  "uncoupled", True),
        ("base neighbor",  "base", "neighbor",  False),
        ("base blf",       "base", "blf",       False),
    ]
    for label, variant, coupling, evolve_phi in configs:
        for lam in [0, 1]:
            d = "%s/%s_bonardi_%s_%s_lambda%d_nu0.5_w1.0" % (base, robot, variant, coupling, lam)
            dbs = sorted(glob.glob(d + "/run_*.sqlite"))
            ds, ps, ems = [], [], []
            for db in dbs:
                try:
                    p = best_params(db)
                    disp, path, em = simulate_em(p, robot, coupling, evolve_phi)
                    ds.append(disp); ps.append(path); ems.append(em)
                except Exception as e:
                    print("ERR %s: %s" % (db, e))
            if ems:
                tag = "%s l=%d" % (label, lam)
                print("%-22s %3d %6.2f %6.2f %6.3f" % (
                    tag, len(ems),
                    statistics.mean(ds), statistics.mean(ps), statistics.mean(ems)
                ))
