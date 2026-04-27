"""Compute Effective Movement (EM) for gecko base BLF vs uncoupled.

EM = straight-line displacement / total path length traveled.
Re-simulates the best params from each run for 30s, tracking core position.
"""
import math
import os
import sys
import sqlite3
import glob
import statistics

import mujoco
import numpy as np

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
from bonardi_brain import (
    BrainBonardi,
    bonardi_structure_from_cpg_structure,
)


def get_struct(coupling):
    body = modular_robots_v1.get("gecko")
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "neighbor":
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    return body, bonardi_structure_from_cpg_structure(cpg), mp


def simulate_em(params, coupling, sim_time=30.0, nu_hz=1.0, w=1.0):
    body, ks, mapping = get_struct(coupling)
    brain = BrainBonardi.from_params(
        params=params, network_structure=ks,
        output_mapping=mapping, nu_hz=nu_hz, w=w,
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
    path_length = 0.0

    while data.time < sim_time:
        if data.time >= last_ctrl + cstep:
            last_ctrl = math.floor(data.time / cstep) * cstep
            ss = SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={},
            )
            sim_scene.handler.handle(ss, ctrl, cstep)
            cur_x, cur_y = data.xpos[core][0], data.xpos[core][1]
            path_length += math.sqrt((cur_x - prev_x)**2 + (cur_y - prev_y)**2)
            prev_x, prev_y = cur_x, cur_y
        mujoco.mj_step(model, data)

    fp = data.xpos[core].copy()
    dx, dy = fp[0] - init_pos[0], fp[1] - init_pos[1]
    displacement = math.sqrt(dx*dx + dy*dy)
    em = displacement / path_length if path_length > 1e-6 else 0.0
    return displacement, path_length, em


def best_params_from_db(db_path):
    con = sqlite3.connect(db_path)
    row = con.execute(
        "SELECT g.serialized_parameters FROM comparison_individual i "
        "JOIN comparison_genotype g ON g.id=i.genotype_id "
        "ORDER BY fitness DESC LIMIT 1"
    ).fetchone()
    con.close()
    return np.array([float(x) for x in row[0].split(";")])


print("%-22s %3s %10s %10s %10s" % ("condition", "n", "displ(m)", "path(m)", "EM"))
print("-" * 60)

configs = [
    ("base uncoupled  l=0", "uncoupled", "results/gecko_bonardi_nu1_w1/gecko_bonardi_base_uncoupled_lambda0_nu1.0_w1.0"),
    ("base uncoupled  l=2", "uncoupled", "results/gecko_bonardi_nu1_w1/gecko_bonardi_base_uncoupled_lambda2_nu1.0_w1.0"),
    ("base blf        l=0", "blf",       "results/gecko_bonardi_nu1_w1/gecko_bonardi_base_blf_lambda0_nu1.0_w1.0"),
    ("base blf        l=2", "blf",       "results/gecko_bonardi_nu1_w1/gecko_bonardi_base_blf_lambda2_nu1.0_w1.0"),
]

for label, coupling, d in configs:
    dbs = sorted(glob.glob(d + "/run_*.sqlite"))
    ds, ps, ems = [], [], []
    for db in dbs:
        try:
            p = best_params_from_db(db)
            disp, path, em = simulate_em(p, coupling)
            ds.append(disp); ps.append(path); ems.append(em)
        except Exception as e:
            print("ERR %s: %s" % (db, e))
    if ems:
        print("%-22s %3d %5.2f+-%.2f %5.2f+-%.2f %5.3f+-%.3f" % (
            label, len(ems),
            statistics.mean(ds), statistics.stdev(ds) if len(ds)>1 else 0,
            statistics.mean(ps), statistics.stdev(ps) if len(ps)>1 else 0,
            statistics.mean(ems), statistics.stdev(ems) if len(ems)>1 else 0,
        ))
