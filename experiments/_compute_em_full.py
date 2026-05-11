"""Compute distance, path, EM for all 360 best controllers in /tmp/fox_results.

Reads dragging from each run's sqlite. Outputs CSV at /tmp/em_full.csv.
"""
import os, sys, glob, math, sqlite3, re
from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np
import mujoco

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


BASE = os.path.expanduser("~/all_runs")
ROBOTS = ["spider", "gecko", "babya", "queen", "insect", "ege2"]


def get_struct(robot, coupling, evolve_phi0):
    body = modular_robots_v1.get(robot)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "neighbor":
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    return body, bonardi_structure_from_cpg_structure(cpg, evolve_phi0=evolve_phi0), mp


def simulate_em(params, robot, coupling, evolve_phi0, sim_time=30.0, nu_hz=0.5, w=1.0):
    body, ks, mapping = get_struct(robot, coupling, evolve_phi0)
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
            path += math.sqrt((cx - prev_x) ** 2 + (cy - prev_y) ** 2)
            prev_x, prev_y = cx, cy
        mujoco.mj_step(model, data)
    fp = data.xpos[core].copy()
    disp = math.sqrt((fp[0] - init_pos[0]) ** 2 + (fp[1] - init_pos[1]) ** 2)
    em = disp / path if path > 1e-6 else 0.0
    return float(disp), float(path), float(em)


def get_drag(sqlite_path):
    try:
        con = sqlite3.connect(sqlite_path)
        r = con.execute(
            "SELECT dragging FROM comparison_individual ORDER BY fitness DESC LIMIT 1"
        ).fetchone()
        con.close()
        return float(r[0]) if r else None
    except Exception:
        return None


PAT = re.compile(
    r".*/(?P<robot>[^/]+)/(?P=robot)_bonardi_(?P<variant>base|phi)_(?P<coupling>uncoupled|neighbor|blf)_lambda(?P<lam>\d+)_nu0\.5_w1\.0/best_params_run_(?P<seed>\d+)\.npy"
)


def task(npy_path):
    m = PAT.match(npy_path)
    if not m:
        return None
    robot = m.group("robot")
    variant = m.group("variant")
    coupling = m.group("coupling")
    lam = int(m.group("lam"))
    seed = int(m.group("seed"))
    evolve_phi0 = (variant == "phi")
    sqlite_path = npy_path.replace("best_params_run_", "run_").replace(".npy", ".sqlite")
    try:
        params = np.load(npy_path)
        disp, path, em = simulate_em(params, robot, coupling, evolve_phi0)
        drag = get_drag(sqlite_path)
        return (robot, variant, coupling, lam, seed, disp, path, em, drag)
    except Exception as e:
        return (robot, variant, coupling, lam, seed, None, None, None, None, str(e))


if __name__ == "__main__":
    files = []
    for r in ROBOTS:
        files.extend(sorted(glob.glob(os.path.join(BASE, r, "*", "best_params_run_*.npy"))))
    print(f"Processing {len(files)} controllers...")
    rows = []
    workers = max(1, os.cpu_count() - 1)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        futures = {ex.submit(task, f): f for f in files}
        done = 0
        for fut in as_completed(futures):
            r = fut.result()
            if r is not None:
                rows.append(r)
            done += 1
            if done % 20 == 0:
                print(f"  {done}/{len(files)}")
    out = os.path.expanduser("~/em_all.csv")
    with open(out, "w") as f:
        f.write("robot,variant,coupling,lambda,seed,distance,path,em,dragging\n")
        for r in sorted(rows):
            if len(r) == 9:
                f.write(",".join(str(x) for x in r) + "\n")
    print(f"wrote {out}")
