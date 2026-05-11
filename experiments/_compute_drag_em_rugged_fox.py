"""Post-hoc compute distance/drag/EM for rugged-evolved controllers on rugged terrain.

Runs on fox. Uses ProcessPoolExecutor for internal parallelism.
"""
import os, sys, glob, math, sqlite3, re
from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np
from pyrr import Vector3
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.standards import modular_robots_v1
from revolve2.standards.terrains import rugged_heightmap
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Pose
from revolve2.simulation.scene.geometry import GeometryHeightmap

from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)
from bonardi_brain import BrainBonardi, bonardi_structure_from_cpg_structure


BASE = os.path.expanduser("~/revolve2/experiments/results")
ROBOTS = ["spider", "gecko", "babya", "queen", "insect", "ege2"]


def get_struct(robot, coupling, evolve_phi0):
    body = modular_robots_v1.get(robot)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    return body, bonardi_structure_from_cpg_structure(cpg, evolve_phi0=evolve_phi0), mp


def simulate(params, robot, coupling, evolve_phi0, sim_time=30.0):
    body, ks, mapping = get_struct(robot, coupling, evolve_phi0)
    brain = BrainBonardi.from_params(
        params=params, network_structure=ks,
        output_mapping=mapping, nu_hz=0.5, w=1.0,
    )
    robot_obj = ModularRobot(body=body, brain=brain)

    heights = rugged_heightmap(size=(20.0, 20.0), num_edges=(500, 500), density=1.5)
    terrain = Terrain(static_geometry=[GeometryHeightmap(
        pose=Pose(), mass=0.0,
        size=Vector3([20.0, 20.0, 0.05]),
        base_thickness=0.2, heights=heights,
    )], friction=1.0)
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot_obj, pose=Pose(position=Vector3([0.0, 0.0, 0.3])))
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
        return None
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
    ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
    non_foot = robot_ids - foot_ids
    cstep = 1.0 / batch.control_frequency
    last_ctrl = 0.0
    mujoco.mj_forward(model, data)
    init_pos = data.xpos[core].copy()
    prev_x, prev_y = init_pos[0], init_pos[1]
    path = 0.0
    total = 0
    drag = 0
    while data.time < sim_time:
        total += 1
        contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
        if any(rg in non_foot for rg, *_ in contacts):
            drag += 1
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
    drag_pct = 100.0 * drag / total if total else 0.0
    return float(disp), float(path), float(em), float(drag_pct)


PAT = re.compile(
    r".*/(?P<robot>[^/]+)_bonardi_rugged_nu0\.5_w1/"
    r"(?P=robot)_bonardi_(?P<variant>base|phi)_(?P<coupling>uncoupled|blf)_"
    r"lambda(?P<lam>\d+)_nu0\.5_w1\.0/best_params_run_(?P<seed>\d+)\.npy"
)


def task(npy_path):
    m = PAT.match(npy_path)
    if not m:
        return None
    robot = m.group("robot")
    variant = m.group("variant")
    coupling = m.group("coupling")
    seed = int(m.group("seed"))
    evolve_phi0 = (variant == "phi")
    try:
        params = np.load(npy_path)
        r = simulate(params, robot, coupling, evolve_phi0)
        if r is None:
            return None
        disp, path, em, drag_pct = r
        return (robot, variant, coupling, seed, disp, path, em, drag_pct)
    except Exception as e:
        return (robot, variant, coupling, seed, None, None, None, None, str(e))


if __name__ == "__main__":
    files = []
    for r in ROBOTS:
        files.extend(sorted(glob.glob(
            os.path.join(BASE, f"{r}_bonardi_rugged_nu0.5_w1", "*", "best_params_run_*.npy")
        )))
    print(f"Processing {len(files)} controllers on rugged terrain...", flush=True)
    rows = []
    workers = max(1, int(os.environ.get("N_WORKERS", os.cpu_count() - 1)))
    print(f"Using {workers} workers", flush=True)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(task, f): f for f in files}
        done = 0
        for fut in as_completed(futs):
            r = fut.result()
            if r is not None:
                rows.append(r)
            done += 1
            if done % 20 == 0:
                print(f"  {done}/{len(files)}", flush=True)
    out = os.path.expanduser("~/em_rugged.csv")
    with open(out, "w") as f:
        f.write("robot,variant,coupling,seed,distance,path,em,dragging_pct\n")
        for r in sorted(rows):
            if len(r) == 8:
                f.write(",".join(str(x) for x in r) + "\n")
    print(f"wrote {out}")
