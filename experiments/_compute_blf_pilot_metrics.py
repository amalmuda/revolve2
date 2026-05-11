"""Post-hoc distance/EM/drag measurement for both BLF pilots
(revolve2 BrainCpgNetworkStatic and Bonardi brain), flat + rugged.

Sources:
  ~/revolve2/experiments/results/revolve2_blf_pilot/
    {robot}_revolve2_blf_{fitness}_{terrain}/best_params_run_N.npy
  ~/revolve2/experiments/results/bonardi_blf_pilot/
    {robot}_bonardi_blf_{fitness}_{terrain}/best_params_run_N.npy

Output: ~/blf_pilot_metrics.csv
"""
import os, sys, math, glob, re
from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np
from pyrr import Vector3
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkStatic
from revolve2.standards import modular_robots_v1
from revolve2.standards.terrains import rugged_heightmap
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryHeightmap, GeometryPlane
from revolve2.simulation.scene.geometry.textures import (
    MapType, Texture, TextureReference,
)
from revolve2.simulation.scene.vector2 import Vector2

from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)
from bonardi_brain import (
    BrainBonardi,
    bonardi_structure_from_cpg_structure,
)


REVOLVE2_BASE = os.path.expanduser("~/revolve2/experiments/results/revolve2_blf_pilot")
BONARDI_BASE  = os.path.expanduser("~/revolve2/experiments/results/bonardi_blf_pilot")
BOUNDS2_BASE  = os.path.expanduser("~/revolve2/experiments/results/revolve2_blf_pilot_bounds2")
SQPEN_BASE    = os.path.expanduser("~/revolve2/experiments/results/revolve2_blf_pilot_sqpen")
CUPEN_BASE    = os.path.expanduser("~/revolve2/experiments/results/revolve2_blf_pilot_cupen")
NEIGHBOR_BASE = os.path.expanduser("~/revolve2/experiments/results/revolve2_neighbor_pilot")
ROBOTS = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
FITS = ["f1", "f2", "f3", "f4"]
TERRAINS = ["flat", "rugged"]
SIM_TIME = 30.0
INITIAL_STATE = 0.5
NU_HZ = 0.5
W = 1.0


def get_struct(robot, controller, coupling="blf"):
    body = modular_robots_v1.get(robot)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "neighbor":
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    if controller == "bonardi":
        ks = bonardi_structure_from_cpg_structure(
            cpg, evolve_phi0=False, evolve_w=False, evolve_X=True, evolve_nu=False,
        )
        return body, ks, mp
    return body, cpg, mp


def build_terrain(terrain):
    if terrain == "rugged":
        heights = rugged_heightmap(size=(20.0, 20.0), num_edges=(500, 500), density=1.5)
        return Terrain(static_geometry=[GeometryHeightmap(
            pose=Pose(), mass=0.0,
            size=Vector3([20.0, 20.0, 0.05]),
            base_thickness=0.2, heights=heights,
        )], friction=1.0)
    if terrain == "rugged_hard":
        heights = rugged_heightmap(size=(20.0, 20.0), num_edges=(500, 500), density=2.5)
        return Terrain(static_geometry=[GeometryHeightmap(
            pose=Pose(), mass=0.0,
            size=Vector3([20.0, 20.0, 0.10]),
            base_thickness=0.2, heights=heights,
        )], friction=1.0)
    return Terrain(static_geometry=[GeometryPlane(
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


def simulate(params, robot, terrain, controller, coupling="blf"):
    body, struct, mapping = get_struct(robot, controller, coupling=coupling)
    if controller == "bonardi":
        brain = BrainBonardi.from_params(
            params=params, network_structure=struct,
            output_mapping=mapping, nu_hz=NU_HZ, w=W,
        )
    else:
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=list(params), cpg_network_structure=struct,
            initial_state_uniform=INITIAL_STATE, output_mapping=mapping,
        )
    robot_obj = ModularRobot(body=body, brain=brain)
    terr = build_terrain(terrain)
    scene = ModularRobotScene(terrain=terr)
    if terrain in ("rugged", "rugged_hard"):
        scene.add_robot(robot_obj, pose=Pose(position=Vector3([0.0, 0.0, 0.3])))
    else:
        scene.add_robot(robot_obj)
    sim_scene, _ = scene.to_simulation_scene()

    batch = make_standard_batch_parameters()
    batch.simulation_time = SIM_TIME
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
    while data.time < SIM_TIME:
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
    return float(disp), float(em), float(drag_pct)


PAT = re.compile(
    r".*/(?P<robot>[^/]+)_(?P<ctrl>revolve2|bonardi)_(?P<coupling>blf|neighbor)_(?P<fit>f[1-4](?:sq|cu)?)_(?P<terrain>flat|rugged|rugged_hard)/"
    r"best_params_run_(?P<seed>\d+)\.npy"
)


def task(npy_path):
    m = PAT.match(npy_path.replace("\\", "/"))
    if not m:
        return None
    robot = m.group("robot")
    controller = m.group("ctrl")
    coupling = m.group("coupling")
    fit = m.group("fit")
    terrain = m.group("terrain")
    seed = int(m.group("seed"))
    bounds = "2" if "_bounds2" in npy_path.replace("\\", "/") else "1"
    try:
        params = np.load(npy_path)
        r = simulate(params, robot, terrain, controller, coupling=coupling)
        if r is None:
            return None
        disp, em, drag_pct = r
        return (controller, coupling, bounds, robot, fit, terrain, seed, disp, em, drag_pct)
    except Exception as e:
        print(f"  ERROR {npy_path}: {e}", flush=True)
        return None


if __name__ == "__main__":
    jobs = []
    ALL_TERRAINS = TERRAINS + ["rugged_hard"]
    SQ_FITS = ["f2sq", "f3sq", "f4sq"]
    CU_FITS = ["f2cu", "f3cu", "f4cu"]
    for r in ROBOTS:
        for f in FITS:
            for t in ALL_TERRAINS:
                pat_r = os.path.join(REVOLVE2_BASE, f"{r}_revolve2_blf_{f}_{t}", "best_params_run_*.npy")
                pat_b = os.path.join(BONARDI_BASE,  f"{r}_bonardi_blf_{f}_{t}",  "best_params_run_*.npy")
                pat_2 = os.path.join(BOUNDS2_BASE,  f"{r}_revolve2_blf_{f}_{t}", "best_params_run_*.npy")
                jobs.extend(sorted(glob.glob(pat_r)))
                jobs.extend(sorted(glob.glob(pat_b)))
                jobs.extend(sorted(glob.glob(pat_2)))
        for f in SQ_FITS:
            pat_sq = os.path.join(SQPEN_BASE, f"{r}_revolve2_blf_{f}_flat", "best_params_run_*.npy")
            jobs.extend(sorted(glob.glob(pat_sq)))
        for f in CU_FITS:
            pat_cu = os.path.join(CUPEN_BASE, f"{r}_revolve2_blf_{f}_flat", "best_params_run_*.npy")
            jobs.extend(sorted(glob.glob(pat_cu)))
        for f in FITS:
            for t in ALL_TERRAINS:
                pat_n = os.path.join(NEIGHBOR_BASE, f"{r}_revolve2_neighbor_{f}_{t}", "best_params_run_*.npy")
                jobs.extend(sorted(glob.glob(pat_n)))
    print(f"Total simulations: {len(jobs)}", flush=True)

    rows = []
    workers = max(1, int(os.environ.get("N_WORKERS", os.cpu_count() - 1)))
    print(f"Using {workers} workers", flush=True)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(task, j): j for j in jobs}
        done = 0
        for fut in as_completed(futs):
            r = fut.result()
            if r is not None:
                rows.append(r)
            done += 1
            if done % 25 == 0 or done == len(jobs):
                print(f"  {done}/{len(jobs)}", flush=True)

    out = os.path.expanduser("~/blf_pilot_metrics.csv")
    with open(out, "w") as f:
        f.write("controller,coupling,bounds,robot,fitness,terrain,seed,distance,em,drag_pct\n")
        for r in sorted(rows):
            f.write(",".join(str(x) for x in r) + "\n")
    print(f"wrote {out} ({len(rows)} rows)", flush=True)
