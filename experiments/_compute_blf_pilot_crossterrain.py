"""Cross-terrain post-hoc evaluation.

For each evolved controller (revolve2 + bonardi pilots), simulate on BOTH
flat and rugged terrains and emit a row per (controller, robot, fitness,
train_terrain, eval_terrain, seed). 480 controllers × 2 eval terrains = 960 rows.

Output: ~/blf_pilot_crossterrain.csv
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
from bonardi_brain import (
    BrainBonardi,
    bonardi_structure_from_cpg_structure,
)


REVOLVE2_BASE = os.path.expanduser("~/revolve2/experiments/results/revolve2_blf_pilot")
BONARDI_BASE  = os.path.expanduser("~/revolve2/experiments/results/bonardi_blf_pilot")
ROBOTS = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
FITS = ["f1", "f2", "f3", "f4"]
TERRAINS = ["flat", "rugged", "rugged_hard"]
SIM_TIME = 30.0
INITIAL_STATE = 0.5
NU_HZ = 0.5
W = 1.0


def get_struct(robot, controller):
    body = modular_robots_v1.get(robot)
    hinges = body.find_modules_of_type(ActiveHinge)
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


def simulate(params, robot, eval_terrain, controller):
    body, struct, mapping = get_struct(robot, controller)
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
    terr = build_terrain(eval_terrain)
    scene = ModularRobotScene(terrain=terr)
    if eval_terrain in ("rugged", "rugged_hard"):
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
    r".*/(?P<robot>[^/]+)_(?P<ctrl>revolve2|bonardi)_blf_(?P<fit>f[1-4])_(?P<train>flat|rugged|rugged_hard)/"
    r"best_params_run_(?P<seed>\d+)\.npy"
)


def task(arg):
    npy_path, eval_terrain = arg
    m = PAT.match(npy_path.replace("\\", "/"))
    if not m:
        return None
    robot = m.group("robot")
    controller = m.group("ctrl")
    fit = m.group("fit")
    train_terrain = m.group("train")
    seed = int(m.group("seed"))
    try:
        params = np.load(npy_path)
        r = simulate(params, robot, eval_terrain, controller)
        if r is None:
            return None
        disp, em, drag_pct = r
        return (controller, robot, fit, train_terrain, eval_terrain, seed, disp, em, drag_pct)
    except Exception as e:
        print(f"  ERROR {npy_path} eval_on={eval_terrain}: {e}", flush=True)
        return None


if __name__ == "__main__":
    npys = []
    for r in ROBOTS:
        for f in FITS:
            for t in TERRAINS:
                pat_r = os.path.join(REVOLVE2_BASE, f"{r}_revolve2_blf_{f}_{t}", "best_params_run_*.npy")
                pat_b = os.path.join(BONARDI_BASE,  f"{r}_bonardi_blf_{f}_{t}",  "best_params_run_*.npy")
                npys.extend(sorted(glob.glob(pat_r)))
                npys.extend(sorted(glob.glob(pat_b)))

    # Each controller × 2 eval terrains
    jobs = [(p, t) for p in npys for t in TERRAINS]
    print(f"Controllers: {len(npys)}, total simulations: {len(jobs)}", flush=True)

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
            if done % 50 == 0 or done == len(jobs):
                print(f"  {done}/{len(jobs)}", flush=True)

    out = os.path.expanduser("~/blf_pilot_crossterrain.csv")
    with open(out, "w") as f:
        f.write("controller,robot,fitness,train_terrain,eval_terrain,seed,distance,em,drag_pct\n")
        for r in sorted(rows):
            f.write(",".join(str(x) for x in r) + "\n")
    print(f"wrote {out} ({len(rows)} rows)", flush=True)
