"""Local cross-terrain transfer test (30 seeds per cell, both directions).

Uses ~/all_runs (flat-trained controllers, 35 seeds available, first 30 used)
and ~/rugged_runs (rugged-trained, 30 seeds) — same source dirs as the
in-distribution metrics in em_all.csv / em_rugged.csv. So the transfer
controllers match the bundle's in-distribution baseline exactly.

  flat-evolved  -> evaluated on RUGGED  (seeds 1..30)
  rugged-evolved -> evaluated on FLAT   (seeds 1..30)

Filtered to lambda=0, phi/uncoupled and base/blf, 6 robots.
Output: ~/em_transfer_30.csv
"""
import os, sys, math, glob, re
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
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryHeightmap, GeometryPlane
from revolve2.simulation.scene.geometry.textures import (
    MapType, Texture, TextureReference,
)
from revolve2.simulation.scene.vector2 import Vector2

from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)
from bonardi_brain import BrainBonardi, bonardi_structure_from_cpg_structure


FLAT_BASE   = os.path.expanduser("~/all_runs")
RUGGED_BASE = os.path.expanduser("~/rugged_runs")
ROBOTS  = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
N_SEEDS = 30
SIM_TIME = 30.0


def get_struct(robot, coupling, evolve_phi0):
    body = modular_robots_v1.get(robot)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    return body, bonardi_structure_from_cpg_structure(cpg, evolve_phi0=evolve_phi0), mp


def build_terrain(eval_terrain):
    if eval_terrain == "rugged":
        heights = rugged_heightmap(size=(20.0, 20.0), num_edges=(500, 500), density=1.5)
        return Terrain(static_geometry=[GeometryHeightmap(
            pose=Pose(), mass=0.0,
            size=Vector3([20.0, 20.0, 0.05]),
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


def simulate(params, robot, coupling, evolve_phi0, eval_terrain):
    body, ks, mapping = get_struct(robot, coupling, evolve_phi0)
    brain = BrainBonardi.from_params(
        params=params, network_structure=ks,
        output_mapping=mapping, nu_hz=0.5, w=1.0,
    )
    robot_obj = ModularRobot(body=body, brain=brain)
    terrain = build_terrain(eval_terrain)
    scene = ModularRobotScene(terrain=terrain)
    if eval_terrain == "rugged":
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


def task(args):
    npy_path, robot, variant, coupling, train_terrain, eval_terrain, seed = args
    evolve_phi0 = (variant == "phi")
    try:
        params = np.load(npy_path)
        r = simulate(params, robot, coupling, evolve_phi0, eval_terrain)
        if r is None:
            return None
        disp, em, drag_pct = r
        return (robot, variant, coupling, train_terrain, eval_terrain,
                seed, disp, em, drag_pct)
    except Exception as e:
        print(f"  ERROR {npy_path}: {e}", flush=True)
        return None


def collect_jobs():
    jobs = []
    for r in ROBOTS:
        for variant, coupling in [("phi", "uncoupled"), ("base", "blf")]:
            # Flat-trained -> evaluate on rugged
            flat_dir = os.path.join(FLAT_BASE, r, f"{r}_bonardi_{variant}_{coupling}_lambda0_nu0.5_w1.0")
            for seed in range(1, N_SEEDS + 1):
                p = os.path.join(flat_dir, f"best_params_run_{seed}.npy")
                if os.path.exists(p):
                    jobs.append((p, r, variant, coupling, "flat", "rugged", seed))
                else:
                    print(f"  MISSING: {p}", flush=True)
            # Rugged-trained -> evaluate on flat
            rug_dir = os.path.join(RUGGED_BASE, r, f"{r}_bonardi_{variant}_{coupling}_lambda0_nu0.5_w1.0")
            for seed in range(1, N_SEEDS + 1):
                p = os.path.join(rug_dir, f"best_params_run_{seed}.npy")
                if os.path.exists(p):
                    jobs.append((p, r, variant, coupling, "rugged", "flat", seed))
                else:
                    print(f"  MISSING: {p}", flush=True)
    return jobs


if __name__ == "__main__":
    jobs = collect_jobs()
    print(f"Total transfer simulations: {len(jobs)}", flush=True)

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
            if done % 50 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    out = os.path.expanduser("~/em_transfer_30.csv")
    with open(out, "w") as f:
        f.write("robot,variant,coupling,train_terrain,eval_terrain,seed,distance,em,drag_pct\n")
        for r in sorted(rows):
            f.write(",".join(str(x) for x in r) + "\n")
    print(f"wrote {out} ({len(rows)} rows, expected {6*2*2*30}=720)", flush=True)
