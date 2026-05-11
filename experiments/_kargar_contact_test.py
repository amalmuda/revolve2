"""Re-simulate spider controllers and measure both my drag % and Kargar's contact rate.

Kargar et al. (2021): c = (C / b) / t
  C = total number of contacts between robot and surface (summed over timesteps)
  b = number of body parts in the robot
  t = simulation duration
Output unit: contacts per body part per second.

My drag %:
  drag = (timesteps where any non-foot module touches ground) / total_timesteps
Output unit: fraction 0..1 (then * 100 for %).
"""
import os, sys, math, csv
from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.body.v1 import BrickV1
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


def count_body_parts(body):
    """Count modules: core + bricks + hinges. The core counts as 1."""
    n_hinges = len(body.find_modules_of_type(ActiveHinge))
    try:
        n_bricks = len(body.find_modules_of_type(BrickV1))
    except Exception:
        n_bricks = 0
    return 1 + n_hinges + n_bricks


def simulate(coupling, evolve_phi0, params_path, sim_time=30.0):
    body = modular_robots_v1.get("spider")
    hinges = body.find_modules_of_type(ActiveHinge)
    n_body_parts = count_body_parts(body)
    if coupling == "uncoupled":
        cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "neighbor":
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    ks = bonardi_structure_from_cpg_structure(cpg, evolve_phi0=evolve_phi0)
    params = np.load(params_path)
    brain = BrainBonardi.from_params(
        params=params, network_structure=ks, output_mapping=mp,
        nu_hz=0.5, w=1.0,
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
    ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
    ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
    non_foot = robot_ids - foot_ids
    cstep = 1.0 / batch.control_frequency
    last_ctrl = 0.0
    mujoco.mj_forward(model, data)

    total = 0
    drag_steps = 0           # my metric: timesteps with any non-foot contact
    contact_event_total = 0  # Kargar's C: total contact pair count summed over timesteps
    while data.time < sim_time:
        total += 1
        contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
        contact_event_total += len(contacts)
        if any(rg in non_foot for rg, *_ in contacts):
            drag_steps += 1
        if data.time >= last_ctrl + cstep:
            last_ctrl = math.floor(data.time / cstep) * cstep
            ss = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
            sim_scene.handler.handle(ss, ctrl, cstep)
        mujoco.mj_step(model, data)

    drag_pct = 100.0 * drag_steps / total if total else 0.0
    kargar_c = (contact_event_total / n_body_parts) / sim_time  # contacts per body part per second
    return drag_pct, kargar_c, n_body_parts


def task(args):
    cell, seed, fp = args
    variant, coupling = cell.split("/")
    coupling_full = {"unc": "uncoupled", "blf": "blf"}[coupling]
    evolve_phi0 = (variant == "phi")
    drag, c, b = simulate(coupling_full, evolve_phi0, fp)
    return cell, seed, drag, c, b


if __name__ == "__main__":
    base_dir = os.path.expanduser("~/all_runs/spider")
    cells = [
        ("phi/unc", "spider_bonardi_phi_uncoupled_lambda0_nu0.5_w1.0"),
        ("base/blf", "spider_bonardi_base_blf_lambda0_nu0.5_w1.0"),
        ("phi/unc-l1", "spider_bonardi_phi_uncoupled_lambda1_nu0.5_w1.0"),
        ("base/blf-l1", "spider_bonardi_base_blf_lambda1_nu0.5_w1.0"),
    ]
    jobs = []
    for cell_label, cell_dir in cells:
        for seed in range(1, 36):
            fp = os.path.join(base_dir, cell_dir, f"best_params_run_{seed}.npy")
            if os.path.exists(fp):
                jobs.append((cell_label, seed, fp))

    print(f"Running {len(jobs)} simulations...")
    rows = []
    workers = max(1, os.cpu_count() - 1)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(task, j) for j in jobs]
        done = 0
        for fut in as_completed(futs):
            r = fut.result()
            rows.append(r)
            done += 1
            if done % 20 == 0:
                print(f"  {done}/{len(jobs)}")

    out = os.path.expanduser("~/kargar_vs_drag_spider.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cell", "seed", "drag_pct", "kargar_c", "body_parts"])
        for r in sorted(rows):
            w.writerow(r)
    print(f"wrote {out}")
