"""Full frequency jitter test: 6 robots x 3 couplings x 5 seeds x 6 jitter x 5 trials."""
import os, sys, math, csv
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
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)
from bonardi_brain import (
    BrainBonardi, BrainBonardiInstance, bonardi_structure_from_cpg_structure,
)


_NU_VEC = None
def _dphi_jitter(self, phi):
    diff = phi[None, :] - phi[:, None] - self._psi
    coupling = (self._W * self._A[None, :] * np.sin(diff)).sum(axis=1)
    if _NU_VEC is not None:
        return _NU_VEC + coupling
    return self._nu_rad + coupling
BrainBonardiInstance._dphi = _dphi_jitter


def run_one(args):
    robot, coupling, evolve_phi0, params_path, jitter, trial_seed, sim_time = args
    global _NU_VEC

    body = modular_robots_v1.get(robot)
    hinges = body.find_modules_of_type(ActiveHinge)
    n = len(hinges)

    rng = np.random.RandomState(trial_seed)
    nu_hz_vec = 0.5 + jitter * rng.randn(n)
    _NU_VEC = 2.0 * np.pi * nu_hz_vec

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
    return float(disp), float(em), float(drag_pct)


def task_wrapper(args):
    key = args[:6]  # robot, coupling, phi0, seed, jitter, trial
    res = run_one(args[6])
    return key, res


ROBOTS = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
COUPLINGS = [
    ("BLF",      "blf",       False),
    ("neighbor", "neighbor",  False),
    ("phi/unc",  "uncoupled", True),
]
SEEDS = [1, 2, 3, 4, 5]
JITTERS = [0.0, 0.005, 0.01, 0.02, 0.05, 0.1]
N_TRIALS = 5
SIM_TIME = 30.0


def params_path(robot, variant, coupling, lam, seed):
    fname = f"{robot}_bonardi_{variant}_{coupling}_lambda{lam}_nu0.5_w1.0/best_params_run_{seed}.npy"
    return os.path.expanduser(f"~/fox_results/{robot}/{fname}")


jobs = []
for robot in ROBOTS:
    for label, coupling, evolve_phi0 in COUPLINGS:
        variant = "phi" if evolve_phi0 else "base"
        for seed in SEEDS:
            fp = params_path(robot, variant, coupling, 0, seed)
            if not os.path.exists(fp):
                print(f"missing: {fp}")
                continue
            for jitter in JITTERS:
                for trial in range(N_TRIALS):
                    key = (robot, label, coupling, evolve_phi0, seed, jitter, trial)
                    args = (robot, coupling, evolve_phi0, fp, jitter, trial, SIM_TIME)
                    jobs.append((key, args))

print(f"Total simulations: {len(jobs)}")

rows = []
workers = max(1, os.cpu_count() - 1)
with ProcessPoolExecutor(max_workers=workers) as ex:
    futs = {ex.submit(run_one, a): k for k, a in jobs}
    done = 0
    for fut in as_completed(futs):
        k = futs[fut]
        r = fut.result()
        if r is not None:
            rows.append((*k, *r))
        done += 1
        if done % 100 == 0:
            print(f"  {done}/{len(jobs)}")

CSV = os.path.expanduser("~/freq_jitter_full.csv")
with open(CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["robot", "label", "coupling", "evolve_phi0", "seed", "jitter", "trial",
                "distance", "em", "drag_pct"])
    for r in sorted(rows):
        w.writerow(r)
print("wrote", CSV)
