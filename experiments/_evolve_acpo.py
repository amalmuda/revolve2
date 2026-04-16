"""Evolve ACPO brain on a single robot/coupling with CMA-ES.

Usage:
    ROBOT=spider COUPLING=blf LAMBDA=0 HZ=1.0 python _evolve_acpo.py
"""
import math
import os
import sys
import time
import numpy as np
import mujoco
import cma
from concurrent.futures import ProcessPoolExecutor, as_completed

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    active_hinges_to_cpg_network_structure_internal_only,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
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

from acpo_brain import BrainAcpo, acpo_structure_from_cpg_structure


ROBOT = os.environ.get("ROBOT", "spider")
COUPLING = os.environ.get("COUPLING", "blf")
LAMBDA = float(os.environ.get("LAMBDA", "0"))
HZ = float(os.environ.get("HZ", "1.0"))
SIM_TIME = float(os.environ.get("SIM_TIME", "20.0"))
GENERATIONS = int(os.environ.get("GENERATIONS", "100"))
POPULATION = int(os.environ.get("POPULATION", "10"))
NUM_WORKERS = int(os.environ.get("WORKERS", "4"))
SEED = int(os.environ.get("SEED", "42"))
OMEGA = 2 * math.pi * HZ

# Hinge range = 1.047 rad (60 deg). Bound A_i to [0, range].
HINGE_RANGE = 1.047197551
A_BOUND = HINGE_RANGE
X_BOUND = 0.5     # offset bound (radians); conservative
W_BOUND = 1.0     # coupling weight bound
PSI_BOUND = math.pi


def get_struct(robot_name, coupling):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg, mp = active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "blf":
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    return body, acpo_structure_from_cpg_structure(cpg), mp


def simulate_one(params, robot_name, coupling, sim_time, lam):
    try:
        body, acpo_s, mapping = get_struct(robot_name, coupling)
        brain = BrainAcpo.from_params(
            params=params, network_structure=acpo_s,
            output_mapping=mapping, omega=OMEGA,
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
        ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
        core = get_robot_core_body_id(model)
        if core is None:
            return 0.0, 0.0, 1.0
        ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
        non_foot = robot_ids - foot_ids
        cstep = 1.0 / batch.control_frequency
        last_ctrl = 0.0
        mujoco.mj_forward(model, data)
        init_pos = data.xpos[core].copy()
        total = drag = 0
        while data.time < sim_time:
            total += 1
            if any(rg in non_foot for rg, *_ in get_contacts_with_ground(model, data, ground_ids, robot_ids)):
                drag += 1
            if data.time >= last_ctrl + cstep:
                last_ctrl = math.floor(data.time / cstep) * cstep
                ss = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
                sim_scene.handler.handle(ss, ctrl, cstep)
            mujoco.mj_step(model, data)
        fp = data.xpos[core].copy()
        dx = fp[0] - init_pos[0]
        dy = fp[1] - init_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)
        dragging = drag / total if total > 0 else 0
        if lam == 0:
            fitness = distance
        else:
            fitness = distance * math.pow(max(0.0, 1.0 - dragging), lam)
        return fitness, distance, dragging
    except Exception as e:
        print(f"  ERROR: {e}")
        return 0.0, 0.0, 1.0


def _eval(args):
    idx, params, robot, coup, sim_time, lam = args
    f, d, dr = simulate_one(params, robot, coup, sim_time, lam)
    return idx, f, d, dr


def main():
    body, acpo_s, mapping = get_struct(ROBOT, COUPLING)
    n = acpo_s.num_oscillators
    nc = acpo_s.num_connections
    n_params = acpo_s.num_params
    print("=" * 60)
    print(f"ACPO evolution (Bonardi-style + evolved coupling weights)")
    print("=" * 60)
    print(f"Robot:       {ROBOT}")
    print(f"Coupling:    {COUPLING}")
    print(f"Lambda:      {LAMBDA}")
    print(f"Oscillators: {n}  Couplings: {nc}")
    print(f"Total params: {n_params} = 2*{n} (A,X) + 2*{nc} (w,psi)")
    print(f"Population:  {POPULATION}  Generations: {GENERATIONS}")
    print(f"Sim time:    {SIM_TIME}s  Seed: {SEED}")
    print(f"Omega:       {OMEGA:.3f} rad/s ({HZ} Hz)")
    print("=" * 60)

    # Bounds: [A_i] x n, [X_i] x n, [w_ij] x nc, [psi_ij] x nc
    lower = [0.0] * n + [-X_BOUND] * n + [-W_BOUND] * nc + [-PSI_BOUND] * nc
    upper = [A_BOUND] * n + [+X_BOUND] * n + [+W_BOUND] * nc + [+PSI_BOUND] * nc
    init_mean = [A_BOUND / 2] * n + [0.0] * n + [0.0] * nc + [0.0] * nc
    init_sigma = 0.5

    opts = cma.CMAOptions()
    opts.set("popsize", POPULATION)
    opts.set("bounds", [lower, upper])
    opts.set("seed", SEED)
    opts.set("verbose", -9)
    opt = cma.CMAEvolutionStrategy(init_mean, init_sigma, opts)

    best_f = float("-inf")
    best_params = None
    best_d = 0.0
    best_dr = 1.0
    t0 = time.time()
    for gen in range(GENERATIONS):
        sols = opt.ask()
        args_list = [(i, np.asarray(s), ROBOT, COUPLING, SIM_TIME, LAMBDA)
                     for i, s in enumerate(sols)]
        results = [None] * len(sols)
        with ProcessPoolExecutor(max_workers=NUM_WORKERS) as ex:
            futures = [ex.submit(_eval, a) for a in args_list]
            for fut in as_completed(futures):
                idx, f, d, dr = fut.result()
                results[idx] = (f, d, dr)
        fits = [r[0] for r in results]
        dists = [r[1] for r in results]
        drags = [r[2] for r in results]
        opt.tell(sols, [-f for f in fits])
        bi = int(np.argmax(fits))
        if fits[bi] > best_f:
            best_f = fits[bi]
            best_params = np.asarray(sols[bi])
            best_d = dists[bi]
            best_dr = drags[bi]
        print(f"Gen {gen+1:3d}/{GENERATIONS} | best={best_f:.3f} (d={best_d:.2f} drag={best_dr*100:.1f}%) | gen_max={np.max(fits):.3f} drag_mean={np.mean(drags):.2f}")

    elapsed = time.time() - t0
    print()
    print("=" * 60)
    print(f"Done in {elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"Best fitness:  {best_f:.3f}")
    print(f"Best distance: {best_d:.3f} m")
    print(f"Best dragging: {best_dr*100:.2f}%")
    print("=" * 60)

    hz_tag = f"{HZ:.2f}".replace(".", "p") + "hz"
    out = f"acpo_{ROBOT}_{COUPLING}_lam{int(LAMBDA)}_{hz_tag}_best.npy"
    np.save(out, best_params)
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
