"""Evolve spider with polar Hopf — both neighbor and BLF coupling, XY and directed fitness."""
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
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
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

from hopf_brain import BrainHopfPolarStatic, hopf_structure_from_cpg_structure
from contact_detection import (
    active_hinges_to_cpg_network_structure_blf,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)

# Config via env or args
ROBOT = os.environ.get("ROBOT", "spider")
COUPLING = os.environ.get("COUPLING", "neighbor")  # "neighbor" or "blf"
DIRECTED = os.environ.get("DIRECTED", "xy") == "y"
LAMBDA_PENALTY = float(os.environ.get("LAMBDA", "0"))
SIM_TIME = 20.0
GENERATIONS = 100
POPULATION = 10
NUM_WORKERS = 4
HZ = float(os.environ.get("HZ", "1.0"))
OMEGA = 2 * math.pi * HZ


def build_struct(robot_name, coupling):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "blf":
        cpg, mp = active_hinges_to_cpg_network_structure_blf(hinges, body)
    else:
        cpg, mp = active_hinges_to_cpg_network_structure_neighbor(hinges)
    return body, hopf_structure_from_cpg_structure(cpg), mp


def simulate_one(params, robot_name, coupling, sim_time, directed, lambda_penalty=0.0):
    try:
        body, hopf_struct, mapping = build_struct(robot_name, coupling)
        brain = BrainHopfPolarStatic.from_params(
            params=params, network_structure=hopf_struct, output_mapping=mapping,
            omega=OMEGA, alpha=1.0,
        )
        robot = ModularRobot(body=body, brain=brain)
        terrain = Terrain(static_geometry=[GeometryPlane(pose=Pose(), mass=0.0, size=Vector2([20.0,20.0]),
            texture=Texture(base_color=Color(200,200,200,255),primary_color=Color(220,220,220,255),
            secondary_color=Color(80,80,80,255),map_type=MapType.MAP2D,
            reference=TextureReference(builtin="checker"), repeat=(50,50)))], friction=1.0)
        scene = ModularRobotScene(terrain=terrain)
        scene.add_robot(robot)
        sim_scene, _ = scene.to_simulation_scene()
        batch = make_standard_batch_parameters()
        batch.simulation_time = sim_time
        model, mj_mapping = scene_to_model(sim_scene, simulation_timestep=batch.simulation_timestep,
                                            cast_shadows=False, fast_sim=True)
        data = mujoco.MjData(model)
        ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
        core_id = get_robot_core_body_id(model)
        if core_id is None:
            return 0.0, 0.0, 1.0
        ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
        non_foot = robot_ids - foot_ids
        mujoco.mj_forward(model, data)
        initial_pos = data.xpos[core_id].copy()
        control_step = 1.0 / batch.control_frequency
        last_ctrl = 0.0
        total = 0
        drag = 0
        while data.time < sim_time:
            total += 1
            contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
            if any(rg in non_foot for rg, *_ in contacts):
                drag += 1
            if data.time >= last_ctrl + control_step:
                last_ctrl = math.floor(data.time/control_step)*control_step
                sim_state = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={})
                sim_scene.handler.handle(sim_state, ctrl, control_step)
            mujoco.mj_step(model, data)
        final_pos = data.xpos[core_id].copy()
        dx = final_pos[0] - initial_pos[0]
        dy = final_pos[1] - initial_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        dragging = drag / total if total > 0 else 0.0
        primary = max(0.0, dy) if directed else distance
        if lambda_penalty == 0:
            fitness = primary
        else:
            fitness = primary * math.pow(max(0.0, 1.0 - dragging), lambda_penalty)
        return fitness, distance, dragging
    except Exception as e:
        print(f"  ERROR: {e}")
        return 0.0, 0.0, 1.0


def _eval_wrapper(args):
    idx, params, robot, coupling, sim_time, directed, lam = args
    f, d, dr = simulate_one(params, robot, coupling, sim_time, directed, lam)
    return idx, f, d, dr


def main():
    body, hopf_struct, mapping = build_struct(ROBOT, COUPLING)
    n = hopf_struct.num_oscillators
    nc = hopf_struct.num_connections
    n_params = n + 2 * nc
    direction_tag = "directed" if DIRECTED else "xy"
    lam_tag = f"lam{int(LAMBDA_PENALTY)}"
    hz_tag = f"{HZ:.2f}hz".replace(".", "p")
    print(f"Polar Hopf {ROBOT} + {COUPLING} + {direction_tag} + {lam_tag} + {hz_tag}")
    print(f"  n={n} osc, nc={nc} pairs, total params = {n_params}")

    # Bounds: mu [0,1], w [-1,1], phi [-pi,pi]
    lower = [0.0] * n + [-1.0] * nc + [-math.pi] * nc
    upper = [1.0] * n + [1.0] * nc + [math.pi] * nc
    initial_mean = [0.5] * n + [0.0] * nc + [0.0] * nc
    initial_sigma = 0.3

    opts = cma.CMAOptions()
    opts.set("popsize", POPULATION)
    opts.set("bounds", [lower, upper])
    opts.set("seed", 42)
    opts.set("verbose", -9)
    opt = cma.CMAEvolutionStrategy(initial_mean, initial_sigma, opts)

    best_fit = float("-inf")
    best_params = None
    t0 = time.time()
    for gen in range(GENERATIONS):
        solutions = opt.ask()
        args_list = [(i, np.asarray(s), ROBOT, COUPLING, SIM_TIME, DIRECTED, LAMBDA_PENALTY)
                     for i, s in enumerate(solutions)]
        results = [None] * len(solutions)
        with ProcessPoolExecutor(max_workers=NUM_WORKERS) as ex:
            futures = [ex.submit(_eval_wrapper, a) for a in args_list]
            for fut in as_completed(futures):
                idx, f, d, dr = fut.result()
                results[idx] = (f, d, dr)
        fits = [r[0] for r in results]
        dists = [r[1] for r in results]
        drags = [r[2] for r in results]
        opt.tell(solutions, [-f for f in fits])
        bi = int(np.argmax(fits))
        if fits[bi] > best_fit:
            best_fit = fits[bi]
            best_params = np.asarray(solutions[bi])
        print(f"Gen {gen+1:3d}/{GENERATIONS} | best={best_fit:.3f} | gen_max={np.max(fits):.3f} | drag_mean={np.mean(drags):.2f}")

    elapsed = time.time() - t0
    print(f"\nDone in {elapsed:.1f}s. Best fitness: {best_fit:.3f}")

    out = f"polar_{ROBOT}_{COUPLING}_{direction_tag}_{lam_tag}_{hz_tag}_alpha1_best.npy"
    np.save(out, best_params)
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
