"""Pilot: BLF coupling on Revolve2's standard BrainCpgNetworkStatic.
Compare 4 fitness functions on flat terrain.

Fitness options (all evolved on flat terrain):
    f1 = distance
    f2 = distance * (1 - drag)
    f3 = distance * em
    f4 = distance * (1 - drag) * em

drag and em are measured during the simulation. drag detection only runs
when the active fitness needs it (f2, f4). em is just from position tracking,
always on.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass

import cma
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pyrr import Vector3

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
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


SIM_TIME = 30.0
INITIAL_STATE = 0.5

FITNESS_NEEDS_DRAG = {"f1": False, "f2": True,  "f3": False, "f4": True,
                      "f2sq": True, "f3sq": False, "f4sq": True,
                      "f2cu": True, "f3cu": False, "f4cu": True}
FITNESS_NEEDS_EM   = {"f1": False, "f2": False, "f3": True,  "f4": True,
                      "f2sq": False, "f3sq": True,  "f4sq": True,
                      "f2cu": False, "f3cu": True,  "f4cu": True}


@dataclass
class EvalResult:
    fitness: float
    distance: float
    drag: float
    em: float


def get_struct(robot_name: str, coupling: str = "blf"):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "neighbor":
        cpg, mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
        return body, cpg, mapping
    cpg, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)
    return body, cpg, mapping


def build_flat_terrain():
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


def build_rugged_terrain():
    heights = rugged_heightmap(size=(20.0, 20.0), num_edges=(500, 500), density=1.5)
    return Terrain(static_geometry=[GeometryHeightmap(
        pose=Pose(), mass=0.0,
        size=Vector3([20.0, 20.0, 0.05]),
        base_thickness=0.2,
        heights=heights,
    )], friction=1.0)


def build_rugged_hard_terrain():
    heights = rugged_heightmap(size=(20.0, 20.0), num_edges=(500, 500), density=2.5)
    return Terrain(static_geometry=[GeometryHeightmap(
        pose=Pose(), mass=0.0,
        size=Vector3([20.0, 20.0, 0.10]),
        base_thickness=0.2,
        heights=heights,
    )], friction=1.0)


def build_terrain(name: str):
    if name == "rugged":
        return build_rugged_terrain()
    if name == "rugged_hard":
        return build_rugged_hard_terrain()
    return build_flat_terrain()


def compute_fitness(fitness_name: str, distance: float, drag: float, em: float) -> float:
    if fitness_name == "f1":
        return distance
    if fitness_name == "f2":
        return distance * max(0.0, 1.0 - drag)
    if fitness_name == "f3":
        return distance * em
    if fitness_name == "f4":
        return distance * max(0.0, 1.0 - drag) * em
    if fitness_name == "f2sq":
        return distance * max(0.0, 1.0 - drag) ** 2
    if fitness_name == "f3sq":
        return distance * em ** 2
    if fitness_name == "f4sq":
        return distance * max(0.0, 1.0 - drag) ** 2 * em ** 2
    if fitness_name == "f2cu":
        return distance * max(0.0, 1.0 - drag) ** 3
    if fitness_name == "f3cu":
        return distance * em ** 3
    if fitness_name == "f4cu":
        return distance * max(0.0, 1.0 - drag) ** 3 * em ** 3
    raise ValueError(f"Unknown fitness: {fitness_name}")


def evaluate(
    norm_params: np.ndarray,
    robot_name: str,
    fitness_name: str,
    lower: np.ndarray,
    upper: np.ndarray,
    terrain_name: str = "flat",
    coupling: str = "blf",
) -> EvalResult:
    try:
        native_params = lower + (norm_params + 1.0) / 2.0 * (upper - lower)

        body, cpg, mapping = get_struct(robot_name, coupling=coupling)
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=list(native_params),
            cpg_network_structure=cpg,
            initial_state_uniform=INITIAL_STATE,
            output_mapping=mapping,
        )
        robot_obj = ModularRobot(body=body, brain=brain)

        terrain = build_terrain(terrain_name)
        scene = ModularRobotScene(terrain=terrain)
        if terrain_name in ("rugged", "rugged_hard"):
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
            return EvalResult(0.0, 0.0, 1.0, 0.0)

        ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)

        need_drag = FITNESS_NEEDS_DRAG[fitness_name]
        need_em = FITNESS_NEEDS_EM[fitness_name]

        if need_drag:
            ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
            non_foot = robot_ids - foot_ids

        cstep = 1.0 / batch.control_frequency
        last_ctrl = 0.0
        mujoco.mj_forward(model, data)
        init_pos = data.xpos[core].copy()
        prev_x, prev_y = init_pos[0], init_pos[1]
        path = 0.0
        total_steps = 0
        drag_steps = 0

        while data.time < SIM_TIME:
            total_steps += 1
            if need_drag:
                contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
                if any(rg in non_foot for rg, *_ in contacts):
                    drag_steps += 1
            if data.time >= last_ctrl + cstep:
                last_ctrl = math.floor(data.time / cstep) * cstep
                ss = SimulationStateImpl(
                    data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={},
                )
                sim_scene.handler.handle(ss, ctrl, cstep)
                if need_em:
                    cx, cy = data.xpos[core][0], data.xpos[core][1]
                    path += math.sqrt((cx - prev_x) ** 2 + (cy - prev_y) ** 2)
                    prev_x, prev_y = cx, cy
            mujoco.mj_step(model, data)

        fp = data.xpos[core].copy()
        dx = fp[0] - init_pos[0]
        dy = fp[1] - init_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)
        drag = drag_steps / total_steps if (need_drag and total_steps > 0) else 0.0
        em = (distance / path) if (need_em and path > 1e-6) else 0.0

        fitness = compute_fitness(fitness_name, distance, drag, em)
        return EvalResult(fitness, distance, drag, em)
    except Exception as e:
        print(f"  ERROR in evaluate: {e}", flush=True)
        return EvalResult(0.0, 0.0, 1.0, 0.0)


def _eval_wrapper(args):
    idx, norm_params, robot, fitness_name, lower, upper, terrain_name, coupling = args
    r = evaluate(norm_params, robot, fitness_name, lower, upper, terrain_name, coupling)
    return idx, r


def run_evolution(args):
    body, cpg, mapping = get_struct(args.robot, coupling=args.coupling)
    n_params = cpg.num_connections

    if n_params == 0:
        raise RuntimeError("No connections to evolve.")

    lower_native = np.full(n_params, -args.bounds_half, dtype=float)
    upper_native = np.full(n_params, +args.bounds_half, dtype=float)

    lower_norm = [-1.0] * n_params
    upper_norm = [+1.0] * n_params
    init_mean = [0.0] * n_params
    init_sigma = 0.5

    experiment_name = f"{args.robot}_revolve2_{args.coupling}_{args.fitness}_{args.terrain}"
    results_dir = os.path.join(args.results_dir, experiment_name)
    os.makedirs(results_dir, exist_ok=True)

    print("=" * 60)
    print("Revolve2 CPG + BLF — fitness pilot")
    print("=" * 60)
    print(f"  Robot:        {args.robot}")
    print(f"  Terrain:      {args.terrain}")
    print(f"  Fitness:      {args.fitness}")
    print(f"  n connections (params): {n_params}")
    print(f"  bounds:       [-{args.bounds_half}, +{args.bounds_half}]")
    print(f"  Population:   {args.population}")
    print(f"  Generations:  {args.generations}")
    print(f"  Sim time:     {args.sim_time}s")
    print(f"  Workers:      {args.workers}")
    print(f"  Run:          {args.run_num}")
    print(f"  Seed:         {args.seed}")
    print(f"  Results dir:  {results_dir}")
    print("=" * 60)

    opts = cma.CMAOptions()
    opts.set("popsize", args.population)
    opts.set("bounds", [lower_norm, upper_norm])
    opts.set("seed", args.seed if args.seed is not None else 0)
    opts.set("verbose", -9)
    opt = cma.CMAEvolutionStrategy(init_mean, init_sigma, opts)

    conv_csv_path = os.path.join(results_dir, f"convergence_run_{args.run_num}.csv")
    conv_csv = open(conv_csv_path, "w", buffering=1)
    conv_csv.write("generation,best_ever_fitness,best_ever_distance,best_ever_drag,best_ever_em,gen_max_fitness,gen_seconds\n")

    best_fit = float("-inf")
    best_params_norm = None
    best_dist = 0.0
    best_drag = 1.0
    best_em = 0.0

    start_time = time.time()
    for gen in range(args.generations):
        gen_start = time.time()
        sols = opt.ask()
        args_list = [
            (i, np.asarray(s), args.robot, args.fitness, lower_native, upper_native, args.terrain, args.coupling)
            for i, s in enumerate(sols)
        ]
        results = [None] * len(sols)
        if args.workers <= 1:
            for a in args_list:
                idx, r = _eval_wrapper(a)
                results[idx] = r
        else:
            with ProcessPoolExecutor(max_workers=args.workers) as ex:
                futures = [ex.submit(_eval_wrapper, a) for a in args_list]
                for fut in as_completed(futures):
                    idx, r = fut.result()
                    results[idx] = r

        fits = [r.fitness for r in results]
        opt.tell(sols, [-f for f in fits])

        gen_best = int(np.argmax(fits))
        if fits[gen_best] > best_fit:
            best_fit = fits[gen_best]
            best_params_norm = np.asarray(sols[gen_best])
            best_dist = results[gen_best].distance
            best_drag = results[gen_best].drag
            best_em = results[gen_best].em

        gen_time = time.time() - gen_start

        conv_csv.write(
            f"{gen + 1},{best_fit:.6f},{best_dist:.6f},{best_drag:.6f},{best_em:.6f},"
            f"{float(np.max(fits)):.6f},{gen_time:.3f}\n"
        )

        if (gen + 1) % 10 == 0 or gen == 0 or (gen + 1) == args.generations:
            print(
                f"Gen {gen + 1:3d}/{args.generations} | "
                f"best_fit={best_fit:.3f} d={best_dist:.2f}m drag={best_drag * 100:.1f}% em={best_em:.3f} | "
                f"{gen_time:.1f}s"
            )

    conv_csv.close()

    if best_params_norm is not None:
        best_native = lower_native + (best_params_norm + 1.0) / 2.0 * (upper_native - lower_native)
        np.save(os.path.join(results_dir, f"best_params_run_{args.run_num}.npy"), best_native)

    total_time = time.time() - start_time
    print("=" * 60)
    print(f"  Best fitness: {best_fit:.4f}")
    print(f"  Best dist:    {best_dist:.4f} m")
    print(f"  Best drag:    {best_drag * 100:.2f}%")
    print(f"  Best em:      {best_em:.4f}")
    print(f"  Total time:   {total_time:.1f}s")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Revolve2 CPG + BLF — fitness pilot")
    parser.add_argument("--robot", type=str, required=True)
    parser.add_argument("--fitness", type=str, required=True,
                        choices=["f1", "f2", "f3", "f4",
                                 "f2sq", "f3sq", "f4sq",
                                 "f2cu", "f3cu", "f4cu"])
    parser.add_argument("--terrain", type=str, default="flat",
                        choices=["flat", "rugged", "rugged_hard"])
    parser.add_argument("--coupling", type=str, default="blf",
                        choices=["blf", "neighbor"])
    parser.add_argument("--bounds-half", type=float, default=1.0,
                        help="Native parameter bounds are [-bounds_half, +bounds_half]")
    parser.add_argument("--sim-time", type=float, default=30.0)
    parser.add_argument("--generations", type=int, default=300)
    parser.add_argument("--population", type=int, default=25)
    parser.add_argument("--workers", type=int, default=25)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--run-num", type=int, default=1)
    parser.add_argument("--results-dir", type=str, default="results/revolve2_blf_pilot")
    args = parser.parse_args()
    run_evolution(args)


if __name__ == "__main__":
    main()
