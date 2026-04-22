"""
Evolve a Bonardi-style CPG for a fixed robot body and coupling topology.

Uses CMA-ES with all parameters normalized to [-1, 1] internally, and
un-normalized to the brain's native ranges at evaluation time.

Parameters evolved (total = 2*n_hinges + n_couplings):
    A_i     in [0, pi/3]        amplitude per hinge
    X_i     in [-pi/3, pi/3]    output offset per hinge
    psi_ij  in [0, 2*pi]        phase lag per coupling link

Fixed:
    nu      natural frequency (Hz), shared
    w       coupling strength, uniform for all edges
    phi_i(0) = 0 for all hinges

Fitness: distance * (1 - dragging)^lambda
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
from sqlalchemy.orm import Session

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from evolution_comparison_database import (
    Base,
    ComparisonExperiment,
    ComparisonGeneration,
    ComparisonGenotype,
    ComparisonIndividual,
    ComparisonPopulation,
)

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
    active_hinges_to_cpg_network_structure_fully_connected,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)
from bonardi_brain import (
    BrainBonardi,
    BonardiNetworkStructure,
    bonardi_structure_from_cpg_structure,
    param_bounds,
)


@dataclass
class EvalResult:
    fitness: float
    distance: float
    dragging: float
    final_x: float = 0.0
    final_y: float = 0.0


def get_structure(
    robot_name: str, coupling: str,
    evolve_phi0: bool = False, evolve_w: bool = False,
):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg, mapping = active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "neighbor":
        cpg, mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
    elif coupling == "blf":
        cpg, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)
    elif coupling == "fully_connected":
        cpg, mapping = active_hinges_to_cpg_network_structure_fully_connected(hinges)
    else:
        raise ValueError(f"Unknown coupling: {coupling}")
    return body, bonardi_structure_from_cpg_structure(
        cpg, evolve_phi0=evolve_phi0, evolve_w=evolve_w,
    ), mapping


def unnormalize(norm_params: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return lower + (norm_params + 1.0) / 2.0 * (upper - lower)


def evaluate(
    norm_params: np.ndarray,
    robot_name: str,
    coupling: str,
    sim_time: float,
    lam: float,
    nu_hz: float,
    w: float,
    lower: np.ndarray,
    upper: np.ndarray,
    evolve_phi0: bool,
    evolve_w: bool,
) -> EvalResult:
    try:
        native_params = unnormalize(np.asarray(norm_params), lower, upper)

        body, ks, mapping = get_structure(
            robot_name, coupling, evolve_phi0=evolve_phi0, evolve_w=evolve_w,
        )
        brain = BrainBonardi.from_params(
            params=native_params,
            network_structure=ks,
            output_mapping=mapping,
            nu_hz=nu_hz,
            w=w,
        )
        robot = ModularRobot(body=body, brain=brain)

        terrain = Terrain(
            static_geometry=[
                GeometryPlane(
                    pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
                    texture=Texture(
                        base_color=Color(200, 200, 200, 255),
                        primary_color=Color(220, 220, 220, 255),
                        secondary_color=Color(80, 80, 80, 255),
                        map_type=MapType.MAP2D,
                        reference=TextureReference(builtin="checker"),
                        repeat=(50, 50),
                    ),
                )
            ],
            friction=1.0,
        )
        scene = ModularRobotScene(terrain=terrain)
        scene.add_robot(robot)
        sim_scene, _ = scene.to_simulation_scene()

        batch = make_standard_batch_parameters()
        batch.simulation_time = sim_time
        model, mj_mapping = scene_to_model(
            sim_scene,
            simulation_timestep=batch.simulation_timestep,
            cast_shadows=False,
            fast_sim=True,
        )
        data = mujoco.MjData(model)

        core = get_robot_core_body_id(model)
        if core is None:
            return EvalResult(0.0, 0.0, 1.0)

        ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)
        ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
        non_foot = robot_ids - foot_ids

        cstep = 1.0 / batch.control_frequency
        last_ctrl = 0.0
        mujoco.mj_forward(model, data)
        init_pos = data.xpos[core].copy()

        total = 0
        drag = 0
        while data.time < sim_time:
            total += 1
            contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
            if any(rg in non_foot for rg, *_ in contacts):
                drag += 1
            if data.time >= last_ctrl + cstep:
                last_ctrl = math.floor(data.time / cstep) * cstep
                ss = SimulationStateImpl(
                    data=data, abstraction_to_mujoco_mapping=mj_mapping, camera_views={}
                )
                sim_scene.handler.handle(ss, ctrl, cstep)
            mujoco.mj_step(model, data)

        fp = data.xpos[core].copy()
        dx = fp[0] - init_pos[0]
        dy = fp[1] - init_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)
        dragging = drag / total if total > 0 else 1.0

        if lam == 0:
            fitness = distance
        else:
            fitness = distance * math.pow(max(0.0, 1.0 - dragging), lam)

        return EvalResult(fitness, distance, dragging, float(fp[0]), float(fp[1]))

    except Exception as e:
        print(f"  ERROR in evaluate: {e}")
        return EvalResult(0.0, 0.0, 1.0, 0.0, 0.0)


def _eval_wrapper(args):
    (idx, norm_params, robot, coup, sim_time, lam, nu_hz, w,
     lower, upper, evolve_phi0, evolve_w) = args
    r = evaluate(norm_params, robot, coup, sim_time, lam, nu_hz, w,
                 lower, upper, evolve_phi0, evolve_w)
    return idx, r


def run_evolution(args):
    body, ks, mapping = get_structure(
        args.robot, args.coupling,
        evolve_phi0=args.evolve_phi0, evolve_w=args.evolve_w,
    )
    n_params = ks.num_params
    n_osc = ks.num_oscillators
    n_conn = ks.num_connections

    # Handle uncoupled robots with 0 oscillator parameters — impossible here
    # because A and X are evolved. But just in case:
    if n_params == 0:
        raise RuntimeError("No parameters to evolve (n_params=0).")

    lower_native, upper_native = param_bounds(ks)
    lower_native = np.asarray(lower_native, dtype=float)
    upper_native = np.asarray(upper_native, dtype=float)

    lower_norm = [-1.0] * n_params
    upper_norm = [+1.0] * n_params
    init_mean = [0.0] * n_params
    init_sigma = 0.5

    nu_hz = args.nu
    w = args.w

    variant = ""
    if args.evolve_phi0:
        variant += "_phi"
    if args.evolve_w:
        variant += "_w"
    if not variant:
        variant = "_base"
    experiment_name = (
        f"{args.robot}_bonardi{variant}_{args.coupling}_lambda{int(args.lambda_penalty)}"
        f"_nu{nu_hz}_w{w}"
    )
    results_dir = os.path.join(args.results_dir, experiment_name)
    os.makedirs(results_dir, exist_ok=True)

    print("=" * 60)
    print("BONARDI-STYLE CPG EVOLUTION")
    print("=" * 60)
    print(f"  Robot:        {args.robot}")
    print(f"  Coupling:     {args.coupling}")
    print(f"  Lambda:       {args.lambda_penalty}")
    print(f"  nu (Hz):      {nu_hz}")
    print(f"  w (coupling): {w}")
    print(f"  n oscillators:{n_osc}")
    print(f"  n couplings:  {n_conn}")
    print(f"  n params:     {n_params}  (2 * {n_osc} + {n_conn})")
    print(f"  Population:   {args.population}")
    print(f"  Generations:  {args.generations}")
    print(f"  Sim time:     {args.sim_time}s")
    print(f"  Workers:      {args.workers}")
    print(f"  Seed:         {args.seed}")
    print(f"  Run:          {args.run_num}")
    print(f"  Init sigma:   {init_sigma}")
    print(f"  Results dir:  {results_dir}")
    print("=" * 60)

    db_path = os.path.join(results_dir, f"run_{args.run_num}.sqlite")
    dbengine = open_database_sqlite(db_path, open_method=OpenMethod.OVERWITE_IF_EXISTS)
    Base.metadata.create_all(dbengine)

    experiment = ComparisonExperiment(
        robot_name=args.robot,
        controller_type="bonardi",
        coupling_mode=args.coupling,
        lambda_penalty=args.lambda_penalty,
        simulation_time=args.sim_time,
        num_generations=args.generations,
        population_size=args.population,
        param_bounds_min=-1.0,
        param_bounds_max=1.0,
        rng_seed=args.seed if args.seed is not None else 0,
        run_number=args.run_num,
        penalty_type="dragging",
        frequency=nu_hz,
        coupling_strength=w,
        num_parameters=n_params,
        num_hinges=n_osc,
    )
    with Session(dbengine) as session:
        session.add(experiment)
        session.commit()

    opts = cma.CMAOptions()
    opts.set("popsize", args.population)
    opts.set("bounds", [lower_norm, upper_norm])
    opts.set("seed", args.seed if args.seed is not None else 0)
    opts.set("verbose", -9)
    opt = cma.CMAEvolutionStrategy(init_mean, init_sigma, opts)

    best_fit = float("-inf")
    best_params_norm = None
    best_dist = 0.0
    best_drag = 1.0

    start_time = time.time()

    for gen in range(args.generations):
        gen_start = time.time()
        sols = opt.ask()

        args_list = [
            (i, np.asarray(s), args.robot, args.coupling, args.sim_time,
             args.lambda_penalty, nu_hz, w, lower_native, upper_native,
             args.evolve_phi0, args.evolve_w)
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
        dists = [r.distance for r in results]
        drags = [r.dragging for r in results]

        opt.tell(sols, [-f for f in fits])

        gen_best = int(np.argmax(fits))
        if fits[gen_best] > best_fit:
            best_fit = fits[gen_best]
            best_params_norm = np.asarray(sols[gen_best])
            best_dist = dists[gen_best]
            best_drag = drags[gen_best]

        gen_time = time.time() - gen_start

        population = ComparisonPopulation(
            individuals=[
                ComparisonIndividual(
                    genotype=ComparisonGenotype.from_parameters(
                        unnormalize(np.asarray(sol), lower_native, upper_native)
                    ),
                    population_index=i,
                    fitness=float(results[i].fitness),
                    distance=float(results[i].distance),
                    dragging=float(results[i].dragging),
                    cost_of_transport=None,
                    final_x=float(results[i].final_x),
                    final_y=float(results[i].final_y),
                    avg_core_height=0.0,
                    min_core_height=0.0,
                )
                for i, sol in enumerate(sols)
            ]
        )

        generation = ComparisonGeneration(
            experiment=experiment,
            population=population,
            generation_index=opt.countiter,
            fitness_mean=float(np.mean(fits)),
            fitness_std=float(np.std(fits)),
            fitness_max=float(np.max(fits)),
            fitness_min=float(np.min(fits)),
            distance_mean=float(np.mean(dists)),
            distance_max=float(np.max(dists)),
            dragging_mean=float(np.mean(drags)),
            cot_mean=None,
            height_mean=0.0,
            best_ever_fitness=best_fit,
            best_ever_distance=best_dist,
            time_seconds=gen_time,
        )

        with Session(dbengine, expire_on_commit=False) as session:
            session.add(generation)
            session.commit()

        if (gen + 1) % 10 == 0 or gen == 0 or (gen + 1) == args.generations:
            print(
                f"Gen {gen + 1:3d}/{args.generations} | "
                f"best={best_fit:.3f} d={best_dist:.2f}m drag={best_drag * 100:.1f}% | "
                f"gen_max={np.max(fits):.3f} drag_mean={np.mean(drags) * 100:.1f}% | "
                f"{gen_time:.1f}s"
            )

    if best_params_norm is not None:
        best_native = unnormalize(best_params_norm, lower_native, upper_native)
        np.save(os.path.join(results_dir, f"best_params_run_{args.run_num}.npy"), best_native)

    total_time = time.time() - start_time
    print("=" * 60)
    print(f"  Best fitness: {best_fit:.4f}")
    print(f"  Best dist:    {best_dist:.4f} m")
    print(f"  Best drag:    {best_drag * 100:.2f}%")
    print(f"  Total time:   {total_time:.1f}s")
    print(f"  Database:     {db_path}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Bonardi-style CPG evolution")
    parser.add_argument("--robot", type=str, required=True)
    parser.add_argument("--coupling", type=str, required=True,
                        choices=["uncoupled", "neighbor", "blf", "fully_connected"])
    parser.add_argument("--lambda", dest="lambda_penalty", type=float, required=True)
    parser.add_argument("--nu", type=float, default=1.0,
                        help="Fixed natural frequency (Hz). Default: 1.0")
    parser.add_argument("--w", type=float, default=1.0,
                        help="Fixed coupling strength. Default: 1.0")
    parser.add_argument("--sim-time", type=float, default=30.0)
    parser.add_argument("--generations", type=int, default=300)
    parser.add_argument("--population", type=int, default=25)
    parser.add_argument("--workers", type=int, default=25)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--run-num", type=int, default=1)
    parser.add_argument("--results-dir", type=str, default="results/bonardi")
    parser.add_argument("--evolve-phi0", action="store_true",
                        help="Also evolve per-oscillator initial phases.")
    parser.add_argument("--evolve-w", action="store_true",
                        help="Also evolve per-edge coupling strengths.")
    args = parser.parse_args()
    run_evolution(args)


if __name__ == "__main__":
    main()
