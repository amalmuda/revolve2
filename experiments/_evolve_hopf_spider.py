"""
Quick local evolution of spider with a Hopf brain using CMA-ES.

Kept deliberately small so it runs on a laptop/workstation in a few minutes.
Uses neighbor coupling (same topology the sanity test used).
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

from hopf_brain import BrainHopfStatic, hopf_structure_from_cpg_structure
from contact_detection import (
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
)


ROBOT = "spider"
COUPLING = "blf"          # "neighbor", "blf", or "uncoupled"
SIM_TIME = 20.0
GENERATIONS = 100
POPULATION = 10
NUM_WORKERS = 4
LAMBDA_PENALTY = 0
OMEGA = 2 * math.pi * 1.0  # 1 Hz, fixed for all oscillators
DIRECTED = True            # True: fitness = max(0, dy); False: distance = sqrt(dx^2+dy^2)


def build_hopf_struct(robot_name, coupling=COUPLING):
    body = modular_robots_v1.get(robot_name)
    hinges = body.find_modules_of_type(ActiveHinge)
    if coupling == "uncoupled":
        cpg_struct, mapping = active_hinges_to_cpg_network_structure_internal_only(hinges)
    elif coupling == "blf":
        cpg_struct, mapping = active_hinges_to_cpg_network_structure_blf(hinges, body)
    else:
        cpg_struct, mapping = active_hinges_to_cpg_network_structure_neighbor(hinges)
    hopf_struct = hopf_structure_from_cpg_structure(cpg_struct)
    return body, hopf_struct, mapping


def simulate_one(params_array, robot_name, sim_time, lambda_penalty):
    """Evaluate a single parameter vector. Returns (fitness, distance, dragging)."""
    try:
        body, hopf_struct, mapping = build_hopf_struct(robot_name)

        brain = BrainHopfStatic.from_params(
            params=params_array,
            network_structure=hopf_struct,
            output_mapping=mapping,
            omega=OMEGA,
        )
        robot = ModularRobot(body=body, brain=brain)

        terrain = Terrain(
            static_geometry=[GeometryPlane(
                pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
                texture=Texture(
                    base_color=Color(200, 200, 200, 255),
                    primary_color=Color(220, 220, 220, 255),
                    secondary_color=Color(80, 80, 80, 255),
                    map_type=MapType.MAP2D,
                    reference=TextureReference(builtin="checker"),
                    repeat=(50, 50),
                ),
            )],
            friction=1.0,
        )
        scene = ModularRobotScene(terrain=terrain)
        scene.add_robot(robot)
        sim_scene, _ = scene.to_simulation_scene()

        batch_params = make_standard_batch_parameters()
        batch_params.simulation_time = sim_time

        model, mj_mapping = scene_to_model(
            sim_scene,
            simulation_timestep=batch_params.simulation_timestep,
            cast_shadows=False,
            fast_sim=True,
        )
        data = mujoco.MjData(model)
        control_interface = ControlInterfaceImpl(
            data=data, abstraction_to_mujoco_mapping=mj_mapping
        )

        core_body_id = get_robot_core_body_id(model)
        if core_body_id is None:
            return 0.0, 0.0, 1.0

        ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
        non_foot_ids = robot_ids - foot_ids

        control_step = 1.0 / batch_params.control_frequency
        last_control_time = 0.0

        mujoco.mj_forward(model, data)
        initial_pos = data.xpos[core_body_id].copy()

        total_steps = 0
        drag_steps = 0

        while data.time < sim_time:
            total_steps += 1
            contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
            if any(rg in non_foot_ids for rg, *_ in contacts):
                drag_steps += 1

            if data.time >= last_control_time + control_step:
                last_control_time = math.floor(data.time / control_step) * control_step
                sim_state = SimulationStateImpl(
                    data=data,
                    abstraction_to_mujoco_mapping=mj_mapping,
                    camera_views={},
                )
                sim_scene.handler.handle(sim_state, control_interface, control_step)

            mujoco.mj_step(model, data)

        final_pos = data.xpos[core_body_id].copy()
        dx = final_pos[0] - initial_pos[0]
        dy = final_pos[1] - initial_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)

        dragging = drag_steps / total_steps if total_steps > 0 else 0.0
        # Use directed (Y-axis) fitness if DIRECTED is True, else total displacement
        primary = max(0.0, dy) if DIRECTED else distance
        if lambda_penalty == 0:
            fitness = primary
        else:
            fitness = primary * math.pow(max(0.0, 1.0 - dragging), lambda_penalty)
        return fitness, distance, dragging
    except Exception as e:
        print(f"  ERROR in eval: {e}")
        return 0.0, 0.0, 1.0


def _eval_wrapper(args):
    idx, params, robot_name, sim_time, lam = args
    f, d, dr = simulate_one(params, robot_name, sim_time, lam)
    return idx, f, d, dr


def main():
    body, hopf_struct, mapping = build_hopf_struct(ROBOT)
    n_params = hopf_struct.num_params
    n_osc = hopf_struct.num_oscillators
    n_coup = hopf_struct.num_connections

    print("=" * 60)
    print(f"Hopf evolution: {ROBOT} ({COUPLING})")
    print("=" * 60)
    print(f"Oscillators: {n_osc}")
    print(f"Couplings:   {n_coup}")
    print(f"Total params: {n_params}  ({n_osc} mu + {n_coup} coupling)")
    print(f"Population:  {POPULATION}")
    print(f"Generations: {GENERATIONS}")
    print(f"Sim time:    {SIM_TIME}s")
    print(f"Workers:     {NUM_WORKERS}")
    print(f"Lambda:      {LAMBDA_PENALTY}")
    print(f"Omega:       {OMEGA:.3f} rad/s ({OMEGA/(2*math.pi):.2f} Hz)")
    print("=" * 60)

    # Parameter bounds: mu in [0, 1], coupling in [-1, 1].
    lower = [0.0] * n_osc + [-1.0] * n_coup
    upper = [1.0] * n_osc + [1.0] * n_coup
    initial_mean = [0.5] * n_osc + [0.0] * n_coup
    initial_sigma = 0.25

    opts = cma.CMAOptions()
    opts.set("popsize", POPULATION)
    opts.set("bounds", [lower, upper])
    opts.set("seed", 42)
    opts.set("verbose", -9)  # quiet

    opt = cma.CMAEvolutionStrategy(initial_mean, initial_sigma, opts)

    best_fitness = float("-inf")
    best_params = None
    best_distance = 0.0

    t_start = time.time()

    for gen in range(GENERATIONS):
        gen_start = time.time()
        solutions = opt.ask()

        args_list = [
            (i, np.asarray(sol), ROBOT, SIM_TIME, LAMBDA_PENALTY)
            for i, sol in enumerate(solutions)
        ]

        results = [None] * len(solutions)
        if NUM_WORKERS <= 1:
            for a in args_list:
                idx, f, d, dr = _eval_wrapper(a)
                results[idx] = (f, d, dr)
        else:
            with ProcessPoolExecutor(max_workers=NUM_WORKERS) as ex:
                futures = [ex.submit(_eval_wrapper, a) for a in args_list]
                for fut in as_completed(futures):
                    idx, f, d, dr = fut.result()
                    results[idx] = (f, d, dr)

        fitnesses = [r[0] for r in results]
        distances = [r[1] for r in results]
        draggings = [r[2] for r in results]

        opt.tell(solutions, [-f for f in fitnesses])

        best_idx = int(np.argmax(fitnesses))
        if fitnesses[best_idx] > best_fitness:
            best_fitness = fitnesses[best_idx]
            best_params = np.asarray(solutions[best_idx])
            best_distance = distances[best_idx]

        gen_time = time.time() - gen_start
        print(
            f"Gen {gen+1:3d}/{GENERATIONS} | "
            f"best={best_fitness:.3f} (d={best_distance:.3f}) | "
            f"gen_mean={np.mean(fitnesses):.3f} gen_max={np.max(fitnesses):.3f} | "
            f"drag_mean={np.mean(draggings):.2f} | "
            f"{gen_time:.1f}s"
        )

    total_time = time.time() - t_start
    print()
    print("=" * 60)
    print(f"Evolution complete in {total_time:.1f}s ({total_time/60:.1f} min)")
    print(f"Best fitness: {best_fitness:.3f}")
    print(f"Best distance: {best_distance:.3f} m")
    print("=" * 60)

    # Save best params
    suffix = "directed" if DIRECTED else "xy"
    out_path = f"hopf_{ROBOT}_{COUPLING}_{suffix}_best.npy"
    np.save(out_path, best_params)
    print(f"Saved best params: {out_path}")


if __name__ == "__main__":
    main()
