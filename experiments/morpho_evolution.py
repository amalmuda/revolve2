"""
Morphological evolution with BLF-structured CPG coupling.

Evolves robot body shapes (direct tree encoding) alongside CPG controllers
(CMA-ES inner loop). Saves best body + params every generation for
visualization while running.

Usage:
    python morpho_evolution.py --coupling blf --seed 42 --results-dir results/morpho_pilot
    python morpho_evolution.py --coupling uncoupled --seed 42
    python morpho_evolution.py --coupling neighbor --seed 42
"""
import argparse
import copy
import math
import os
import pickle
import random
import sys
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cma
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.body.v1 import BodyV1, CoreV1, ActiveHingeV1, BrickV1
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2
import mujoco

from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
    get_robot_core_body_id,
    identify_geometry_types,
    get_contacts_with_ground,
)


ROTATIONS = [0.0, np.pi / 2.0, -np.pi / 2.0, np.pi]


# ==================== Body operations ====================

def get_attachment_points(module):
    if isinstance(module, CoreV1):
        slots = {0: "front", 1: "right", 2: "back", 3: "left"}
    elif isinstance(module, BrickV1):
        slots = {0: "front", 1: "right", 2: "left"}
    elif isinstance(module, ActiveHingeV1):
        slots = {0: "attachment"}
    else:
        return {}
    result = {}
    for idx in slots:
        result[idx] = idx in module.children
    return result


def get_all_modules(body):
    modules = []
    def traverse(mod):
        modules.append(mod)
        for child in mod.children.values():
            traverse(child)
    traverse(body.core)
    return modules


def count_hinges(body):
    return len(body.find_modules_of_type(ActiveHinge))


def count_modules(body):
    return len(get_all_modules(body))


def random_body(rng, max_modules=20, min_hinges=2):
    for _ in range(100):
        body = BodyV1()
        _grow(body.core_v1, 0, 4, rng, [1], max_modules)
        if count_hinges(body) >= min_hinges and count_modules(body) <= max_modules:
            return body
    body = BodyV1()
    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment = BrickV1(0.0)
    body.core_v1.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment = BrickV1(0.0)
    return body


def _grow(module, depth, max_depth, rng, count, max_modules):
    if depth >= max_depth or count[0] >= max_modules:
        return
    slots = get_attachment_points(module)
    for slot_idx, occupied in slots.items():
        if occupied or count[0] >= max_modules:
            continue
        if rng.random() < 0.4:
            rotation = rng.choice(ROTATIONS)
            if rng.random() < 0.4:
                child = ActiveHingeV1(rotation)
            else:
                child = BrickV1(rotation)
            module.set_child(child, slot_idx)
            count[0] += 1
            _grow(child, depth + 1, max_depth, rng, count, max_modules)


def clone_body(body):
    return copy.deepcopy(body)


def mutate_body(body, rng, max_modules=20, min_hinges=2):
    body = clone_body(body)
    modules = get_all_modules(body)
    op = rng.choice(["add", "remove", "change_rotation"])

    if op == "add" and count_modules(body) < max_modules:
        candidates = []
        for mod in modules:
            for idx, occupied in get_attachment_points(mod).items():
                if not occupied:
                    candidates.append((mod, idx))
        if candidates:
            mod, idx = rng.choice(candidates)
            rotation = rng.choice(ROTATIONS)
            child = ActiveHingeV1(rotation) if rng.random() < 0.5 else BrickV1(rotation)
            mod.set_child(child, idx)

    elif op == "remove":
        leaves = [m for m in modules if len(m.children) == 0 and not isinstance(m, CoreV1)]
        if leaves:
            leaf = rng.choice(leaves)
            if leaf._parent and leaf._parent_child_index in leaf._parent.children:
                del leaf._parent.children[leaf._parent_child_index]

    elif op == "change_rotation":
        # Replace a random non-core module with same type but different rotation
        non_core = [m for m in modules if not isinstance(m, CoreV1) and m._parent is not None]
        if non_core:
            mod = rng.choice(non_core)
            new_rot = rng.choice(ROTATIONS)
            parent = mod._parent
            child_idx = mod._parent_child_index
            if parent is not None and child_idx is not None:
                # Create replacement with new rotation, same type
                if isinstance(mod, ActiveHingeV1):
                    replacement = ActiveHingeV1(new_rot)
                else:
                    replacement = BrickV1(new_rot)
                # Move children to replacement
                for ci, child in list(mod.children.items()):
                    child._parent = None
                    child._parent_child_index = None
                    replacement.set_child(child, ci)
                # Replace in parent
                del parent.children[child_idx]
                parent.set_child(replacement, child_idx)

    if count_hinges(body) < min_hinges:
        return mutate_body(body, rng, max_modules, min_hinges)
    return body


def crossover_bodies(parent_a, parent_b, rng, max_modules=20, min_hinges=2):
    child = clone_body(parent_a)
    modules_b = get_all_modules(parent_b)
    non_core_b = [m for m in modules_b if not isinstance(m, CoreV1) and m._parent is not None]
    if not non_core_b:
        return child

    donor = rng.choice(non_core_b)
    donor_copy = copy.deepcopy(donor)
    donor_copy._parent = None
    donor_copy._parent_child_index = None

    child_modules = get_all_modules(child)
    candidates = []
    for mod in child_modules:
        for idx, occupied in get_attachment_points(mod).items():
            if not occupied:
                candidates.append((mod, idx))
    if not candidates:
        return child

    target_mod, target_idx = rng.choice(candidates)
    target_mod.set_child(donor_copy, target_idx)

    while count_modules(child) > max_modules:
        modules = get_all_modules(child)
        leaves = [m for m in modules if len(m.children) == 0 and not isinstance(m, CoreV1)]
        if not leaves:
            break
        leaf = rng.choice(leaves)
        if leaf._parent and leaf._parent_child_index in leaf._parent.children:
            del leaf._parent.children[leaf._parent_child_index]

    if count_hinges(child) < min_hinges:
        return clone_body(parent_a)
    return child


# ==================== CPG + Simulation ====================

def get_cpg_structure(coupling, body, hinges):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    if coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)
    return active_hinges_to_cpg_network_structure_neighbor(hinges)


def simulate_robot(body, params, coupling, sim_time):
    try:
        hinges = body.find_modules_of_type(ActiveHinge)
        if len(hinges) < 2:
            return 0.0, 1.0

        cpg_struct, output_mapping = get_cpg_structure(coupling, body, hinges)
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=params, cpg_network_structure=cpg_struct,
            initial_state_uniform=math.sqrt(2) * 0.5, output_mapping=output_mapping,
        )
        robot = ModularRobot(body=body, brain=brain)

        terrain = Terrain(
            static_geometry=[GeometryPlane(pose=Pose(), mass=0.0, size=Vector2([20.0, 20.0]),
                texture=Texture(base_color=Color(200,200,200,255), primary_color=Color(220,220,220,255),
                    secondary_color=Color(80,80,80,255), map_type=MapType.MAP2D,
                    reference=TextureReference(builtin="checker"), repeat=(50, 50)))],
            friction=1.0)
        scene = ModularRobotScene(terrain=terrain)
        scene.add_robot(robot)
        simulation_scene, _ = scene.to_simulation_scene()

        batch_params = make_standard_batch_parameters()
        batch_params.simulation_time = sim_time

        model, mapping = scene_to_model(simulation_scene,
            simulation_timestep=batch_params.simulation_timestep, cast_shadows=False, fast_sim=True)
        data = mujoco.MjData(model)

        core_body_id = get_robot_core_body_id(model)
        if core_body_id is None:
            return 0.0, 1.0

        control_interface = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mapping)
        ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
        non_foot_ids = robot_ids - foot_ids

        control_step = 1.0 / batch_params.control_frequency
        last_control_time = 0.0
        mujoco.mj_forward(model, data)
        init_pos = data.xpos[core_body_id].copy()
        total_ts = 0
        drag_ts = 0

        while data.time < sim_time:
            total_ts += 1
            contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
            for rg, _, _, _ in contacts:
                if rg in non_foot_ids:
                    drag_ts += 1
                    break
            if data.time >= last_control_time + control_step:
                last_control_time = math.floor(data.time / control_step) * control_step
                state = SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mapping, camera_views={})
                simulation_scene.handler.handle(state, control_interface, control_step)
            mujoco.mj_step(model, data)

        final_pos = data.xpos[core_body_id].copy()
        dx, dy = final_pos[0] - init_pos[0], final_pos[1] - init_pos[1]
        return math.sqrt(dx*dx + dy*dy), drag_ts / total_ts if total_ts > 0 else 1.0
    except Exception as e:
        print("  SIM ERROR: %s" % e)
        return 0.0, 1.0


def _eval_one(args):
    """Worker function for parallel CMA-ES evaluation."""
    idx, params, body_pkl, coupling, sim_time, lambda_penalty = args
    body = pickle.loads(body_pkl)
    d, dr = simulate_robot(body, np.array(params), coupling, sim_time)
    f = d * math.pow(1 - dr, lambda_penalty)
    return idx, f, d, dr


def optimize_controller(body, coupling, inner_pop, inner_gens, sim_time, rng,
                        param_bounds=1.0, lambda_penalty=2.0, workers=1):
    hinges = body.find_modules_of_type(ActiveHinge)
    if len(hinges) < 2:
        return 0.0, 0.0, 1.0, None

    cpg_struct, _ = get_cpg_structure(coupling, body, hinges)
    n_params = cpg_struct.num_connections

    opts = cma.CMAOptions()
    opts.set("seed", int(rng.random() * 1e6))
    opts.set("popsize", inner_pop)
    opts.set("bounds", [[-param_bounds] * n_params, [param_bounds] * n_params])
    opts.set("verbose", -9)

    opt = cma.CMAEvolutionStrategy([0.0] * n_params, param_bounds / 2.0, opts)

    best_fitness = float("-inf")
    best_params = None
    best_dist = 0.0
    best_drag = 1.0

    # Pickle body once for passing to workers
    body_pkl = pickle.dumps(body)

    for _ in range(inner_gens):
        solutions = opt.ask()

        if workers <= 1:
            # Sequential
            results = []
            for i, sol in enumerate(solutions):
                results.append(_eval_one((i, sol, body_pkl, coupling, sim_time, lambda_penalty)))
        else:
            # Parallel
            from concurrent.futures import ProcessPoolExecutor, as_completed
            args_list = [(i, sol, body_pkl, coupling, sim_time, lambda_penalty)
                         for i, sol in enumerate(solutions)]
            results = [None] * len(solutions)
            with ProcessPoolExecutor(max_workers=workers) as executor:
                futures = [executor.submit(_eval_one, a) for a in args_list]
                for future in as_completed(futures):
                    idx, f, d, dr = future.result()
                    results[idx] = (idx, f, d, dr)

        fitnesses = []
        for idx, f, d, dr in results:
            fitnesses.append(f)
            if f > best_fitness:
                best_fitness = f
                best_params = np.array(solutions[idx])
                best_dist = d
                best_drag = dr
        opt.tell(solutions, [-f for f in fitnesses])

    return best_fitness, best_dist, best_drag, best_params


# ==================== Main ====================

def main():
    parser = argparse.ArgumentParser(description="Morphological evolution")
    parser.add_argument("--coupling", type=str, required=True, choices=["uncoupled", "neighbor", "blf"])
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--results-dir", type=str, default="results/morpho_pilot")
    parser.add_argument("--outer-pop", type=int, default=10)
    parser.add_argument("--outer-gens", type=int, default=10)
    parser.add_argument("--inner-pop", type=int, default=5)
    parser.add_argument("--inner-gens", type=int, default=10)
    parser.add_argument("--sim-time", type=float, default=10.0)
    parser.add_argument("--lambda", dest="lambda_penalty", type=float, default=2.0)
    parser.add_argument("--max-modules", type=int, default=20)
    parser.add_argument("--min-hinges", type=int, default=2)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args()

    coupling = args.coupling
    seed = args.seed
    rng = random.Random(seed)

    # Output directory
    out_dir = os.path.join(args.results_dir, "%s_seed%d" % (coupling, seed))
    os.makedirs(out_dir, exist_ok=True)

    # Log file (appended each gen — can be tailed while running)
    log_path = os.path.join(out_dir, "evolution_log.csv")
    with open(log_path, "w") as f:
        f.write("generation,individual,modules,hinges,n_params,fitness,distance,dragging\n")

    print("=" * 60)
    print("MORPHOLOGICAL EVOLUTION")
    print("=" * 60)
    print("  Coupling:     %s" % coupling)
    print("  Seed:         %d" % seed)
    print("  Outer:        pop=%d, gens=%d" % (args.outer_pop, args.outer_gens))
    print("  Inner:        pop=%d, gens=%d" % (args.inner_pop, args.inner_gens))
    print("  Sim time:     %.1fs" % args.sim_time)
    print("  Lambda:       %.1f" % args.lambda_penalty)
    print("  Output:       %s" % out_dir)
    print("=" * 60)

    # Initialize population
    print("\nGenerating initial population...")
    population = []
    for i in range(args.outer_pop):
        body = random_body(rng, args.max_modules, args.min_hinges)
        print("  Individual %d: %d modules, %d hinges" % (i, count_modules(body), count_hinges(body)))
        population.append(body)

    best_ever_fitness = float("-inf")

    for gen in range(1, args.outer_gens + 1):
        print("\n--- Generation %d ---" % gen, flush=True)
        gen_start = time.time()

        eval_results = []
        for i, body in enumerate(population):
            n_h = count_hinges(body)
            n_m = count_modules(body)
            fitness, dist, drag, params = optimize_controller(
                body, coupling, args.inner_pop, args.inner_gens,
                args.sim_time, rng, lambda_penalty=args.lambda_penalty,
                workers=args.workers,
            )

            # Get param count
            hinges = body.find_modules_of_type(ActiveHinge)
            cpg_struct, _ = get_cpg_structure(coupling, body, hinges)
            n_params = cpg_struct.num_connections

            eval_results.append((body, fitness, dist, drag, params))
            print("  [%d] %dm %dh %dp -> fit=%.3f dist=%.2f drag=%.1f%%" % (
                i, n_m, n_h, n_params, fitness, dist, drag * 100), flush=True)

            # Append to log
            with open(log_path, "a") as f:
                f.write("%d,%d,%d,%d,%d,%.6f,%.4f,%.4f\n" % (
                    gen, i, n_m, n_h, n_params, fitness, dist, drag))

        eval_results.sort(key=lambda x: -x[1])
        best = eval_results[0]
        best_body, best_fit, best_dist, best_drag, best_params = best

        print("  BEST: fit=%.3f dist=%.2f drag=%.1f%% (%dm %dh)" % (
            best_fit, best_dist, best_drag * 100,
            count_modules(best_body), count_hinges(best_body)), flush=True)

        # Save best body + params this generation
        gen_dir = os.path.join(out_dir, "gen_%03d" % gen)
        os.makedirs(gen_dir, exist_ok=True)
        with open(os.path.join(gen_dir, "best_body.pkl"), "wb") as f:
            pickle.dump(best_body, f)
        if best_params is not None:
            np.save(os.path.join(gen_dir, "best_params.npy"), best_params)

        # Save best-ever
        if best_fit > best_ever_fitness:
            best_ever_fitness = best_fit
            with open(os.path.join(out_dir, "best_ever_body.pkl"), "wb") as f:
                pickle.dump(best_body, f)
            if best_params is not None:
                np.save(os.path.join(out_dir, "best_ever_params.npy"), best_params)

        # Selection + reproduction
        n_parents = max(2, args.outer_pop * 2 // 5)
        parents = [r[0] for r in eval_results[:n_parents]]

        new_pop = [clone_body(parents[0])]  # elitism
        while len(new_pop) < args.outer_pop:
            if rng.random() < 0.5 and len(parents) >= 2:
                child = crossover_bodies(rng.choice(parents), rng.choice(parents), rng,
                                         args.max_modules, args.min_hinges)
            else:
                child = mutate_body(rng.choice(parents), rng, args.max_modules, args.min_hinges)
            new_pop.append(child)

        population = new_pop
        print("  Gen time: %.1fs" % (time.time() - gen_start), flush=True)

    print("\n" + "=" * 60)
    print("EVOLUTION COMPLETE")
    print("  Best ever fitness: %.4f" % best_ever_fitness)
    print("  Results: %s" % out_dir)
    print("=" * 60)


if __name__ == "__main__":
    main()
