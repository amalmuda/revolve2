"""
Morphological evolution pilot.
Evolves robot body shapes alongside CPG controllers.

Body encoding: direct tree manipulation of Revolve2 BodyV1 objects.
Controller: CMA-ES optimized CPG weights (uncoupled or BLF-structured).

Pilot settings (tiny, just to test pipeline):
  Outer: pop 5, gens 3
  Inner: CMA-ES pop 3, gens 5
  Sim: 5 seconds
"""
import copy
import math
import os
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


# ==================== Config ====================

PILOT = True  # tiny params for testing

if PILOT:
    OUTER_POP = 5
    OUTER_GENS = 3
    INNER_POP = 3
    INNER_GENS = 5
    SIM_TIME = 5.0
else:
    OUTER_POP = 25
    OUTER_GENS = 50
    INNER_POP = 10
    INNER_GENS = 30
    SIM_TIME = 10.0

MAX_MODULES = 20
MIN_HINGES = 2
LAMBDA_PENALTY = 2.0
PARAM_BOUNDS = 1.0
COUPLING_MODE = "blf"  # set per-run; overridden by main()

ROTATIONS = [0.0, np.pi / 2.0, -np.pi / 2.0, np.pi]


# ==================== Body operations ====================

def get_attachment_points(module):
    """Get dict of (slot_index, is_occupied) for a module."""
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
    """Return list of all modules in the body."""
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


def random_body(rng):
    """Generate a random body with at least MIN_HINGES active hinges."""
    for _ in range(100):  # retry until valid
        body = BodyV1()
        _grow(body.core_v1, depth=0, max_depth=4, rng=rng, count=[1])
        if count_hinges(body) >= MIN_HINGES and count_modules(body) <= MAX_MODULES:
            return body
    # Fallback: minimal valid body
    body = BodyV1()
    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment = BrickV1(0.0)
    body.core_v1.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment = BrickV1(0.0)
    return body


def _grow(module, depth, max_depth, rng, count):
    """Recursively grow a body tree."""
    if depth >= max_depth or count[0] >= MAX_MODULES:
        return
    slots = get_attachment_points(module)
    for slot_idx, occupied in slots.items():
        if occupied:
            continue
        if rng.random() < 0.4:  # probability of adding a child
            if count[0] >= MAX_MODULES:
                break
            # Choose module type
            r = rng.random()
            rotation = rng.choice(ROTATIONS)
            if r < 0.4:
                child = ActiveHingeV1(rotation)
            else:
                child = BrickV1(rotation)
            module.set_child(child, slot_idx)
            count[0] += 1
            _grow(child, depth + 1, max_depth, rng, count)


def clone_body(body):
    """Deep copy a body."""
    return copy.deepcopy(body)


def mutate_body(body, rng):
    """Mutate a body in place. Returns the mutated body."""
    body = clone_body(body)
    modules = get_all_modules(body)

    op = rng.choice(["add", "remove", "change_rotation"])

    if op == "add" and count_modules(body) < MAX_MODULES:
        # Find modules with empty slots
        candidates = []
        for mod in modules:
            slots = get_attachment_points(mod)
            for idx, occupied in slots.items():
                if not occupied:
                    candidates.append((mod, idx))
        if candidates:
            mod, idx = rng.choice(candidates)
            rotation = rng.choice(ROTATIONS)
            if rng.random() < 0.5:
                child = ActiveHingeV1(rotation)
            else:
                child = BrickV1(rotation)
            mod.set_child(child, idx)

    elif op == "remove":
        # Find leaf modules (not core, no children)
        leaves = [m for m in modules if len(m.children) == 0 and not isinstance(m, CoreV1)]
        if leaves:
            leaf = rng.choice(leaves)
            parent = leaf._parent
            if parent is not None:
                # Remove from parent's children
                child_idx = leaf._parent_child_index
                if child_idx is not None and child_idx in parent.children:
                    del parent.children[child_idx]

    elif op == "change_rotation":
        non_core = [m for m in modules if not isinstance(m, CoreV1)]
        if non_core:
            mod = rng.choice(non_core)
            mod._orientation = rng.choice(ROTATIONS)

    # Validate
    if count_hinges(body) < MIN_HINGES:
        return mutate_body(body, rng)  # retry if invalid

    return body


def crossover_bodies(parent_a, parent_b, rng):
    """Simple crossover: take core from parent_a, replace one branch with one from parent_b."""
    child = clone_body(parent_a)

    modules_b = get_all_modules(parent_b)
    # Find a non-core subtree in parent_b
    non_core_b = [m for m in modules_b if not isinstance(m, CoreV1) and m._parent is not None]
    if not non_core_b:
        return child

    # Pick a random subtree from B
    donor = rng.choice(non_core_b)
    donor_copy = copy.deepcopy(donor)
    # Detach from copied parent so set_child won't assert
    donor_copy._parent = None
    donor_copy._parent_child_index = None

    # Find a random EMPTY attachment point in child
    child_modules = get_all_modules(child)
    candidates = []
    for mod in child_modules:
        slots = get_attachment_points(mod)
        for idx, occupied in slots.items():
            if not occupied:
                candidates.append((mod, idx))
    if not candidates:
        return child

    target_mod, target_idx = rng.choice(candidates)
    target_mod.set_child(donor_copy, target_idx)

    # Trim if too many modules
    while count_modules(child) > MAX_MODULES:
        modules = get_all_modules(child)
        leaves = [m for m in modules if len(m.children) == 0 and not isinstance(m, CoreV1)]
        if not leaves:
            break
        leaf = rng.choice(leaves)
        if leaf._parent and leaf._parent_child_index in leaf._parent.children:
            del leaf._parent.children[leaf._parent_child_index]

    if count_hinges(child) < MIN_HINGES:
        return clone_body(parent_a)  # fallback

    return child


# ==================== Simulation ====================

def get_cpg_structure(coupling, body, hinges):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    if coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)
    return active_hinges_to_cpg_network_structure_neighbor(hinges)


def simulate_robot(body, params, coupling, sim_time):
    """Run one simulation, return (distance, dragging)."""
    try:
        hinges = body.find_modules_of_type(ActiveHinge)
        if len(hinges) < MIN_HINGES:
            return 0.0, 1.0

        cpg_struct, output_mapping = get_cpg_structure(coupling, body, hinges)
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=params,
            cpg_network_structure=cpg_struct,
            initial_state_uniform=math.sqrt(2) * 0.5,
            output_mapping=output_mapping,
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
        simulation_scene, _ = scene.to_simulation_scene()

        batch_params = make_standard_batch_parameters()
        batch_params.simulation_time = sim_time

        model, mapping = scene_to_model(
            simulation_scene,
            simulation_timestep=batch_params.simulation_timestep,
            cast_shadows=False,
            fast_sim=True,
        )
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
        dx = final_pos[0] - init_pos[0]
        dy = final_pos[1] - init_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)
        dragging = drag_ts / total_ts if total_ts > 0 else 1.0

        return distance, dragging

    except Exception as e:
        print("  SIM ERROR: %s" % e)
        return 0.0, 1.0


def optimize_controller(body, coupling, inner_pop, inner_gens, sim_time, rng):
    """CMA-ES inner loop: find best CPG weights for a given body."""
    hinges = body.find_modules_of_type(ActiveHinge)
    if len(hinges) < MIN_HINGES:
        return 0.0, 0.0, 1.0, None  # fitness, distance, dragging, params

    cpg_struct, _ = get_cpg_structure(coupling, body, hinges)
    n_params = cpg_struct.num_connections

    initial_mean = [0.0] * n_params
    initial_std = PARAM_BOUNDS / 2.0
    bounds = [[-PARAM_BOUNDS] * n_params, [PARAM_BOUNDS] * n_params]

    opts = cma.CMAOptions()
    opts.set("seed", int(rng.random() * 1e6))
    opts.set("popsize", inner_pop)
    opts.set("bounds", bounds)
    opts.set("verbose", -9)  # silence CMA-ES

    opt = cma.CMAEvolutionStrategy(initial_mean, initial_std, opts)

    best_fitness = float("-inf")
    best_params = None
    best_distance = 0.0
    best_dragging = 1.0

    for gen in range(inner_gens):
        solutions = opt.ask()
        fitnesses = []
        for sol in solutions:
            d, dr = simulate_robot(body, np.array(sol), coupling, sim_time)
            f = d * math.pow(1 - dr, LAMBDA_PENALTY)
            fitnesses.append(f)
            if f > best_fitness:
                best_fitness = f
                best_params = np.array(sol)
                best_distance = d
                best_dragging = dr
        opt.tell(solutions, [-f for f in fitnesses])

    return best_fitness, best_distance, best_dragging, best_params


# ==================== Main evolution ====================

def main():
    rng = random.Random(42)
    np_rng = np.random.default_rng(42)

    print("=" * 60)
    print("MORPHOLOGICAL EVOLUTION PILOT")
    print("=" * 60)
    print("  Coupling:     %s" % COUPLING_MODE)
    print("  Outer:        pop=%d, gens=%d" % (OUTER_POP, OUTER_GENS))
    print("  Inner:        pop=%d, gens=%d" % (INNER_POP, INNER_GENS))
    print("  Sim time:     %.1fs" % SIM_TIME)
    print("  Lambda:       %.1f" % LAMBDA_PENALTY)
    print("  Max modules:  %d" % MAX_MODULES)
    print("=" * 60)

    # Initialize population
    print("\nGenerating initial population...")
    population = []
    for i in range(OUTER_POP):
        body = random_body(rng)
        n_h = count_hinges(body)
        n_m = count_modules(body)
        print("  Individual %d: %d modules, %d hinges" % (i, n_m, n_h))
        population.append(body)

    # Evolution loop
    for gen in range(OUTER_GENS):
        print("\n--- Generation %d ---" % (gen + 1))
        gen_start = time.time()

        # Evaluate each individual
        eval_results = []
        for i, body in enumerate(population):
            n_h = count_hinges(body)
            n_m = count_modules(body)
            fitness, dist, drag, params = optimize_controller(
                body, COUPLING_MODE, INNER_POP, INNER_GENS, SIM_TIME, rng
            )
            eval_results.append((body, fitness, dist, drag, params))
            print("  [%d] %d modules, %d hinges -> fitness=%.3f dist=%.2f drag=%.1f%%" % (
                i, n_m, n_h, fitness, dist, drag * 100
            ))

        # Sort by fitness
        eval_results.sort(key=lambda x: -x[1])

        best = eval_results[0]
        print("  BEST: fitness=%.3f dist=%.2f drag=%.1f%% (%d modules, %d hinges)" % (
            best[1], best[2], best[3] * 100,
            count_modules(best[0]), count_hinges(best[0])
        ))

        # Selection + reproduction
        # Keep top 40% as parents
        n_parents = max(2, OUTER_POP * 2 // 5)
        parents = [r[0] for r in eval_results[:n_parents]]

        new_pop = []
        # Elitism: keep best
        new_pop.append(clone_body(parents[0]))

        while len(new_pop) < OUTER_POP:
            if rng.random() < 0.5 and len(parents) >= 2:
                # Crossover
                p1 = rng.choice(parents)
                p2 = rng.choice(parents)
                child = crossover_bodies(p1, p2, rng)
            else:
                # Mutation
                parent = rng.choice(parents)
                child = mutate_body(parent, rng)
            new_pop.append(child)

        population = new_pop
        gen_time = time.time() - gen_start
        print("  Generation time: %.1fs" % gen_time)

    print("\n" + "=" * 60)
    print("PILOT COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()
