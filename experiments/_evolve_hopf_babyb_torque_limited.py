"""
Hopf babyb BLF 1 Hz lambda=0 100 gens WITH torque limits enforced.

Same seed (42) and same hyperparameters as _evolve_hopf_babyb.py, except
the MjModel is patched post-build to enforce forcerange=+/-0.948 Nm.
One-off: no changes to revolve2 core.
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

from hopf_brain import BrainHopfStatic, hopf_structure_from_cpg_structure


ROBOT = "babyb"
COUPLING = "blf"
SIM_TIME = 20.0
GENERATIONS = 100
POPULATION = 10
NUM_WORKERS = 4
LAMBDA_PENALTY = 0
OMEGA = 2 * math.pi * 1.0          # 1 Hz
DIRECTED = False                    # XY distance, matches original babyb Hopf run
CMA_SEED = 42                       # matches original
TORQUE_LIMIT = 0.948013269


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
    try:
        body, hopf_struct, mapping = build_hopf_struct(robot_name)
        brain = BrainHopfStatic.from_params(
            params=params_array, network_structure=hopf_struct,
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

        # *** TORQUE LIMIT PATCH ***
        model.actuator_forcerange[:, 0] = -TORQUE_LIMIT
        model.actuator_forcerange[:, 1] = +TORQUE_LIMIT
        model.actuator_forcelimited[:] = 1

        data = mujoco.MjData(model)
        ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)

        core = get_robot_core_body_id(model)
        if core is None:
            return 0.0, 0.0, 1.0
        ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
        non_foot = robot_ids - foot_ids
        ctrl_step = 1.0 / batch.control_frequency
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
            if data.time >= last_ctrl + ctrl_step:
                last_ctrl = math.floor(data.time / ctrl_step) * ctrl_step
                sim_state = SimulationStateImpl(
                    data=data, abstraction_to_mujoco_mapping=mj_mapping,
                    camera_views={},
                )
                sim_scene.handler.handle(sim_state, ctrl, ctrl_step)
            mujoco.mj_step(model, data)

        fp = data.xpos[core].copy()
        dx = fp[0] - init_pos[0]
        dy = fp[1] - init_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)
        dragging = drag / total if total > 0 else 0
        primary = max(0.0, dy) if DIRECTED else distance
        if lambda_penalty == 0:
            fitness = primary
        else:
            fitness = primary * math.pow(max(0.0, 1.0 - dragging), lambda_penalty)
        return fitness, distance, dragging
    except Exception as e:
        print(f"  ERROR: {e}")
        return 0.0, 0.0, 1.0


def _eval(args):
    idx, params, robot, sim_time, lam = args
    f, d, dr = simulate_one(params, robot, sim_time, lam)
    return idx, f, d, dr


def main():
    body, hopf_struct, mapping = build_hopf_struct(ROBOT)
    n_osc = hopf_struct.num_oscillators
    n_coup = hopf_struct.num_connections
    n_params = hopf_struct.num_params

    print("=" * 60)
    print(f"Hopf evolution WITH TORQUE LIMITS ({TORQUE_LIMIT} Nm)")
    print("=" * 60)
    print(f"Robot:       {ROBOT}")
    print(f"Coupling:    {COUPLING}")
    print(f"Lambda:      {LAMBDA_PENALTY}")
    print(f"Oscillators: {n_osc} mu + {n_coup} coupling = {n_params} params")
    print(f"Population:  {POPULATION}")
    print(f"Generations: {GENERATIONS}")
    print(f"Sim time:    {SIM_TIME}s")
    print(f"Workers:     {NUM_WORKERS}")
    print(f"CMA seed:    {CMA_SEED}")
    print(f"Omega:       {OMEGA:.3f} rad/s ({OMEGA/(2*math.pi):.2f} Hz)")
    print(f"Directed:    {DIRECTED}")
    print("=" * 60)

    lower = [0.0] * n_osc + [-1.0] * n_coup
    upper = [1.0] * n_osc + [1.0] * n_coup
    initial_mean = [0.5] * n_osc + [0.0] * n_coup
    initial_sigma = 0.25

    opts = cma.CMAOptions()
    opts.set("popsize", POPULATION)
    opts.set("bounds", [lower, upper])
    opts.set("seed", CMA_SEED)
    opts.set("verbose", -9)

    opt = cma.CMAEvolutionStrategy(initial_mean, initial_sigma, opts)

    best_f = float("-inf")
    best_params = None
    best_d = 0.0
    best_dr = 1.0
    t0 = time.time()

    for gen in range(GENERATIONS):
        t_gen = time.time()
        sols = opt.ask()
        args_list = [(i, np.asarray(s), ROBOT, SIM_TIME, LAMBDA_PENALTY)
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

        print(
            f"Gen {gen+1:3d}/{GENERATIONS} | "
            f"best={best_f:.3f} (d={best_d:.2f} drag={best_dr*100:.1f}%) | "
            f"gen_mean={np.mean(fits):.3f} gen_max={np.max(fits):.3f} | "
            f"{time.time()-t_gen:.1f}s"
        )

    elapsed = time.time() - t0
    print()
    print("=" * 60)
    print(f"Evolution complete in {elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"Best fitness:  {best_f:.3f}")
    print(f"Best distance: {best_d:.3f} m")
    print(f"Best dragging: {best_dr*100:.2f}%")
    print("=" * 60)

    out = f"hopf_{ROBOT}_{COUPLING}_xy_TORQUELIMITED_best.npy"
    np.save(out, best_params)
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
