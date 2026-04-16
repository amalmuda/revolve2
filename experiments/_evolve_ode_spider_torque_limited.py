"""
One-off experiment: spider ODE-CPG, BLF coupling, lambda=0, 100 generations,
WITH per-joint torque limits enforced at 0.948 Nm.

This is NOT a template for general use. The torque patching is done
in-process after scene_to_model, so nothing in revolve2 core changes.
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
    BrainCpgNetworkStatic,
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


ROBOT = "spider"
COUPLING = "blf"
LAMBDA_PENALTY = 0
SIM_TIME = 30.0
GENERATIONS = 100
POPULATION = 25          # matches Fox setup
NUM_WORKERS = 4
TORQUE_LIMIT = 0.948013269  # Nm, matches ActiveHingeV1 effort


def get_cpg_structure(coupling, body, hinges):
    if coupling == "uncoupled":
        return active_hinges_to_cpg_network_structure_internal_only(hinges)
    if coupling == "neighbor":
        return active_hinges_to_cpg_network_structure_neighbor(hinges)
    if coupling == "blf":
        return active_hinges_to_cpg_network_structure_blf(hinges, body)


def _patch_torque_limits(model, limit):
    """
    Patch an already-built MjModel in-place to enforce per-actuator force
    limits. Applies to all actuators.

    Writes a new XML with forcerange/forcelimited set, since MjModel fields
    are read-only once compiled.
    """
    # Modify MjModel's actuator arrays directly if possible. These are
    # backed by writable numpy views in the python bindings.
    model.actuator_forcerange[:, 0] = -limit
    model.actuator_forcerange[:, 1] = +limit
    model.actuator_forcelimited[:] = 1


def simulate_one(params_array, robot_name, coupling, sim_time, lambda_penalty):
    try:
        body = modular_robots_v1.get(robot_name)
        hinges = body.find_modules_of_type(ActiveHinge)
        cpg_struct, mapping = get_cpg_structure(coupling, body, hinges)
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=params_array,
            cpg_network_structure=cpg_struct,
            initial_state_uniform=math.sqrt(2) * 0.5,
            output_mapping=mapping,
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
            sim_scene,
            simulation_timestep=batch.simulation_timestep,
            cast_shadows=False,
            fast_sim=True,
        )

        # *** TORQUE LIMIT PATCH ***
        _patch_torque_limits(model, TORQUE_LIMIT)

        data = mujoco.MjData(model)
        ctrl = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mj_mapping)

        core_id = get_robot_core_body_id(model)
        if core_id is None:
            return 0.0, 0.0, 1.0

        ground_ids, robot_ids, foot_ids = identify_geometry_types(model)
        non_foot_ids = robot_ids - foot_ids

        control_step = 1.0 / batch.control_frequency
        last_ctrl = 0.0
        mujoco.mj_forward(model, data)
        init_pos = data.xpos[core_id].copy()

        total = 0
        drag = 0
        while data.time < sim_time:
            total += 1
            contacts = get_contacts_with_ground(model, data, ground_ids, robot_ids)
            if any(rg in non_foot_ids for rg, *_ in contacts):
                drag += 1
            if data.time >= last_ctrl + control_step:
                last_ctrl = math.floor(data.time / control_step) * control_step
                sim_state = SimulationStateImpl(
                    data=data,
                    abstraction_to_mujoco_mapping=mj_mapping,
                    camera_views={},
                )
                sim_scene.handler.handle(sim_state, ctrl, control_step)
            mujoco.mj_step(model, data)

        fp = data.xpos[core_id].copy()
        dx = fp[0] - init_pos[0]
        dy = fp[1] - init_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)
        dragging = drag / total if total > 0 else 0.0
        if lambda_penalty == 0:
            fitness = distance
        else:
            fitness = distance * math.pow(max(0.0, 1.0 - dragging), lambda_penalty)
        return fitness, distance, dragging
    except Exception as e:
        print(f"  ERROR in eval: {e}")
        return 0.0, 0.0, 1.0


def _eval(args):
    idx, params, robot, coup, sim_time, lam = args
    f, d, dr = simulate_one(params, robot, coup, sim_time, lam)
    return idx, f, d, dr


def main():
    body = modular_robots_v1.get(ROBOT)
    hinges = body.find_modules_of_type(ActiveHinge)
    cpg_struct, _ = get_cpg_structure(COUPLING, body, hinges)
    n_params = cpg_struct.num_connections
    # For ODE-CPG, num_connections = internal weights + coupling weights.
    # For BLF spider (4 hips, 4 knees): 8 internal + 10 coupling = 18.

    print("=" * 60)
    print(f"ODE-CPG evolution WITH TORQUE LIMITS ({TORQUE_LIMIT} Nm)")
    print("=" * 60)
    print(f"Robot:       {ROBOT}")
    print(f"Coupling:    {COUPLING}")
    print(f"Lambda:      {LAMBDA_PENALTY}")
    print(f"Params:      {n_params}")
    print(f"Population:  {POPULATION}")
    print(f"Generations: {GENERATIONS}")
    print(f"Sim time:    {SIM_TIME}s")
    print(f"Workers:     {NUM_WORKERS}")
    print("=" * 60)

    opts = cma.CMAOptions()
    opts.set("popsize", POPULATION)
    opts.set("bounds", [[-1.0] * n_params, [1.0] * n_params])
    opts.set("seed", 42)
    opts.set("verbose", -9)

    opt = cma.CMAEvolutionStrategy([0.0] * n_params, 0.5, opts)

    best_f = float("-inf")
    best_params = None
    best_d = 0.0
    best_dr = 1.0

    t0 = time.time()
    for gen in range(GENERATIONS):
        t_gen = time.time()
        sols = opt.ask()
        args_list = [(i, np.asarray(s), ROBOT, COUPLING, SIM_TIME, LAMBDA_PENALTY)
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

    out = f"ode_{ROBOT}_{COUPLING}_l{LAMBDA_PENALTY}_TORQUELIMITED_best.npy"
    np.save(out, best_params)
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
