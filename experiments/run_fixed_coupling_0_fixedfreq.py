"""
Evolution for coupled sine brain with FIXED coupling=0 and FIXED frequency=0.5Hz.

Only evolves: amplitude, phase, offset per hinge (24 params for spider).
Coupling and frequency are FIXED for fair comparison.
"""

import math
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass

import cma
import mujoco
import numpy as np

from brain_sine_coupled import BrainCoupledSine, get_num_coupled_sine_params
from contact_detection import (
    ContactTracker,
    LocomotionTracker,
    calculate_locomotion_metrics,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
    get_robot_mass,
    calculate_actuator_energy,
    quaternion_to_euler,
    Terrain,
)

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot_simulation import ModularRobotScene
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.standards import modular_robots_v1
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.simulation.scene import Pose, Color
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2

# FIXED VALUES
COUPLING_STRENGTH = 0.0
FREQUENCY = 0.5  # Fixed frequency


@dataclass
class EvalResult:
    """Evaluation result with all metrics."""
    distance: float
    contact_m1: float
    contact_m2: float
    fitness: float


def simulate_robot(
    robot: ModularRobot,
    simulation_time: float = 10.0,
    terrain_friction: float = 1.0,
) -> tuple[ContactTracker, LocomotionTracker]:
    """Simulate a robot and track contacts."""
    terrain = Terrain(
        static_geometry=[
            GeometryPlane(
                pose=Pose(),
                mass=0.0,
                size=Vector2([20.0, 20.0]),
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
        friction=terrain_friction,
    )

    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)

    batch_params = make_standard_batch_parameters()
    simulation_scene, _ = scene.to_simulation_scene()

    model, mujoco_mapping = scene_to_model(
        simulation_scene,
        simulation_timestep=batch_params.simulation_timestep,
        cast_shadows=False,
        fast_sim=True,
    )
    data = mujoco.MjData(model)

    ground_geom_ids, robot_geom_ids, foot_geom_ids = identify_geometry_types(model)
    non_foot_geom_ids = robot_geom_ids - foot_geom_ids

    for geom_id in ground_geom_ids:
        model.geom_friction[geom_id] = [terrain_friction, 0.005, 0.0001]

    core_body_id = get_robot_core_body_id(model)

    contact_tracker = ContactTracker()
    contact_tracker.ground_geom_ids = ground_geom_ids
    contact_tracker.all_robot_geom_ids = robot_geom_ids
    contact_tracker.foot_geom_ids = foot_geom_ids
    contact_tracker.non_foot_geom_ids = non_foot_geom_ids
    contact_tracker.foot_slip_threshold = 0.1

    loco_tracker = LocomotionTracker()
    loco_tracker.robot_mass = get_robot_mass(model)
    loco_tracker.simulation_timestep = batch_params.simulation_timestep

    control_interface = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mujoco_mapping)

    mujoco.mj_forward(model, data)

    if core_body_id is not None:
        loco_tracker.start_position = tuple(data.xpos[core_body_id])
        loco_tracker.previous_position = loco_tracker.start_position

    control_freq = batch_params.control_frequency
    control_step = 1.0 / control_freq
    last_control_time = 0.0

    handler = simulation_scene.handler

    while data.time < simulation_time:
        if data.time >= last_control_time + control_step:
            last_control_time = int(data.time / control_step) * control_step

            from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
            state = SimulationStateImpl(data, mujoco_mapping, {})
            handler.handle(state, control_interface, control_step)

        mujoco.mj_step(model, data)

        contact_tracker.total_timesteps += 1
        has_body_contact = False
        has_bad_contact = False

        contacts = get_contacts_with_ground(model, data, ground_geom_ids, robot_geom_ids)
        for robot_geom, _, force, pos in contacts:
            if robot_geom in non_foot_geom_ids:
                has_body_contact = True
                has_bad_contact = True

        for foot_id in foot_geom_ids:
            geom_body = model.geom_bodyid[foot_id]
            curr_pos = data.xpos[geom_body].copy()

            if foot_id in contact_tracker.foot_positions_prev:
                prev_pos = contact_tracker.foot_positions_prev[foot_id]
                velocity = np.linalg.norm(curr_pos - prev_pos) / model.opt.timestep

                is_in_contact = any(
                    robot_geom == foot_id for robot_geom, _, _, _ in contacts
                )
                if is_in_contact and velocity > contact_tracker.foot_slip_threshold:
                    has_bad_contact = True

            contact_tracker.foot_positions_prev[foot_id] = curr_pos

        if has_body_contact:
            contact_tracker.timesteps_with_non_foot_contact += 1
        if has_bad_contact:
            contact_tracker.timesteps_with_bad_contact += 1

        if core_body_id is not None:
            curr_pos = data.xpos[core_body_id]
            loco_tracker.heights.append(curr_pos[2])

            dx = curr_pos[0] - loco_tracker.previous_position[0]
            dy = curr_pos[1] - loco_tracker.previous_position[1]
            loco_tracker.total_path_length += math.sqrt(dx*dx + dy*dy)
            loco_tracker.previous_position = tuple(curr_pos)

            quat = data.xquat[core_body_id]
            roll, pitch, _ = quaternion_to_euler(quat)
            loco_tracker.roll_angles.append(roll)
            loco_tracker.pitch_angles.append(pitch)

        loco_tracker.total_energy += calculate_actuator_energy(model, data, model.opt.timestep)

    if core_body_id is not None:
        loco_tracker.end_position = tuple(data.xpos[core_body_id])

    return contact_tracker, loco_tracker


def evaluate_params(
    params: np.ndarray,
    robot_name: str = "spider",
    simulation_time: float = 10.0,
) -> EvalResult:
    """
    Evaluate sine parameters with FIXED coupling and frequency.

    Params format: [amp_0, phase_0, offset_0, ...]
    Total: 3*n_hinges
    """
    try:
        body = modular_robots_v1.get(robot_name)

        brain = BrainCoupledSine.from_parameters(
            body,
            params,
            frequency=FREQUENCY,
            coupling_strength=COUPLING_STRENGTH,
        )
        robot = ModularRobot(body=body, brain=brain)

        contact_tracker, loco_tracker = simulate_robot(
            robot, simulation_time=simulation_time
        )

        metrics = calculate_locomotion_metrics(loco_tracker, contact_tracker, simulation_time)

        fitness = metrics.distance

        return EvalResult(
            distance=metrics.distance,
            contact_m1=metrics.contact_metric_1,
            contact_m2=metrics.contact_metric_2,
            fitness=fitness,
        )

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return EvalResult(0.0, 1.0, 1.0, 0.0)


def _eval_wrapper(args):
    idx, params, robot_name, sim_time = args
    result = evaluate_params(params, robot_name, sim_time)
    return idx, result


def main():
    print("\n" + "=" * 70)
    print(f"FIXED COUPLING={COUPLING_STRENGTH}, FREQ={FREQUENCY}Hz - 250 Generations")
    print("=" * 70)

    robot_name = "spider"
    simulation_time = 10.0
    num_generations = 250
    num_workers = 4
    seed = 42

    body = modular_robots_v1.get(robot_name)
    n_hinges = len(body.find_modules_of_type(ActiveHinge))
    n_params = 3 * n_hinges  # amp + phase + offset per hinge (NO frequency)

    print(f"\nConfiguration:")
    print(f"  Robot:          {robot_name}")
    print(f"  Hinges:         {n_hinges}")
    print(f"  Parameters:     {n_params} (amp + phase + offset per hinge)")
    print(f"  Coupling:       FIXED at {COUPLING_STRENGTH}")
    print(f"  Frequency:      FIXED at {FREQUENCY} Hz")
    print(f"  Sim time:       {simulation_time}s")
    print(f"  Workers:        {num_workers}")
    print(f"  Seed:           {seed}")
    print(f"  Sigma:          0.5")

    # Initial mean
    initial_mean = []
    for _ in range(n_hinges):
        initial_mean.append(0.5)   # amplitude
        initial_mean.append(0.0)   # phase
        initial_mean.append(0.0)   # offset

    # Bounds
    lower_bounds = []
    upper_bounds = []
    for _ in range(n_hinges):
        lower_bounds.extend([0.0, -math.pi, -0.5])
        upper_bounds.extend([1.0, math.pi, 0.5])

    options = cma.CMAOptions()
    options.set("seed", seed)
    options.set("bounds", [lower_bounds, upper_bounds])

    opt = cma.CMAEvolutionStrategy(initial_mean, 0.5, options)

    best_fitness = float("-inf")
    best_params = None
    best_result = None

    history = {
        "generation": [],
        "mean_fitness": [],
        "max_fitness": [],
        "best_distance": [],
        "best_contact": [],
    }

    print("\n" + "-" * 70)
    print(f"{'Gen':>4} | {'Mean Fit':>9} | {'Max Fit':>9} | {'Best Dist':>10} | "
          f"{'Body Cont':>9} | {'Time':>6}")
    print("-" * 70)

    start_time = time.time()

    for gen in range(num_generations):
        gen_start = time.time()

        solutions = opt.ask()
        pop_size = len(solutions)

        results = [None] * pop_size
        args_list = [
            (i, np.array(s), robot_name, simulation_time)
            for i, s in enumerate(solutions)
        ]

        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            futures = [executor.submit(_eval_wrapper, args) for args in args_list]
            for future in as_completed(futures):
                idx, result = future.result()
                results[idx] = result

        fitnesses = [r.fitness for r in results]
        opt.tell(solutions, [-f for f in fitnesses])

        gen_best_idx = np.argmax(fitnesses)
        if fitnesses[gen_best_idx] > best_fitness:
            best_fitness = fitnesses[gen_best_idx]
            best_params = np.array(solutions[gen_best_idx])
            best_result = results[gen_best_idx]

        gen_time = time.time() - gen_start
        mean_fit = np.mean(fitnesses)
        max_fit = np.max(fitnesses)
        best_r = results[gen_best_idx]

        history["generation"].append(gen + 1)
        history["mean_fitness"].append(mean_fit)
        history["max_fitness"].append(max_fit)
        history["best_distance"].append(best_r.distance)
        history["best_contact"].append(best_r.contact_m1)

        print(f"{gen+1:4d} | {mean_fit:9.4f} | {max_fit:9.4f} | {best_r.distance:10.4f}m | "
              f"{best_r.contact_m1:8.1%} | {gen_time:5.1f}s")

        # Log best params every 25 generations
        if (gen + 1) % 25 == 0 and best_params is not None:
            print(f"\n--- Best params at gen {gen+1} (dist={best_result.distance:.3f}m, contact={best_result.contact_m1:.1%}) ---")
            amps = best_params[0::3]
            phases = best_params[1::3]
            offsets = best_params[2::3]
            print(f"Amplitudes: {amps}")
            print(f"Phases: {phases}")
            print(f"Offsets: {offsets}")
            print("-" * 70)

    total_time = time.time() - start_time

    print("-" * 70)
    print("\n" + "=" * 70)
    print(f"EVOLUTION COMPLETE - COUPLING={COUPLING_STRENGTH}, FREQ={FREQUENCY}Hz")
    print("=" * 70)
    print(f"\nTotal time: {total_time:.1f}s ({total_time/60:.1f} min)")

    print(f"\n--- Best Solution ---")
    print(f"  Fitness:          {best_fitness:.4f}")
    print(f"  Distance:         {best_result.distance:.4f} m")
    print(f"  Body contact (M1):{best_result.contact_m1:.1%}")
    print(f"  Contact+slip (M2):{best_result.contact_m2:.1%}")
    print(f"  Coupling:         {COUPLING_STRENGTH} (FIXED)")
    print(f"  Frequency:        {FREQUENCY} Hz (FIXED)")

    print(f"\nBest parameters:")
    for i in range(n_hinges):
        amp = best_params[3*i]
        phase = best_params[3*i + 1]
        offset = best_params[3*i + 2]
        print(f"  Hinge {i}: amp={amp:.3f}, phase={phase:+.3f}, offset={offset:+.3f}")

    results_file = f"coupling{COUPLING_STRENGTH:.0f}_freq{FREQUENCY}_seed{seed}.npz"
    np.savez(
        results_file,
        best_params=best_params,
        best_fitness=best_fitness,
        best_distance=best_result.distance,
        best_contact_m1=best_result.contact_m1,
        best_contact_m2=best_result.contact_m2,
        coupling_strength=COUPLING_STRENGTH,
        frequency=FREQUENCY,
        history_generation=history["generation"],
        history_mean_fitness=history["mean_fitness"],
        history_max_fitness=history["max_fitness"],
        history_best_distance=history["best_distance"],
        history_best_contact=history["best_contact"],
        seed=seed,
        num_generations=num_generations,
        simulation_time=simulation_time,
    )
    print(f"\nResults saved to: {results_file}")


if __name__ == "__main__":
    main()
