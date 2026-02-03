"""
Quick sine brain evolution with contact metrics.
Runs 10 generations for spider robot.

Uses the standard revolve2 simulation infrastructure with contact tracking.
"""

import math
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass

import cma
import mujoco
import numpy as np

from brain_sine import BrainSine, get_num_sine_params
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
from revolve2.standards.terrains import flat
from revolve2.simulation.scene import Pose, Color
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2


@dataclass
class EvalResult:
    """Evaluation result with all metrics."""
    distance: float
    contact_m1: float  # Body contact %
    contact_m2: float  # Body contact + foot slip %
    cost_of_transport: float
    stability: float
    straightness: float
    fitness: float


def simulate_sine_robot(
    robot: ModularRobot,
    simulation_time: float = 10.0,
    terrain_friction: float = 1.0,
) -> tuple[ContactTracker, LocomotionTracker]:
    """
    Simulate a robot with a sine brain and track contacts.

    Returns the contact and locomotion trackers.
    """
    # Create terrain
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

    # Create scene
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)

    # Convert to simulation scene
    batch_params = make_standard_batch_parameters()
    simulation_scene, _ = scene.to_simulation_scene()

    # Create MuJoCo model
    model, mujoco_mapping = scene_to_model(
        simulation_scene,
        simulation_timestep=batch_params.simulation_timestep,
        cast_shadows=False,
        fast_sim=True,
    )
    data = mujoco.MjData(model)

    # Identify geometries
    ground_geom_ids, robot_geom_ids, foot_geom_ids = identify_geometry_types(model)
    non_foot_geom_ids = robot_geom_ids - foot_geom_ids

    # Set terrain friction
    for geom_id in ground_geom_ids:
        model.geom_friction[geom_id] = [terrain_friction, 0.005, 0.0001]

    # Get core body for tracking
    core_body_id = get_robot_core_body_id(model)

    # Initialize trackers
    contact_tracker = ContactTracker()
    contact_tracker.ground_geom_ids = ground_geom_ids
    contact_tracker.all_robot_geom_ids = robot_geom_ids
    contact_tracker.foot_geom_ids = foot_geom_ids
    contact_tracker.non_foot_geom_ids = non_foot_geom_ids
    contact_tracker.foot_slip_threshold = 0.1

    loco_tracker = LocomotionTracker()
    loco_tracker.robot_mass = get_robot_mass(model)
    loco_tracker.simulation_timestep = batch_params.simulation_timestep

    # Create control interface
    control_interface = ControlInterfaceImpl(data=data, abstraction_to_mujoco_mapping=mujoco_mapping)

    # Initialize simulation
    mujoco.mj_forward(model, data)

    # Get initial position
    if core_body_id is not None:
        loco_tracker.start_position = tuple(data.xpos[core_body_id])
        loco_tracker.previous_position = loco_tracker.start_position

    # Control parameters
    control_freq = batch_params.control_frequency
    control_step = 1.0 / control_freq
    last_control_time = 0.0

    # Get the simulation handler (contains the brain instance)
    handler = simulation_scene.handler

    # Main simulation loop
    while data.time < simulation_time:
        # Control at control frequency
        if data.time >= last_control_time + control_step:
            last_control_time = int(data.time / control_step) * control_step

            # Create simulation state for handler
            from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
            state = SimulationStateImpl(data, mujoco_mapping, {})

            # Call handler (which calls brain.control)
            handler.handle(state, control_interface, control_step)

        # Physics step
        mujoco.mj_step(model, data)

        # Track contacts
        contact_tracker.total_timesteps += 1
        has_body_contact = False
        has_bad_contact = False

        # Check ground contacts
        contacts = get_contacts_with_ground(model, data, ground_geom_ids, robot_geom_ids)
        for robot_geom, _, force, pos in contacts:
            if robot_geom in non_foot_geom_ids:
                has_body_contact = True
                has_bad_contact = True

        # Check foot slip
        for foot_id in foot_geom_ids:
            geom_body = model.geom_bodyid[foot_id]
            curr_pos = data.xpos[geom_body].copy()

            if foot_id in contact_tracker.foot_positions_prev:
                prev_pos = contact_tracker.foot_positions_prev[foot_id]
                velocity = np.linalg.norm(curr_pos - prev_pos) / model.opt.timestep

                # Check if foot is in contact and slipping
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

        # Track locomotion
        if core_body_id is not None:
            curr_pos = data.xpos[core_body_id]
            loco_tracker.heights.append(curr_pos[2])

            # Path length
            dx = curr_pos[0] - loco_tracker.previous_position[0]
            dy = curr_pos[1] - loco_tracker.previous_position[1]
            loco_tracker.total_path_length += math.sqrt(dx*dx + dy*dy)
            loco_tracker.previous_position = tuple(curr_pos)

            # Orientation
            quat = data.xquat[core_body_id]
            roll, pitch, _ = quaternion_to_euler(quat)
            loco_tracker.roll_angles.append(roll)
            loco_tracker.pitch_angles.append(pitch)

        # Energy
        loco_tracker.total_energy += calculate_actuator_energy(model, data, model.opt.timestep)

    # Final position
    if core_body_id is not None:
        loco_tracker.end_position = tuple(data.xpos[core_body_id])

    return contact_tracker, loco_tracker


def evaluate_sine_params(
    params: np.ndarray,
    robot_name: str = "spider",
    frequency: float = 1.0,
    simulation_time: float = 10.0,
    lambda_penalty: float = 0.5,
) -> EvalResult:
    """
    Evaluate sine parameters with full contact metrics.
    """
    try:
        # Build robot
        body = modular_robots_v1.get(robot_name)
        brain = BrainSine.from_parameters(body, params, frequency=frequency)
        robot = ModularRobot(body=body, brain=brain)

        # Simulate
        contact_tracker, loco_tracker = simulate_sine_robot(
            robot, simulation_time=simulation_time
        )

        # Calculate metrics
        metrics = calculate_locomotion_metrics(loco_tracker, contact_tracker, simulation_time)

        # Calculate fitness
        contact = metrics.contact_metric_1
        fitness = metrics.distance * math.exp(-lambda_penalty * contact)

        return EvalResult(
            distance=metrics.distance,
            contact_m1=metrics.contact_metric_1,
            contact_m2=metrics.contact_metric_2,
            cost_of_transport=metrics.cost_of_transport,
            stability=metrics.stability_combined,
            straightness=metrics.straightness,
            fitness=fitness,
        )

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return EvalResult(0.0, 1.0, 1.0, float('inf'), float('inf'), 0.0, 0.0)


def _eval_wrapper(args):
    """Wrapper for parallel evaluation."""
    idx, params, robot_name, freq, sim_time, lambda_pen = args
    result = evaluate_sine_params(params, robot_name, freq, sim_time, lambda_pen)
    return idx, result


def main():
    """Run 10 generation evolution."""
    print("\n" + "=" * 70)
    print("SINE BRAIN EVOLUTION - Spider Robot - 10 Generations")
    print("=" * 70)

    robot_name = "spider"
    frequency = 1.0
    simulation_time = 10.0
    num_generations = 10
    num_workers = 4
    lambda_penalty = 0.5
    seed = 42

    # Get robot info
    body = modular_robots_v1.get(robot_name)
    n_hinges = len(body.find_modules_of_type(ActiveHinge))
    n_params = get_num_sine_params(body)

    print(f"\nConfiguration:")
    print(f"  Robot:          {robot_name}")
    print(f"  Hinges:         {n_hinges}")
    print(f"  Parameters:     {n_params} (amplitude + phase per hinge)")
    print(f"  Frequency:      {frequency} Hz")
    print(f"  Sim time:       {simulation_time}s")
    print(f"  Lambda:         {lambda_penalty}")
    print(f"  Workers:        {num_workers}")
    print(f"  Seed:           {seed}")

    # Initialize CMA-ES
    initial_mean = []
    for _ in range(n_hinges):
        initial_mean.append(0.5)  # amplitude
        initial_mean.append(0.0)  # phase

    options = cma.CMAOptions()
    options.set("seed", seed)
    options.set("bounds", [
        [0.0, -math.pi] * n_hinges,
        [1.0, math.pi] * n_hinges,
    ])

    opt = cma.CMAEvolutionStrategy(initial_mean, 0.3, options)

    # Track best
    best_fitness = float("-inf")
    best_params = None
    best_result = None

    print("\n" + "-" * 70)
    print(f"{'Gen':>4} | {'Mean Fit':>9} | {'Max Fit':>9} | {'Best Dist':>10} | "
          f"{'Body Cont':>9} | {'Slip':>9} | {'Time':>6}")
    print("-" * 70)

    start_time = time.time()

    for gen in range(num_generations):
        gen_start = time.time()

        # Get population
        solutions = opt.ask()
        pop_size = len(solutions)

        # Evaluate in parallel
        results = [None] * pop_size
        args_list = [
            (i, np.array(s), robot_name, frequency, simulation_time, lambda_penalty)
            for i, s in enumerate(solutions)
        ]

        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            futures = [executor.submit(_eval_wrapper, args) for args in args_list]
            for future in as_completed(futures):
                idx, result = future.result()
                results[idx] = result

        # Extract fitness
        fitnesses = [r.fitness for r in results]

        # Update CMA-ES
        opt.tell(solutions, [-f for f in fitnesses])

        # Track best
        gen_best_idx = np.argmax(fitnesses)
        if fitnesses[gen_best_idx] > best_fitness:
            best_fitness = fitnesses[gen_best_idx]
            best_params = np.array(solutions[gen_best_idx])
            best_result = results[gen_best_idx]

        # Stats
        gen_time = time.time() - gen_start
        mean_fit = np.mean(fitnesses)
        max_fit = np.max(fitnesses)
        best_r = results[gen_best_idx]

        print(f"{gen+1:4d} | {mean_fit:9.4f} | {max_fit:9.4f} | {best_r.distance:10.4f}m | "
              f"{best_r.contact_m1:8.1%} | {best_r.contact_m2:8.1%} | {gen_time:5.1f}s")

    total_time = time.time() - start_time

    print("-" * 70)
    print("\n" + "=" * 70)
    print("EVOLUTION COMPLETE")
    print("=" * 70)
    print(f"\nTotal time: {total_time:.1f}s ({total_time/60:.1f} min)")

    print(f"\n--- Best Solution ---")
    print(f"  Fitness:          {best_fitness:.4f}")
    print(f"  Distance:         {best_result.distance:.4f} m")
    print(f"  Body contact (M1):{best_result.contact_m1:.1%}")
    print(f"  Contact+slip (M2):{best_result.contact_m2:.1%}")
    print(f"  Cost of Transport:{best_result.cost_of_transport:.2f}")
    print(f"  Stability:        {best_result.stability:.4f}")
    print(f"  Straightness:     {best_result.straightness:.1%}")

    print(f"\nBest parameters:")
    for i in range(n_hinges):
        amp = best_params[2*i]
        phase = best_params[2*i + 1]
        print(f"  Hinge {i}: amp={amp:.3f} rad ({math.degrees(amp):5.1f}°), "
              f"phase={phase:+.3f} rad ({math.degrees(phase):+6.1f}°)")

    # Visualize?
    print("\n" + "=" * 70)
    response = input("Visualize best robot? (y/n): ").strip().lower()
    if response == 'y':
        print("Running visualization...")
        from revolve2.modular_robot_simulation import simulate_scenes
        from revolve2.simulators.mujoco_simulator import LocalSimulator

        body = modular_robots_v1.get(robot_name)
        brain = BrainSine.from_parameters(body, best_params, frequency=frequency)
        robot = ModularRobot(body=body, brain=brain)

        scene = ModularRobotScene(terrain=flat())
        scene.add_robot(robot)

        simulator = LocalSimulator(headless=False, num_simulators=1)
        batch_params = make_standard_batch_parameters()
        batch_params.simulation_time = int(simulation_time)

        simulate_scenes(simulator=simulator, batch_parameters=batch_params, scenes=scene)


if __name__ == "__main__":
    main()
