"""
Unified evolution script for CPG comparison experiments.

Supports:
- Controller types: ode_cpg (Revolve2) or sine (parametric)
- Coupling modes: uncoupled, neighbor, blf
- Robots: spider, gecko
- Contact penalty: lambda parameter
- Metrics: distance, dragging (m1), CoT per generation

Results saved to SQLite database for easy analysis.

Usage:
    python evolve_comparison.py --robot spider --controller sine --coupling blf --lambda 1.0
"""

import argparse
import math
import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import cma
import mujoco
import numpy as np
from sqlalchemy.orm import Session

# Import sine controllers
from brain_sine import BrainSine, get_num_sine_params
from brain_sine_coupled import BrainCoupledSine
from brain_cpg_blf import brain_coupled_sine_blf_from_parameters
from blf_analyzer import analyze_body, JointType
from brain_cpg_offset import BrainCpgNetworkStaticWithOffset

# Import Revolve2 CPG
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkStatic,
    CpgNetworkStructure,
    active_hinges_to_cpg_network_structure_neighbor,
)
from revolve2.modular_robot_simulation import ModularRobotScene, Terrain
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl
from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.standards import modular_robots_v1, modular_robots_v2
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from revolve2.simulation.scene import Color, Pose
from revolve2.simulation.scene.geometry import GeometryPlane
from revolve2.simulation.scene.geometry.textures import MapType, Texture, TextureReference
from revolve2.simulation.scene.vector2 import Vector2

# Import contact detection utilities
from contact_detection import (
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_blf,
    identify_geometry_types,
    get_contacts_with_ground,
    get_robot_core_body_id,
    get_robot_mass,
    calculate_actuator_energy,
    ContactTracker,
)

# Import database
from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from evolution_comparison_database import (
    Base,
    ComparisonExperiment,
    ComparisonGeneration,
    ComparisonGenotype,
    ComparisonIndividual,
    ComparisonPopulation,
)


@dataclass
class EvaluationResult:
    """Result from a single evaluation."""
    distance: float
    dragging: float  # m1 contact metric
    cot: float  # Cost of Transport
    fitness: float
    final_x: float
    final_y: float
    avg_core_height: float = 0.0  # Average core body height during simulation
    min_core_height: float = 0.0  # Minimum core body height during simulation


def get_body(robot_name: str):
    """Get robot body by name."""
    if robot_name in ["spider", "ant", "beetle", "babya", "babyb", "blokky",
                       "garrix", "gecko", "insect", "linkin", "longleg",
                       "park", "penguin", "pentapod", "queen", "salamander",
                       "squarish", "snake", "spider9", "stingray", "tinlicker",
                       "turtle", "ww", "zappa"]:
        try:
            return modular_robots_v1.get(robot_name)
        except:
            pass
        try:
            return modular_robots_v2.get(robot_name)
        except:
            raise ValueError(f"Unknown robot: {robot_name}")
    raise ValueError(f"Unknown robot: {robot_name}")


def create_terrain() -> Terrain:
    """Create flat terrain for simulation."""
    return Terrain(
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
        friction=1.0,
    )


def simulate_with_metrics(
    robot: ModularRobot,
    simulation_time: float,
    batch_params,
) -> EvaluationResult:
    """
    Simulate robot and calculate all metrics including contact detection.

    :param robot: The robot to simulate.
    :param simulation_time: Simulation duration in seconds.
    :param batch_params: Simulation batch parameters.
    :returns: EvaluationResult with all metrics.
    """
    # Create scene
    terrain = create_terrain()
    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)

    # Convert to simulation scene
    simulation_scene, robot_to_mbs_mapping = scene.to_simulation_scene()

    # Create MuJoCo model
    model, mujoco_mapping = scene_to_model(
        simulation_scene,
        simulation_timestep=batch_params.simulation_timestep,
        cast_shadows=False,
        fast_sim=True,
    )
    data = mujoco.MjData(model)

    # Identify geometry types for contact detection
    ground_geom_ids, robot_geom_ids, foot_geom_ids = identify_geometry_types(model)
    non_foot_geom_ids = robot_geom_ids - foot_geom_ids

    # Get robot core body for position tracking
    core_body_id = get_robot_core_body_id(model)
    if core_body_id is None:
        return EvaluationResult(0.0, 1.0, float('inf'), 0.0, 0.0, 0.0)

    robot_mass = get_robot_mass(model)

    # Create contact tracker
    contact_tracker = ContactTracker()
    contact_tracker.ground_geom_ids = ground_geom_ids
    contact_tracker.foot_geom_ids = foot_geom_ids
    contact_tracker.non_foot_geom_ids = non_foot_geom_ids

    # Create control interface
    control_interface = ControlInterfaceImpl(
        data=data,
        abstraction_to_mujoco_mapping=mujoco_mapping
    )

    # Simulation loop
    control_step = 1.0 / batch_params.control_frequency
    last_control_time = 0.0
    total_energy = 0.0

    mujoco.mj_forward(model, data)

    # Record initial position
    initial_pos = data.xpos[core_body_id].copy()

    # Track core heights for height penalty
    core_heights = []

    while data.time < simulation_time:
        contact_tracker.total_timesteps += 1

        # Track contacts
        contacts = get_contacts_with_ground(model, data, ground_geom_ids, robot_geom_ids)
        has_body_contact = False

        for robot_geom, ground_geom, force, position in contacts:
            if robot_geom in non_foot_geom_ids:
                has_body_contact = True
                break

        if has_body_contact:
            contact_tracker.timesteps_with_non_foot_contact += 1

        # Track core body height (z-coordinate)
        core_height = data.xpos[core_body_id][2]
        core_heights.append(core_height)

        # Track energy
        energy = calculate_actuator_energy(model, data, batch_params.simulation_timestep)
        total_energy += energy

        # Control step
        if data.time >= last_control_time + control_step:
            last_control_time = math.floor(data.time / control_step) * control_step
            simulation_state = SimulationStateImpl(
                data=data,
                abstraction_to_mujoco_mapping=mujoco_mapping,
                camera_views={},
            )
            simulation_scene.handler.handle(simulation_state, control_interface, control_step)

        mujoco.mj_step(model, data)

    # Calculate final metrics
    final_pos = data.xpos[core_body_id].copy()

    dx = final_pos[0] - initial_pos[0]
    dy = final_pos[1] - initial_pos[1]
    distance = math.sqrt(dx**2 + dy**2)

    # M1: Proportion of timesteps with non-foot contact (dragging)
    if contact_tracker.total_timesteps > 0:
        dragging = contact_tracker.timesteps_with_non_foot_contact / contact_tracker.total_timesteps
    else:
        dragging = 0.0

    # Cost of Transport: energy / (mass * gravity * distance)
    GRAVITY = 9.81
    if distance > 0.001 and robot_mass > 0:
        cot = total_energy / (robot_mass * GRAVITY * distance)
    else:
        cot = float('inf')

    # Core height statistics
    if core_heights:
        avg_core_height = float(np.mean(core_heights))
        min_core_height = float(np.min(core_heights))
    else:
        avg_core_height = 0.0
        min_core_height = 0.0

    return EvaluationResult(
        distance=distance,
        dragging=dragging,
        cot=cot if cot != float('inf') else 0.0,  # Use 0 for CSV compatibility
        fitness=0.0,  # Calculated later with lambda
        final_x=float(final_pos[0]),
        final_y=float(final_pos[1]),
        avg_core_height=avg_core_height,
        min_core_height=min_core_height,
    )


def calculate_fitness(
    result: EvaluationResult,
    lambda_penalty: float,
    penalty_type: str = "dragging",
) -> float:
    """
    Calculate fitness based on penalty type.

    Penalty types:
    - "dragging": fitness = distance * (1 - dragging)^lambda
    - "height": fitness = distance * avg_core_height^lambda (rewards keeping body up)
    - "min_height": fitness = distance * min_core_height^lambda (penalizes any sag)
    - "combined": fitness = distance * (1 - dragging)^lambda * avg_core_height
    """
    if penalty_type == "dragging":
        return result.distance * math.pow(1 - result.dragging, lambda_penalty)
    elif penalty_type == "height":
        # Use normalized height (typically 0.05-0.15m for these robots)
        # Scale to make it comparable to dragging penalty
        height_factor = max(0.0, result.avg_core_height * 10)  # Scale up
        return result.distance * math.pow(height_factor, lambda_penalty)
    elif penalty_type == "min_height":
        # Penalize based on minimum height (more strict)
        height_factor = max(0.0, result.min_core_height * 10)
        return result.distance * math.pow(height_factor, lambda_penalty)
    elif penalty_type == "combined":
        # Combine dragging penalty with height reward
        dragging_factor = math.pow(1 - result.dragging, lambda_penalty)
        height_factor = max(0.01, result.avg_core_height * 10)
        return result.distance * dragging_factor * height_factor
    else:
        # Default to dragging
        return result.distance * math.pow(1 - result.dragging, lambda_penalty)


def evaluate_ode_cpg(
    params: np.ndarray,
    robot_name: str,
    coupling: str,
    simulation_time: float,
    lambda_penalty: float,
    penalty_type: str = "dragging",
) -> EvaluationResult:
    """Evaluate ODE CPG controller with full contact detection."""
    try:
        body = get_body(robot_name)
        active_hinges = body.find_modules_of_type(ActiveHinge)

        # Get CPG structure based on coupling mode
        if coupling == "uncoupled":
            cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_internal_only(active_hinges)
        elif coupling == "blf":
            cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_blf(active_hinges, body)
        else:  # neighbor
            cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

        # Create brain from parameters
        brain = BrainCpgNetworkStatic.uniform_from_params(
            params=params,
            cpg_network_structure=cpg_structure,
            initial_state_uniform=math.sqrt(2) * 0.5,
            output_mapping=output_mapping,
        )

        robot = ModularRobot(body=body, brain=brain)

        # Setup batch parameters
        batch_params = make_standard_batch_parameters()
        batch_params.simulation_time = simulation_time

        # Run simulation with contact detection
        result = simulate_with_metrics(robot, simulation_time, batch_params)

        # Calculate fitness with penalty
        result.fitness = calculate_fitness(result, lambda_penalty, penalty_type)

        return result

    except Exception as e:
        print(f"ODE CPG evaluation error: {e}")
        return EvaluationResult(0.0, 1.0, 0.0, 0.0, 0.0, 0.0)


def evaluate_ode_cpg_offset(
    params: np.ndarray,
    robot_name: str,
    coupling: str,
    simulation_time: float,
    lambda_penalty: float,
    penalty_type: str = "dragging",
) -> EvaluationResult:
    """Evaluate ODE CPG controller WITH OFFSET parameters.

    This adds per-joint offset parameters to the standard Revolve2 CPG.
    Output = (cpg_state + offset) * hinge_range

    Parameters layout: [cpg_weights..., offsets...]
    """
    try:
        body = get_body(robot_name)
        active_hinges = body.find_modules_of_type(ActiveHinge)
        n_hinges = len(active_hinges)

        # Get CPG structure based on coupling mode
        if coupling == "uncoupled":
            cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_internal_only(active_hinges)
        elif coupling == "blf":
            cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_blf(active_hinges, body)
        else:  # neighbor
            cpg_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

        # Create brain with offset from parameters
        brain = BrainCpgNetworkStaticWithOffset.uniform_from_params(
            params=params,
            cpg_network_structure=cpg_structure,
            initial_state_uniform=math.sqrt(2) * 0.5,
            output_mapping=output_mapping,
        )

        robot = ModularRobot(body=body, brain=brain)

        # Setup batch parameters
        batch_params = make_standard_batch_parameters()
        batch_params.simulation_time = simulation_time

        # Run simulation with contact detection
        result = simulate_with_metrics(robot, simulation_time, batch_params)

        # Calculate fitness with penalty
        result.fitness = calculate_fitness(result, lambda_penalty, penalty_type)

        return result

    except Exception as e:
        print(f"ODE CPG Offset evaluation error: {e}")
        import traceback
        traceback.print_exc()
        return EvaluationResult(0.0, 1.0, 0.0, 0.0, 0.0, 0.0)


def evaluate_sine(
    params: np.ndarray,
    robot_name: str,
    coupling: str,
    simulation_time: float,
    lambda_penalty: float,
    penalty_type: str = "dragging",
    frequency: float = 0.2,  # Bonardi et al. uses 0.2 Hz
    coupling_strength: float = 0.5,
) -> EvaluationResult:
    """Evaluate Sine CPG controller with full contact detection."""
    try:
        body = get_body(robot_name)

        # Create brain based on coupling mode
        if coupling in ("uncoupled", "uncoupled_bounded"):
            # uncoupled_bounded uses BLF amplitude bounds but no coupling
            brain = BrainSine.from_parameters(body, params, frequency=frequency)
        elif coupling in ("blf", "blf_bounded"):
            # Both blf and blf_bounded use BLF coupling, difference is in amplitude bounds
            brain, _ = brain_coupled_sine_blf_from_parameters(
                body, params, frequency=frequency, coupling_strength=coupling_strength
            )
        else:  # neighbor
            brain = BrainCoupledSine.from_parameters(
                body, params, frequency=frequency, coupling_strength=coupling_strength
            )

        robot = ModularRobot(body=body, brain=brain)

        # Setup batch parameters
        batch_params = make_standard_batch_parameters()
        batch_params.simulation_time = simulation_time

        # Run simulation with contact detection
        result = simulate_with_metrics(robot, simulation_time, batch_params)

        # Calculate fitness with penalty
        result.fitness = calculate_fitness(result, lambda_penalty, penalty_type)

        return result

    except Exception as e:
        print(f"Sine evaluation error: {e}")
        return EvaluationResult(0.0, 1.0, 0.0, 0.0, 0.0, 0.0)


def get_num_params(robot_name: str, controller: str, coupling: str) -> int:
    """Get number of parameters for given configuration."""
    body = get_body(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(active_hinges)

    if controller == "sine":
        # All sine controllers: 3 params per hinge (amp, phase, offset)
        return 3 * n_hinges
    else:  # ode_cpg or ode_cpg_offset
        if coupling == "uncoupled":
            cpg_structure, _ = active_hinges_to_cpg_network_structure_internal_only(active_hinges)
        elif coupling in ("blf", "blf_bounded"):
            cpg_structure, _ = active_hinges_to_cpg_network_structure_blf(active_hinges, body)
        else:  # neighbor
            cpg_structure, _ = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

        base_params = cpg_structure.num_connections
        if controller == "ode_cpg_offset":
            # Add offset parameter per hinge
            return base_params + n_hinges
        return base_params


def get_bounds(controller: str, n_params: int, n_hinges: int,
               robot_name: str = None, coupling: str = None, use_paper_bounds: bool = True):
    """Get parameter bounds.

    For sine + blf_bounded, uses bioinspired amplitude bounds scaled for Revolve2:
    - Spine: [0, 1.0] rad - full range for body undulation
    - Hip: [0, 0.75] rad - large range for leg swing
    - Knee: [0, 0.25] rad - restricted for distal precision
    - Ankle: [0, 0.25] rad - restricted for distal precision

    (Revolve2 V2 hinge physical limit: ±1.047 rad)

    For sine + blf (and others), uses uniform [0, 1] bounds.
    """
    if controller == "sine":
        # Check if we should use BLF-based bounds (paper bounds)
        # blf_bounded and uncoupled_bounded get paper bounds; blf/uncoupled use uniform bounds
        if coupling in ("blf_bounded", "uncoupled_bounded") and robot_name is not None and use_paper_bounds:
            # Use BLF analysis to get joint-specific amplitude bounds
            body = get_body(robot_name)
            blf_result = analyze_body(body)
            active_hinges = body.find_modules_of_type(ActiveHinge)

            # Bioinspired amplitude bounds scaled for Revolve2 V2 hinges
            # (Physical limit: ±1.047 rad, so max amplitude ~1.0 rad)
            # Maintains Bonardi et al. proportions: spine > hip > knee/ankle
            AMP_SPINE = 1.0    # Full range for body undulation
            AMP_HIP = 0.75     # Large range for leg swing
            AMP_KNEE = 0.25    # Restricted for distal precision
            AMP_ANKLE = 0.25   # Restricted for distal precision
            AMP_DEFAULT = 1.0  # Default for unclassified

            # Build bounds per hinge based on BLF classification
            lower = []
            upper = []

            # Create a mapping from hinge index to joint type
            joint_type_map = {}
            for joint_info in blf_result.joints:
                joint_type_map[joint_info.index] = joint_info.joint_type

            for i in range(n_hinges):
                joint_type = joint_type_map.get(i, JointType.UNCLASSIFIED)

                if joint_type == JointType.SPINE:
                    amp_max = AMP_SPINE
                elif joint_type == JointType.HIP:
                    amp_max = AMP_HIP
                elif joint_type == JointType.KNEE:
                    amp_max = AMP_KNEE
                elif joint_type == JointType.ANKLE:
                    amp_max = AMP_ANKLE
                else:
                    amp_max = AMP_DEFAULT

                # [amp, phase, offset] per hinge
                lower.extend([0.0, -math.pi, -0.5])
                upper.extend([amp_max, math.pi, 0.5])

            return lower, upper
        else:
            # Standard uniform bounds: [amp, phase, offset] per hinge
            lower = []
            upper = []
            for _ in range(n_hinges):
                lower.extend([0.0, -math.pi, -0.5])  # amp, phase, offset
                upper.extend([1.0, math.pi, 0.5])
            return lower, upper
    elif controller == "ode_cpg_offset":
        # CPG weights in [-1, 1], offsets in [-0.5, 0.5]
        n_cpg_params = n_params - n_hinges  # total - offsets = cpg weights
        lower = [-1.0] * n_cpg_params + [-0.5] * n_hinges
        upper = [1.0] * n_cpg_params + [0.5] * n_hinges
        return lower, upper
    else:  # ode_cpg
        return [-1.0] * n_params, [1.0] * n_params


def _eval_wrapper(args):
    """Wrapper for parallel evaluation."""
    idx, params, robot_name, controller, coupling, sim_time, lambda_penalty, penalty_type = args

    if controller == "sine":
        result = evaluate_sine(params, robot_name, coupling, sim_time, lambda_penalty, penalty_type)
    elif controller == "ode_cpg_offset":
        result = evaluate_ode_cpg_offset(params, robot_name, coupling, sim_time, lambda_penalty, penalty_type)
    else:  # ode_cpg
        result = evaluate_ode_cpg(params, robot_name, coupling, sim_time, lambda_penalty, penalty_type)

    return idx, result


def run_evolution(
    robot_name: str,
    controller: str,
    coupling: str,
    lambda_penalty: float,
    penalty_type: str = "dragging",
    simulation_time: float = 30.0,
    num_generations: int = 300,
    population_size: int = 25,
    num_workers: int = 25,
    seed: int = None,
    results_dir: str = "results",
    run_num: int = 1,
):
    """Run CMA-ES evolution with results saved to SQLite database."""

    if seed is None:
        seed = int(time.time()) % 2**32

    # Get configuration
    body = get_body(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    n_hinges = len(active_hinges)
    n_params = get_num_params(robot_name, controller, coupling)
    lower_bounds, upper_bounds = get_bounds(controller, n_params, n_hinges,
                                            robot_name=robot_name, coupling=coupling)

    # Create results directory
    experiment_name = f"{robot_name}_{controller}_{coupling}_lambda{int(lambda_penalty)}_{penalty_type}"
    experiment_dir = f"{results_dir}/{experiment_name}"
    os.makedirs(experiment_dir, exist_ok=True)

    # Database file path
    db_path = f"{experiment_dir}/run_{run_num}.sqlite"

    print("\n" + "=" * 60)
    print("EVOLUTION COMPARISON EXPERIMENT")
    print("=" * 60)
    print(f"  Robot:          {robot_name} ({n_hinges} hinges)")
    print(f"  Controller:     {controller}")
    print(f"  Coupling:       {coupling}")
    print(f"  Lambda:         {lambda_penalty}")
    print(f"  Penalty Type:   {penalty_type}")
    print(f"  Parameters:     {n_params}")
    print(f"  Sim Time:       {simulation_time}s")
    print(f"  Generations:    {num_generations}")
    print(f"  Population:     {population_size}")
    print(f"  Workers:        {num_workers}")
    print(f"  Seed:           {seed}")
    print(f"  Run:            {run_num}")
    print(f"  Database:       {db_path}")
    print("=" * 60 + "\n")

    # Setup database
    dbengine = open_database_sqlite(
        db_path, open_method=OpenMethod.OVERWITE_IF_EXISTS
    )
    Base.metadata.create_all(dbengine)

    # Save experiment configuration
    experiment = ComparisonExperiment(
        robot_name=robot_name,
        controller_type=controller,
        coupling_mode=coupling,
        lambda_penalty=lambda_penalty,
        simulation_time=simulation_time,
        num_generations=num_generations,
        population_size=population_size,
        param_bounds_min=lower_bounds[0],
        param_bounds_max=upper_bounds[0],
        rng_seed=seed,
        run_number=run_num,
        penalty_type=penalty_type,
        frequency=0.2 if controller == "sine" else None,  # Bonardi et al. uses 0.2 Hz
        coupling_strength=0.5 if controller == "sine" and coupling != "uncoupled" else None,
        num_parameters=n_params,
        num_hinges=n_hinges,
    )
    with Session(dbengine) as session:
        session.add(experiment)
        session.commit()

    # Initialize CMA-ES
    initial_mean = [(l + u) / 2 for l, u in zip(lower_bounds, upper_bounds)]
    initial_std = (upper_bounds[0] - lower_bounds[0]) / 4

    options = cma.CMAOptions()
    options.set("seed", seed)
    options.set("popsize", population_size)
    options.set("bounds", [lower_bounds, upper_bounds])

    opt = cma.CMAEvolutionStrategy(initial_mean, initial_std, options)

    # Track best
    best_fitness = float("-inf")
    best_params = None
    best_distance = 0.0

    print("Starting evolution...")
    start_time = time.time()

    while opt.countiter < num_generations:
        gen_start = time.time()

        # Get population
        solutions = opt.ask()

        # Evaluate
        results = [None] * len(solutions)
        args_list = [
            (i, np.array(params), robot_name, controller, coupling,
             simulation_time, lambda_penalty, penalty_type)
            for i, params in enumerate(solutions)
        ]

        if num_workers <= 1:
            for args in args_list:
                idx, result = _eval_wrapper(args)
                results[idx] = result
        else:
            with ProcessPoolExecutor(max_workers=num_workers) as executor:
                futures = [executor.submit(_eval_wrapper, args) for args in args_list]
                for future in as_completed(futures):
                    idx, result = future.result()
                    results[idx] = result

        # Extract metrics
        fitnesses = [r.fitness for r in results]
        distances = [r.distance for r in results]
        draggings = [r.dragging for r in results]
        cots = [r.cot for r in results]
        heights = [r.avg_core_height for r in results]

        # Tell CMA-ES
        opt.tell(solutions, [-f for f in fitnesses])

        # Track best
        gen_best_idx = np.argmax(fitnesses)
        if fitnesses[gen_best_idx] > best_fitness:
            best_fitness = fitnesses[gen_best_idx]
            best_params = np.array(solutions[gen_best_idx])
            best_distance = distances[gen_best_idx]

        # Calculate generation time
        gen_time = time.time() - gen_start

        # Save to database
        population = ComparisonPopulation(
            individuals=[
                ComparisonIndividual(
                    genotype=ComparisonGenotype.from_parameters(np.array(params)),
                    population_index=i,
                    fitness=float(fitness),
                    distance=float(r.distance),
                    dragging=float(r.dragging),
                    cost_of_transport=float(r.cot) if r.cot != float('inf') else None,
                    final_x=float(r.final_x),
                    final_y=float(r.final_y),
                    avg_core_height=float(r.avg_core_height),
                    min_core_height=float(r.min_core_height),
                )
                for i, (params, fitness, r) in enumerate(zip(solutions, fitnesses, results))
            ]
        )

        generation = ComparisonGeneration(
            experiment=experiment,
            population=population,
            generation_index=opt.countiter,
            fitness_mean=float(np.mean(fitnesses)),
            fitness_std=float(np.std(fitnesses)),
            fitness_max=float(np.max(fitnesses)),
            fitness_min=float(np.min(fitnesses)),
            distance_mean=float(np.mean(distances)),
            distance_max=float(np.max(distances)),
            dragging_mean=float(np.mean(draggings)),
            cot_mean=float(np.mean([c for c in cots if c != float('inf')])) if any(c != float('inf') for c in cots) else None,
            height_mean=float(np.mean(heights)),
            best_ever_fitness=best_fitness,
            best_ever_distance=best_distance,
            time_seconds=gen_time,
        )

        with Session(dbengine, expire_on_commit=False) as session:
            session.add(generation)
            session.commit()

        # Print progress
        if opt.countiter % 10 == 0 or opt.countiter == num_generations:
            print(f"Gen {opt.countiter:3d}: fitness={np.mean(fitnesses):.4f}/{np.max(fitnesses):.4f}, "
                  f"dist={np.mean(distances):.3f}/{np.max(distances):.3f}, "
                  f"best_ever={best_fitness:.4f} ({gen_time:.1f}s)")

    # Final summary
    total_time = time.time() - start_time
    print("\n" + "=" * 60)
    print("EVOLUTION COMPLETE")
    print("=" * 60)
    print(f"  Total time:     {total_time:.1f}s ({total_time/3600:.2f} hours)")
    print(f"  Best fitness:   {best_fitness:.4f}")
    print(f"  Best distance:  {best_distance:.4f}m")
    print(f"  Database:       {db_path}")
    print("=" * 60)

    # Save best params as .npy for easy loading
    params_path = f"{experiment_dir}/best_params_run_{run_num}.npy"
    np.save(params_path, best_params)
    print(f"  Best params:    {params_path}")

    return best_params, best_fitness


def main():
    parser = argparse.ArgumentParser(description="Unified CPG evolution for comparison experiments")

    parser.add_argument("--robot", type=str, required=True,
                        choices=["spider", "gecko"],
                        help="Robot name")
    parser.add_argument("--controller", type=str, required=True,
                        choices=["ode_cpg", "ode_cpg_offset", "sine"],
                        help="Controller type (ode_cpg_offset adds per-joint offset parameters)")
    parser.add_argument("--coupling", type=str, required=True,
                        choices=["uncoupled", "neighbor", "blf", "blf_bounded", "uncoupled_bounded"],
                        help="Coupling mode (blf_bounded/uncoupled_bounded use paper amplitude bounds)")
    parser.add_argument("--lambda", dest="lambda_penalty", type=float, required=True,
                        help="Contact penalty lambda (0=no penalty, 1=penalty)")
    parser.add_argument("--penalty-type", type=str, default="dragging",
                        choices=["dragging", "height", "min_height", "combined"],
                        help="Penalty type: dragging (penalize non-foot contact), "
                             "height (reward avg core height), "
                             "min_height (reward min core height), "
                             "combined (both dragging + height)")

    parser.add_argument("--sim-time", type=float, default=30.0,
                        help="Simulation time in seconds (default: 30)")
    parser.add_argument("--generations", type=int, default=300,
                        help="Number of generations (default: 300)")
    parser.add_argument("--population", type=int, default=25,
                        help="Population size (default: 25)")
    parser.add_argument("--workers", type=int, default=25,
                        help="Number of parallel workers (default: 25)")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed")
    parser.add_argument("--run-num", type=int, default=1,
                        help="Run number for multiple seeds")
    parser.add_argument("--results-dir", type=str, default="results/comparison",
                        help="Results directory")

    args = parser.parse_args()

    run_evolution(
        robot_name=args.robot,
        controller=args.controller,
        coupling=args.coupling,
        lambda_penalty=args.lambda_penalty,
        penalty_type=args.penalty_type,
        simulation_time=args.sim_time,
        num_generations=args.generations,
        population_size=args.population,
        num_workers=args.workers,
        seed=args.seed,
        results_dir=args.results_dir,
        run_num=args.run_num,
    )


if __name__ == "__main__":
    main()
