"""
Main evolution script for CPG parameter optimization with contact penalty.

Usage:
    python evolve.py                    # Use config file settings
    python evolve.py --robot gecko      # Override robot
    python evolve.py --generations 50   # Override generations
    python evolve.py --no-database      # Disable database logging
    python evolve.py --visualize        # Show best robot after evolution
"""

import argparse
import time
from datetime import datetime

import cma
import numpy as np
from sqlalchemy.orm import Session

import evolution_config as config
from evolution_evaluator import Evaluator
from evolution_database import (
    Base,
    Experiment,
    Generation,
    Genotype,
    Individual,
    Population,
)
from contact_detection import (
    simulate_with_contact_detection,
    active_hinges_to_cpg_network_structure_internal_only,
    active_hinges_to_cpg_network_structure_cross_leg,
    active_hinges_to_cpg_network_structure_diagonal,
    active_hinges_to_cpg_network_structure_full_cross_leg,
    active_hinges_to_cpg_network_structure_fully_connected,
    active_hinges_to_cpg_network_structure_blf,
)
from blf import get_blf_parameter_bounds, get_blf_symmetry_expansion_info
from core_centric import (
    active_hinges_to_cpg_network_structure_core_centric,
    get_core_centric_parameter_bounds,
    get_cc_sym_expansion_info,
    get_cc_sym_parameter_bounds,
)

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import seed_from_time
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from revolve2.standards import modular_robots_v1


def get_num_cpg_params(robot_name: str, no_coupling: bool = False, cross_leg_coupling: bool = False, diagonal_coupling: bool = False, full_cross_leg_coupling: bool = False, fully_connected_coupling: bool = False, blf_coupling: bool = False, blf_symmetry: bool = False, core_centric_coupling: bool = False, core_centric_symmetry: bool = False) -> int:
    """Get the number of CPG parameters for a robot.

    Args:
        robot_name: Name of the robot.
        no_coupling: If True, return only internal params (no external coupling).
        cross_leg_coupling: If True, return params for cross-leg coupling mode.
        diagonal_coupling: If True, return params for diagonal coupling mode.
        full_cross_leg_coupling: If True, return params for full cross-leg coupling mode.
        fully_connected_coupling: If True, every hinge connected to every other hinge.
        blf_coupling: If True, return params for BLF (Body/Limb Finder) coupling.
        blf_symmetry: If True, use BLF-SYM (symmetric limbs share parameters).
        core_centric_coupling: If True, use Core-Centric coupling (bio-inspired reduced network).
        core_centric_symmetry: If True, use CC-SYM (Core-Centric with symmetry).
    """
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    if core_centric_coupling or core_centric_symmetry:
        cpg_network_structure, _ = active_hinges_to_cpg_network_structure_core_centric(active_hinges, body)
        if core_centric_symmetry:
            # Return reduced param count for CC-SYM
            # num_unique_params already includes external weights
            info = get_cc_sym_expansion_info(body)
            return info["num_unique_params"]
    elif blf_coupling:
        cpg_network_structure, _ = active_hinges_to_cpg_network_structure_blf(active_hinges, body, use_symmetry=blf_symmetry)
        if blf_symmetry:
            # Return reduced param count for BLF-SYM
            info = get_blf_symmetry_expansion_info(body)
            return info["num_unique_params"]
    elif fully_connected_coupling:
        cpg_network_structure, _ = active_hinges_to_cpg_network_structure_fully_connected(active_hinges)
    elif full_cross_leg_coupling:
        cpg_network_structure, _ = active_hinges_to_cpg_network_structure_full_cross_leg(active_hinges, body)
    elif diagonal_coupling:
        cpg_network_structure, _ = active_hinges_to_cpg_network_structure_diagonal(active_hinges, body)
    elif cross_leg_coupling:
        cpg_network_structure, _ = active_hinges_to_cpg_network_structure_cross_leg(active_hinges, body)
    elif no_coupling:
        cpg_network_structure, _ = active_hinges_to_cpg_network_structure_internal_only(active_hinges)
    else:
        cpg_network_structure, _ = active_hinges_to_cpg_network_structure_neighbor(active_hinges)
    return cpg_network_structure.num_connections


def run_evolution(
    robot_name: str = None,
    contact_metric: str = None,
    lambda_penalty: float = None,
    simulation_time: int = None,
    num_generations: int = None,
    initial_std: float = None,
    param_bounds: tuple[float, float] = None,
    population_size: int = None,
    use_database: bool = None,
    database_file: str = None,
    seed: int = None,
    experiment_name: str = None,
    experiment_notes: str = None,
    num_workers: int = None,
    log_every: int = None,
    visualize_best: bool = False,
    terrain_friction: float = 1.0,
    run_num: int = None,
    fitness_formula: str = "exponential",
    results_dir: str = "results",
    no_coupling: bool = False,
    cross_leg_coupling: bool = False,
    diagonal_coupling: bool = False,
    full_cross_leg_coupling: bool = False,
    fully_connected_coupling: bool = False,
    blf_coupling: bool = False,
    blf_symmetry: bool = False,
    core_centric_coupling: bool = False,
    core_centric_symmetry: bool = False,
) -> tuple[np.ndarray, float]:
    """
    Run CMA-ES evolution to optimize CPG parameters.

    All parameters default to values from config file if not provided.

    Returns:
        Tuple of (best_params, best_fitness)
    """
    # Load defaults from config
    robot_name = robot_name or config.ROBOT_NAME
    contact_metric = contact_metric or config.CONTACT_METRIC
    lambda_penalty = lambda_penalty if lambda_penalty is not None else config.LAMBDA_PENALTY
    simulation_time = simulation_time or config.SIMULATION_TIME
    num_generations = num_generations or config.NUM_GENERATIONS
    param_bounds = param_bounds or config.PARAM_BOUNDS
    population_size = population_size or config.POPULATION_SIZE
    use_database = use_database if use_database is not None else config.USE_DATABASE
    database_file = database_file or config.DATABASE_FILE
    seed = seed if seed is not None else config.SEED
    experiment_name = experiment_name or config.EXPERIMENT_NAME
    experiment_notes = experiment_notes or config.EXPERIMENT_NOTES
    num_workers = num_workers or config.NUM_SIMULATORS
    log_every = log_every or config.LOG_EVERY

    # Handle initial_std - auto-calculate from bounds if None
    if initial_std is None:
        initial_std = config.INITIAL_STD
    if initial_std is None:
        # Auto-calculate: std = 1/4 of parameter range
        param_range = param_bounds[1] - param_bounds[0]
        if param_range <= 0:
            raise ValueError(f"Invalid param_bounds: {param_bounds}. Upper bound must be greater than lower bound.")
        initial_std = param_range / 4.0

    # Generate seed if not provided
    if seed is None:
        seed = seed_from_time() % 2**32

    # Auto-generate experiment name and notes if not provided
    if experiment_name is None:
        experiment_name = f"{robot_name}_{contact_metric}_lambda{lambda_penalty}"
    if experiment_notes is None:
        experiment_notes = (
            f"Evolving {robot_name} CPG parameters with {contact_metric} contact penalty "
            f"(lambda={lambda_penalty}, bounds={param_bounds})"
        )

    # Setup logging
    setup_logging()

    # Print configuration
    print("\n" + "=" * 60)
    print("EVOLUTION CONFIGURATION")
    print("=" * 60)
    print(f"  Robot:           {robot_name}")
    print(f"  Contact Metric:  {contact_metric}")
    print(f"  Fitness Formula: {fitness_formula}")
    print(f"  Lambda Penalty:  {lambda_penalty}")
    print(f"  Simulation Time: {simulation_time}s")
    print(f"  Generations:     {num_generations}")
    print(f"  Initial Std:     {initial_std}")
    print(f"  Param Bounds:    {param_bounds}")
    print(f"  Workers:         {num_workers}")
    print(f"  Database:        {use_database}")
    print(f"  Seed:            {seed}")
    print(f"  Friction:        {terrain_friction}")
    print(f"  No Coupling:     {no_coupling}")
    print(f"  Cross-Leg:       {cross_leg_coupling}")
    print(f"  Diagonal:        {diagonal_coupling}")
    print(f"  Full Cross-Leg:  {full_cross_leg_coupling}")
    print(f"  Fully Connected: {fully_connected_coupling}")
    print(f"  BLF Coupling:    {blf_coupling}")
    print(f"  BLF Symmetry:    {blf_symmetry}")
    print(f"  Core-Centric:    {core_centric_coupling}")
    print(f"  CC-SYM:          {core_centric_symmetry}")
    print("=" * 60 + "\n")

    # Get number of CPG parameters
    num_params = get_num_cpg_params(robot_name, no_coupling=no_coupling, cross_leg_coupling=cross_leg_coupling, diagonal_coupling=diagonal_coupling, full_cross_leg_coupling=full_cross_leg_coupling, fully_connected_coupling=fully_connected_coupling, blf_coupling=blf_coupling, blf_symmetry=blf_symmetry, core_centric_coupling=core_centric_coupling, core_centric_symmetry=core_centric_symmetry)
    coupling_mode = "CC-SYM" if core_centric_symmetry else ("core-centric" if core_centric_coupling else ("BLF-SYM" if (blf_coupling and blf_symmetry) else ("BLF" if blf_coupling else ("fully-connected" if fully_connected_coupling else ("full-cross-leg" if full_cross_leg_coupling else ("diagonal" if diagonal_coupling else ("cross-leg" if cross_leg_coupling else ("internal only" if no_coupling else "neighbor"))))))))
    print(f"Number of CPG parameters for {robot_name}: {num_params} ({coupling_mode})")

    # Initialize evaluator
    evaluator = Evaluator(
        robot_name=robot_name,
        simulation_time=simulation_time,
        contact_metric=contact_metric,
        lambda_penalty=lambda_penalty,
        num_workers=num_workers,
        terrain_friction=terrain_friction,
        fitness_formula=fitness_formula,
        warmup_time=config.WARMUP_TIME,
        no_coupling=no_coupling,
        cross_leg_coupling=cross_leg_coupling,
        diagonal_coupling=diagonal_coupling,
        full_cross_leg_coupling=full_cross_leg_coupling,
        fully_connected_coupling=fully_connected_coupling,
        blf_coupling=blf_coupling,
        blf_symmetry=blf_symmetry,
        core_centric_coupling=core_centric_coupling or core_centric_symmetry,  # CC-SYM uses CC structure
        core_centric_symmetry=core_centric_symmetry,
        param_bounds=param_bounds,
    )

    # Setup database if enabled
    dbengine = None
    experiment = None
    if use_database:
        import os

        # Format values for directory/file names
        def format_val(val):
            if val == int(val):
                return str(int(val))
            return str(val).replace(".", "_").replace("-", "neg")

        bounds_str = f"bounds{format_val(param_bounds[1])}"
        lambda_str = f"lambda{format_val(lambda_penalty)}"

        # Organized directory structure:
        # results/{robot}_{metric}_{formula}_{bounds}_{lambda}/run_{n}.sqlite
        experiment_dir = f"{results_dir}/{robot_name}_{contact_metric}_{fitness_formula}_{bounds_str}_{lambda_str}"

        if run_num is not None:
            db_filename = f"run_{run_num}.sqlite"
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            db_filename = f"seed{seed}_{timestamp}.sqlite"

        db_path = f"{experiment_dir}/{db_filename}"

        # Ensure directory exists
        os.makedirs(experiment_dir, exist_ok=True)

        print(f"Database: {db_path}")

        dbengine = open_database_sqlite(
            db_path, open_method=OpenMethod.NOT_EXISTS_AND_CREATE
        )
        Base.metadata.create_all(dbengine)

        # Save experiment configuration
        experiment = Experiment(
            name=experiment_name,
            robot_name=robot_name,
            contact_metric=contact_metric,
            lambda_penalty=lambda_penalty,
            simulation_time=simulation_time,
            num_generations=num_generations,
            initial_std=initial_std,
            param_bounds_min=param_bounds[0],
            param_bounds_max=param_bounds[1],
            terrain_friction=terrain_friction,
            rng_seed=seed,
            notes=experiment_notes,
            fitness_formula=fitness_formula,
            warmup_time=config.WARMUP_TIME,
        )
        with Session(dbengine) as session:
            session.add(experiment)
            session.commit()


    # Initialize CMA-ES
    options = cma.CMAOptions()
    options.set("seed", seed)
    if population_size:
        options.set("popsize", population_size)

    # Get parameter bounds (BLF/Core-Centric use per-parameter joint-type specific bounds)
    if core_centric_symmetry:
        body = modular_robots_v1.get(robot_name)
        lower_bounds, upper_bounds = get_cc_sym_parameter_bounds(
            body,
            external_weight_bounds=param_bounds,  # Use user-specified bounds for coupling weights
        )
        options.set("bounds", [lower_bounds, upper_bounds])
        # Start at center of per-parameter bounds
        initial_mean = [(lo + hi) / 2 for lo, hi in zip(lower_bounds, upper_bounds)]
        print(f"Using CC-SYM per-joint amplitude bounds (paper Table I)")
        print(f"  Reduced params: symmetric limbs share amplitude/offset")
        print(f"  Internal weights: joint-type specific [0, π/6] to [0, 2π/3]")
        print(f"  External weights: {param_bounds}")
    elif core_centric_coupling:
        body = modular_robots_v1.get(robot_name)
        lower_bounds, upper_bounds = get_core_centric_parameter_bounds(
            body,
            external_weight_bounds=param_bounds,  # Use user-specified bounds for coupling weights
        )
        options.set("bounds", [lower_bounds, upper_bounds])
        # Start at center of per-parameter bounds
        initial_mean = [(lo + hi) / 2 for lo, hi in zip(lower_bounds, upper_bounds)]
        print(f"Using Core-Centric per-joint amplitude bounds (paper Table I)")
        print(f"  Internal weights: joint-type specific [0, π/6] to [0, 2π/3]")
        print(f"  External weights: {param_bounds}")
    elif blf_coupling:
        body = modular_robots_v1.get(robot_name)
        lower_bounds, upper_bounds = get_blf_parameter_bounds(
            body,
            use_symmetry=blf_symmetry,
            external_weight_bounds=param_bounds,  # Use user-specified bounds for coupling weights
        )
        options.set("bounds", [lower_bounds, upper_bounds])
        # Start at center of per-parameter bounds
        initial_mean = [(lo + hi) / 2 for lo, hi in zip(lower_bounds, upper_bounds)]
        print(f"Using BLF per-joint amplitude bounds (paper Table I)")
        print(f"  Internal weights: joint-type specific [0, π/6] to [0, 2π/3]")
        print(f"  External weights: {param_bounds}")
    else:
        options.set("bounds", list(param_bounds))
        initial_mean = [0.0] * num_params  # Start at center of parameter space

    opt = cma.CMAEvolutionStrategy(initial_mean, initial_std, options)

    # Track best solution
    best_fitness = float("-inf")
    best_params = None
    best_metrics = None

    # Evolution loop
    print("\nStarting evolution...")
    start_time = time.time()

    while opt.countiter < num_generations:
        gen_start = time.time()

        # Get population from CMA-ES
        solutions = opt.ask()

        # Evaluate population
        fitnesses = evaluator.evaluate([np.array(s) for s in solutions])

        # Tell CMA-ES (negate because CMA-ES minimizes)
        opt.tell(solutions, -fitnesses)

        # Track best
        gen_best_idx = np.argmax(fitnesses)
        if fitnesses[gen_best_idx] > best_fitness:
            best_fitness = fitnesses[gen_best_idx]
            best_params = np.array(solutions[gen_best_idx])
            best_metrics = evaluator.last_metrics[gen_best_idx]

        # Get stats
        stats = evaluator.get_stats(fitnesses)

        # Log progress
        if (opt.countiter % log_every == 0) or (opt.countiter == num_generations):
            gen_time = time.time() - gen_start
            print(f"\nGeneration {opt.countiter}/{num_generations} ({gen_time:.1f}s)")
            print(f"  Fitness: mean={stats['fitness_mean']:.4f}, max={stats['fitness_max']:.4f}, best_ever={best_fitness:.4f}")
            if best_metrics:
                print(f"  Best: dist={best_metrics.distance:.3f}m, M1={best_metrics.contact_metric_1:.2%}, M2={best_metrics.contact_metric_2:.2%}")
                print(f"        CoT={best_metrics.cost_of_transport:.2f}, stability={best_metrics.stability_combined:.3f}, straightness={best_metrics.straightness:.2%}")

        # Save to database
        if use_database and dbengine and experiment:
            population = Population(
                individuals=[
                    Individual(
                        genotype=Genotype.from_parameters(np.array(params)),
                        population_index=i,
                        fitness=float(fitness),
                        distance=m.distance if m else None,
                        contact_m1=m.contact_metric_1 if m else None,
                        contact_m2=m.contact_metric_2 if m else None,
                        cost_of_transport=m.cost_of_transport if m else None,
                        stability=m.stability_combined if m else None,
                        straightness=m.straightness if m else None,
                        energy=m.total_energy if m else None,
                        path_length=m.total_path_length if m else None,
                    )
                    for i, (params, fitness, m) in enumerate(
                        zip(solutions, fitnesses, evaluator.last_metrics)
                    )
                ]
            )

            generation = Generation(
                experiment=experiment,
                population=population,
                generation_index=opt.countiter,
                fitness_mean=stats["fitness_mean"],
                fitness_std=stats["fitness_std"],
                fitness_max=stats["fitness_max"],
                fitness_min=stats["fitness_min"],
                best_distance=stats.get("best_distance"),
                best_contact_m1=stats.get("best_contact_m1"),
                best_cot=stats.get("best_cot"),
            )

            with Session(dbengine, expire_on_commit=False) as session:
                session.add(generation)
                session.commit()

    # Final summary
    total_time = time.time() - start_time
    print("\n" + "=" * 60)
    print("EVOLUTION COMPLETE")
    print("=" * 60)
    print(f"  Total Time:      {total_time:.1f}s ({total_time/60:.1f} min)")
    print(f"  Best Fitness:    {best_fitness:.4f}")
    if best_metrics:
        print(f"  Best Distance:   {best_metrics.distance:.4f}m")
        print(f"  Best Contact M1: {best_metrics.contact_metric_1:.2%}")
        print(f"  Best Contact M2: {best_metrics.contact_metric_2:.2%} (body+slip)")
        print(f"  Best CoT:        {best_metrics.cost_of_transport:.4f}")
        print(f"  Best Stability:  {best_metrics.stability_combined:.4f}")
        print(f"  Best Straightness: {best_metrics.straightness:.2%}")
    print(f"\n  Best Parameters:")
    print(f"    {best_params.tolist()}")
    print("=" * 60)

    # Visualize best robot if requested
    if visualize_best and best_params is not None:
        print("\nVisualizing best robot...")
        # For CC-SYM, we need to expand the params before visualization
        vis_params = best_params.tolist()
        if core_centric_symmetry:
            from core_centric import expand_cc_sym_params
            body = modular_robots_v1.get(robot_name)
            vis_params = expand_cc_sym_params(vis_params, body, external_weight_bounds=param_bounds)
        simulate_with_contact_detection(
            robot_name=robot_name,
            simulation_time=simulation_time,
            verbose=True,
            cpg_params=vis_params,
            headless=False,
            warmup_time=config.WARMUP_TIME,
            no_coupling=no_coupling,
            cross_leg_coupling=cross_leg_coupling,
            diagonal_coupling=diagonal_coupling,
            full_cross_leg_coupling=full_cross_leg_coupling,
            blf_coupling=blf_coupling,
            blf_symmetry=blf_symmetry,
            core_centric_coupling=core_centric_coupling or core_centric_symmetry,
        )

    return best_params, best_fitness


def main():
    """Main entry point with command line arguments."""
    parser = argparse.ArgumentParser(
        description="Evolve CPG parameters with contact penalty"
    )

    # Robot configuration
    parser.add_argument("--robot", type=str, help="Robot name (e.g., spider, gecko)")
    parser.add_argument("--contact-metric", type=str, choices=["m1", "m2"],
                        help="Contact metric to use")
    parser.add_argument("--lambda", dest="lambda_penalty", type=float,
                        help="Contact penalty weight (0-1)")

    # Simulation
    parser.add_argument("--sim-time", type=int, help="Simulation time in seconds")
    parser.add_argument("--workers", type=int, help="Number of parallel workers")

    # Evolution
    parser.add_argument("--generations", type=int, help="Number of generations")
    parser.add_argument("--population", type=int, help="Population size")
    parser.add_argument("--std", type=float, help="Initial standard deviation")
    parser.add_argument("--bounds", type=float, nargs=2, help="Parameter bounds (min max)")
    parser.add_argument("--friction", type=float, default=1.0, help="Terrain friction (default 1.0)")
    parser.add_argument("--fitness-formula", type=str, choices=["exponential", "power", "linear"],
                        default="exponential", help="Fitness formula (default: exponential)")

    # Database
    parser.add_argument("--no-database", action="store_true", help="Disable database logging")
    parser.add_argument("--database", type=str, help="Database file name")

    # CPG structure
    parser.add_argument("--no-coupling", action="store_true",
                        help="Use only internal CPG weights (no external coupling between hinges)")
    parser.add_argument("--cross-leg-coupling", action="store_true",
                        help="Use cross-leg coupling (neighbor + shoulder-to-shoulder connections)")
    parser.add_argument("--diagonal-coupling", action="store_true",
                        help="Use diagonal coupling (neighbor + true diagonal leg connections)")
    parser.add_argument("--full-cross-leg-coupling", action="store_true",
                        help="Use full cross-leg coupling (complete graph among all 4 actual leg hinges)")
    parser.add_argument("--fully-connected", action="store_true",
                        help="Use fully connected coupling (every hinge to every hinge, 36 params for spider)")
    parser.add_argument("--blf-coupling", action="store_true",
                        help="Use BLF (Body/Limb Finder) coupling from EPFL paper")
    parser.add_argument("--blf-symmetry", action="store_true",
                        help="Use BLF-SYM (symmetric limbs share parameters)")
    parser.add_argument("--core-centric", action="store_true",
                        help="Use Core-Centric coupling (bio-inspired reduced network)")
    parser.add_argument("--core-centric-sym", action="store_true",
                        help="Use CC-SYM (Core-Centric + symmetry, reduced params)")

    # Other
    parser.add_argument("--seed", type=int, help="Random seed")
    parser.add_argument("--name", type=str, help="Experiment name")
    parser.add_argument("--visualize", action="store_true", help="Visualize best robot after")
    parser.add_argument("--run-num", type=int, help="Run number (for organized output)")
    parser.add_argument("--results-dir", type=str, default="results", help="Base results directory")

    args = parser.parse_args()

    # Build kwargs from args
    kwargs = {}
    if args.robot:
        kwargs["robot_name"] = args.robot
    if args.contact_metric:
        kwargs["contact_metric"] = args.contact_metric
    if args.lambda_penalty is not None:
        kwargs["lambda_penalty"] = args.lambda_penalty
    if args.sim_time:
        kwargs["simulation_time"] = args.sim_time
    if args.workers:
        kwargs["num_workers"] = args.workers
    if args.generations:
        kwargs["num_generations"] = args.generations
    if args.population:
        kwargs["population_size"] = args.population
    if args.std:
        kwargs["initial_std"] = args.std
    if args.bounds:
        kwargs["param_bounds"] = tuple(args.bounds)
    if args.no_database:
        kwargs["use_database"] = False
    if args.database:
        kwargs["database_file"] = args.database
    if args.seed is not None:
        kwargs["seed"] = args.seed
    if args.name:
        kwargs["experiment_name"] = args.name
    if args.visualize:
        kwargs["visualize_best"] = True
    if args.friction is not None:
        kwargs["terrain_friction"] = args.friction
    if args.run_num:
        kwargs["run_num"] = args.run_num
    if args.fitness_formula:
        kwargs["fitness_formula"] = args.fitness_formula
    if args.results_dir:
        kwargs["results_dir"] = args.results_dir
    if args.no_coupling:
        kwargs["no_coupling"] = True
    if args.cross_leg_coupling:
        kwargs["cross_leg_coupling"] = True
    if args.diagonal_coupling:
        kwargs["diagonal_coupling"] = True
    if args.full_cross_leg_coupling:
        kwargs["full_cross_leg_coupling"] = True
    if args.fully_connected:
        kwargs["fully_connected_coupling"] = True
    if args.blf_coupling:
        kwargs["blf_coupling"] = True
    if args.blf_symmetry:
        kwargs["blf_symmetry"] = True
    if args.core_centric:
        kwargs["core_centric_coupling"] = True
    if args.core_centric_sym:
        kwargs["core_centric_symmetry"] = True

    run_evolution(**kwargs)


if __name__ == "__main__":
    main()
