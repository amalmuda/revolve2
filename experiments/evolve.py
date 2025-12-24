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
from contact_detection import simulate_with_contact_detection

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import seed_from_time
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from revolve2.standards import modular_robots_v1


def get_num_cpg_params(robot_name: str) -> int:
    """Get the number of CPG parameters for a robot."""
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
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
    print("=" * 60 + "\n")

    # Get number of CPG parameters
    num_params = get_num_cpg_params(robot_name)
    print(f"Number of CPG parameters for {robot_name}: {num_params}")

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
    initial_mean = [0.0] * num_params  # Start at center of parameter space
    options = cma.CMAOptions()
    options.set("bounds", list(param_bounds))
    options.set("seed", seed)
    if population_size:
        options.set("popsize", population_size)

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
        simulate_with_contact_detection(
            robot_name=robot_name,
            simulation_time=simulation_time,
            verbose=True,
            cpg_params=best_params.tolist(),
            headless=False,
            warmup_time=config.WARMUP_TIME,
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

    run_evolution(**kwargs)


if __name__ == "__main__":
    main()
