"""Main script for the experiment."""

import csv
import logging
import os

import cma
import numpy as np
import config
from evaluator_contact_penalty import Evaluator

from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import seed_from_time
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)


def main() -> None:
    """Run the experiment."""
    # Get run ID from SLURM array task ID if available
    run_id = int(os.environ.get("SLURM_ARRAY_TASK_ID", "0"))

    # Setup logging with unique filename
    log_filename = f"log_run{run_id}.txt" if run_id > 0 else "log.txt"
    setup_logging(file_name=log_filename)

    logging.info(f"Starting run {run_id}")

    # Find all active hinges in the body
    active_hinges = config.BODY.find_modules_of_type(ActiveHinge)

    # Create a structure for the CPG network from these hinges.
    # This also returns a mapping between active hinges and the index of there corresponding cpg in the network.
    (
        cpg_network_structure,
        output_mapping,
    ) = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

    # Intialize the evaluator that will be used to evaluate robots.
    evaluator = Evaluator(
        headless=True,
        num_simulators=config.NUM_SIMULATORS,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
        output_mapping=output_mapping,
    )

    # Initial parameter values for the brain.
    initial_mean = cpg_network_structure.num_connections * [0.5]

    # We use the CMA-ES optimizer from the cma python package.
    # Initialize the cma optimizer.
    options = cma.CMAOptions()
    options.set("bounds", config.BOUNDS)
    # The cma package uses its own internal rng.
    # Instead of creating our own numpy rng, we use our seed to initialize cma.
    rng_seed = seed_from_time() % 2**32  # Cma seed must be smaller than 2**32.
    options.set("seed", rng_seed)
    opt = cma.CMAEvolutionStrategy(initial_mean, config.INITIAL_STD, options)

    generation_index = 0

    # Track evolution history for plotting
    history = {
        'generation': [],
        'best_fitness': [],
        'best_distance': [],
        'best_contact_ratio': [],
    }

    # Run cma for the defined number of generations.
    logging.info("Start optimization process.")
    while generation_index < config.NUM_GENERATIONS:
        logging.info(f"Generation {generation_index + 1} / {config.NUM_GENERATIONS}.")

        # Get the sampled solutions(parameters) from cma.
        solutions = opt.ask()

        # Evaluate them. Invert because fitness maximizes, but cma minimizes.
        fitnesses = -evaluator.evaluate(solutions)

        # Tell cma the fitnesses.
        opt.tell(solutions, fitnesses)

        # Find best solution in this generation
        best_idx = fitnesses.argmin()  # Min negative fitness = max actual fitness
        best_distance = evaluator.last_distances[best_idx]
        best_contact_ratio = evaluator.last_contact_ratios[best_idx]
        best_fitness = -fitnesses[best_idx]  # Convert back to positive

        # Store in history
        history['generation'].append(generation_index + 1)
        history['best_fitness'].append(best_fitness)
        history['best_distance'].append(best_distance)
        history['best_contact_ratio'].append(best_contact_ratio)

        logging.info(f"{opt.result.xbest=} {opt.result.fbest=}")
        logging.info(f"Best this gen: distance={best_distance:.4f}m, non_leaf_contact={best_contact_ratio:.1%}")

        # Increase the generation index counter.
        generation_index += 1

    # Save evolution history as CSV for plotting
    csv_filename = f"evolution_run{run_id}.csv" if run_id > 0 else "evolution.csv"
    with open(csv_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['generation', 'fitness', 'distance', 'contact_ratio'])
        for i in range(len(history['generation'])):
            writer.writerow([
                history['generation'][i],
                history['best_fitness'][i],
                history['best_distance'][i],
                history['best_contact_ratio'][i]
            ])
    logging.info(f"Evolution history saved to {csv_filename}")

    # Save final results
    results_filename = f"results_run{run_id}.npz" if run_id > 0 else "results.npz"
    np.savez(
        results_filename,
        best_params=opt.result.xbest,
        best_fitness=-opt.result.fbest,
        run_id=run_id,
        num_generations=config.NUM_GENERATIONS,
    )
    logging.info(f"Results saved to {results_filename}")
    logging.info(f"Final best fitness: {-opt.result.fbest:.4f}")


if __name__ == "__main__":
    main()
