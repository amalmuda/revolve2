"""Plot evolution results from CSV files."""

import argparse
import csv
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def plot_single_run(csv_file: str) -> None:
    """Plot evolution for a single run."""
    # Read CSV
    generations = []
    fitness = []
    distance = []
    contact_ratio = []

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            generations.append(int(row['generation']))
            fitness.append(float(row['fitness']))
            distance.append(float(row['distance']))
            contact_ratio.append(float(row['contact_ratio']))

    # Create figure
    fig, axes = plt.subplots(3, 1, figsize=(10, 12))

    # Plot distance (primary metric)
    axes[0].plot(generations, distance, 'g-', linewidth=2)
    axes[0].set_xlabel('Generation')
    axes[0].set_ylabel('Distance (m)')
    axes[0].set_title('Best Distance Over Generations')
    axes[0].grid(True, alpha=0.3)

    # Plot fitness (penalized)
    axes[1].plot(generations, fitness, 'b-', linewidth=2)
    axes[1].set_xlabel('Generation')
    axes[1].set_ylabel('Penalized Fitness')
    axes[1].set_title('Best Penalized Fitness Over Generations')
    axes[1].grid(True, alpha=0.3)

    # Plot contact ratio
    axes[2].plot(generations, contact_ratio, 'r-', linewidth=2)
    axes[2].set_xlabel('Generation')
    axes[2].set_ylabel('Non-Leaf Contact Ratio')
    axes[2].set_title('Non-Leaf Contact Ratio Over Generations')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim([0, 1])

    plt.tight_layout()
    plt.savefig(csv_file.replace('.csv', '_plot.png'), dpi=150)
    print(f"Saved plot to {csv_file.replace('.csv', '_plot.png')}")
    plt.close()


def plot_all_runs(run_ids: list[int], experiment_name: str = "contact_penalty") -> None:
    """Plot comparison of all runs."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 14))

    for run_id in run_ids:
        csv_file = f"results/{experiment_name}/evolution_run{run_id}.csv"
        if not Path(csv_file).exists():
            print(f"Warning: {csv_file} not found, skipping")
            continue

        # Read CSV
        generations = []
        fitness = []
        distance = []
        contact_ratio = []

        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                generations.append(int(row['generation']))
                fitness.append(float(row['fitness']))
                distance.append(float(row['distance']))
                contact_ratio.append(float(row['contact_ratio']))

        # Plot
        label = f"Run {run_id}"
        axes[0].plot(generations, distance, linewidth=1.5, label=label, alpha=0.7)
        axes[1].plot(generations, fitness, linewidth=1.5, label=label, alpha=0.7)
        axes[2].plot(generations, contact_ratio, linewidth=1.5, label=label, alpha=0.7)

    # Configure axes
    axes[0].set_xlabel('Generation')
    axes[0].set_ylabel('Distance (m)')
    axes[0].set_title('Best Distance Over Generations (All Runs)')
    axes[0].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel('Generation')
    axes[1].set_ylabel('Penalized Fitness')
    axes[1].set_title('Best Penalized Fitness Over Generations (All Runs)')
    axes[1].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    axes[1].grid(True, alpha=0.3)

    axes[2].set_xlabel('Generation')
    axes[2].set_ylabel('Non-Leaf Contact Ratio')
    axes[2].set_title('Non-Leaf Contact Ratio Over Generations (All Runs)')
    axes[2].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim([0, 1])

    plt.tight_layout()
    plt.savefig('evolution_all_runs.png', dpi=150, bbox_inches='tight')
    print(f"Saved comparison plot to evolution_all_runs.png")
    plt.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot evolution results from CSV files")
    parser.add_argument(
        "--experiment",
        "-e",
        type=str,
        default="contact_penalty",
        help="Experiment name (default: contact_penalty)",
    )
    parser.add_argument(
        "--runs",
        "-r",
        type=str,
        default="1-10",
        help="Run IDs to plot, e.g., '1-10' or '1,3,5' (default: 1-10)",
    )
    args = parser.parse_args()

    experiment_name = args.experiment

    # Parse run IDs
    if "-" in args.runs:
        start, end = map(int, args.runs.split("-"))
        run_ids = list(range(start, end + 1))
    else:
        run_ids = [int(x) for x in args.runs.split(",")]

    # Find all result directories
    results_base = Path("results")
    if not results_base.exists():
        print("No results directory found!")
        exit(1)

    experiment_dir = results_base / experiment_name
    if not experiment_dir.exists():
        print(f"Experiment directory not found: {experiment_dir}")
        exit(1)

    print(f"Plotting results for experiment: {experiment_name}")
    print(f"Run IDs: {run_ids}")

    # Plot individual runs
    for run_id in run_ids:
        csv_file = f"results/{experiment_name}/evolution_run{run_id}.csv"
        if Path(csv_file).exists():
            print(f"Plotting run {run_id}...")
            plot_single_run(csv_file)

    # Plot comparison in results directory
    print("\nCreating comparison plot...")
    plot_all_runs(run_ids, experiment_name)
    # Move comparison plot to results directory
    if Path("evolution_all_runs.png").exists():
        import shutil
        shutil.move("evolution_all_runs.png", f"results/{experiment_name}/comparison_all_runs.png")
        print(f"Saved comparison plot to results/{experiment_name}/comparison_all_runs.png")

    print("\nDone!")