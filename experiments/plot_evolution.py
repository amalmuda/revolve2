"""Plot evolution results from CSV files."""

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

    # Plot fitness
    axes[0].plot(generations, fitness, 'b-', linewidth=2)
    axes[0].set_xlabel('Generation')
    axes[0].set_ylabel('Fitness (m)')
    axes[0].set_title('Best Fitness Over Generations')
    axes[0].grid(True, alpha=0.3)

    # Plot distance
    axes[1].plot(generations, distance, 'g-', linewidth=2)
    axes[1].set_xlabel('Generation')
    axes[1].set_ylabel('Distance (m)')
    axes[1].set_title('Best Distance Over Generations')
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


def plot_all_runs(run_ids: list[int]) -> None:
    """Plot comparison of all runs."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 14))

    for run_id in run_ids:
        csv_file = f"evolution_run{run_id}.csv"
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
        axes[0].plot(generations, fitness, linewidth=1.5, label=label, alpha=0.7)
        axes[1].plot(generations, distance, linewidth=1.5, label=label, alpha=0.7)
        axes[2].plot(generations, contact_ratio, linewidth=1.5, label=label, alpha=0.7)

    # Configure axes
    axes[0].set_xlabel('Generation')
    axes[0].set_ylabel('Fitness (m)')
    axes[0].set_title('Best Fitness Over Generations (All Runs)')
    axes[0].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel('Generation')
    axes[1].set_ylabel('Distance (m)')
    axes[1].set_title('Best Distance Over Generations (All Runs)')
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
    # Plot all runs 1-10
    run_ids = list(range(1, 11))

    # Plot individual runs
    for run_id in run_ids:
        csv_file = f"evolution_run{run_id}.csv"
        if Path(csv_file).exists():
            plot_single_run(csv_file)

    # Plot comparison
    plot_all_runs(run_ids)

    print("Done!")