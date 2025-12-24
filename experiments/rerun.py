"""Rerun a spider robot with given CPG parameters."""

import math
import sys
from pathlib import Path

# Ensure imports work from any directory
sys.path.insert(0, str(Path(__file__).parent))

from contact_detection import simulate_with_contact_detection
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from revolve2.standards import modular_robots_v1


# Default parameters (from previous evolution)
DEFAULT_PARAMS = [
    0.97560332, -0.98052375, 0.70084728, -0.06435287,
    0.69862624, -0.05989215, -0.99993381, 0.99979145,
    0.99602341, 0.64765969, 0.72665882, -0.90461619
]


def main() -> None:
    """Run visualization with CPG parameters."""
    robot_name = "spider"
    sim_time = 30  # Match evolution config (30s, no warmup)

    # Get number of CPG parameters for spider
    body = modular_robots_v1.get(robot_name)
    active_hinges = body.find_modules_of_type(ActiveHinge)
    cpg_structure, _ = active_hinges_to_cpg_network_structure_neighbor(active_hinges)
    num_params = cpg_structure.num_connections

    print(f"Spider robot requires {num_params} CPG parameters")
    print(f"Simulation time: {sim_time}s")
    print()

    # Get parameters from user
    print("Enter CPG parameters (comma or space separated)")
    print("Or press Enter to use default parameters:")
    print(f"Default: {DEFAULT_PARAMS}")
    print()

    user_input = input("> ").strip()

    if not user_input:
        cpg_params = DEFAULT_PARAMS
        print("Using default parameters")
    else:
        # Parse input - handle both comma and space separated
        user_input = user_input.replace(",", " ")
        try:
            cpg_params = [float(x) for x in user_input.split()]
            if len(cpg_params) != num_params:
                print(f"Error: Expected {num_params} parameters, got {len(cpg_params)}")
                sys.exit(1)
        except ValueError as e:
            print(f"Error parsing parameters: {e}")
            sys.exit(1)

    print()
    print(f"Running simulation with parameters:")
    print(f"  {cpg_params}")
    print()

    # Run simulation with viewer (shadows off for better visibility)
    # Match evolution config: 30s, no warmup
    tracker, metrics = simulate_with_contact_detection(
        robot_name=robot_name,
        simulation_time=sim_time,
        verbose=True,
        cpg_params=cpg_params,
        headless=False,
        cast_shadows=False,
        warmup_time=0.0,
    )

    # Print fitness for different formulas and lambda values
    d = metrics.distance
    c = metrics.contact_metric_1

    print()
    print("=" * 60)
    print("FITNESS VALUES (M1 contact)")
    print("=" * 60)

    print("\nEXPONENTIAL: distance * exp(-λ * contact)")
    for lam in [2, 5, 8, 10]:
        fitness = d * math.exp(-lam * c)
        print(f"  λ={lam}: {fitness:.4f}")

    print("\nPOWER: distance * (1 - contact)^λ")
    for lam in [0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4]:
        fitness = d * math.pow(1 - c, lam)
        print(f"  λ={lam}: {fitness:.4f}")

    print("\nLINEAR: distance * (1 - λ * contact)")
    for lam in [0.5, 1, 1.5, 2, 2.5]:
        fitness = d * (1 - lam * c)
        print(f"  λ={lam}: {fitness:.4f}")


if __name__ == "__main__":
    main()
