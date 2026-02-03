"""Rerun a spider robot with given CPG parameters."""

import argparse
import ast
import math
import sys
from pathlib import Path

# Ensure imports work from any directory
sys.path.insert(0, str(Path(__file__).parent))

from contact_detection import simulate_with_contact_detection
from blf import expand_blf_sym_params
from core_centric import expand_cc_sym_params
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
    parser = argparse.ArgumentParser(description="Visualize robot with CPG parameters")
    parser.add_argument("--robot", type=str, default="spider", help="Robot name")
    parser.add_argument("--time", type=int, default=30, help="Simulation time in seconds")
    parser.add_argument("--params", type=str, help="CPG parameters as list (e.g. '[0.1, 0.2, ...]')")
    parser.add_argument("--fully-connected", action="store_true", help="Use fully connected coupling")
    parser.add_argument("--blf", action="store_true", help="Use BLF coupling")
    parser.add_argument("--blf-sym", action="store_true", help="Use BLF-SYM coupling (reduced params, will expand)")
    parser.add_argument("--cross-leg", action="store_true", help="Use cross-leg coupling")
    parser.add_argument("--full-cross-leg", action="store_true", help="Use full cross-leg coupling")
    parser.add_argument("--core-centric", action="store_true", help="Use Core-Centric coupling (bio-inspired)")
    parser.add_argument("--core-centric-sym", action="store_true", help="Use CC-SYM coupling (reduced params, will expand)")
    args = parser.parse_args()

    robot_name = args.robot
    sim_time = args.time

    print(f"Robot: {robot_name}")
    print(f"Simulation time: {sim_time}s")

    # Determine coupling mode
    coupling_mode = "neighbor"
    if args.fully_connected:
        coupling_mode = "fully-connected"
    elif args.core_centric_sym:
        coupling_mode = "cc-sym"
    elif args.core_centric:
        coupling_mode = "core-centric"
    elif args.blf_sym:
        coupling_mode = "blf-sym"
    elif args.blf:
        coupling_mode = "blf"
    elif args.cross_leg:
        coupling_mode = "cross-leg"
    elif args.full_cross_leg:
        coupling_mode = "full-cross-leg"
    print(f"Coupling: {coupling_mode}")
    print()

    # Parse parameters from command line or prompt
    if args.params:
        try:
            cpg_params = ast.literal_eval(args.params)
            if not isinstance(cpg_params, list):
                raise ValueError("Parameters must be a list")
        except (ValueError, SyntaxError) as e:
            print(f"Error parsing parameters: {e}")
            sys.exit(1)
    else:
        print("Enter CPG parameters (comma or space separated)")
        print("Or press Enter to use default parameters:")
        print(f"Default: {DEFAULT_PARAMS}")
        print()
        user_input = input("> ").strip()

        if not user_input:
            cpg_params = DEFAULT_PARAMS
            print("Using default parameters")
        else:
            user_input = user_input.replace(",", " ").replace("[", "").replace("]", "")
            try:
                cpg_params = [float(x) for x in user_input.split()]
            except ValueError as e:
                print(f"Error parsing parameters: {e}")
                sys.exit(1)

    print()
    print(f"Running simulation with {len(cpg_params)} parameters")

    # If using symmetry modes, expand reduced params to full params
    if args.core_centric_sym:
        body = modular_robots_v1.get(robot_name)
        cpg_params = expand_cc_sym_params(cpg_params, body, external_weight_bounds=(-1.0, 1.0))
        print(f"Expanded CC-SYM params to {len(cpg_params)} full params")
    elif args.blf_sym:
        body = modular_robots_v1.get(robot_name)
        cpg_params = expand_blf_sym_params(cpg_params, body, external_weight_bounds=(-1.0, 1.0))
        print(f"Expanded BLF-SYM params to {len(cpg_params)} full params")
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
        fully_connected_coupling=args.fully_connected,
        blf_coupling=args.blf or args.blf_sym,  # BLF-SYM uses BLF structure
        cross_leg_coupling=args.cross_leg,
        full_cross_leg_coupling=args.full_cross_leg,
        core_centric_coupling=args.core_centric or args.core_centric_sym,  # CC-SYM uses CC structure
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
