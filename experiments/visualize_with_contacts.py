"""Visualize robot with contact detection - uses standard infrastructure like rerun.py."""

import config
import numpy as np
from evaluator import Evaluator

from revolve2.experimentation.logging import setup_logging
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)

# Optimized parameters from rerun.py
PARAMS = np.array(
    [-0.00151238,  0.41833362, -0.07792383,  0.86703611,  0.78084571,
     0.20071105, -0.11151541,  0.38434698,  0.64011063, -0.76854658,
     0.03632331,  0.62366759]
)


def main() -> None:
    """Run visualization with contact info."""
    setup_logging()

    # Find all active hinges in the body
    active_hinges = config.BODY.find_modules_of_type(ActiveHinge)

    # Create CPG network structure
    (
        cpg_network_structure,
        output_mapping,
    ) = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

    # Create the evaluator - EXACTLY like rerun.py
    evaluator = Evaluator(
        headless=False,  # Show visualization!
        num_simulators=1,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
        output_mapping=output_mapping,
    )

    print("Running simulation with visualization...")
    print("(Contact detection not available with standard evaluator)")
    print("Close the viewer window when done.\n")

    # Evaluate - this will show the robot
    fitness = evaluator.evaluate([PARAMS])

    print(f"\nFitness (xy displacement): {fitness[0]:.4f} m")
    print("\nFor contact detection, use evaluator_with_contacts_viz.py")
    print("which tracks:")
    print("  - Which bodies touch ground")
    print("  - How often each body is in contact")
    print("  - Total contact counts per simulation step")


if __name__ == "__main__":
    main()