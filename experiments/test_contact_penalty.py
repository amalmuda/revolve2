"""Test the contact penalty fitness function."""

import numpy as np
from evaluator_contact_penalty import Evaluator
import config
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)

# Use the optimized parameters from rerun.py
PARAMS = np.array(
    [-0.00151238,  0.41833362, -0.07792383,  0.86703611,  0.78084571,
     0.20071105, -0.11151541,  0.38434698,  0.64011063, -0.76854658,
     0.03632331,  0.62366759]
)

def main() -> None:
    """Test the contact penalty evaluator."""
    print("Testing contact penalty fitness function...")
    print("Fitness = distance * (1 - non_leaf_contact_ratio)")
    print()

    # Find all active hinges in the body
    active_hinges = config.BODY.find_modules_of_type(ActiveHinge)

    # Create CPG network structure
    (
        cpg_network_structure,
        output_mapping,
    ) = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

    # Create evaluator
    evaluator = Evaluator(
        headless=True,
        num_simulators=1,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
        output_mapping=output_mapping,
    )

    # Evaluate with the optimized parameters
    print("Evaluating robot with optimized parameters...")
    fitness = evaluator.evaluate([PARAMS])

    distance = evaluator.last_distances[0]
    contact_ratio = evaluator.last_contact_ratios[0]

    print(f"\nResults:")
    print(f"  Distance traveled:               {distance:.4f} m")
    print(f"  Non-leaf contact ratio:          {contact_ratio:.1%}")
    print(f"  Fitness (with contact penalty):  {fitness[0]:.4f}")
    print(f"  Fitness multiplier:              {(1.0 - contact_ratio):.1%} (applied to distance)")
    print()
    print("Note: Lower non-leaf contact ratio = higher fitness")
    print("      Perfect walking (only leaf nodes touch) = no penalty")

if __name__ == "__main__":
    main()
