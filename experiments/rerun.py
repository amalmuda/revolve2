"""Rerun a robot with given body and parameters."""

import config
import numpy as np
from evaluator import Evaluator as VisualizationEvaluator
from evaluator_contact_penalty import Evaluator as ContactEvaluator

from revolve2.experimentation.logging import setup_logging
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)

# These are set of parameters that we optimized using CMA-ES.
# You can copy your own parameters from the optimization output log.
PARAMS = np.array(
      [ 2.93062479,  0.02055893, -0.03104385,  2.98877133,  3.95851415,
       -2.96959369, -3.86391343,  3.8630212 ,  3.99054494, -3.46590107,
       -3.30362038,  1.9740831 ]
)


def main() -> None:
    """Perform the rerun."""
    setup_logging()

    # Find all active hinges in the body
    active_hinges = config.BODY.find_modules_of_type(ActiveHinge)

    # Create a structure for the CPG network from these hinges.
    # This also returns a mapping between active hinges and the index of there corresponding cpg in the network.
    (
        cpg_network_structure,
        output_mapping,
    ) = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

    # Create visualization evaluator
    viz_evaluator = VisualizationEvaluator(
        headless=False,
        num_simulators=1,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
        output_mapping=output_mapping,
    )

    # Create contact penalty evaluator for metrics
    contact_evaluator = ContactEvaluator(
        headless=True,
        num_simulators=1,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
        output_mapping=output_mapping,
    )

    # Show the robot with visualization
    viz_evaluator.evaluate([PARAMS])

    # Get metrics with contact penalty evaluator
    fitness_values = contact_evaluator.evaluate([PARAMS])
    fitness = fitness_values[0]
    distance = contact_evaluator.last_distances[0]
    contact_ratio = contact_evaluator.last_contact_ratios[0]

    # Display results
    print("\n" + "="*50)
    print("EVALUATION RESULTS")
    print("="*50)
    print(f"Distance traveled:          {distance:.4f} m")
    print(f"Non-leaf contact ratio:     {contact_ratio:.1%}")
    print(f"Penalized fitness:          {fitness:.4f}")
    print(f"Fitness multiplier:         {(1 - contact_ratio):.1%} (applied to distance)")
    print("="*50)
    print("\nParameters used:")
    print(PARAMS)
    print("="*50)


if __name__ == "__main__":
    main()