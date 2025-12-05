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
PARAMS = np.array([-0.58249078,  0.92819868,  0.60360065, -0.68093017,  0.98086714,
       -0.99759271, -0.94409706,  0.94526203,  0.94590059, -0.99497549,
        0.76800099, -0.76883791]
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

    # Create contact penalty evaluators for both modes
    contact_evaluator_counting = ContactEvaluator(
        headless=True,
        num_simulators=1,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
        output_mapping=output_mapping,
        contact_mode="counting",
    )

    contact_evaluator_binary = ContactEvaluator(
        headless=True,
        num_simulators=1,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
        output_mapping=output_mapping,
        contact_mode="binary",
    )

    # Show the robot with visualization
    viz_evaluator.evaluate([PARAMS])

    # Get metrics with COUNTING mode
    fitness_counting = contact_evaluator_counting.evaluate([PARAMS])[0]
    distance_counting = contact_evaluator_counting.last_distances[0]
    contact_ratio_counting = contact_evaluator_counting.last_contact_ratios[0]

    # Get metrics with BINARY mode
    fitness_binary = contact_evaluator_binary.evaluate([PARAMS])[0]
    distance_binary = contact_evaluator_binary.last_distances[0]
    contact_ratio_binary = contact_evaluator_binary.last_contact_ratios[0]

    # Display results
    print("\n" + "="*70)
    print("EVALUATION RESULTS - COMPARISON OF CONTACT MODES")
    print("="*70)
    print(f"\n{'Metric':<35} {'COUNTING':<17} {'BINARY':<17}")
    print("-"*70)
    print(f"{'Distance traveled (m):':<35} {distance_counting:<17.4f} {distance_binary:<17.4f}")
    print(f"{'Non-leaf contact ratio:':<35} {contact_ratio_counting:<17.1%} {contact_ratio_binary:<17.1%}")
    print(f"{'Penalized fitness:':<35} {fitness_counting:<17.4f} {fitness_binary:<17.4f}")
    print(f"{'Fitness multiplier:':<35} {(1 - contact_ratio_counting**0.5)**2:<17.1%} {(1 - contact_ratio_binary**0.5)**2:<17.1%}")
    print("="*70)

    print("\nCONTACT MODE EXPLANATIONS:")
    print("-"*70)
    print("COUNTING: Ratio = (sum of non-leaf bodies touching) / (total non-leaf bodies × timesteps)")
    print("          Penalizes based on HOW MANY bodies are dragging")
    print("\nBINARY:   Ratio = (timesteps with ANY non-leaf contact) / (total timesteps)")
    print("          Penalizes based on WHETHER any body is dragging (not how many)")
    print("="*70)

    print("\nParameters used:")
    print(PARAMS)
    print("="*70)


if __name__ == "__main__":
    main()