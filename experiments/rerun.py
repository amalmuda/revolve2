"""Rerun a robot with given body and parameters."""

import config
import numpy as np
from evaluator import Evaluator

from revolve2.experimentation.logging import setup_logging
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)

# These are set of parameters that we optimized using CMA-ES.
# You can copy your own parameters from the optimization output log.
PARAMS = np.array(
      [0.62248148, -0.95225013, -0.80867508,  0.85181333,  0.99094499,
       -0.98966582, -0.80806261,  0.76988206,  0.79472574, -0.63070308,
        0.99524088, -0.60691992]
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

    # Create the evaluator.
    evaluator = Evaluator(
        headless=False,
        num_simulators=1,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
        output_mapping=output_mapping,
    )

    # Show the robot.
    evaluator.evaluate([PARAMS])


if __name__ == "__main__":
    main()