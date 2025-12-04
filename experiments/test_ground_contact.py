"""Test script to verify ground contact detection."""

import config
import numpy as np
from evaluator import Evaluator

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)

# Find all active hinges in the body
active_hinges = config.BODY.find_modules_of_type(ActiveHinge)

# Create a structure for the CPG network from these hinges.
(
    cpg_network_structure,
    output_mapping,
) = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

# Create the evaluator.
evaluator = Evaluator(
    headless=True,
    num_simulators=1,
    cpg_network_structure=cpg_network_structure,
    body=config.BODY,
    output_mapping=output_mapping,
)

# Test with a single random solution
test_params = np.random.uniform(-1.0, 1.0, cpg_network_structure.num_connections)

# Evaluate with contact information
fitnesses, contact_info = evaluator.evaluate_with_contact_info([test_params])

print(f"Fitness (xy displacement): {fitnesses[0]:.4f}")
print(f"\nGround Contact Information:")
print(f"  Total sampled states: {contact_info[0]['total_states']}")
print(f"  Average core height: {contact_info[0]['avg_core_height']:.4f} m")
print(f"  Min core height: {contact_info[0]['min_core_height']:.4f} m")
print(f"  Max core height: {contact_info[0]['max_core_height']:.4f} m")
print(f"  States in contact (core < 0.05m): {contact_info[0]['states_in_contact']}")
print(f"  Contact ratio: {contact_info[0]['contact_ratio']:.2%}")
