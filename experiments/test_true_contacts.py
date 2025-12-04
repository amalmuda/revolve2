"""Test script for true contact detection."""

import config
import numpy as np
from evaluator_with_true_contacts import EvaluatorWithTrueContacts

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

# Create the evaluator
evaluator = EvaluatorWithTrueContacts(
    cpg_network_structure=cpg_network_structure,
    body=config.BODY,
    output_mapping=output_mapping,
)

# Use optimized parameters from rerun.py
test_params = np.array(
    [-0.00151238,  0.41833362, -0.07792383,  0.86703611,  0.78084571,
     0.20071105, -0.11151541,  0.38434698,  0.64011063, -0.76854658,
     0.03632331,  0.62366759]
)

print("Running simulation with true contact detection...")

# Evaluate with true contact information
fitnesses, contact_info = evaluator.evaluate_with_true_contacts([test_params])

print(f"\nFitness (xy displacement): {fitnesses[0]:.4f}")
print(f"\nTrue Ground Contact Information:")
print(f"  Total contacts detected: {contact_info[0]['total_contacts']}")
print(f"  Average contacts per step: {contact_info[0]['avg_contacts_per_step']:.2f}")
print(f"  Number of robot bodies with contact: {contact_info[0]['num_bodies_with_contact']}")
print(f"  Total robot bodies: {contact_info[0]['total_steps']}")
print(f"\nPer-body contact counts:")
for body_id, count in sorted(contact_info[0]['body_contact_counts'].items()):
    ratio = contact_info[0]['body_contact_ratios'][body_id]
    print(f"    Body {body_id}: {count} contacts ({ratio:.2%} of steps)")
