"""Test contact detection with visualization."""

import config
import numpy as np
from evaluator_with_contacts_viz import EvaluatorWithContactsViz

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
)

# Setup
active_hinges = config.BODY.find_modules_of_type(ActiveHinge)
cpg_network_structure, output_mapping = active_hinges_to_cpg_network_structure_neighbor(
    active_hinges
)

evaluator = EvaluatorWithContactsViz(
    cpg_network_structure=cpg_network_structure,
    body=config.BODY,
    output_mapping=output_mapping,
)

# Optimized parameters from rerun.py
test_params = np.array(
    [0.97560332, -0.98052375, 0.70084728, -0.06435287, 0.69862624, -0.05989215, -0.99993381, 0.99979145, 0.99602341, 0.64765969, 0.72665882, -0.90461619]
)

print("Running simulation with contact detection and visualization...")

# Evaluate
fitnesses, contact_info = evaluator.evaluate_with_contacts(
    [test_params], headless=True
)

print(f"\nFitness (xy displacement): {fitnesses[0]:.4f} m")
print(f"\nTrue Ground Contact Information:")
print(f"  Total contacts: {contact_info[0]['total_contacts']}")
print(f"  Avg contacts/step: {contact_info[0]['avg_contacts_per_step']:.2f}")
print(f"  Bodies in contact: {contact_info[0]['num_bodies_with_contact']}")
print(f"  Total steps: {contact_info[0]['total_steps']}")

# Define body type mapping - each body is Core, ActiveHinge, or Brick
body_types = {
    2: ("Core", "Non-Leaf"),
    3: ("ActiveHinge", "Non-Leaf"),
    4: ("Brick", "LEAF"),
    5: ("ActiveHinge", "Non-Leaf"),
    6: ("Brick", "LEAF"),
    7: ("ActiveHinge", "Non-Leaf"),
    8: ("Brick", "LEAF"),
    9: ("ActiveHinge", "Non-Leaf"),
    10: ("Brick", "LEAF"),
}

print(f"\nDetailed Per-Body Contact Analysis:")
print("=" * 70)
print(f"{'Body ID':<10} {'Contacts':<12} {'%':<10} {'Module Type':<15} {'Leaf?':<10}")
print("=" * 70)

for body_id, count in sorted(contact_info[0]['body_contact_counts'].items()):
    ratio = contact_info[0]['body_contact_ratios'][body_id]
    if body_id in body_types:
        module_type, leaf_status = body_types[body_id]
        print(f"{body_id:<10} {count:<12} {ratio*100:6.1f}%    {module_type:<15} {leaf_status:<10}")
    else:
        print(f"{body_id:<10} {count:<12} {ratio*100:6.1f}%    UNKNOWN")

print("=" * 70)

# Summary by module type
core_bodies = [2]
hinge_bodies = [3, 5, 7, 9]
brick_bodies = [4, 6, 8, 10]

core_contacts = sum(contact_info[0]['body_contact_counts'].get(b, 0) for b in core_bodies)
hinge_contacts = sum(contact_info[0]['body_contact_counts'].get(b, 0) for b in hinge_bodies)
brick_contacts = sum(contact_info[0]['body_contact_counts'].get(b, 0) for b in brick_bodies)

print(f"\nSummary by Module Type:")
print(f"  Core contacts:         {core_contacts:6d} ({core_contacts/contact_info[0]['total_contacts']*100:5.1f}%)")
print(f"  ActiveHinge contacts:  {hinge_contacts:6d} ({hinge_contacts/contact_info[0]['total_contacts']*100:5.1f}%)")
print(f"  Brick contacts:        {brick_contacts:6d} ({brick_contacts/contact_info[0]['total_contacts']*100:5.1f}%)")

print(f"\nSummary by Leaf Status:")
leaf_contacts = brick_contacts  # All bricks are leaf nodes
non_leaf_contacts = core_contacts + hinge_contacts  # Core and hinges are non-leaf

print(f"  LEAF nodes (Bricks):     {leaf_contacts:6d} ({leaf_contacts/contact_info[0]['total_contacts']*100:5.1f}%)")
print(f"  NON-LEAF nodes:          {non_leaf_contacts:6d} ({non_leaf_contacts/contact_info[0]['total_contacts']*100:5.1f}%)")
