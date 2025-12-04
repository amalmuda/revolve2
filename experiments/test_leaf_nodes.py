"""Test script to check leaf and non-leaf nodes in the robot body."""

import config
from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.base import ActiveHinge, Core

# Get the body
body = config.BODY

# Find all modules
all_modules = body.find_modules_of_type(Module)

# Find core modules
core_modules = body.find_modules_of_type(Core)

# Separate leaf and non-leaf nodes
leaf_nodes = [module for module in all_modules if len(module.children) == 0]
non_leaf_nodes = [module for module in all_modules if len(module.children) > 0]

# Also check for active hinges specifically
active_hinges = body.find_modules_of_type(ActiveHinge)
leaf_hinges = [hinge for hinge in active_hinges if len(hinge.children) == 0]
non_leaf_hinges = [hinge for hinge in active_hinges if len(hinge.children) > 0]

print(f"Robot body: {type(body).__name__}")
print(f"\nTotal modules: {len(all_modules)}")
print(f"Core modules: {len(core_modules)}")
print(f"Leaf nodes: {len(leaf_nodes)}")
print(f"Non-leaf nodes: {len(non_leaf_nodes)}")

print(f"\nActive hinges: {len(active_hinges)}")
print(f"Leaf hinges: {len(leaf_hinges)}")
print(f"Non-leaf hinges: {len(non_leaf_hinges)}")

print("\nLeaf node types:")
for node in leaf_nodes:
    print(f"  - {type(node).__name__}")

print("\nNon-leaf node types:")
for node in non_leaf_nodes:
    print(f"  - {type(node).__name__}")