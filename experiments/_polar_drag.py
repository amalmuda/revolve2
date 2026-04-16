"""Compute dragging for all 4 polar spider experiments."""
import sys
sys.path.insert(0, ".")
from _evolve_polar_spider import simulate_one
import numpy as np

runs = [
    ("polar_spider_neighbor_xy_best.npy", "neighbor", False),
    ("polar_spider_neighbor_directed_best.npy", "neighbor", True),
    ("polar_spider_blf_xy_best.npy", "blf", False),
    ("polar_spider_blf_directed_best.npy", "blf", True),
]

print("{:<45}{:<10}{}".format("Run", "Distance", "Drag"))
print("-" * 70)
for name, coupling, directed in runs:
    p = np.load(name)
    f, d, dr = simulate_one(p, "spider", coupling, 20.0, directed)
    print("{:<45}{:<10.3f}{:.2%}".format(name, d, dr))
