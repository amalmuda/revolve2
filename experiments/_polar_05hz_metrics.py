"""Get distance and dragging for completed 0.5Hz polar spider runs."""
import sys
sys.path.insert(0, ".")
from _evolve_polar_spider import simulate_one
import numpy as np

runs = [
    ("polar_spider_neighbor_xy_lam0_0p50hz_best.npy", "neighbor", False, 0),
    ("polar_spider_neighbor_xy_lam1_0p50hz_best.npy", "neighbor", False, 1),
    ("polar_spider_neighbor_directed_lam0_0p50hz_best.npy", "neighbor", True, 0),
    ("polar_spider_neighbor_directed_lam1_0p50hz_best.npy", "neighbor", True, 1),
    ("polar_spider_blf_xy_lam0_0p50hz_best.npy", "blf", False, 0),
]

# Note: simulate_one in current script uses OMEGA from env via HZ env var.
# Need to set HZ=0.5 in env before running this script.
import os
os.environ["HZ"] = "0.5"
import importlib, _evolve_polar_spider as evp
importlib.reload(evp)

print("{:<55}{:<10}{:<8}{}".format("Run", "Distance", "Drag", "Fitness"))
print("-" * 85)
for name, coupling, directed, lam in runs:
    p = np.load(name)
    f, d, dr = evp.simulate_one(p, "spider", coupling, 20.0, directed, float(lam))
    print("{:<55}{:<10.3f}{:<8.2%}{:.3f}".format(name, d, dr, f))
