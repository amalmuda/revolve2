"""Get distance and dragging for all 8 polar 0.5Hz spider runs."""
import os
os.environ["HZ"] = "0.5"
import sys
sys.path.insert(0, ".")
import importlib
import _evolve_polar_spider as evp
importlib.reload(evp)
import numpy as np

runs = [
    ("polar_spider_neighbor_xy_lam0_0p50hz_best.npy", "neighbor", False, 0),
    ("polar_spider_neighbor_xy_lam1_0p50hz_best.npy", "neighbor", False, 1),
    ("polar_spider_neighbor_directed_lam0_0p50hz_best.npy", "neighbor", True, 0),
    ("polar_spider_neighbor_directed_lam1_0p50hz_best.npy", "neighbor", True, 1),
    ("polar_spider_blf_xy_lam0_0p50hz_best.npy", "blf", False, 0),
    ("polar_spider_blf_xy_lam1_0p50hz_best.npy", "blf", False, 1),
    ("polar_spider_blf_directed_lam0_0p50hz_best.npy", "blf", True, 0),
    ("polar_spider_blf_directed_lam1_0p50hz_best.npy", "blf", True, 1),
]

print("{:<12}{:<12}{:<5}{:<10}{:<10}{}".format("Coupling", "Fitness", "Lam", "Distance", "Drag", "Fitness"))
print("-" * 65)
for name, coupling, directed, lam in runs:
    p = np.load(name)
    f, d, dr = evp.simulate_one(p, "spider", coupling, 20.0, directed, float(lam))
    fit_mode = "directed" if directed else "xy"
    print("{:<12}{:<12}{:<5}{:<10.3f}{:<10.2%}{:.3f}".format(coupling, fit_mode, lam, d, dr, f))
