"""Analyze neighbor vs BLF coupling on spider Hopf experiments."""
import numpy as np
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import active_hinges_to_cpg_network_structure_neighbor
from revolve2.standards import modular_robots_v1
from blf import BodyLimbFinder, JointType
from contact_detection import active_hinges_to_cpg_network_structure_blf

body = modular_robots_v1.get("spider")
hinges = body.find_modules_of_type(ActiveHinge)

# Get topology info
n_cpg, _ = active_hinges_to_cpg_network_structure_neighbor(hinges)
b_cpg, _ = active_hinges_to_cpg_network_structure_blf(hinges, body)

print("Spider topology:")
print(f"  Neighbor: {n_cpg.num_cpgs} oscillators, {len(n_cpg.connections)} connections")
print(f"  BLF:      {b_cpg.num_cpgs} oscillators, {len(b_cpg.connections)} connections")
print(f"  BLF adds {len(b_cpg.connections) - len(n_cpg.connections)} extra connections")
print()

# What are BLF's extra connections?
result = BodyLimbFinder(body).analyze()
hip_indices = sorted([i for i, jt in result.articulations.items() if jt == JointType.HIP])
knee_indices = sorted([i for i, jt in result.articulations.items() if jt == JointType.KNEE])

# Map node indices to CPG indices in BLF (BLF order: spine, hip, knee)
hip_cpgs = list(range(0, len(hip_indices)))  # 0-3 for spider hips
knee_cpgs = list(range(len(hip_indices), len(hip_indices) + len(knee_indices)))  # 4-7

print("BLF connection breakdown:")
hh = hk = kk = 0
for pair in b_cpg.connections:
    lo, hi = pair.cpg_index_lowest.index, pair.cpg_index_highest.index
    if lo in hip_cpgs and hi in hip_cpgs:
        hh += 1
    elif (lo in hip_cpgs and hi in knee_cpgs) or (lo in knee_cpgs and hi in hip_cpgs):
        hk += 1
print(f"  hip-hip: {hh} (BLF only — neighbor has 0)")
print(f"  hip-knee chain: {hk} (also in neighbor, 4 of these)")
print()

# Load evolved params
print("=" * 60)
print("Evolved coupling weight analysis (Hopf cartesian, 1 Hz)")
print("=" * 60)

# Neighbor: 8 mu + 4 coupling
n_params = np.load("hopf_spider_1hz_100gen.npy")
n_couplings = n_params[8:]
print("Neighbor (4 coupling weights, all hip-knee within leg):")
for i, c in enumerate(n_couplings):
    print(f"  pair {i}: {c:.3f}")
print(f"  abs_mean: {np.mean(np.abs(n_couplings)):.3f}, std: {np.std(n_couplings):.3f}")
print()

# BLF: 8 mu + 10 coupling
b_params = np.load("hopf_spider_blf_best.npy")
b_couplings = b_params[8:]
print("BLF (10 coupling weights, 6 hip-hip + 4 hip-knee chain):")
print("  hip-hip pairs (extra in BLF):")
for i in range(6):
    print(f"    pair {i}: {b_couplings[i]:.3f}")
print("  hip-knee chain:")
for i in range(6, 10):
    print(f"    pair {i}: {b_couplings[i]:.3f}")
print(f"  abs_mean (all): {np.mean(np.abs(b_couplings)):.3f}")
print(f"  abs_mean (hip-hip only): {np.mean(np.abs(b_couplings[:6])):.3f}")
print(f"  abs_mean (hip-knee only): {np.mean(np.abs(b_couplings[6:])):.3f}")
print()

# Polar
print("=" * 60)
print("Polar Hopf coupling analysis")
print("=" * 60)
np_params = np.load("polar_spider_neighbor_xy_best.npy")
n_w = np_params[8:12]
n_phi = np_params[12:16]
print("Polar Neighbor (4 weights + 4 phase offsets):")
for i in range(4):
    print(f"  pair {i}: w={n_w[i]:+.3f}, phi={n_phi[i]:+.3f} rad ({np.degrees(n_phi[i]):+.0f} deg)")
print()

bp_params = np.load("polar_spider_blf_xy_best.npy")
bp_w = bp_params[8:18]
bp_phi = bp_params[18:28]
print("Polar BLF (10 weights + 10 phase offsets):")
print("  hip-hip (BLF only):")
for i in range(6):
    print(f"    pair {i}: w={bp_w[i]:+.3f}, phi={bp_phi[i]:+.3f} rad ({np.degrees(bp_phi[i]):+.0f} deg)")
print("  hip-knee chain:")
for i in range(6, 10):
    print(f"    pair {i}: w={bp_w[i]:+.3f}, phi={bp_phi[i]:+.3f} rad ({np.degrees(bp_phi[i]):+.0f} deg)")
