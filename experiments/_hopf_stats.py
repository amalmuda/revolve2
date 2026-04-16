"""Show evolved Hopf parameters from local experiments."""
import os
import numpy as np

files = [
    ("hopf_spider_025hz.npy", "Spider 0.25Hz, 10 gens"),
    ("hopf_spider_025hz_100gen.npy", "Spider 0.25Hz, 100 gens"),
    ("hopf_spider_1hz_100gen.npy", "Spider 1Hz, 100 gens"),
]

# Spider with neighbor coupling: 8 oscillators + 4 couplings = 12 params
n_int = 8
n_coup = 4

print(f"{'Run':<35}{'mu_min':<8}{'mu_max':<8}{'mu_mean':<9}{'cpl[0]':<9}{'cpl[1]':<9}{'cpl[2]':<9}{'cpl[3]':<9}")
print("-" * 95)

for fname, label in files:
    path = os.path.expanduser(f"~/masteroppgave/revolve2/experiments/{fname}")
    if not os.path.exists(path):
        print(f"{label:<35} (file missing)")
        continue
    p = np.load(path)
    mus = p[:n_int]
    coups = p[n_int:n_int + n_coup]
    print(f"{label:<35}{mus.min():<8.3f}{mus.max():<8.3f}{mus.mean():<9.3f}", end="")
    for c in coups:
        print(f"{c:<9.3f}", end="")
    print()

# Detailed view of the best one
print("\n\nDetailed: Spider 1Hz 100 gens")
print("-" * 50)
p = np.load(os.path.expanduser("~/masteroppgave/revolve2/experiments/hopf_spider_1hz_100gen.npy"))
print("mu (amplitude^2) per oscillator:")
for i, m in enumerate(p[:n_int]):
    amp = float(np.sqrt(max(0, m)))
    print(f"  osc {i}: mu={m:.3f}  -> amplitude={amp:.3f}")

print("\ncoupling weights (neighbor pairs):")
for i, c in enumerate(p[n_int:n_int + n_coup]):
    print(f"  pair {i}: {c:.3f}")

print(f"\ncoupling stats:")
print(f"  mean: {np.mean(p[n_int:]):.3f}")
print(f"  abs mean: {np.mean(np.abs(p[n_int:])):.3f}")
print(f"  std: {np.std(p[n_int:]):.3f}")
