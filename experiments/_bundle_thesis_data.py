"""Bundle thesis-ready data for the phi/unc vs base/blf comparison.

Output: ~/thesis_data/
  per_seed.csv          - one row per (robot, cell, seed): distance, dragging_pct, em
  summary.csv           - one row per cell: mean/std/median/q25/q75 for each metric
  convergence.csv       - per-generation aggregates of best_ever_fitness across 35 seeds
  README.md             - explains every file/column and the experiment setup
"""
import os, csv
import numpy as np
import pandas as pd

EM_CSV = os.path.expanduser("~/em_all.csv")
CONV_CSV = os.path.expanduser("~/convergence_all.csv")
OUT = os.path.expanduser("~/thesis_data")
os.makedirs(OUT, exist_ok=True)

ROBOT_ORDER = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
ROBOT_LABEL = {
    "spider": "spider", "gecko": "gecko", "babya": "babya",
    "queen": "queen", "insect": "insect", "ege2": "ege",
}
# Filter to just these two cells:
CELLS = [
    ("phi",  "uncoupled"),  # initial phases evolved, no coupling
    ("base", "blf"),        # BLF coupling, initial phases fixed at 0
]

em = pd.read_csv(EM_CSV)
em["dragging_pct"] = em["dragging"] * 100.0
em["robot_label"] = em["robot"].map(ROBOT_LABEL)
em = em[em.apply(lambda r: (r["variant"], r["coupling"]) in CELLS, axis=1)]
em = em.sort_values(["robot", "variant", "coupling", "lambda", "seed"])

# 1) per_seed.csv
per_seed = em[["robot_label", "variant", "coupling", "lambda", "seed",
               "distance", "dragging_pct", "em"]].rename(columns={
    "robot_label": "robot",
    "lambda": "lambda_penalty",
    "dragging_pct": "dragging_pct",
})
per_seed.to_csv(os.path.join(OUT, "per_seed.csv"), index=False, float_format="%.4f")

# 2) summary.csv
summary_rows = []
for robot in ROBOT_ORDER:
    for variant, coupling in CELLS:
        for lam in (0, 1):
            sub = em[(em.robot == robot) & (em.variant == variant)
                     & (em.coupling == coupling) & (em["lambda"] == lam)]
            if sub.empty:
                continue
            row = {
                "robot": ROBOT_LABEL[robot],
                "variant": variant,
                "coupling": coupling,
                "lambda_penalty": lam,
                "n_seeds": len(sub),
            }
            for metric in ("distance", "dragging_pct", "em"):
                vals = sub[metric]
                row[f"{metric}_mean"] = vals.mean()
                row[f"{metric}_std"]  = vals.std()
                row[f"{metric}_median"] = vals.median()
                row[f"{metric}_q25"] = vals.quantile(0.25)
                row[f"{metric}_q75"] = vals.quantile(0.75)
                row[f"{metric}_min"] = vals.min()
                row[f"{metric}_max"] = vals.max()
            summary_rows.append(row)
pd.DataFrame(summary_rows).to_csv(
    os.path.join(OUT, "summary.csv"), index=False, float_format="%.4f"
)

# 3) convergence.csv
conv = pd.read_csv(CONV_CSV)
conv = conv[conv.apply(lambda r: (r["variant"], r["coupling"]) in CELLS, axis=1)]
conv["robot_label"] = conv["robot"].map(ROBOT_LABEL)

agg = (conv.groupby(["robot_label", "variant", "coupling", "lambda", "generation"])
            ["best_ever_fitness"]
            .agg(median="median",
                 q25=lambda s: s.quantile(0.25),
                 q75=lambda s: s.quantile(0.75),
                 mean="mean",
                 std="std",
                 n="count")
            .reset_index()
            .rename(columns={"robot_label": "robot",
                             "lambda": "lambda_penalty"})
            .sort_values(["robot", "variant", "coupling", "lambda_penalty", "generation"]))
agg.to_csv(os.path.join(OUT, "convergence.csv"), index=False, float_format="%.4f")

# 4) README.md
readme = """# Thesis data bundle — phi/uncoupled vs base/blf

Two cells from the Bonardi 0.5 Hz CMA-ES sweep, all 6 morphologies, both lambda
values, n=35 evolved seeds per cell.

## Experimental setup

CPG: Bonardi-style phase-amplitude oscillators, one per active hinge.

  - dphi_i/dt  = 2*pi*nu  +  sum_j w_ij * A_j * sin(phi_j - phi_i - psi_ij)
  - theta_i(t) = A_i * sin(phi_i(t))  +  X_i

Always evolved by CMA-ES (popsize 25, sigma 0.5, 300 generations):
  - A_i  in [0, pi/3]        amplitude per oscillator
  - X_i  in [-pi/3, pi/3]    output offset per oscillator
  - psi_ij in [0, 2*pi]      phase lag per coupled edge

Variant axis (controls phi_i(0)):
  - base : phi_i(0) = 0 for all oscillators (synchronized at t=0)
  - phi  : phi_i(0) is also evolved (per-oscillator, in [0, 2*pi])

Coupling axis (which oscillator pairs are connected by w_ij = 1.0):
  - uncoupled : no edges (w_ij = 0 everywhere)
  - blf       : Body Limb Finder; couples within-limb hinges only (Bonardi 2014)

Fixed across all runs:
  - nu = 0.5 Hz       natural frequency, same for every oscillator
  - w  = 1.0          coupling weight, uniform on every connected edge
  - simulation_time   = 30 s
  - control frequency = 20 Hz
  - friction          = 1.0
  - terrain           = flat checker plane

Fitness function (lambda_penalty):
  - 0 (no penalty)    : fitness = distance
  - 1 (with penalty)  : fitness = distance * (1 - dragging_fraction)
                        where dragging_fraction is the share of timesteps
                        any non-foot module touched the ground.

## Morphologies (6)

  spider, gecko, babya  : revolve2 standard bodies (unmodified)
  queen, insect         : revolve2 standard bodies with manual hinge-rotation tweaks
                          (intended to make them physically capable of locomotion)
  ege                   : custom body designed for this study

The bodies live in revolve2/standards/revolve2/standards/modular_robots_v1.py.

## How the 35 seeds were assembled

Each seed corresponds to one independent CMA-ES run with a different random seed.

  - 5 seeds : original test batch, preserved before the full sweep was started
  - 25 seeds: fox cluster, full 300-generation sweep
  -  5 seeds: robin-hpc cluster, parallel sweep with the same code & bodies
  total 35

All three sources used the same modular_robots_v1.py with the modified queen
and insect bodies (verified by file modtime comparison).

## Files

per_seed.csv
  Columns: robot, variant, coupling, lambda_penalty, seed, distance, dragging_pct, em
  Rows: 6 robots * 2 cells * 2 lambdas * 35 seeds = 840
  Usage: boxplots, statistical tests, scatter plots.

summary.csv
  Columns: robot, variant, coupling, lambda_penalty, n_seeds,
           {distance,dragging_pct,em}_{mean,std,median,q25,q75,min,max}
  Rows: 6 robots * 2 cells * 2 lambdas = 24
  Usage: tables, headline numbers.

convergence.csv
  Columns: robot, variant, coupling, lambda_penalty, generation,
           median, q25, q75, mean, std, n
  Rows: 6 robots * 2 cells * 2 lambdas * 300 generations = 7200
  Usage: convergence plots (median line + IQR shaded band).
  best_ever_fitness is the running best fitness up to each generation.

## Metrics defined

  distance     : Euclidean displacement (m) of the core link between t=0 and t=30s
  dragging_pct : percentage of simulation timesteps where any non-foot module
                 touches the ground (0-100 %)
  em           : effective movement = displacement / total_path_length of the
                 core link; 1.0 = walks straight, 0 = wobbles in place
"""
with open(os.path.join(OUT, "README.md"), "w") as f:
    f.write(readme)

print(f"Wrote bundle to: {OUT}")
print(f"  per_seed.csv:    {len(per_seed)} rows")
print(f"  summary.csv:     {len(summary_rows)} rows")
print(f"  convergence.csv: {len(agg)} rows")
print(f"  README.md")
