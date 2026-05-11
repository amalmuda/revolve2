# Thesis data bundle — phi/uncoupled vs base/blf

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
  fitness = distance (lambda=0; drag is descriptive only)
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
