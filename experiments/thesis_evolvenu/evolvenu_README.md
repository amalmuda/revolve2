# thesis_evolvenu bundle (n=30)

In-distribution comparison of two CPG controllers across 6 morphologies on flat and rugged terrain, with **per-joint natural frequency evolved** (νᵢ ∈ [0.2, 1.0] Hz). Mirrors the thesis_terrain bundle structure but with per-joint ν instead of fixed ν=0.5.

## Design
- **Robots (6):** spider, gecko, babya, queen, insect, ege
- **Controllers (2):** phi/uncoupled +ν vs base/BLF +ν
- **Terrains (2):** flat, rugged (Perlin heightmap, max h=5cm)
- **Seeds:** 30 per cell (720 evaluations total)
- **νᵢ bounds:** [0.2, 1.0] Hz
- **Eval simulation:** 30 s, control freq 20 Hz, w=1.0
- **Fitness:** distance only (lambda=0); drag measured post-hoc

## Files
- evolvenu_per_seed.csv (720 rows: robot, variant, coupling, terrain, seed, distance, em, drag_pct)
- evolvenu_summary.csv (24 rows: per-cell stats)
- evolvenu_convergence.csv (per-generation aggregates from training logs; sampled every 10 gens — 31 gen-points × 24 cells = 744 rows)
- evolvenu_distance_boxplots.pdf, evolvenu_em_boxplots.pdf, evolvenu_dragging_boxplots.pdf
- evolvenu_convergence.pdf

## Note on convergence resolution
Per-generation history is sampled every 10 generations (gen 1, 10, 20, ..., 300) due to limitations of the SQLite database storage on the HPC's NFS during training. The aggregated trend is reliable; only resolution is reduced.

## Headline finding
- BLF+ν benefits massively from evolving frequency (+37 to +112% distance, all p<0.0001 at n=30, δ ≈ +0.85 to +1.0).
- phi+ν barely benefits or hurts.
- **The phi-vs-BLF comparison REVERSES**: BLF+ν beats phi+ν in all 12 cells with p<0.005, gap 21–54%.
