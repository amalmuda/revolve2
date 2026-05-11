# thesis_terrain bundle (30/30 symmetric)

Experiment 1: in-distribution comparison of two CPG controllers across 6 morphologies on flat and rugged terrain.

## Design
- **Robots (6):** spider, gecko, babya, queen, insect, ege
- **Controllers (2):** phi/uncoupled (initial-phase evolved, no coupling) vs base/BLF (BLF coupling, no evolved initial phase)
- **Terrains (2):** flat plane, rugged Perlin-noise heightmap (max h=5cm)
- **Seeds:** 30 per cell on every terrain (6 robots x 2 controllers x 2 terrains x 30 seeds = 720 evaluations)
- **Fitness during evolution:** distance only (lambda=0)
- **Eval simulation:** 30 s, control freq 20 Hz, ν=0.5 Hz, w=1.0

## Files
| file | description |
|---|---|
| `per_seed.csv` | 720 rows: robot, variant, coupling, terrain, seed, distance (m), em (0-1), drag_pct (0-100) |
| `summary.csv` | 24 rows: mean / std / median / IQR / min / max per (robot, variant, coupling, terrain) |
| `convergence.csv` | per-generation aggregates (median, IQR, mean, std, n=30) |
| `distance_boxplots.pdf` | 2 rows (flat, rugged) x 6 cols (robots), 2 boxes per panel (controllers) |
| `em_boxplots.pdf` | same layout, EM metric |
| `dragging_boxplots.pdf` | same layout, dragging % metric |
| `convergence.pdf` | per-robot, per-terrain best-fitness curves with std band |

## Notes
- Robot label `ege2` (raw data) unified to `ege` for consistency.
- This bundle is the symmetric 30/30 replacement for the earlier 35/30 version.
