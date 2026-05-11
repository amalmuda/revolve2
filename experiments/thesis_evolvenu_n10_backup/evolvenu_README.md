# thesis_evolvenu bundle (n=10)

In-distribution comparison of two CPG controllers across 6 morphologies on flat and rugged terrain, with **per-joint natural frequency evolved** (νᵢ ∈ [0.2, 1.0] Hz). Mirrors the thesis_terrain bundle structure but with per-joint ν instead of fixed ν=0.5.

## Design
- **Robots (6):** spider, gecko, babya, queen, insect, ege
- **Controllers (2):** phi/uncoupled +ν (initial-phase evolved + per-joint ν, no coupling) vs base/BLF +ν (BLF coupling + per-joint ν, no evolved init phase)
- **Terrains (2):** flat, rugged (Perlin heightmap, max h=5cm)
- **Seeds:** 10 per cell (6 × 2 × 2 × 10 = 240 evaluations)
- **νᵢ bounds:** [0.2, 1.0] Hz (centred on the original fixed ν=0.5)
- **Eval simulation:** 30 s, control freq 20 Hz, w=1.0
- **Fitness:** distance only (lambda=0); drag measured post-hoc

## Files
| file | description |
|---|---|
| evolvenu_per_seed.csv | 240 rows: robot, variant, coupling, terrain, seed, distance (m), em (0-1), drag_pct (0-100) |
| evolvenu_summary.csv | 24 rows: per-cell stats |
| evolvenu_convergence.csv | per-generation aggregates (median, IQR, mean, std, n) |
| evolvenu_distance_boxplots.pdf | 2x6 panels, distance |
| evolvenu_em_boxplots.pdf | 2x6 panels, EM |
| evolvenu_dragging_boxplots.pdf | 2x6 panels, drag % |
| evolvenu_convergence.pdf | best-fitness curves with std band |

## Headline finding
- BLF+ν benefits massively from evolving frequency (+65 to +120% distance vs fixed ν).
- phi+ν barely benefits or hurts.
- The phi-vs-BLF comparison REVERSES: BLF+ν beats phi+ν in nearly every cell.
