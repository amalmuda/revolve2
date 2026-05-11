# thesis_evolvenu_transfer bundle (n=10)

Cross-terrain transfer test for evolve_ν controllers: each evolved on flat or rugged is re-evaluated on the opposite terrain. Mirrors the thesis_transfer bundle structure.

## Design
- **Robots (6):** spider, gecko, babya, queen, insect, ege
- **Controllers (2):** phi/uncoupled +ν, base/BLF +ν
- **Train terrains:** flat, rugged
- **Eval terrains:** flat, rugged
- **Seeds:** 10 per cell on every direction (6 × 2 × 4 × 10 = 480 evaluations)
- **Source controllers:** the same as in thesis_evolvenu (in-distribution rows are identical to thesis_evolvenu/per_seed.csv)

## Files
| file | description |
|---|---|
| evolvenu_transfer_per_seed.csv | 480 rows: robot, variant, coupling, train_terrain, eval_terrain, seed, distance, em, drag_pct, transfer |
| evolvenu_transfer_summary.csv | 48 rows: mean/std/median per (robot, controller, train, eval) |
| evolvenu_transfer_distance_boxplots.pdf | 2 rows (controllers) x 6 cols (robots), 4 boxes per panel |
| evolvenu_transfer_em_boxplots.pdf | same layout, EM |
| evolvenu_transfer_dragging_boxplots.pdf | same layout, dragging % |

## Headline finding
- BLF+ν dominates in-distribution but **transfers worse** than phi+ν, especially flat→rugged.
- The freedom that helps BLF+ν shine in-distribution is also room to overfit.
- On rugged→flat transfer, BLF+ν retains its advantage; on flat→rugged it weakens.
