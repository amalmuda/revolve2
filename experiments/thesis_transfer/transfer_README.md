# thesis_transfer bundle (30/30 symmetric)

Cross-terrain transfer test: controllers evolved on one terrain re-evaluated on the other, compared against the in-distribution baseline.

## Design
- **Robots (6):** spider, gecko, babya, queen, insect, ege
- **Controllers (2):** phi/uncoupled, base/BLF
- **Train terrains:** flat, rugged
- **Eval terrains:** flat, rugged
- **Seeds:** 30 per cell on every direction (6 robots x 2 controllers x 4 directions x 30 = 1440 evaluations)
- **Eval simulation:** 30 s, control freq 20 Hz, nu=0.5 Hz, w=1.0
- **Source controllers:** ~/all_runs (flat-trained) and ~/rugged_runs (rugged-trained), seeds 1-30 — same controllers as the in-distribution metrics in the thesis_terrain bundle.

## Files
| file | description |
|---|---|
| transfer_per_seed.csv | 1440 rows: robot, variant, coupling, train_terrain, eval_terrain, seed, distance, em, drag_pct, transfer (bool) |
| transfer_summary.csv | 48 rows: mean / std / median per (robot, controller, train, eval) |
| transfer_distance_boxplots.pdf | 2 rows (controllers) x 6 cols (robots), 4 boxes per panel: flat->flat, flat->rugged, rugged->rugged, rugged->flat |
| transfer_em_boxplots.pdf | same layout, EM |
| transfer_dragging_boxplots.pdf | same layout, dragging % |

## How to read it
For each (robot, controller, eval_terrain) compare:
- **Native** controllers: trained on the same terrain they are evaluated on
- **Transferred** controllers: trained on the OTHER terrain

A controller that generalises well shows little drop from native to transferred.

