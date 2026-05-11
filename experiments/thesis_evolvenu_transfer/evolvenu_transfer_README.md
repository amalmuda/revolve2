# thesis_evolvenu_transfer bundle (n=30)

Cross-terrain transfer test for evolve_ν controllers. Mirrors thesis_transfer structure.

## Design
- **Robots (6):** spider, gecko, babya, queen, insect, ege
- **Controllers (2):** phi/uncoupled +ν, base/BLF +ν
- **Train terrains:** flat, rugged
- **Eval terrains:** flat, rugged
- **Seeds:** 30 per cell on every direction (1440 evaluations)
- **Source controllers:** identical to thesis_evolvenu (in-distribution rows match exactly)

## Files
- evolvenu_transfer_per_seed.csv (1440 rows)
- evolvenu_transfer_summary.csv (48 rows)
- evolvenu_transfer_distance_boxplots.pdf, evolvenu_transfer_em_boxplots.pdf, evolvenu_transfer_dragging_boxplots.pdf

## Headline finding
- BLF+ν dominates in-distribution but **transfers worse** than phi+ν, especially flat→rugged.
- The freedom that helps BLF+ν shine in-distribution is also room to overfit.
- On rugged→flat transfer, BLF+ν retains advantage; on flat→rugged it weakens.
