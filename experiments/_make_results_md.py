"""Generate a markdown with each individual run's distance/drag/EM."""
import os
import pandas as pd

CSV = os.path.expanduser("~/em_full.csv")
OUT = os.path.expanduser("~/masteroppgave/revolve2/experiments/results_summary.md")

ROBOT_LABEL = {
    "spider": "spider", "gecko": "gecko", "babya": "babya",
    "queen": "queen", "insect": "insect", "ege2": "custom robot",
}
ROBOT_ORDER = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
CONDS = [
    ("base", "uncoupled"), ("base", "neighbor"), ("base", "blf"),
]

df = pd.read_csv(CSV)
df["dragging_pct"] = df["dragging"] * 100.0

with open(OUT, "w") as f:
    f.write("# Bonardi 0.5 Hz sweep — per-run results\n\n")
    f.write("All 5 seeds for each cell. 6 morphologies x 3 couplings x 2 fitness functions = 36 cells = 180 runs.\n\n")
    f.write("Columns: seed, distance (m), drag (%), EM (displacement/path).\n\n")

    for robot in ROBOT_ORDER:
        f.write(f"## {ROBOT_LABEL[robot]}\n\n")
        for variant, coupling in CONDS:
            for lam in (0, 1):
                sub = df[(df.robot == robot) & (df.variant == variant)
                         & (df.coupling == coupling) & (df["lambda"] == lam)
                         ].sort_values("seed")
                if sub.empty:
                    continue
                f.write(f"### {variant}/{coupling}, lambda={lam}\n\n")
                f.write("| seed | distance (m) | drag (%) | EM    |\n")
                f.write("|------|--------------|----------|-------|\n")
                for _, r in sub.iterrows():
                    f.write(f"| {int(r['seed'])}    | {r['distance']:.2f}         | {r['dragging_pct']:.1f}     | {r['em']:.3f} |\n")
                f.write("\n")
        f.write("\n")

print(f"wrote {OUT}")
