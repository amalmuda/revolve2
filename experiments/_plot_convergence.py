"""Plot convergence curves: best_ever_fitness vs generation.

One page per fitness function (lambda=0, lambda=1), 6 robot panels each.
Lines: condition (6). Shaded: +/- std across seeds.
"""
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

CSV = "/tmp/convergence.csv"
OUT = os.path.expanduser("~/convergence.pdf")

if not os.path.exists(CSV):
    raise FileNotFoundError(CSV)

ROBOT_ORDER = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
ROBOT_LABEL = {
    "spider": "spider", "gecko": "gecko", "babya": "babya",
    "queen": "queen", "insect": "insect", "ege2": "custom robot",
}
COND_ORDER = [
    ("base", "uncoupled"), ("base", "neighbor"), ("base", "blf"),
    ("phi",  "uncoupled"), ("phi",  "neighbor"), ("phi",  "blf"),
]
COND_LABEL = {
    ("base", "uncoupled"): "base/unc",
    ("base", "neighbor"):  "base/neigh",
    ("base", "blf"):       "base/blf",
    ("phi",  "uncoupled"): "phi/unc",
    ("phi",  "neighbor"):  "phi/neigh",
    ("phi",  "blf"):       "phi/blf",
}
COND_COLOR = {
    ("base", "uncoupled"): "#6baed6",
    ("base", "neighbor"):  "#3182bd",
    ("base", "blf"):       "#08519c",
    ("phi",  "uncoupled"): "#fdae6b",
    ("phi",  "neighbor"):  "#e6550d",
    ("phi",  "blf"):       "#a63603",
}

df = pd.read_csv(CSV)
print("loaded", len(df), "rows; robots:", df.robot.unique())


def plot_one_lambda(pdf, lam):
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), sharex=True)
    axes = axes.flatten()
    for i, robot in enumerate(ROBOT_ORDER):
        ax = axes[i]
        for v, c in COND_ORDER:
            sub = df[(df.robot == robot) & (df.variant == v)
                     & (df.coupling == c) & (df["lambda"] == lam)]
            if sub.empty:
                continue
            agg = sub.groupby("generation")["best_ever_fitness"].agg(["mean", "std"]).reset_index()
            color = COND_COLOR[(v, c)]
            ax.plot(agg["generation"], agg["mean"], label=COND_LABEL[(v, c)],
                    color=color, lw=1.4)
            ax.fill_between(agg["generation"], agg["mean"] - agg["std"],
                            agg["mean"] + agg["std"], color=color, alpha=0.15)
        ax.set_title(ROBOT_LABEL[robot], fontsize=12, fontweight="bold")
        ax.grid(alpha=0.3)
        if i % 3 == 0:
            ax.set_ylabel("Best fitness so far", fontsize=10)
        if i >= 3:
            ax.set_xlabel("Generation", fontsize=10)
        if i == 0:
            ax.legend(fontsize=8, loc="lower right", ncol=2, framealpha=0.9)
    fig.suptitle(f"Convergence (best ever fitness vs generation), lambda={lam}",
                 fontsize=14, fontweight="bold", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


with PdfPages(OUT) as pdf:
    plot_one_lambda(pdf, 0)
    plot_one_lambda(pdf, 1)

print(f"wrote {OUT}")
