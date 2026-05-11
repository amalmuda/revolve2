"""Convergence plot for the 3-condition design: phi/unc, base/blf, phi/blf.

Two pages (lambda=0, lambda=1), 6 robot panels each. Lines are condition means
across 5 seeds, shaded bands are +/- std.
"""
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

CSV = os.path.expanduser("~/convergence.csv")
OUT = os.path.expanduser("~/convergence_3cond.pdf")

ROBOT_ORDER = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
ROBOT_LABEL = {
    "spider": "spider", "gecko": "gecko", "babya": "babya",
    "queen": "queen", "insect": "insect", "ege2": "custom robot",
}
CONDS = [
    ("phi",  "uncoupled", "phi/unc",  "#fdae6b"),
    ("base", "blf",       "base/blf", "#08519c"),
    ("phi",  "blf",       "phi/blf",  "#a63603"),
]

df = pd.read_csv(CSV)


def plot_one_lambda(pdf, lam):
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), sharex=True)
    axes = axes.flatten()
    for i, robot in enumerate(ROBOT_ORDER):
        ax = axes[i]
        for v, c, lbl, color in CONDS:
            sub = df[(df.robot == robot) & (df.variant == v)
                     & (df.coupling == c) & (df["lambda"] == lam)]
            if sub.empty:
                continue
            agg = sub.groupby("generation")["best_ever_fitness"].agg(["mean", "std"]).reset_index()
            ax.plot(agg["generation"], agg["mean"], label=lbl, color=color, lw=1.6)
            ax.fill_between(agg["generation"],
                            agg["mean"] - agg["std"],
                            agg["mean"] + agg["std"],
                            color=color, alpha=0.18)
        ax.set_title(ROBOT_LABEL[robot], fontsize=12, fontweight="bold")
        ax.grid(alpha=0.3)
        if i % 3 == 0:
            ax.set_ylabel("Best fitness so far", fontsize=10)
        if i >= 3:
            ax.set_xlabel("Generation", fontsize=10)
        if i == 0:
            ax.legend(fontsize=10, loc="lower right", framealpha=0.9)
    fig.suptitle(f"Convergence (3-condition design), lambda={lam}",
                 fontsize=14, fontweight="bold", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


with PdfPages(OUT) as pdf:
    plot_one_lambda(pdf, 0)
    plot_one_lambda(pdf, 1)

print(f"wrote {OUT}")
