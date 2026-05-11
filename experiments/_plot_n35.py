"""Final thesis plots for the 35-seed dataset.

Output: ~/results_n35.pdf with 5 pages (distance/drag/EM boxplots, then convergence lambda=0/1).

Each page has:
  - Bold title at top (e.g., "Distance")
  - Small italic info line(s) explaining what's evolved/fixed and what lambda means
  - Shared legend
  - 6 robot panels in 2x3 grid

Boxplots:
  - x-axis: 6 conditions, base block then phi block, dashed separator
  - hue: lambda=0 vs lambda=1
  - y-axis: metric (forced to 0 for drag)

Convergence:
  - x: generation
  - y: best fitness so far
  - line: median across 35 seeds
  - shaded band: IQR (25-75 percentile)
  - 6 lines per panel, paired colors by coupling
"""
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import seaborn as sns

EM_CSV = os.path.expanduser("~/em_all.csv")
CONV_CSV = os.path.expanduser("~/convergence_all.csv")
OUT = os.path.expanduser("~/results_n35.pdf")

ROBOT_ORDER = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
ROBOT_LABEL = {
    "spider": "spider", "gecko": "gecko", "babya": "babya",
    "queen": "queen", "insect": "insect", "ege2": "ege",
}
COND_ORDER = [
    "base|uncoupled", "base|neighbor", "base|blf",
    "phi|uncoupled",  "phi|neighbor",  "phi|blf",
]
COND_LABEL = {
    "base|uncoupled": "base/unc",
    "base|neighbor":  "base/neigh",
    "base|blf":       "base/blf",
    "phi|uncoupled":  "phi/unc",
    "phi|neighbor":   "phi/neigh",
    "phi|blf":        "phi/blf",
}
COND_COLOR = {
    "base|uncoupled": "#a6cee3",
    "phi|uncoupled":  "#1f78b4",
    "base|neighbor":  "#fdbf6f",
    "phi|neighbor":   "#ff7f00",
    "base|blf":       "#b2df8a",
    "phi|blf":        "#33a02c",
}

L0_COLOR = "#9ecae1"
L1_COLOR = "#08519c"

INFO_LINE_1 = (
    r"$\bf{Always\ evolved}$: $A$ (amplitude), $X$ (offset), $\psi_{ij}$ (phase lag per coupled pair).   "
    r"$\bf{base}$: $\phi_i(0)=0$ for all oscillators (start synchronized).   "
    r"$\bf{phi}$: also evolves $\phi_i(0)$.   "
    r"$\bf{Fixed}$: $\nu = 0.5$ Hz (natural frequency, same for every joint), $w = 1.0$ (coupling weight, same for every coupled pair)."
)
INFO_LINE_2 = (
    r"$\bf{Couplings}$: $\bf{unc}$ (no edges), $\bf{neigh}$ (adjacent hinges), $\bf{blf}$ (within-limb).   "
    r"$\bf{Fitness\ function}$: $\bf{no\ penalty} \Rightarrow$ fitness = distance; "
    r"$\bf{with\ penalty} \Rightarrow$ fitness = distance $\times$ (1 - dragging).   "
    r"$\bf{n=35}$ evolved seeds per cell."
)


def add_header(fig, title, info_lines, lam_tag=None):
    """Add bold title + italic info text + reserve top space for legend."""
    fig.suptitle(title, fontsize=22, fontweight="bold", y=0.995, x=0.02, ha="left")
    if lam_tag is not None:
        fig.text(0.98, 0.995, lam_tag, fontsize=20, fontweight="bold",
                 ha="right", va="top")
    for k, line in enumerate(info_lines):
        fig.text(0.02, 0.965 - k * 0.018, line, fontsize=11, style="italic",
                 ha="left", va="top")


def make_boxplot_page(pdf, df, metric, ylabel, title, force_zero=False):
    sns.set_theme(style="whitegrid", context="paper")
    fig, axes = plt.subplots(2, 3, figsize=(22, 14))
    axes = axes.flatten()
    for i, robot in enumerate(ROBOT_ORDER):
        ax = axes[i]
        sub = df[df.robot == robot]
        sns.boxplot(
            data=sub, x="condition", y=metric, hue="lambda_lbl",
            order=COND_ORDER, hue_order=["lambda=0", "lambda=1"],
            ax=ax, fliersize=2, linewidth=1.0, width=0.7,
            palette={"lambda=0": L0_COLOR, "lambda=1": L1_COLOR},
        )
        ax.axvline(2.5, color="#666", linestyle="--", linewidth=1.0, alpha=0.7, zorder=0)
        ax.set_xticks(range(len(COND_ORDER)))
        ax.set_xticklabels([COND_LABEL[c] for c in COND_ORDER],
                           rotation=20, ha="right", fontsize=12)
        ax.set_xlabel("")
        ax.set_ylabel(ylabel if i % 3 == 0 else "", fontsize=14)
        ax.set_title(ROBOT_LABEL[robot], fontsize=18, fontweight="bold")
        ax.tick_params(axis="y", labelsize=12)
        leg = ax.get_legend()
        if leg is not None:
            leg.remove()
        if force_zero:
            ax.set_ylim(bottom=0)

    add_header(fig, title, [INFO_LINE_1, INFO_LINE_2])
    handles = [
        plt.Rectangle((0, 0), 1, 1, color=L0_COLOR, ec="#333"),
        plt.Rectangle((0, 0), 1, 1, color=L1_COLOR, ec="#333"),
    ]
    fig.legend(handles, ["no penalty (distance only)",
                         r"with penalty (distance $\times$ (1 - drag))"],
               loc="upper center", ncol=2, fontsize=14, frameon=False,
               bbox_to_anchor=(0.5, 0.92))
    fig.tight_layout(rect=[0, 0, 1, 0.89])
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def make_convergence_page(pdf, conv_df, lam):
    sns.set_theme(style="whitegrid", context="paper")
    fig, axes = plt.subplots(2, 3, figsize=(22, 14), sharex=True)
    axes = axes.flatten()
    for i, robot in enumerate(ROBOT_ORDER):
        ax = axes[i]
        for v_c in COND_ORDER:
            v, c = v_c.split("|")
            sub = conv_df[(conv_df.robot == robot)
                          & (conv_df.variant == v)
                          & (conv_df.coupling == c)
                          & (conv_df["lambda"] == lam)]
            if sub.empty:
                continue
            agg = (sub.groupby("generation")["best_ever_fitness"]
                      .agg(median="median",
                           q25=lambda s: s.quantile(0.25),
                           q75=lambda s: s.quantile(0.75))
                      .reset_index())
            color = COND_COLOR[v_c]
            ax.plot(agg["generation"], agg["median"], lw=2.0, color=color,
                    label=COND_LABEL[v_c])
            ax.fill_between(agg["generation"], agg["q25"], agg["q75"],
                            color=color, alpha=0.18)
        ax.set_title(ROBOT_LABEL[robot], fontsize=18, fontweight="bold")
        ax.set_ylabel("Best fitness so far" if i % 3 == 0 else "", fontsize=14)
        if i >= 3:
            ax.set_xlabel("Generation", fontsize=14)
        ax.tick_params(labelsize=12)
        ax.grid(alpha=0.3)

    title = "Convergence (best fitness so far across generations)"
    lam_label = "no penalty (distance only)" if lam == 0 else r"with penalty (distance $\times$ (1 - drag))"
    add_header(fig, title, [
        INFO_LINE_1,
        r"Line = median across 35 seeds; shaded band = IQR (25-75 percentile). "
        r"$\bf{Fitness\ function}$: $\bf{no\ penalty} \Rightarrow$ fitness = distance; "
        r"$\bf{with\ penalty} \Rightarrow$ fitness = distance $\times$ (1 - dragging).",
    ], lam_tag=lam_label)

    handles, labels = [], []
    for v_c in COND_ORDER:
        handles.append(plt.Line2D([0], [0], color=COND_COLOR[v_c], lw=3))
        labels.append(COND_LABEL[v_c])
    fig.legend(handles, labels, loc="upper center", ncol=6, fontsize=13,
               frameon=False, bbox_to_anchor=(0.5, 0.92))
    fig.tight_layout(rect=[0, 0, 1, 0.89])
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


# ---------- main ----------

em = pd.read_csv(EM_CSV)
em["dragging_pct"] = em["dragging"] * 100.0
em["condition"] = em["variant"] + "|" + em["coupling"]
em["lambda_lbl"] = em["lambda"].map({0: "lambda=0", 1: "lambda=1"})

conv = pd.read_csv(CONV_CSV)

with PdfPages(OUT) as pdf:
    make_boxplot_page(pdf, em, "distance",     "Distance (m)",
                      "Distance per condition", force_zero=False)
    make_boxplot_page(pdf, em, "dragging_pct", "Dragging (%)",
                      "Dragging per condition", force_zero=True)
    make_boxplot_page(pdf, em, "em",           "Effective Movement (displacement / path)",
                      "Effective Movement per condition", force_zero=False)
    make_convergence_page(pdf, conv, 0)
    make_convergence_page(pdf, conv, 1)

print(f"wrote {OUT}")
