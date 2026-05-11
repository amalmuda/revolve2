"""Polished thesis-quality boxplots + summary tables for the 360-cell sweep.

Outputs:
  - 3 plot pages (distance, dragging, EM) with per-robot panels
  - 3 table pages (mean +/- std) one per metric

All into /tmp/results.pdf.

Robot label "ege2" is rendered as "custom robot".
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import seaborn as sns

import os
CSV = os.path.expanduser("~/em_full.csv")
OUT_PDF = os.path.expanduser("~/results.pdf")

ROBOT_ORDER = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
ROBOT_LABEL = {
    "spider": "spider",
    "gecko": "gecko",
    "babya": "babya",
    "queen": "queen",
    "insect": "insect",
    "ege2": "custom robot",
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

L0_COLOR = "#a6cee3"
L1_COLOR = "#1f78b4"
L0_DOT   = "#08519c"
L1_DOT   = "#08306b"

df = pd.read_csv(CSV)
df["dragging_pct"] = df["dragging"] * 100.0
df["condition"] = df["variant"] + "|" + df["coupling"]
df["lambda_lbl"] = df["lambda"].map({0: "lambda=0", 1: "lambda=1"})

sns.set_theme(style="whitegrid", context="paper")


def make_panel(ax, sub, metric, ylabel, force_zero=False):
    sns.boxplot(
        data=sub, x="condition", y=metric, hue="lambda_lbl",
        order=COND_ORDER, hue_order=["lambda=0", "lambda=1"],
        ax=ax, fliersize=0, linewidth=0.8,
        palette={"lambda=0": L0_COLOR, "lambda=1": L1_COLOR},
    )
    sns.stripplot(
        data=sub, x="condition", y=metric, hue="lambda_lbl",
        order=COND_ORDER, hue_order=["lambda=0", "lambda=1"],
        ax=ax, dodge=True, size=3, alpha=0.85,
        palette={"lambda=0": L0_DOT, "lambda=1": L1_DOT},
        legend=False,
    )
    # Vertical separator between base block and phi block
    ax.axvline(2.5, color="#888", linestyle="--", linewidth=0.8, alpha=0.7, zorder=0)
    ax.set_xticks(range(len(COND_ORDER)))
    ax.set_xticklabels([COND_LABEL[c] for c in COND_ORDER], rotation=25, ha="right", fontsize=9)
    ax.set_xlabel("")
    ax.set_ylabel(ylabel, fontsize=10)
    leg = ax.get_legend()
    if leg is not None:
        leg.remove()
    if force_zero:
        ax.set_ylim(bottom=0)


def plot_metric(pdf, metric, ylabel, suptitle, force_zero=False):
    fig, axes = plt.subplots(2, 3, figsize=(18, 11))
    axes = axes.flatten()
    for i, robot in enumerate(ROBOT_ORDER):
        sub = df[df["robot"] == robot]
        ax = axes[i]
        make_panel(ax, sub, metric, ylabel if (i % 3 == 0) else "", force_zero=force_zero)
        ax.set_title(ROBOT_LABEL[robot], fontsize=13, fontweight="bold")
    handles = [
        plt.Rectangle((0, 0), 1, 1, color=L0_COLOR, ec="#333"),
        plt.Rectangle((0, 0), 1, 1, color=L1_COLOR, ec="#333"),
    ]
    labels = [r"$\lambda=0$ (distance only)",
              r"$\lambda=1$ (distance $\times$ (1 - drag))"]
    fig.legend(handles, labels, loc="upper center", ncol=2, fontsize=11, frameon=False,
               bbox_to_anchor=(0.5, 0.965))
    fig.suptitle(suptitle, fontsize=14, y=0.995, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def stat_one(sub, metric, cond, lam):
    s = sub[(sub.condition == cond) & (sub["lambda"] == lam)][metric]
    if len(s) == 0:
        return np.nan, np.nan, 0
    return s.mean(), s.std() if len(s) > 1 else 0.0, len(s)


def render_table(pdf, metric, title, fmt="{:.2f}"):
    cols = []
    for cond in COND_ORDER:
        for lam in (0, 1):
            cols.append((COND_LABEL[cond], f"$\\lambda$={lam}"))
    table_data = []
    for robot in ROBOT_ORDER:
        sub = df[df["robot"] == robot]
        row = []
        for cond in COND_ORDER:
            for lam in (0, 1):
                m, s, n = stat_one(sub, metric, cond, lam)
                if np.isnan(m):
                    row.append("-")
                else:
                    row.append(f"{fmt.format(m)}\n$\\pm${fmt.format(s)}")
        table_data.append(row)

    fig, ax = plt.subplots(figsize=(20, 6))
    ax.axis("off")
    col_labels = [f"{c[0]}\n{c[1]}" for c in cols]
    row_labels = [ROBOT_LABEL[r] for r in ROBOT_ORDER]
    table = ax.table(
        cellText=table_data,
        colLabels=col_labels,
        rowLabels=row_labels,
        cellLoc="center",
        rowLoc="center",
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.5)
    table.scale(1.0, 2.4)
    for j in range(len(cols)):
        cell = table[(0, j)]
        lam = j % 2
        cell.set_facecolor(L0_COLOR if lam == 0 else L1_COLOR)
        cell.set_text_props(weight="bold", color="white" if lam == 1 else "black")
    for i in range(len(ROBOT_ORDER)):
        cell = table[(i + 1, -1)]
        cell.set_facecolor("#eee")
        cell.set_text_props(weight="bold")
    fig.suptitle(title, fontsize=14, fontweight="bold", y=0.96)
    n_min = df.groupby(["robot","condition","lambda"]).size().min()
    n_max = df.groupby(["robot","condition","lambda"]).size().max()
    n_str = f"n={n_min}-{n_max} seeds" if n_min != n_max else f"n={n_min} seeds"
    fig.text(0.5, 0.04,
             f"Cells show mean $\\pm$ std ({n_str}).",
             ha="center", fontsize=9, style="italic")
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def render_intro(pdf):
    fig = plt.figure(figsize=(11, 14))
    ax = fig.add_subplot(111)
    ax.axis("off")
    intro = r"""
$\bf{Bonardi\ CPG\ sweep:\ 6\ morphologies\ \times\ 6\ conditions\ \times\ 2\ fitness\ functions\ \times\ 5\ seeds}$

We evolve a Bonardi-style phase-amplitude CPG controller on 6 robot morphologies. Each oscillator drives one
hinge with output  $\theta_i(t) = A_i \sin(\phi_i(t)) + X_i$.  Phases evolve via:
   $\dot{\phi}_i = 2\pi\nu + \sum_j w_{ij}\,A_j\,\sin(\phi_j - \phi_i - \psi_{ij})$
with fixed $\nu = 0.5$ Hz and $w = 1$.

CMA-ES (pycma, popsize 25, sigma 0.5, 300 generations) optimises a parameter vector in the normalised box
$[-1, 1]^d$, mapped to native ranges:
   $A_i \in [0,\ \pi/3]$        amplitude per joint
   $X_i \in [-\pi/3,\ \pi/3]$   output offset per joint
   $\psi_{ij} \in [0,\ 2\pi]$    phase lag per edge
   $\phi_i(0) \in [0,\ 2\pi]$    initial phase per joint  (only when "phi" variant)

$\bf{Variant\ axis\ (rows\ in\ each\ plot):}$
  $\bf{base}$  — evolves $A$, $X$, $\psi$.  Initial phases $\phi_i(0)$ fixed at 0.
  $\bf{phi}$   — also evolves $\phi_i(0)$. One extra parameter per joint.

$\bf{Coupling\ axis\ (columns\ within\ each\ row):}$
  $\bf{uncoupled}$ — no edges; hinges never see each other ($w_{ij}=0$ everywhere).
  $\bf{neighbor}$  — edges between physically adjacent hinges (revolve2 default).
  $\bf{blf}$       — Body Limb Finder: only hinges within the same limb are coupled (Bonardi 2014).

$\bf{Fitness\ functions\ (\lambda):}$
  $\bf{\lambda=0}$  — fitness = distance.  Pure pursuit of forward displacement.
  $\bf{\lambda=1}$  — fitness = distance $\times$ (1 - dragging fraction).  Penalises body-on-ground time.

$\bf{Metrics\ (one\ page\ each):}$
  $\bf{Distance}$  (m): displacement of the core link from start to end of the 30 s simulation.
  $\bf{Dragging}$  (\%): fraction of timesteps where any non-foot module touches the ground.
  $\bf{EM}$ (Effective Movement): displacement / total path length of the core. 1.0 = walks straight,
                                   0 = wobbles in place.

$\bf{Each\ plot\ panel}$ is one robot. Conditions on the x-axis go base/unc, base/neigh, base/blf, then a
dashed separator, then phi/unc, phi/neigh, phi/blf. Light blue boxes = $\lambda=0$, dark blue = $\lambda=1$.
Boxes show median + IQR; whiskers are 1.5 IQR; dots are the 5 individual seed runs.

Robot "custom robot" is the new modular body designed for this study.

$\bf{How\ to\ read\ each\ plot:}$

  $\bf{Distance\ plot:}$
    - base/unc box short and low $\rightarrow$ without coordination, the robot barely moves.
    - Within base block (left), boxes climb base/unc < base/neigh < base/blf $\rightarrow$ stronger
      coupling helps.
    - Phi block (right) is flat at the level of base/blf $\rightarrow$ initial phases alone match
      structured coupling.
    - Light vs dark within a condition $\rightarrow$ does the penalty cost distance? Mostly no.

  $\bf{Dragging\ plot:}$
    - Light blue (lambda=0) boxes typically high $\rightarrow$ robots drag when not penalised.
    - Dark blue (lambda=1) boxes near zero on most robots $\rightarrow$ the penalty works.
    - Insect/queen dark boxes stay high $\rightarrow$ morphology floor; some bodies can't be
      cleaned up by the controller alone.

  $\bf{EM\ plot:}$
    - base/unc box is low (~0.6) $\rightarrow$ uncoordinated controllers wobble in place.
    - All other boxes plateau at ~0.85+ $\rightarrow$ any coordination saturates directional
      efficiency.
    - Insect/queen with lambda=1 dip in EM $\rightarrow$ penalty hurts directionality on those
      heavy bodies.

$\bf{Headline\ comparisons\ to\ make\ by\ eye:}$
  1. base/unc vs anything else $\rightarrow$ "coordination matters".
  2. base/blf vs phi/unc        $\rightarrow$ "phi alone matches structured coupling" (the equivalence claim).
  3. phi/blf  vs phi/unc        $\rightarrow$ "combining phi with coupling doesn't stack".
  4. light vs dark within a condition $\rightarrow$ "what does the penalty cost?".
"""
    ax.text(0.02, 0.98, intro, ha="left", va="top", fontsize=11, family="serif",
            transform=ax.transAxes, wrap=True)
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


with PdfPages(OUT_PDF) as pdf:
    n_min = df.groupby(["robot","condition","lambda"]).size().min()
    n_max = df.groupby(["robot","condition","lambda"]).size().max()
    n_str = f"n={n_min}-{n_max} seeds" if n_min != n_max else f"n={n_min} seeds"
    plot_metric(
        pdf, "distance", "Distance (m)",
        f"Distance per condition ({n_str})",
        force_zero=False,
    )
    plot_metric(
        pdf, "dragging_pct", "Dragging (%)",
        f"Dragging % per condition ({n_str})",
        force_zero=True,
    )
    plot_metric(
        pdf, "em", "Effective Movement (displacement / path)",
        f"Effective Movement per condition ({n_str})",
        force_zero=False,
    )

print(f"wrote {OUT_PDF}")
