"""Bundle thesis-ready data comparing phi/unc vs base/blf on flat vs rugged.

Lambda=0 only. Flat = 35 seeds, rugged = 30 seeds (asymmetric n; documented).
"""
import os, sqlite3, glob, csv, re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import seaborn as sns

EM_FLAT = os.path.expanduser("~/em_all.csv")
EM_RUGGED = os.path.expanduser("~/em_rugged.csv")
CONV_FLAT = os.path.expanduser("~/convergence_all.csv")
RUGGED_RUNS = os.path.expanduser("~/rugged_runs")
OUT = os.path.expanduser("~/thesis_terrain")
os.makedirs(OUT, exist_ok=True)

ROBOT_ORDER = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
ROBOT_LABEL = {
    "spider": "spider", "gecko": "gecko", "babya": "babya",
    "queen": "queen", "insect": "insect", "ege2": "ege",
}
CELLS = [("phi", "uncoupled"), ("base", "blf")]
COND_LABEL = {("phi", "uncoupled"): "phi/unc", ("base", "blf"): "base/blf"}
COND_ORDER_LBL = ["phi/unc", "base/blf"]
TERRAIN_COLOR = {"flat": "#a6cee3", "rugged": "#b2df8a"}
TERRAIN_DARK  = {"flat": "#1f78b4", "rugged": "#33a02c"}


# ---------- 1. per_seed.csv ----------

flat = pd.read_csv(EM_FLAT)
flat = flat[(flat["lambda"] == 0) &
            flat.apply(lambda r: (r["variant"], r["coupling"]) in CELLS, axis=1)].copy()
flat["terrain"] = "flat"
flat["dragging_pct"] = flat["dragging"] * 100.0
flat = flat[["robot", "variant", "coupling", "terrain", "seed",
             "distance", "dragging_pct", "em"]].rename(
    columns={"dragging_pct": "drag_pct"})

rugged = pd.read_csv(EM_RUGGED)
rugged["terrain"] = "rugged"
rugged = rugged[["robot", "variant", "coupling", "terrain", "seed",
                 "distance", "dragging_pct", "em"]].rename(
    columns={"dragging_pct": "drag_pct"})

per_seed = pd.concat([flat, rugged], ignore_index=True)
per_seed["robot_label"] = per_seed["robot"].map(ROBOT_LABEL)
per_seed = per_seed[["robot_label", "robot", "variant", "coupling",
                     "terrain", "seed", "distance", "drag_pct", "em"]]
per_seed.to_csv(os.path.join(OUT, "per_seed.csv"), index=False, float_format="%.4f")

# ---------- 2. summary.csv ----------

summary_rows = []
for robot in ROBOT_ORDER:
    for v, c in CELLS:
        for terrain in ("flat", "rugged"):
            sub = per_seed[(per_seed.robot == robot) & (per_seed.variant == v)
                           & (per_seed.coupling == c) & (per_seed.terrain == terrain)]
            if sub.empty:
                continue
            row = {
                "robot": ROBOT_LABEL[robot],
                "variant": v, "coupling": c, "terrain": terrain,
                "n_seeds": len(sub),
            }
            for metric in ("distance", "drag_pct", "em"):
                vals = sub[metric]
                row[f"{metric}_mean"] = vals.mean()
                row[f"{metric}_std"] = vals.std()
                row[f"{metric}_median"] = vals.median()
                row[f"{metric}_q25"] = vals.quantile(0.25)
                row[f"{metric}_q75"] = vals.quantile(0.75)
                row[f"{metric}_min"] = vals.min()
                row[f"{metric}_max"] = vals.max()
            summary_rows.append(row)
pd.DataFrame(summary_rows).to_csv(
    os.path.join(OUT, "summary.csv"), index=False, float_format="%.4f")

# ---------- 3. convergence.csv ----------

# Flat: from existing convergence_all.csv (filter to lambda=0 + 2 cells)
conv_flat = pd.read_csv(CONV_FLAT)
conv_flat = conv_flat[
    (conv_flat["lambda"] == 0) &
    conv_flat.apply(lambda r: (r["variant"], r["coupling"]) in CELLS, axis=1)
].copy()
conv_flat["terrain"] = "flat"

# Rugged: extract from local sqlites
rugged_rows = []
PAT = re.compile(
    r".*/(?P<robot>[^/]+)/(?P=robot)_bonardi_(?P<variant>base|phi)_(?P<coupling>uncoupled|blf)_lambda0_nu0\.5_w1\.0/run_(?P<seed>\d+)\.sqlite"
)
for r in ROBOT_ORDER:
    for db in sorted(glob.glob(os.path.join(RUGGED_RUNS, r, "*", "run_*.sqlite"))):
        m = PAT.match(db)
        if not m:
            continue
        try:
            con = sqlite3.connect(db)
            for gen, bef in con.execute(
                "SELECT generation_index, best_ever_fitness FROM comparison_generation ORDER BY generation_index"
            ):
                rugged_rows.append((m["robot"], m["variant"], m["coupling"],
                                    int(m["seed"]), int(gen), float(bef)))
            con.close()
        except Exception as e:
            print(f"WARN {db}: {e}")
conv_rugged = pd.DataFrame(rugged_rows, columns=["robot", "variant", "coupling",
                                                  "seed", "generation",
                                                  "best_ever_fitness"])
conv_rugged["lambda"] = 0
conv_rugged["terrain"] = "rugged"

conv_all = pd.concat([conv_flat, conv_rugged], ignore_index=True)
conv_all["robot_label"] = conv_all["robot"].map(ROBOT_LABEL)
agg = (conv_all.groupby(["robot_label", "variant", "coupling", "terrain", "generation"])
                ["best_ever_fitness"]
                .agg(median="median",
                     q25=lambda s: s.quantile(0.25),
                     q75=lambda s: s.quantile(0.75),
                     mean="mean", std="std", n="count")
                .reset_index()
                .rename(columns={"robot_label": "robot"})
                .sort_values(["robot", "variant", "coupling", "terrain", "generation"]))
agg.to_csv(os.path.join(OUT, "convergence.csv"), index=False, float_format="%.4f")

# ---------- 4. README.md ----------

readme = """# Thesis terrain bundle - phi/unc vs base/blf, flat vs rugged

Bonardi 0.5 Hz CPG sweep, lambda=0 (no drag penalty in fitness).

## Cells

  - 2 controllers : phi/uncoupled, base/blf
  - 2 terrains    : flat, rugged
  - 6 morphologies: spider, gecko, babya, queen, insect, ege

n per cell:
  - flat    : 35 seeds (assembled from fox 25 seeds + backup 5 + robin 5)
  - rugged  : 30 seeds (fox sweep)

Asymmetric n is documented; for fair statistical comparisons, sub-sample flat
to n=30 by taking the first 30 seeds (or any deterministic subset).

## Setup recap

CPG: Bonardi-style phase-amplitude oscillators, one per active hinge.
  dphi_i/dt  = 2*pi*nu  +  sum_j w_ij * A_j * sin(phi_j - phi_i - psi_ij)
  theta_i(t) = A_i * sin(phi_i(t)) + X_i

Always evolved by CMA-ES (popsize 25, sigma 0.5, 300 generations):
  A_i  in [0, pi/3]        amplitude
  X_i  in [-pi/3, pi/3]    output offset
  psi_ij in [0, 2*pi]      phase lag per coupled edge

Variant axis:
  base : phi_i(0) = 0 for all oscillators
  phi  : phi_i(0) is also evolved per oscillator

Coupling axis:
  uncoupled : no edges (w_ij = 0 everywhere)
  blf       : Body Limb Finder; couples within-limb hinges (Bonardi 2014)

Fixed: nu = 0.5 Hz (same for every oscillator), w = 1.0 (all coupled edges),
       sim_time = 30 s, control_frequency = 20 Hz, friction = 1.0.

Rugged terrain: 20x20 m, GeometryHeightmap with Perlin-noise rugged_heightmap
(density=1.5, 500x500 grid), max bump height 5 cm. Robot spawns at z=0.3 m.
Robot evolved on rugged from scratch (not transferred from flat).

## Files

per_seed.csv   - one row per (robot, controller, terrain, seed)
                 columns: robot, variant, coupling, terrain, seed,
                          distance, drag_pct, em

summary.csv    - per-cell aggregates: mean, std, median, q25, q75, min, max
                 for distance, drag_pct, em

convergence.csv - per-generation aggregates of best_ever_fitness across seeds
                  columns: robot, variant, coupling, terrain, generation,
                           median, q25, q75, mean, std, n

boxplots.pdf      - 3 pages (distance / drag / EM), 6 robot panels each.
                    x-axis: phi/unc and base/blf; hue: flat vs rugged.

convergence.pdf   - 2 pages (one per terrain), 6 robot panels each.
                    Each panel: 2 lines (phi/unc, base/blf), median of seeds,
                    shaded band = IQR (25-75 percentile).
"""
with open(os.path.join(OUT, "README.md"), "w") as f:
    f.write(readme)


# ---------- 5. boxplots.pdf ----------

INFO = (
    r"$\bf{Always\ evolved}$: $A$, $X$, $\psi_{ij}$.   "
    r"$\bf{base}$: $\phi_i(0)=0$ for all oscillators.   "
    r"$\bf{phi}$: also evolves $\phi_i(0)$.   "
    r"$\bf{Fixed}$: $\nu = 0.5$ Hz, $w = 1.0$. Lambda=0 (fitness = distance, drag is descriptive)."
)
INFO_N = "Flat n=35 seeds, rugged n=30 seeds."


def add_header(fig, title, info_lines):
    fig.suptitle(title, fontsize=22, fontweight="bold", y=0.995, x=0.02, ha="left")
    for k, line in enumerate(info_lines):
        fig.text(0.02, 0.965 - k * 0.018, line, fontsize=11, style="italic",
                 ha="left", va="top")


def make_boxplot_page(pdf, df, metric, ylabel, title, force_zero=False):
    sns.set_theme(style="whitegrid", context="paper")
    fig, axes = plt.subplots(2, 3, figsize=(22, 14))
    axes = axes.flatten()
    df = df.copy()
    df["cond_lbl"] = df.apply(
        lambda r: COND_LABEL[(r["variant"], r["coupling"])], axis=1)
    for i, robot in enumerate(ROBOT_ORDER):
        ax = axes[i]
        sub = df[df.robot == robot]
        sns.boxplot(
            data=sub, x="cond_lbl", y=metric, hue="terrain",
            order=COND_ORDER_LBL, hue_order=["flat", "rugged"],
            ax=ax, fliersize=2, linewidth=1.0, width=0.6,
            palette={"flat": TERRAIN_COLOR["flat"],
                     "rugged": TERRAIN_COLOR["rugged"]},
        )
        ax.set_xticks(range(len(COND_ORDER_LBL)))
        ax.set_xticklabels(COND_ORDER_LBL, fontsize=14)
        ax.set_xlabel("")
        ax.set_ylabel(ylabel if i % 3 == 0 else "", fontsize=14)
        ax.set_title(ROBOT_LABEL[robot], fontsize=18, fontweight="bold")
        ax.tick_params(axis="y", labelsize=12)
        leg = ax.get_legend()
        if leg is not None:
            leg.remove()
        if force_zero:
            ax.set_ylim(bottom=0)

    add_header(fig, title, [INFO, INFO_N])
    handles = [
        plt.Rectangle((0, 0), 1, 1, color=TERRAIN_COLOR["flat"], ec="#333"),
        plt.Rectangle((0, 0), 1, 1, color=TERRAIN_COLOR["rugged"], ec="#333"),
    ]
    fig.legend(handles, ["flat (n=35)", "rugged (n=30)"],
               loc="upper center", ncol=2, fontsize=14, frameon=False,
               bbox_to_anchor=(0.5, 0.92))
    fig.tight_layout(rect=[0, 0, 1, 0.89])
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


with PdfPages(os.path.join(OUT, "boxplots.pdf")) as pdf:
    make_boxplot_page(pdf, per_seed, "distance",
                      "Distance (m)", "Distance per condition", force_zero=False)
    make_boxplot_page(pdf, per_seed, "drag_pct",
                      "Dragging (%)", "Dragging per condition", force_zero=True)
    make_boxplot_page(pdf, per_seed, "em",
                      "Effective Movement (displacement / path)",
                      "Effective Movement per condition", force_zero=False)


# ---------- 6. convergence.pdf ----------

def make_conv_page(pdf, conv_df, terrain_filter):
    sns.set_theme(style="whitegrid", context="paper")
    fig, axes = plt.subplots(2, 3, figsize=(22, 14), sharex=True)
    axes = axes.flatten()
    for i, robot in enumerate(ROBOT_ORDER):
        ax = axes[i]
        for v, c in CELLS:
            sub = conv_df[(conv_df.robot == ROBOT_LABEL[robot])
                          & (conv_df.variant == v)
                          & (conv_df.coupling == c)
                          & (conv_df.terrain == terrain_filter)]
            if sub.empty:
                continue
            color = "#1f78b4" if v == "phi" else "#33a02c"
            ax.plot(sub["generation"], sub["median"], lw=2.2, color=color,
                    label=COND_LABEL[(v, c)])
            ax.fill_between(sub["generation"], sub["q25"], sub["q75"],
                            color=color, alpha=0.2)
        ax.set_title(ROBOT_LABEL[robot], fontsize=18, fontweight="bold")
        ax.set_ylabel("Best fitness so far" if i % 3 == 0 else "", fontsize=14)
        if i >= 3:
            ax.set_xlabel("Generation", fontsize=14)
        ax.tick_params(labelsize=12)
        ax.grid(alpha=0.3)

    title = f"Convergence (best fitness so far) - {terrain_filter} terrain"
    add_header(fig, title, [
        INFO,
        "Line = median across seeds; shaded band = IQR (25-75 percentile). " + INFO_N,
    ])
    handles = [
        plt.Line2D([0], [0], color="#1f78b4", lw=3),
        plt.Line2D([0], [0], color="#33a02c", lw=3),
    ]
    fig.legend(handles, ["phi/unc", "base/blf"],
               loc="upper center", ncol=2, fontsize=14, frameon=False,
               bbox_to_anchor=(0.5, 0.92))
    fig.tight_layout(rect=[0, 0, 1, 0.89])
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


with PdfPages(os.path.join(OUT, "convergence.pdf")) as pdf:
    make_conv_page(pdf, agg, "flat")
    make_conv_page(pdf, agg, "rugged")


print(f"Wrote bundle to: {OUT}")
print(f"  per_seed.csv:    {len(per_seed)} rows")
print(f"  summary.csv:     {len(summary_rows)} rows")
print(f"  convergence.csv: {len(agg)} rows")
print(f"  boxplots.pdf, convergence.pdf, README.md")
