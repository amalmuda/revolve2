"""
Generate a PDF with distance and dragging tables for spider and gecko
across all 4 couplings (uncoupled, neighbour, structured, fully connected)
and all 4 lambda values. Mean ± std across 30 runs per cell.
"""
import os
import sqlite3
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


FINAL_RESULTS = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/results/final_results"
)
FC_TSV = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/_fc_distance_dragging.tsv"
)
OUTPUT_PDF = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/distance_dragging_tables.pdf"
)


def get_final_distance_dragging(db):
    try:
        conn = sqlite3.connect(db)
        cur = conn.cursor()
        cur.execute("""
            SELECT i.distance, i.dragging FROM comparison_individual i
            JOIN comparison_population p ON i.population_id = p.id
            JOIN comparison_generation g ON g.population_id = p.id
            WHERE g.generation_index = (SELECT MAX(generation_index) FROM comparison_generation)
            ORDER BY i.fitness DESC LIMIT 1
        """)
        row = cur.fetchone()
        conn.close()
        if row and row[0] is not None:
            return float(row[0]), float(row[1]) * 100.0
    except Exception:
        pass
    return None, None


def collect_cell(robot, coupling, lam, fc_data):
    dists, drags = [], []
    if coupling == "fully_connected":
        for run, d, dr in fc_data.get((robot, lam), []):
            dists.append(d)
            drags.append(dr)
    else:
        exp_dir = "%s_ode_cpg_%s_lambda%d_dragging" % (robot, coupling, lam)
        exp_path = os.path.join(FINAL_RESULTS, exp_dir)
        if os.path.isdir(exp_path):
            for run in range(1, 31):
                db = os.path.join(exp_path, "run_%d.sqlite" % run)
                if os.path.exists(db):
                    d, dr = get_final_distance_dragging(db)
                    if d is not None:
                        dists.append(d)
                        drags.append(dr)
    return dists, drags


def load_fc():
    fc = {}
    if not os.path.exists(FC_TSV):
        return fc
    with open(FC_TSV) as f:
        next(f)
        for line in f:
            parts = line.strip().split("\t")
            if len(parts) != 5:
                continue
            robot, lam, run, d, dr = parts
            fc.setdefault((robot, int(lam)), []).append((int(run), float(d), float(dr) * 100.0))
    return fc


def build_table_data(robot, metric, fc_data):
    """Build a 2D list for one table. metric is 'distance' or 'dragging'."""
    couplings = [
        ("uncoupled", "No coupling"),
        ("neighbor", "Neighbour"),
        ("blf", "Structured"),
        ("fully_connected", "Fully connected"),
    ]
    lambdas = [0, 1, 2, 3]
    rows = []
    for cpl_key, cpl_label in couplings:
        row = [cpl_label]
        for lam in lambdas:
            dists, drags = collect_cell(robot, cpl_key, lam, fc_data)
            values = dists if metric == "distance" else drags
            if values:
                if metric == "distance":
                    row.append("%.2f ± %.2f" % (np.mean(values), np.std(values)))
                else:
                    row.append("%.1f ± %.1f" % (np.mean(values), np.std(values)))
            else:
                row.append("—")
        rows.append(row)
    return rows


def draw_table(ax, title, data):
    ax.axis("off")
    ax.set_title(title, fontsize=13, weight="bold", pad=12)
    col_labels = ["Coupling", "λ = 0", "λ = 1", "λ = 2", "λ = 3"]
    table = ax.table(
        cellText=data,
        colLabels=col_labels,
        cellLoc="center",
        loc="center",
        colWidths=[0.22, 0.195, 0.195, 0.195, 0.195],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 1.8)
    # Header styling
    for i in range(len(col_labels)):
        cell = table[0, i]
        cell.set_facecolor("#4a7abc")
        cell.set_text_props(color="white", weight="bold")
    # First column (coupling names) styling
    for i in range(1, len(data) + 1):
        cell = table[i, 0]
        cell.set_facecolor("#e8eef7")
        cell.set_text_props(weight="bold")


def main():
    fc_data = load_fc()

    fig = plt.figure(figsize=(11, 13))
    fig.suptitle(
        "Distance and dragging results (mean ± std, 30 runs per cell)",
        fontsize=15, weight="bold", y=0.98,
    )

    gs = fig.add_gridspec(4, 1, hspace=0.5, top=0.93, bottom=0.03)

    # Spider Distance
    ax1 = fig.add_subplot(gs[0])
    draw_table(ax1, "Spider — Distance (m)", build_table_data("spider", "distance", fc_data))

    # Spider Dragging
    ax2 = fig.add_subplot(gs[1])
    draw_table(ax2, "Spider — Dragging (%)", build_table_data("spider", "dragging", fc_data))

    # Gecko Distance
    ax3 = fig.add_subplot(gs[2])
    draw_table(ax3, "Gecko — Distance (m)", build_table_data("gecko", "distance", fc_data))

    # Gecko Dragging
    ax4 = fig.add_subplot(gs[3])
    draw_table(ax4, "Gecko — Dragging (%)", build_table_data("gecko", "dragging", fc_data))

    plt.savefig(OUTPUT_PDF, bbox_inches="tight")
    print("Saved: %s" % OUTPUT_PDF)


if __name__ == "__main__":
    main()
