"""
Extract distance and dragging per run for spider and gecko,
grouped by coupling and lambda. Output a markdown file for manual inspection.
"""
import os
import sqlite3
import numpy as np


FINAL_RESULTS = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/results/final_results"
)
FC_LOCAL = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/results/fully_connected_local"
)
FC_TSV = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/_fc_distance_dragging.tsv"
)
OUTPUT_MD = os.path.expanduser(
    "~/masteroppgave/revolve2/experiments/runs_distance_dragging.md"
)


def load_fc_tsv():
    """Load the fully connected results from the TSV (queried on fox)."""
    if not os.path.exists(FC_TSV):
        return {}
    fc = {}  # (robot, lam) -> list of (run, dist, drag_pct)
    with open(FC_TSV) as f:
        next(f)  # skip header
        for line in f:
            parts = line.strip().split("\t")
            if len(parts) != 5:
                continue
            robot, lam, run, dist, drag = parts
            key = (robot, int(lam))
            fc.setdefault(key, []).append((int(run), float(dist), float(drag) * 100.0))
    return fc


def get_distance_dragging(db_path):
    """Get final-gen best individual distance and dragging."""
    try:
        conn = sqlite3.connect(db_path)
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
        if row is None or row[0] is None or row[1] is None:
            return None, None
        return float(row[0]), float(row[1]) * 100.0  # dragging as percentage
    except Exception:
        return None, None


def collect_runs(robot, coupling, lam, fc_data):
    """
    Collect (run_id, distance, dragging) for one (robot, coupling, lambda) cell.
    Returns list of tuples and list of skipped run notes.
    """
    runs = []
    skipped = []

    if coupling == "fully_connected":
        # Use the TSV from fox
        runs = sorted(fc_data.get((robot, lam), []))
        return runs, skipped

    base = FINAL_RESULTS
    exp_dir = "%s_ode_cpg_%s_lambda%d_dragging" % (robot, coupling, lam)
    exp_path = os.path.join(base, exp_dir)
    if not os.path.isdir(exp_path):
        return runs, skipped  # entire cell missing

    for run_num in range(1, 31):
        db = os.path.join(exp_path, "run_%d.sqlite" % run_num)
        if not os.path.exists(db):
            continue
        d, dr = get_distance_dragging(db)
        if d is None:
            skipped.append("%s %s lambda=%d run=%d (db read failed)" % (robot, coupling, lam, run_num))
            continue
        runs.append((run_num, d, dr))

    return runs, skipped


def fmt(x, dec):
    return ("%.*f" % (dec, x)) if x is not None else "—"


def main():
    morphologies = [
        ("spider", "Spider"),
        ("gecko", "Gecko"),
    ]
    couplings = [
        ("uncoupled", "Uncoupled"),
        ("neighbor", "Neighbour"),
        ("blf", "Structured"),
        ("fully_connected", "Fully connected"),
    ]
    lambdas = [0, 1, 2, 3]

    all_skipped = []
    fc_data = load_fc_tsv()
    # data[robot_label][lambda][coupling_label] = list of (run, dist, drag)
    data = {}

    for robot_key, robot_label in morphologies:
        data[robot_label] = {}
        for lam in lambdas:
            data[robot_label][lam] = {}
            for cpl_key, cpl_label in couplings:
                runs, skipped = collect_runs(robot_key, cpl_key, lam, fc_data)
                data[robot_label][lam][cpl_label] = runs
                all_skipped.extend(skipped)

    # ===== Build markdown =====
    lines = []
    lines.append("# Distance and dragging per run")
    lines.append("")
    lines.append("Final distance and dragging from the best individual at the final")
    lines.append("generation of each CMA-ES run. Read directly from the saved SQLite")
    lines.append("databases — no controller re-evaluation.")
    lines.append("")
    lines.append("- Distance: meters, 2 decimal places")
    lines.append("- Dragging: percentage of timesteps with non-foot ground contact, 1 decimal place")
    lines.append("- Source for uncoupled / neighbour / structured: `final_results/`")
    lines.append("- Source for fully connected: `fully_connected_local/`")
    lines.append("")

    for robot_label in ["Spider", "Gecko"]:
        lines.append("# %s" % robot_label)
        lines.append("")

        # Summary table
        lines.append("## Summary")
        lines.append("")
        lines.append("| Coupling | λ | n runs | Distance (m) | Dragging (%) |")
        lines.append("|---|---|---|---|---|")
        for cpl_label in ["Uncoupled", "Neighbour", "Structured", "Fully connected"]:
            for lam in lambdas:
                cell = data[robot_label][lam][cpl_label]
                n = len(cell)
                if n == 0:
                    lines.append("| %s | %d | 0 | — | — |" % (cpl_label, lam))
                    continue
                dists = [r[1] for r in cell]
                drags = [r[2] for r in cell]
                d_str = "%.2f ± %.2f" % (np.mean(dists), np.std(dists))
                dr_str = "%.1f ± %.1f" % (np.mean(drags), np.std(drags))
                lines.append("| %s | %d | %d | %s | %s |" % (cpl_label, lam, n, d_str, dr_str))
        lines.append("")

        # Per-run tables
        for lam in lambdas:
            lines.append("## λ = %d" % lam)
            lines.append("")
            for cpl_label in ["Uncoupled", "Neighbour", "Structured", "Fully connected"]:
                cell = data[robot_label][lam][cpl_label]
                lines.append("### %s" % cpl_label)
                lines.append("")
                if not cell:
                    lines.append("_No runs available._")
                    lines.append("")
                    continue
                lines.append("| Run | Distance (m) | Dragging (%) |")
                lines.append("|---|---|---|")
                for run_num, d, dr in sorted(cell):
                    lines.append("| %d | %.2f | %.1f |" % (run_num, d, dr))
                lines.append("")

    # Skipped section
    lines.append("# Skipped runs")
    lines.append("")
    if not all_skipped:
        lines.append("_None._")
    else:
        for s in all_skipped:
            lines.append("- %s" % s)
    lines.append("")

    with open(OUTPUT_MD, "w") as f:
        f.write("\n".join(lines) + "\n")

    total = sum(len(data[r][l][c]) for r in data for l in data[r] for c in data[r][l])
    print("Wrote %d runs to %s" % (total, OUTPUT_MD))
    if all_skipped:
        print("Skipped %d runs (see end of file)" % len(all_skipped))


if __name__ == "__main__":
    main()
