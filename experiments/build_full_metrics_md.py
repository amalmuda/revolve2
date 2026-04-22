"""
Build a comprehensive markdown file with all 5 metrics per run for spider and gecko
(original morphologies). Merges distance/dragging from sqlite with balance/hhs/em
from precomputed text files.

Output: full_metrics.md
"""
import os
import sqlite3
import numpy as np
from collections import defaultdict


FINAL_RESULTS = os.path.expanduser("~/masteroppgave/revolve2/experiments/results/final_results")
BALANCE_FILE = os.path.expanduser("~/masteroppgave/revolve2/experiments/balance_results.txt")
HHS_FILE = os.path.expanduser("~/masteroppgave/revolve2/experiments/hhs_results.txt")
EM_FILE = os.path.expanduser("~/masteroppgave/revolve2/experiments/em_results.txt")
OUTPUT = os.path.expanduser("~/masteroppgave/revolve2/experiments/full_metrics.md")


def normalize_coupling(name):
    n = name.lower().replace("_", " ").strip()
    if "no coupling" in n or n == "uncoupled":
        return "Uncoupled"
    if n == "neighbour" or n == "neighbor":
        return "Neighbour"
    if n == "structured" or n == "blf":
        return "Structured"
    return name


def load_metric_file(path, metric_col):
    out = {}
    with open(path) as f:
        header = f.readline().strip().split("\t")
        col_idx = header.index(metric_col)
        for line in f:
            parts = line.strip().split("\t")
            if len(parts) < len(header):
                continue
            robot = parts[0].lower()
            coupling = normalize_coupling(parts[1])
            lam = int(parts[2])
            run = int(parts[3])
            try:
                val = float(parts[col_idx])
            except (ValueError, IndexError):
                continue
            out[(robot, coupling, lam, run)] = val
    return out


def load_distance_dragging():
    out = {}
    for robot in ["spider", "gecko"]:
        for cpl_key, cpl_label in [("uncoupled", "Uncoupled"), ("neighbor", "Neighbour"), ("blf", "Structured")]:
            for lam in [0, 1, 2, 3]:
                exp_dir = "%s_ode_cpg_%s_lambda%d_dragging" % (robot, cpl_key, lam)
                exp_path = os.path.join(FINAL_RESULTS, exp_dir)
                if not os.path.isdir(exp_path):
                    continue
                for run in range(1, 31):
                    db = os.path.join(exp_path, "run_%d.sqlite" % run)
                    if not os.path.exists(db):
                        continue
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
                            out[(robot, cpl_label, lam, run)] = (float(row[0]), float(row[1]) * 100.0)
                    except Exception:
                        pass
    return out


def main():
    print("Loading data...")
    dd = load_distance_dragging()
    balance = load_metric_file(BALANCE_FILE, "balance")
    hhs = load_metric_file(HHS_FILE, "hhs")
    em = load_metric_file(EM_FILE, "em")

    # Merge into unified dict
    merged = {}
    skipped = []
    for key in dd:
        robot, cpl, lam, run = key
        d, dr = dd[key]
        b = balance.get(key)
        h = hhs.get(key)
        e = em.get(key)
        if b is None or h is None or e is None:
            skipped.append("%s %s λ=%d run=%d: missing balance/hhs/em" % (robot, cpl, lam, run))
            continue
        merged[key] = {"distance": d, "dragging": dr, "balance": b, "hhs": h, "em": e}

    print("Merged %d runs (%d skipped)" % (len(merged), len(skipped)))

    # Build markdown
    lines = []
    lines.append("# Full metrics — Spider and Gecko")
    lines.append("")
    lines.append("Per-run distance, dragging, balance, HHS, and EM for every CMA-ES run.")
    lines.append("")
    lines.append("## Metric definitions")
    lines.append("")
    lines.append("| Metric | Definition | Range | Better |")
    lines.append("|---|---|---|---|")
    lines.append("| Distance (m) | XY displacement of core over 30 s | ≥ 0 | Higher |")
    lines.append("| Dragging (%) | % timesteps with non-foot ground contact | 0–100 | Lower |")
    lines.append("| Balance | Kargar: 1 − (Σ\\|roll\\|+Σ\\|pitch\\|)/(n×360) | 0–1 | Higher |")
    lines.append("| HHS (m) | Head height stability: Σ\\|Δz\\| | ≥ 0 | Lower |")
    lines.append("| EM | Effective movement: displacement / path length | 0–1 | Higher |")
    lines.append("")
    lines.append("## Experiment settings")
    lines.append("")
    lines.append("- Pop 25, 300 generations, bounds [-1, 1], 30 s simulation, 30 runs per cell")
    lines.append("- Initial state: uniform ±√2/2")
    lines.append("- Fitness: distance × (1 − dragging)^λ")
    lines.append("- Source: `final_results/` (distance/dragging from sqlite), `balance_results.txt`,")
    lines.append("  `hhs_results.txt`, `em_results.txt` (from re-simulation of saved best params)")
    lines.append("")

    morphologies = [("spider", "Spider"), ("gecko", "Gecko")]
    couplings = ["Uncoupled", "Neighbour", "Structured"]
    lambdas = [0, 1, 2, 3]

    for robot_key, robot_label in morphologies:
        lines.append("# %s" % robot_label)
        lines.append("")

        # Summary table
        lines.append("## Summary (mean ± std)")
        lines.append("")
        lines.append("| Coupling | λ | n | Distance (m) | Dragging (%) | Balance | HHS (m) | EM |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for cpl in couplings:
            for lam in lambdas:
                cell = [v for k, v in merged.items() if k[0] == robot_key and k[1] == cpl and k[2] == lam]
                if not cell:
                    lines.append("| %s | %d | 0 | — | — | — | — | — |" % (cpl, lam))
                    continue
                n = len(cell)
                d = np.array([c["distance"] for c in cell])
                dr = np.array([c["dragging"] for c in cell])
                b = np.array([c["balance"] for c in cell])
                h = np.array([c["hhs"] for c in cell])
                e = np.array([c["em"] for c in cell])
                lines.append("| %s | %d | %d | %.2f ± %.2f | %.1f ± %.1f | %.3f ± %.3f | %.3f ± %.3f | %.3f ± %.3f |" % (
                    cpl, lam, n,
                    d.mean(), d.std(),
                    dr.mean(), dr.std(),
                    b.mean(), b.std(),
                    h.mean(), h.std(),
                    e.mean(), e.std(),
                ))
        lines.append("")

        # Per-run tables
        for lam in lambdas:
            lines.append("## λ = %d" % lam)
            lines.append("")
            for cpl in couplings:
                cell = [(k[3], v) for k, v in merged.items()
                        if k[0] == robot_key and k[1] == cpl and k[2] == lam]
                lines.append("### %s" % cpl)
                lines.append("")
                if not cell:
                    lines.append("_No data._")
                    lines.append("")
                    continue
                lines.append("| Run | Distance (m) | Dragging (%) | Balance | HHS (m) | EM |")
                lines.append("|---|---|---|---|---|---|")
                for run, v in sorted(cell):
                    lines.append("| %d | %.2f | %.1f | %.3f | %.3f | %.3f |" % (
                        run, v["distance"], v["dragging"], v["balance"], v["hhs"], v["em"]
                    ))
                lines.append("")

    # Skipped
    lines.append("# Skipped runs")
    lines.append("")
    if not skipped:
        lines.append("_None._")
    else:
        for s in skipped:
            lines.append("- %s" % s)
    lines.append("")

    with open(OUTPUT, "w") as f:
        f.write("\n".join(lines) + "\n")

    print("Wrote %d runs to %s" % (len(merged), OUTPUT))


if __name__ == "__main__":
    main()
