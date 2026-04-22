"""
Extract smoothness (HHS), straightness (EM), and balance per run.
Merges balance_results.txt + hhs_results.txt + em_results.txt with FC data
from _fc_quality_metrics.txt.

Output: runs_quality_metrics.md
"""
import os
import numpy as np


BALANCE_FILE = os.path.expanduser("~/masteroppgave/revolve2/experiments/balance_results.txt")
HHS_FILE = os.path.expanduser("~/masteroppgave/revolve2/experiments/hhs_results.txt")
EM_FILE = os.path.expanduser("~/masteroppgave/revolve2/experiments/em_results.txt")
FC_FILE = os.path.expanduser("~/masteroppgave/revolve2/experiments/_fc_quality_metrics.txt")
OUTPUT = os.path.expanduser("~/masteroppgave/revolve2/experiments/runs_quality_metrics.md")


def load_metric(path, metric_col):
    """Load a single metric file. Returns dict[(robot, coupling, lam, run)] -> value."""
    out = {}
    if not os.path.exists(path):
        return out
    with open(path) as f:
        header = f.readline().strip().split("\t")
        col_idx = header.index(metric_col)
        for line in f:
            parts = line.strip().split("\t")
            if len(parts) < len(header):
                continue
            robot = parts[0].lower()
            coupling = parts[1]
            lam = int(parts[2])
            run = int(parts[3])
            try:
                val = float(parts[col_idx])
            except (ValueError, IndexError):
                continue
            out[(robot, coupling, lam, run)] = val
    return out


def normalize_coupling(name):
    """Normalize coupling labels across files."""
    n = name.lower().replace("_", " ").strip()
    if "no coupling" in n or n == "uncoupled":
        return "Uncoupled"
    if n == "neighbour" or n == "neighbor":
        return "Neighbour"
    if n == "structured" or n == "blf":
        return "Structured"
    if "fully" in n:
        return "Fully connected"
    return name


def main():
    # Load each metric — note files use morphology+coupling+lambda+run as key
    balance = load_metric(BALANCE_FILE, "balance")
    hhs = load_metric(HHS_FILE, "hhs")
    em = load_metric(EM_FILE, "em")
    fc_balance = load_metric(FC_FILE, "balance")
    fc_hhs = load_metric(FC_FILE, "hhs")
    fc_em = load_metric(FC_FILE, "em")

    # Merge — build unified dict[(robot, coupling_label, lam, run)] -> {balance, hhs, em}
    merged = {}
    skipped = []

    def add(d, metric_name, src):
        for key, val in src.items():
            robot, coupling_raw, lam, run = key
            coupling = normalize_coupling(coupling_raw)
            k = (robot, coupling, lam, run)
            d.setdefault(k, {})[metric_name] = val

    add(merged, "balance", balance)
    add(merged, "hhs", hhs)
    add(merged, "em", em)
    add(merged, "balance", fc_balance)
    add(merged, "hhs", fc_hhs)
    add(merged, "em", fc_em)

    # Filter out runs missing any metric
    complete = {}
    for k, v in merged.items():
        if all(m in v for m in ("balance", "hhs", "em")):
            complete[k] = v
        else:
            missing = [m for m in ("balance", "hhs", "em") if m not in v]
            skipped.append("%s %s lambda=%d run=%d: missing %s" % (k[0], k[1], k[2], k[3], ", ".join(missing)))

    # Build markdown
    lines = []
    lines.append("# Quality metrics per run")
    lines.append("")
    lines.append("Per-run smoothness (HHS), straightness (EM), and balance for every CMA-ES run,")
    lines.append("computed by re-simulating the saved best controller for 30 s in MuJoCo and")
    lines.append("recording physics traces. No re-evaluation of fitness or modification of saved data.")
    lines.append("")
    lines.append("## Metric definitions (taken from existing scripts)")
    lines.append("")
    lines.append("- **smoothness** = `hhs` (Head Height Stability), defined in `compute_hhs.py` as")
    lines.append("  the cumulative absolute change in core z-position over the simulation:")
    lines.append("  `HHS = Σ |pz(t+1) − pz(t)|`. Unit: meters. **Lower is smoother** (less vertical")
    lines.append("  bouncing). Range: ≥ 0.")
    lines.append("- **straightness** = `em` (Effective Movement), defined in `compute_em.py` as")
    lines.append("  displacement / total path length: `EM = D / T`. Range [0, 1]. **Higher is")
    lines.append("  straighter**; 1.0 = perfectly straight, lower = more wandering.")
    lines.append("- **balance** = Kargar et al. (2021) balance metric, defined in `compute_balance.py`:")
    lines.append("  `B = 1 − (Σ|roll| + Σ|pitch|) / (n_steps × 360)` with roll/pitch in degrees clamped")
    lines.append("  to [0, 180]. Range [0, 1]. **Higher is more balanced**; 1.0 = perfectly level.")
    lines.append("")
    lines.append("## Source files")
    lines.append("")
    lines.append("- `balance_results.txt`, `hhs_results.txt`, `em_results.txt` — uncoupled / neighbour /")
    lines.append("  structured runs (computed previously by the corresponding `compute_*.py` scripts).")
    lines.append("- `_fc_quality_metrics.txt` — fully-connected runs (computed by")
    lines.append("  `compute_fc_quality_metrics.py`, which runs all three metrics in one simulation pass).")
    lines.append("")

    morphologies = [("spider", "Spider"), ("gecko", "Gecko")]
    couplings = ["Uncoupled", "Neighbour", "Structured", "Fully connected"]
    lambdas = [0, 1, 2, 3]

    for robot_key, robot_label in morphologies:
        lines.append("# %s" % robot_label)
        lines.append("")

        # Summary
        lines.append("## Summary")
        lines.append("")
        lines.append("| Coupling | λ | n runs | Smoothness (HHS, m) | Straightness (EM) | Balance |")
        lines.append("|---|---|---|---|---|---|")
        for cpl in couplings:
            for lam in lambdas:
                cell = [v for k, v in complete.items() if k[0] == robot_key and k[1] == cpl and k[2] == lam]
                if not cell:
                    lines.append("| %s | %d | 0 | — | — | — |" % (cpl, lam))
                    continue
                hhs_vals = [v["hhs"] for v in cell]
                em_vals = [v["em"] for v in cell]
                bal_vals = [v["balance"] for v in cell]
                lines.append("| %s | %d | %d | %.3f ± %.3f | %.3f ± %.3f | %.3f ± %.3f |" % (
                    cpl, lam, len(cell),
                    np.mean(hhs_vals), np.std(hhs_vals),
                    np.mean(em_vals), np.std(em_vals),
                    np.mean(bal_vals), np.std(bal_vals),
                ))
        lines.append("")

        # Per-run sections
        for lam in lambdas:
            lines.append("## λ = %d" % lam)
            lines.append("")
            for cpl in couplings:
                cell = [(k[3], v) for k, v in complete.items() if k[0] == robot_key and k[1] == cpl and k[2] == lam]
                lines.append("### %s" % cpl)
                lines.append("")
                if not cell:
                    lines.append("_No runs available._")
                    lines.append("")
                    continue
                lines.append("| Run | Smoothness (HHS) | Straightness (EM) | Balance |")
                lines.append("|---|---|---|---|")
                for run, v in sorted(cell):
                    lines.append("| %d | %.3f | %.3f | %.3f |" % (run, v["hhs"], v["em"], v["balance"]))
                lines.append("")

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

    print("Wrote %d runs to %s" % (len(complete), OUTPUT))
    if skipped:
        print("Skipped %d runs (see end of file)" % len(skipped))


if __name__ == "__main__":
    main()
