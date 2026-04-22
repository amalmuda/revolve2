"""Summarize evolved Kuramoto parameters per coupling for lambda=0.

Params layout per individual:
  [A_0..A_{n-1}, phi0_0..phi0_{n-1}, K_0..K_{nc-1}, Delta_0..Delta_{nc-1}]
"""
import sqlite3
import glob
import math
import statistics
import numpy as np

N_HINGES = 8  # spider
COUPLINGS = {"uncoupled": 0, "neighbor": 4, "blf": 10}


def best_genome(db_path):
    con = sqlite3.connect(db_path)
    row = con.execute(
        "SELECT g.serialized_parameters FROM comparison_individual i "
        "JOIN comparison_genotype g ON g.id = i.genotype_id "
        "ORDER BY i.fitness DESC LIMIT 1"
    ).fetchone()
    con.close()
    if not row:
        return None
    return np.array([float(x) for x in row[0].split(";")])


for coupling, nc in COUPLINGS.items():
    d = "results/spider_kuramoto_hz0.2/spider_kuramoto_%s_lambda0_hz0.2" % coupling
    dbs = sorted(glob.glob(d + "/run_*.sqlite"))
    genomes = []
    for db in dbs:
        g = best_genome(db)
        if g is not None and len(g) == 2 * N_HINGES + 2 * nc:
            genomes.append(g)
    if not genomes:
        continue
    arr = np.vstack(genomes)

    A = arr[:, :N_HINGES]
    phi0 = arr[:, N_HINGES:2 * N_HINGES]
    K = arr[:, 2 * N_HINGES:2 * N_HINGES + nc] if nc > 0 else None
    Delta = arr[:, 2 * N_HINGES + nc:] if nc > 0 else None

    print("=" * 60)
    print("%s (n runs = %d)" % (coupling.upper(), len(genomes)))
    print("=" * 60)

    print("\nAmplitudes A (rad, range [0, pi/3=1.047]):")
    for i in range(N_HINGES):
        print("  Hinge %d: %.3f +/- %.3f" % (i, A[:, i].mean(), A[:, i].std()))
    print("  Overall mean: %.3f rad (%.1f deg)" % (A.mean(), math.degrees(A.mean())))

    print("\nInitial phases phi0 (rad, range [0, 2pi]):")
    for i in range(N_HINGES):
        print("  Hinge %d: %.3f +/- %.3f" % (i, phi0[:, i].mean(), phi0[:, i].std()))

    if K is not None:
        print("\nCoupling weights K (range [0, 2.0]):")
        for i in range(nc):
            print("  Link %2d: K=%.3f +/- %.3f  Delta=%.3f +/- %.3f" % (
                i, K[:, i].mean(), K[:, i].std(), Delta[:, i].mean(), Delta[:, i].std()
            ))
        print("  K mean: %.3f, Delta mean: %.3f rad" % (K.mean(), Delta.mean()))

    print()
