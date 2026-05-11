"""Pull distance + dragging tables for the full 6 robots x 6 conditions x 2 lambdas factorial."""
import sqlite3, glob, statistics, sys

robots = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
conditions = [
    ("base uncoupled", "base", "uncoupled"),
    ("base neighbor",  "base", "neighbor"),
    ("base blf",       "base", "blf"),
    ("phi  uncoupled", "phi",  "uncoupled"),
    ("phi  neighbor",  "phi",  "neighbor"),
    ("phi  blf",       "phi",  "blf"),
]

for robot in robots:
    print("\n=== %s ===" % robot)
    print("%-22s %-8s  %-13s  %-13s" % ("condition", "lam n", "dist (m)", "drag (%)"))
    print("-" * 65)
    base = "results/%s_bonardi_nu0.5_w1" % robot
    for label, variant, coupling in conditions:
        for lam in (0, 1):
            d = "%s/%s_bonardi_%s_%s_lambda%d_nu0.5_w1.0" % (base, robot, variant, coupling, lam)
            ds, gs = [], []
            for db in sorted(glob.glob(d + "/run_*.sqlite")):
                try:
                    con = sqlite3.connect(db)
                    r = con.execute(
                        "SELECT distance, dragging FROM comparison_individual ORDER BY fitness DESC LIMIT 1"
                    ).fetchone()
                    if r:
                        ds.append(r[0])
                        gs.append(r[1])
                    con.close()
                except sqlite3.OperationalError as e:
                    print("  WARN: %s -> %s" % (db, e))
            if not ds:
                continue
            n = len(ds)
            dm = statistics.mean(ds)
            ds_std = statistics.stdev(ds) if n > 1 else 0.0
            gm = statistics.mean(gs) * 100
            gs_std = statistics.stdev(gs) * 100 if n > 1 else 0.0
            print("%-22s lam=%d n=%d  %5.2f +- %4.2f  %5.1f +- %4.1f" % (
                label, lam, n, dm, ds_std, gm, gs_std
            ))
