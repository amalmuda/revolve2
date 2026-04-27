"""Query 0.5 Hz Bonardi small test results."""
import sqlite3, glob, statistics, sys

robot = sys.argv[1] if len(sys.argv) > 1 else "spider"
print("== %s ==" % robot)
print("%-22s %3s %12s %12s" % ("condition", "n", "dist(m)", "drag(%)"))
print("-" * 60)

results_dir = "results/%s_bonardi_nu0.5_w1" % robot
configs = [
    ("base uncoupled", "base", "uncoupled"),
    ("phi  uncoupled", "phi",  "uncoupled"),
    ("base neighbor",  "base", "neighbor"),
    ("base blf",       "base", "blf"),
]

for label, variant, coupling in configs:
    for lam in [0, 1]:
        d = "%s/%s_bonardi_%s_%s_lambda%d_nu0.5_w1.0" % (results_dir, robot, variant, coupling, lam)
        dbs = sorted(glob.glob(d + "/run_*.sqlite"))
        ds, gs = [], []
        for db in dbs:
            con = sqlite3.connect(db)
            r = con.execute("SELECT distance, dragging FROM comparison_individual ORDER BY fitness DESC LIMIT 1").fetchone()
            if r: ds.append(r[0]); gs.append(r[1])
            con.close()
        if ds:
            dm = statistics.mean(ds); ds_std = statistics.stdev(ds) if len(ds) > 1 else 0
            gm = statistics.mean(gs)*100; gs_std = statistics.stdev(gs)*100 if len(gs) > 1 else 0
            print("%-22s %3d %5.2f+-%.2f %5.1f+-%.1f" % ("%s l=%d" % (label, lam), len(ds), dm, ds_std, gm, gs_std))
