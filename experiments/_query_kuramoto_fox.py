"""Query fox Kuramoto results: distance + dragging summary per coupling for lambda=0."""
import sqlite3, glob, statistics

print("%-12s %3s %10s %9s %10s %9s" % ("coupling", "n", "dist mean", "dist std", "drag mean", "drag std"))
print("-" * 60)
for coupling in ["uncoupled", "neighbor", "blf"]:
    d = "results/spider_kuramoto_hz0.2/spider_kuramoto_%s_lambda0_hz0.2" % coupling
    dbs = sorted(glob.glob(d + "/run_*.sqlite"))
    dists, drags = [], []
    for db in dbs:
        con = sqlite3.connect(db)
        row = con.execute("SELECT distance, dragging FROM comparison_individual ORDER BY fitness DESC LIMIT 1").fetchone()
        if row:
            dists.append(row[0])
            drags.append(row[1])
        con.close()
    if dists:
        dm = statistics.mean(dists)
        ds = statistics.stdev(dists) if len(dists) > 1 else 0
        gm = statistics.mean(drags) * 100
        gs = statistics.stdev(drags) * 100 if len(drags) > 1 else 0
        print("%-12s %3d %10.3f %9.3f %10.1f %9.1f" % (coupling, len(dists), dm, ds, gm, gs))
