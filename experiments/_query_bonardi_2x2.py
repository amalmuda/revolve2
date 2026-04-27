"""Full 2x2 Bonardi comparison table."""
import sqlite3, glob, statistics

VARIANTS = [
    ("base", "results/spider_bonardi_nu1_w1", ["spider_bonardi_base", "spider_bonardi"]),
    ("+phi0", "results/spider_bonardi_phi_nu1_w1", ["spider_bonardi_phi"]),
    ("+w", "results/spider_bonardi_w_nu1", ["spider_bonardi_w"]),
    ("+phi0+w", "results/spider_bonardi_phi_w_nu1", ["spider_bonardi_phi_w"]),
]

print("%-10s %-10s %3s %3s %10s %10s" % ("variant", "coupling", "lam", "n", "dist(m)", "drag(%)"))
print("-" * 60)

for vname, vdir, prefixes in VARIANTS:
    for c in ["uncoupled", "neighbor", "blf"]:
        for l in [0, 2]:
            ds, gs = [], []
            for prefix in prefixes:
                d = "%s/%s_%s_lambda%d_nu1.0_w1.0" % (vdir, prefix, c, l)
                dbs = sorted(glob.glob(d + "/run_*.sqlite"))
                for db in dbs:
                    con = sqlite3.connect(db)
                    r = con.execute("SELECT distance, dragging FROM comparison_individual ORDER BY fitness DESC LIMIT 1").fetchone()
                    if r: ds.append(r[0]); gs.append(r[1])
                    con.close()
            if ds:
                dm = statistics.mean(ds); ds_std = statistics.stdev(ds) if len(ds)>1 else 0
                gm = statistics.mean(gs)*100; gs_std = statistics.stdev(gs)*100 if len(gs)>1 else 0
                print("%-10s %-10s %3d %3d %5.2f+-%.2f %5.1f+-%.1f" % (vname, c, l, len(ds), dm, ds_std, gm, gs_std))
            else:
                print("%-10s %-10s %3d %3d  (no data)" % (vname, c, l, 0))
