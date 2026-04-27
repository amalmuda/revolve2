"""Find best gecko run per condition for lambda=2."""
import sqlite3, glob

configs = [
    ("base_uncoupled", "results/gecko_bonardi_nu1_w1/gecko_bonardi_base_uncoupled_lambda2_nu1.0_w1.0"),
    ("phi_uncoupled", "results/gecko_bonardi_phi_nu1_w1/gecko_bonardi_phi_uncoupled_lambda2_nu1.0_w1.0"),
    ("phi_neighbor", "results/gecko_bonardi_phi_nu1_w1/gecko_bonardi_phi_neighbor_lambda2_nu1.0_w1.0"),
    ("phi_blf", "results/gecko_bonardi_phi_nu1_w1/gecko_bonardi_phi_blf_lambda2_nu1.0_w1.0"),
]

for name, d in configs:
    best = None
    for db in sorted(glob.glob(d + "/run_*.sqlite")):
        con = sqlite3.connect(db)
        r = con.execute("SELECT fitness, distance, dragging FROM comparison_individual ORDER BY fitness DESC LIMIT 1").fetchone()
        con.close()
        if r and (best is None or r[0] > best[0]):
            best = (r[0], r[1], r[2], db)
    if best:
        fname = best[3].split("/")[-1]
        print("%s: fit=%.2f dist=%.2fm drag=%.2f%% file=%s" % (name, best[0], best[1], best[2]*100, fname))
    else:
        print("%s: no data" % name)
