"""t-test between couplings at each lambda."""
import sqlite3, glob
from scipy import stats

def get_dists(coupling, lam):
    dists = []
    pattern = "results/spider_kuramoto_hz0.2/spider_kuramoto_%s_lambda%d_hz0.2/run_*.sqlite" % (coupling, lam)
    for db in sorted(glob.glob(pattern)):
        con = sqlite3.connect(db)
        r = con.execute("SELECT distance FROM comparison_individual ORDER BY fitness DESC LIMIT 1").fetchone()
        if r:
            dists.append(r[0])
        con.close()
    return dists

for lam in [0, 2]:
    print("lambda=%d:" % lam)
    u = get_dists("uncoupled", lam)
    n = get_dists("neighbor", lam)
    b = get_dists("blf", lam)
    _, p1 = stats.ttest_ind(u, n)
    _, p2 = stats.ttest_ind(u, b)
    _, p3 = stats.ttest_ind(n, b)
    print("  uncoupled vs neighbor: p=%.4f (n=%d vs %d)" % (p1, len(u), len(n)))
    print("  uncoupled vs blf:      p=%.4f (n=%d vs %d)" % (p2, len(u), len(b)))
    print("  neighbor vs blf:       p=%.4f (n=%d vs %d)" % (p3, len(n), len(b)))
    print()
