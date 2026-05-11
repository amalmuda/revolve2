"""Query rugged sweep distances on fox per cell."""
import sqlite3, glob, statistics, sys

robots = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
conds = [("phi", "uncoupled"), ("base", "blf")]

print(f"{'robot':10} {'phi/unc dist (mean+-std)':30} {'base/blf dist (mean+-std)':30} {'diff':>10}")
print("-" * 85)
for r in robots:
    means = {}
    for v, c in conds:
        d = f"results/{r}_bonardi_rugged_nu0.5_w1/{r}_bonardi_{v}_{c}_lambda0_nu0.5_w1.0"
        ds = []
        for db in sorted(glob.glob(d + "/run_*.sqlite")):
            try:
                con = sqlite3.connect(db)
                row = con.execute("SELECT distance FROM comparison_individual ORDER BY fitness DESC LIMIT 1").fetchone()
                if row:
                    ds.append(row[0])
                con.close()
            except Exception:
                pass
        if ds:
            mean = statistics.mean(ds); std = statistics.stdev(ds) if len(ds) > 1 else 0
            means[(v, c)] = mean
            label = f"{v}/{c[:3]}"
        else:
            mean, std = float("nan"), 0
            means[(v, c)] = float("nan")
    diff = means.get(("base", "blf"), float("nan")) - means.get(("phi", "uncoupled"), float("nan"))
    phi = means.get(("phi", "uncoupled"))
    blf = means.get(("base", "blf"))
    # re-fetch std for printing
    def stats(v, c):
        d = f"results/{r}_bonardi_rugged_nu0.5_w1/{r}_bonardi_{v}_{c}_lambda0_nu0.5_w1.0"
        ds = []
        for db in sorted(glob.glob(d + "/run_*.sqlite")):
            try:
                con = sqlite3.connect(db)
                row = con.execute("SELECT distance FROM comparison_individual ORDER BY fitness DESC LIMIT 1").fetchone()
                if row:
                    ds.append(row[0])
                con.close()
            except Exception:
                pass
        if not ds: return ("", 0)
        return (f"{statistics.mean(ds):5.2f} +- {statistics.stdev(ds) if len(ds)>1 else 0:.2f}  (n={len(ds)})", statistics.mean(ds))
    s1, _ = stats("phi", "uncoupled")
    s2, _ = stats("base", "blf")
    print(f"{r:10} {s1:30} {s2:30} {diff:+10.2f}")
