"""Query noX results so far on fox."""
import sqlite3, glob, statistics
robots = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
conds = [("base","uncoupled"),("base","neighbor"),("base","blf"),
         ("phi","uncoupled"),("phi","neighbor"),("phi","blf")]
for r in robots:
    print(f"\n=== {r} (noX) ===")
    print(f"{'cond':22} lam  n   dist          drag(%)")
    print("-"*60)
    for v,c in conds:
        for lam in (0,1):
            d = f"results/{r}_bonardi_nu0.5_w1/{r}_bonardi_{v}_noX_{c}_lambda{lam}_nu0.5_w1.0"
            ds, gs = [], []
            for db in sorted(glob.glob(d+"/run_*.sqlite")):
                try:
                    con = sqlite3.connect(db)
                    row = con.execute("SELECT distance, dragging FROM comparison_individual ORDER BY fitness DESC LIMIT 1").fetchone()
                    if row:
                        ds.append(row[0]); gs.append(row[1])
                    con.close()
                except: pass
            if ds:
                m = statistics.mean(ds); s = statistics.stdev(ds) if len(ds)>1 else 0
                gm = statistics.mean(gs)*100; gs2 = statistics.stdev(gs)*100 if len(gs)>1 else 0
                print(f"{v}/{c:18} {lam}  {len(ds):2}  {m:5.2f}+-{s:4.2f}  {gm:5.1f}+-{gs2:4.1f}")
