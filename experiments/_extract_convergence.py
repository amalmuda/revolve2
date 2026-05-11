"""Extract per-generation best_ever_fitness from each run sqlite.

Outputs /tmp/convergence.csv with columns:
  robot, variant, coupling, lambda, seed, generation, best_ever_fitness
"""
import os, sqlite3, glob, re, csv

BASE = os.path.expanduser("~/all_runs")
ROBOTS = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
PAT = re.compile(
    r".*/(?P<robot>[^/]+)/(?P=robot)_bonardi_(?P<variant>base|phi)_(?P<coupling>uncoupled|neighbor|blf)_lambda(?P<lam>\d+)_nu0\.5_w1\.0/run_(?P<seed>\d+)\.sqlite"
)

rows = []
for r in ROBOTS:
    for db in sorted(glob.glob(os.path.join(BASE, r, "*", "run_*.sqlite"))):
        m = PAT.match(db)
        if not m:
            continue
        if "_noX_" in db:
            continue
        try:
            con = sqlite3.connect(db)
            cur = con.execute(
                "SELECT generation_index, best_ever_fitness FROM comparison_generation ORDER BY generation_index"
            )
            for gen, bef in cur:
                rows.append((m["robot"], m["variant"], m["coupling"], int(m["lam"]),
                             int(m["seed"]), int(gen), float(bef)))
            con.close()
        except Exception as e:
            print(f"WARN {db}: {e}")

out = os.path.expanduser("~/convergence_all.csv")
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["robot", "variant", "coupling", "lambda", "seed", "generation", "best_ever_fitness"])
    w.writerows(rows)
print(f"wrote {out} with {len(rows)} rows")
