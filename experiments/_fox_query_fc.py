"""Query fox for fully connected distance/dragging. Run on fox, output TSV."""
import sqlite3
import os
import sys

base = os.path.expanduser("~/revolve2/experiments/results/fully_connected_experiments")
output = os.path.expanduser("~/fc_distance_dragging.tsv")

lines = ["morphology\tlambda\trun\tdistance\tdragging"]

for robot in ["spider", "gecko"]:
    for lam in [0, 1, 2, 3]:
        exp_dir = "%s_ode_cpg_fully_connected_lambda%d_dragging" % (robot, lam)
        exp_path = os.path.join(base, exp_dir)
        if not os.path.isdir(exp_path):
            continue
        for run_num in range(1, 31):
            db = os.path.join(exp_path, "run_%d.sqlite" % run_num)
            if not os.path.exists(db):
                continue
            try:
                conn = sqlite3.connect(db)
                cur = conn.cursor()
                cur.execute("""
                    SELECT i.distance, i.dragging FROM comparison_individual i
                    JOIN comparison_population p ON i.population_id = p.id
                    JOIN comparison_generation g ON g.population_id = p.id
                    WHERE g.generation_index = (SELECT MAX(generation_index) FROM comparison_generation)
                    ORDER BY i.fitness DESC LIMIT 1
                """)
                row = cur.fetchone()
                conn.close()
                if row and row[0] is not None and row[1] is not None:
                    lines.append("%s\t%d\t%d\t%.6f\t%.6f" % (robot, lam, run_num, row[0], row[1]))
            except Exception as e:
                print("ERROR %s run %d: %s" % (exp_dir, run_num, e), file=sys.stderr)

with open(output, "w") as f:
    f.write("\n".join(lines) + "\n")

print("Wrote %d rows to %s" % (len(lines) - 1, output))
