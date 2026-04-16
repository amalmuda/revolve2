"""Show convergence curves from a Fox SQLite db."""
import sqlite3
con = sqlite3.connect("/tmp/run_1.sqlite")
cur = con.cursor()
rows = cur.execute(
    "SELECT generation_index, fitness_max, fitness_mean, best_ever_fitness, "
    "distance_max, dragging_mean FROM comparison_generation ORDER BY generation_index"
).fetchall()
print(f"Total generations recorded: {len(rows)}")
print()
print("{:<6}{:<10}{:<10}{:<12}{:<10}{:<10}".format(
    "Gen", "fit_max", "fit_mean", "best_ever", "dist_max", "drag_mean"))
print("-" * 60)
for r in rows[::20]:
    g, fm, fmean, be, dm, dr = r
    print("{:<6}{:<10.3f}{:<10.3f}{:<12.3f}{:<10.3f}{:<10.3f}".format(g, fm, fmean, be, dm, dr))
g, fm, fmean, be, dm, dr = rows[-1]
print("{:<6}{:<10.3f}{:<10.3f}{:<12.3f}{:<10.3f}{:<10.3f}  <- final".format(g, fm, fmean, be, dm, dr))

# Find when best_ever stopped improving
plateau_gen = None
for i in range(1, len(rows)):
    if rows[i][3] == rows[-1][3] and plateau_gen is None:
        plateau_gen = rows[i][0]
        break
print()
print(f"best_ever first reached final value at generation: {plateau_gen}")
print(f"i.e. last {len(rows) - (plateau_gen if plateau_gen else 0)} generations were redundant")
