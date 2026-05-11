"""Merge backup, fox_runs, robin_runs into ~/all_runs/ with renumbered seeds:

  backup (1-5)   -> run_1..5
  fox_runs (1-25) -> run_6..30
  robin_runs (1-5) -> run_31..35
"""
import os, shutil, re, glob

SOURCES = [
    (os.path.expanduser("~/fox_results_5seeds_backup_20260428_2125"), 0),    # offset 0 -> 1..5
    (os.path.expanduser("~/fox_runs"),                                  5),  # offset 5 -> 6..30
    (os.path.expanduser("~/robin_runs"),                               30),  # offset 30 -> 31..35
]
DEST = os.path.expanduser("~/all_runs")

ROBOTS = ["spider", "gecko", "babya", "queen", "insect", "ege2"]
CELLS = []
for v in ("base", "phi"):
    for c in ("uncoupled", "neighbor", "blf"):
        for lam in (0, 1):
            CELLS.append(f"_bonardi_{v}_{c}_lambda{lam}_nu0.5_w1.0")

run_re = re.compile(r"(?:best_params_)?run_(\d+)\.(npy|sqlite)$")

n_total = 0
for r in ROBOTS:
    for cell_suffix in CELLS:
        cell = f"{r}{cell_suffix}"
        dst_cell = os.path.join(DEST, r, cell)
        os.makedirs(dst_cell, exist_ok=True)
        for src_root, offset in SOURCES:
            src_cell = os.path.join(src_root, r, cell)
            if not os.path.isdir(src_cell):
                continue
            for f in os.listdir(src_cell):
                m = run_re.search(f)
                if not m:
                    continue
                old_idx = int(m.group(1))
                if old_idx < 1 or old_idx > 25:  # restrict to 1..25 for fox, 1..5 for others
                    continue
                new_idx = old_idx + offset
                src = os.path.join(src_cell, f)
                if f.startswith("best_params_run_"):
                    dst = os.path.join(dst_cell, f"best_params_run_{new_idx}.npy")
                elif f.startswith("run_") and f.endswith(".sqlite"):
                    dst = os.path.join(dst_cell, f"run_{new_idx}.sqlite")
                else:
                    continue
                if not os.path.exists(dst):
                    try:
                        os.link(src, dst)
                    except OSError:
                        shutil.copy2(src, dst)
                    n_total += 1

print(f"Linked/copied {n_total} files into {DEST}")
print()
print("Per-robot, per-cell counts (npy):")
for r in ROBOTS:
    print(f"\n=== {r} ===")
    for cell_suffix in CELLS:
        cell = f"{r}{cell_suffix}"
        n = len(glob.glob(os.path.join(DEST, r, cell, "best_params_run_*.npy")))
        print(f"  {cell_suffix}: {n}")
