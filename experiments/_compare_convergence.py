"""Compare phi/uncoupled vs base/blf convergence."""
import pandas as pd
import os
df = pd.read_csv(os.path.expanduser("~/convergence.csv"))
robots = ['spider','gecko','babya','queen','insect','ege2']
print("robot      lam  phi/unc_final  base/blf_final  phi95%gen  blf95%gen")
print("-" * 75)
for r in robots:
    for lam in (0, 1):
        results = {}
        for v, c, lbl in [("phi", "uncoupled", "phi"), ("base", "blf", "blf")]:
            sub = df[(df.robot == r) & (df.variant == v)
                     & (df.coupling == c) & (df["lambda"] == lam)]
            if sub.empty:
                continue
            mean = sub.groupby("generation")["best_ever_fitness"].mean()
            final = mean.iloc[-1]
            target = 0.95 * final
            reach = int(mean[mean >= target].index[0]) if (mean >= target).any() else 300
            results[lbl] = (final, reach)
        if "phi" in results and "blf" in results:
            pf, pr = results["phi"]
            bf, br = results["blf"]
            print(f"{r:<10} {lam:<3}  {pf:<13.2f}  {bf:<14.2f}  {pr:<9}  {br:<9}")
