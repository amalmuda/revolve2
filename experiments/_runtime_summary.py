"""Aggregate fox/robin slurm elapsed times."""
import subprocess, sys
from collections import defaultdict
import statistics

JOBS = ",".join([
    "3401405", "3401406", "3401491",                # main 4-cond (queen, insect, ege)
    "3401598", "3401599", "3401600",                # phi-couplings (spider, gecko, babya)
    "3401601", "3401602", "3401603",                # phi-couplings (queen, insect, ege)
    "3401618", "3401619",                           # ege2 main + phi-couplings
])
out = subprocess.check_output(["sacct", "-X", "-j", JOBS,
                                "--format=JobName%14,Elapsed", "-nP"]).decode()
groups = defaultdict(list)
for line in out.splitlines():
    parts = line.strip().split("|")
    if len(parts) != 2:
        continue
    name, t = parts
    if ":" not in t:
        continue
    h, m, s = t.split(":")
    groups[name].append(int(h)*3600 + int(m)*60 + int(s))

print(f"{'job':14}  {'n':>3}  {'mean(min)':>10}  {'range(min)':>14}  {'total(h)':>9}")
print("-"*60)
total_h = 0
for k, v in sorted(groups.items()):
    if not v:
        continue
    m = statistics.mean(v); mx = max(v); mn = min(v); n = len(v)
    total_h += sum(v)/3600
    print(f"{k:14}  {n:3}  {m/60:10.1f}  {mn/60:6.0f}-{mx/60:>5.0f}  {sum(v)/3600:9.1f}")
print(f"\nGrand total core-hours (single CPU equivalent): {total_h:.1f}")
print(f"With 25 cpus per task: {total_h * 25:.1f} cpu-hours equivalent")
