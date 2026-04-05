"""Merge individual HHS result files into one consolidated file."""
import os
import glob

output_dir = os.path.expanduser("~/revolve2/experiments/results/hhs_results")
merged_path = os.path.expanduser("~/revolve2/experiments/hhs_results.txt")

files = sorted(glob.glob(os.path.join(output_dir, "hhs_*.txt")))
print("Found %d result files" % len(files))

lines = ["morphology\tcoupling\tlambda\trun\thhs\tdistance\tdragging"]
for f in files:
    with open(f) as fh:
        content = fh.read().strip()
        if content:
            lines.append(content)

with open(merged_path, "w") as fh:
    fh.write("\n".join(lines) + "\n")

print("Merged %d results to: %s" % (len(lines) - 1, merged_path))
