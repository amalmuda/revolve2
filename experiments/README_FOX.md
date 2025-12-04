# Running Spider Contact Penalty Experiments on Fox

## Setup

1. **Activate virtual environment:**
   ```bash
   source ~/masteroppgave/.venv/bin/activate
   ```

2. **Create logs directory:**
   ```bash
   cd ~/masteroppgave/revolve2/experiments
   mkdir -p logs
   ```

## Running Experiments

### Submit job array (10 independent runs):
```bash
sbatch slurm/run_contact_evolution.sh
```

This will run 10 independent evolution experiments (array tasks 1-10), each with:
- 500 generations
- 11 parallel simulators
- Contact penalty fitness: `distance * (1 - non_leaf_contact_ratio)`
- Unique random seed per run

### Check job status:
```bash
squeue -u $USER
```

### Cancel jobs:
```bash
# Cancel specific job
scancel <job_id>

# Cancel all your jobs
scancel -u $USER
```

## Output Files

Each run produces:
- `logs/slurm-<job_id>_<array_id>.out` - stdout log
- `logs/slurm-<job_id>_<array_id>.err` - stderr log
- `log_run<array_id>.txt` - detailed optimization log
- **`evolution_run<array_id>.csv`** - evolution history for plotting (generation, fitness, distance, contact_ratio)
- `results_run<array_id>.npz` - best parameters and fitness

## Plotting Results

After experiments complete, plot the results:

```bash
python plot_evolution.py
```

This creates:
- `evolution_run<N>_plot.png` - individual plot for each run
- `evolution_all_runs.png` - comparison of all runs

Or plot manually:
```python
import csv
import matplotlib.pyplot as plt

# Read CSV
gen, fit, dist, contact = [], [], [], []
with open("evolution_run1.csv", 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        gen.append(int(row['generation']))
        fit.append(float(row['fitness']))
        dist.append(float(row['distance']))
        contact.append(float(row['contact_ratio']))

# Plot
plt.plot(gen, fit)
plt.xlabel('Generation')
plt.ylabel('Fitness (m)')
plt.show()
```

## Load Final Results

```python
import numpy as np

# Load best parameters from run 1
data = np.load("results_run1.npz")
best_params = data["best_params"]
best_fitness = data["best_fitness"]
run_id = data["run_id"]

print(f"Run {run_id}: Best fitness = {best_fitness:.4f}")
```

## SLURM Configuration

Current settings in `slurm/run_contact_evolution.sh`:
- **Account:** ec29
- **Time limit:** 5 hours
- **CPUs:** 11 (matches NUM_SIMULATORS in config)
- **Memory:** 4GB per CPU
- **Array size:** 1-10 (run 10 independent experiments)

To change array size, edit the line:
```bash
#SBATCH --array=1-10
```

For example, `--array=1-30` would run 30 independent experiments.

## Monitoring Progress

View live log output:
```bash
tail -f logs/slurm-<job_id>_<array_id>.out
```

View optimization details:
```bash
tail -f log_run<array_id>.txt
```

## Notes

- Each array task uses different random seed (from timestamp)
- NUM_SIMULATORS=11 in config matches cpus-per-task
- Results are saved automatically when optimization completes
- Evolution history is saved every generation to CSV
- CSV format makes it easy to plot with any tool (Python, R, Excel, etc.)