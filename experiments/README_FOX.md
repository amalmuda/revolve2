# Running Spider Contact Penalty Experiments on Fox

## Setup

1. **Activate virtual environment:**
   ```bash
   source ~/masteroppgave/.venv/bin/activate
   ```

2. **No additional setup needed:**
   - SLURM will automatically create the logs directory when you submit your job

## Running Experiments

### Configuring Experiment Name

By default, experiments are automatically named using the SLURM job ID (e.g., `exp_12345`).

To set a custom experiment name, edit `slurm/run_contact_evolution.sh` and uncomment line 22:

```bash
# Uncomment this line and set your custom name:
export EXPERIMENT_NAME="contact_penalty"
```

Examples of custom names:
- `export EXPERIMENT_NAME="contact_penalty"` - descriptive name
- `export EXPERIMENT_NAME="baseline"` - baseline experiment
- `export EXPERIMENT_NAME="no_penalty_test"` - alternative fitness

**Automatic naming (default):**
- If you don't set a custom name, the experiment will be named `exp_{SLURM_JOB_ID}`
- Example: If your SLURM job ID is 12345, experiment name will be `exp_12345`

All runs from the same SLURM job array will be saved in `results/{EXPERIMENT_NAME}/`.

### Submit job array (10 independent runs):
```bash
sbatch slurm/run_contact_evolution.sh
```

This will run 10 independent evolution experiments (array tasks 1-10), each with:
- 500 generations
- 11 parallel simulators
- Contact penalty fitness: `distance * (1 - non_leaf_contact_ratio)`
- Unique random seed per run
- Results saved to `results/{EXPERIMENT_NAME}/`

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

All runs from the same experiment are in one folder:

```
results/
  contact_penalty/
    log_run1.txt         - optimization log for run 1
    evolution_run1.csv   - evolution history for run 1
    results_run1.npz     - best parameters for run 1
    evolution_run1_plot.png - plots for run 1 (after running plot_evolution.py)

    log_run2.txt         - optimization log for run 2
    evolution_run2.csv   - evolution history for run 2
    results_run2.npz     - best parameters for run 2
    ...

    comparison_all_runs.png - comparison of all runs (after running plot_evolution.py)
```

SLURM logs are organized in `slurm/` directory:
- `slurm/logs_<job_id>/slurm-<job_id>_<array_id>.out` - stdout log for each array task
- `slurm/logs_<job_id>/slurm-<job_id>_<array_id>.err` - stderr log for each array task

If you set a custom experiment name, a symlink is created:
- `slurm/logs_{EXPERIMENT_NAME}/` → points to `logs_<job_id>/`

Example for job 12345 with custom name "contact_penalty":
```
slurm/
  logs_12345/                    - actual directory with SLURM logs
    slurm-12345_1.out            - SLURM output for run 1
    slurm-12345_1.err            - SLURM errors for run 1
    slurm-12345_2.out            - SLURM output for run 2
    slurm-12345_2.err            - SLURM errors for run 2
    ...
  logs_contact_penalty/          - symlink to logs_12345/
```

## Plotting Results

After experiments complete, plot the results:

```bash
# Plot default experiment (contact_penalty) with runs 1-10
python plot_evolution.py

# Plot specific experiment
python plot_evolution.py --experiment my_experiment_name

# Plot specific runs (e.g., runs 1-5)
python plot_evolution.py --experiment my_experiment_name --runs 1-5

# Plot specific run IDs (e.g., runs 1, 3, 5)
python plot_evolution.py --experiment my_experiment_name --runs 1,3,5
```

This creates:
- `results/{EXPERIMENT_NAME}/evolution_run<N>_plot.png` - individual plot for each run
- `results/{EXPERIMENT_NAME}/comparison_all_runs.png` - comparison of all runs

Or plot manually:
```python
import csv
import matplotlib.pyplot as plt

# Read CSV
gen, fit, dist, contact = [], [], [], []
with open("results/contact_penalty/evolution_run1.csv", 'r') as f:
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
data = np.load("results/contact_penalty/results_run1.npz")
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

View live SLURM log output for a specific run:
```bash
tail -f slurm/logs_<job_id>/slurm-<job_id>_<array_id>.out
```

View optimization details:
```bash
tail -f results/{EXPERIMENT_NAME}/log_run<array_id>.txt
```

Examples:
```bash
# SLURM logs for job 12345, run 1
tail -f slurm/logs_12345/slurm-12345_1.out

# Or if using custom experiment name "contact_penalty"
tail -f slurm/logs_contact_penalty/slurm-12345_1.out

# Optimization log for experiment "contact_penalty", run 1
tail -f results/contact_penalty/log_run1.txt

# Or for auto-named experiment (job 12345), run 1
tail -f results/exp_12345/log_run1.txt
```

## Notes

- **Automatic naming**: Experiments are automatically named `exp_{JOB_ID}` unless you set a custom name
- **SLURM logs location**: All SLURM logs are in `slurm/logs_{JOB_ID}/` directory
- **Symlinks for custom names**: If you set a custom experiment name, `slurm/logs_{EXPERIMENT_NAME}/` symlink is created
- Each array task uses different random seed (from timestamp)
- NUM_SIMULATORS=11 in config matches cpus-per-task
- Results are saved automatically when optimization completes
- Evolution history is saved every generation to CSV
- CSV format makes it easy to plot with any tool (Python, R, Excel, etc.)