# Contact Mode Comparison Experiment

This experiment compares the two contact penalty modes (binary vs counting) by running them in parallel.

## What it does

The `compare_contact_modes.sh` script runs:
- **Tasks 1-10**: Binary mode (10 replicates)
- **Tasks 11-20**: Counting mode (10 replicates)

Results are saved separately:
- `results/binary_<JOB_ID>/`
- `results/counting_<JOB_ID>/`

## How to run

```bash
# Navigate to experiments directory
cd ~/revolve2/experiments

# Submit the comparison job
sbatch slurm/compare_contact_modes.sh
```

## Results

After the job completes, you'll have:

- **Binary mode results**: `results/binary_<JOB_ID>/`
  - `evolution_run1.csv` through `evolution_run10.csv`
  - `results_run1.npz` through `results_run10.npz`
  - `log_run1.txt` through `log_run10.txt`

- **Counting mode results**: `results/counting_<JOB_ID>/`
  - `evolution_run1.csv` through `evolution_run10.csv`
  - `results_run1.npz` through `results_run10.npz`
  - `log_run1.txt` through `log_run10.txt`

- **SLURM logs**: `slurm/logs_<JOB_ID>/`
  - `slurm-<JOB_ID>_1.out` through `slurm-<JOB_ID>_20.out` (stdout)
  - `slurm-<JOB_ID>_1.err` through `slurm-<JOB_ID>_20.err` (stderr)

## Monitoring progress

```bash
# Check job status
squeue -u $USER

# Check specific job
squeue -j <JOB_ID>

# View live output for a specific task (e.g., task 1 = binary run 1)
tail -f slurm/logs_<JOB_ID>/slurm-<JOB_ID>_1.out

# View live output for counting mode task 1 (task 11)
tail -f slurm/logs_<JOB_ID>/slurm-<JOB_ID>_11.out
```

## Customizing

To change the number of replicates per mode, edit the script:
- Change `--array=1-20` to `--array=1-N` where N = 2 × (replicates per mode)
- Update the condition `if [ $SLURM_ARRAY_TASK_ID -le 10 ]` to use N/2
- Update the `RUN_ID` calculation for counting mode

## Contact Mode Differences

**Binary mode:**
- Penalizes based on WHETHER any non-leaf body is dragging (yes/no per timestep)
- Higher penalty for any dragging behavior
- Better matches visual perception of dragging

**Counting mode:**
- Penalizes based on HOW MANY non-leaf bodies are dragging
- Lower penalty if only one body drags
- Can underestimate dragging if averaged across many bodies
