#!/bin/bash
#SBATCH --job-name=blf_pilot_metrics
#SBATCH --partition=main
#SBATCH --time=1:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=2G
#SBATCH --output=slurm/logs/blf_pilot_metrics/%j.log
#SBATCH --error=slurm/logs/blf_pilot_metrics/%j.err

# Post-hoc metrics for the BLF revolve2 pilot.
# Re-simulates each saved best_params_run_N.npy and writes
# distance/em/drag_pct to ~/blf_pilot_metrics.csv.
# Single node, 120 workers; should finish in 1-2 minutes for ~120-240 sims.

cd ~/revolve2/experiments
mkdir -p slurm/logs/blf_pilot_metrics
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate revolve2

export N_WORKERS=25

python -u _compute_blf_pilot_metrics.py
echo "rc=$?"
