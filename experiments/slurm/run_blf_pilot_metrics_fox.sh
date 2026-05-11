#!/bin/bash
#SBATCH --job-name=blf_metrics
#SBATCH --account=ec29
#SBATCH --time=1:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=2G
#SBATCH --output=slurm/logs/blf_pilot_metrics/%j.log
#SBATCH --error=slurm/logs/blf_pilot_metrics/%j.err

cd ~/revolve2/experiments
mkdir -p slurm/logs/blf_pilot_metrics
source ~/myenv/bin/activate

export N_WORKERS=25

python -u _compute_blf_pilot_metrics.py
echo "rc=$?"
