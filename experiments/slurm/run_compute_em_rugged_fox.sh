#!/bin/bash
#SBATCH --job-name=em_rugged
#SBATCH --account=ec29
#SBATCH --time=1:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --output=slurm/logs/em_rugged.log
#SBATCH --error=slurm/logs/em_rugged.err

cd ~/revolve2/experiments
source ~/myenv/bin/activate
N_WORKERS=25 python -u _compute_drag_em_rugged_fox.py
