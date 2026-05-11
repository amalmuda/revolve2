#!/bin/bash
#SBATCH --job-name=tr_evnu
#SBATCH --account=ec29
#SBATCH --time=1:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --output=slurm/logs/transfer_evolvenu.log
#SBATCH --error=slurm/logs/transfer_evolvenu.err

cd ~/revolve2/experiments
source ~/myenv/bin/activate
N_WORKERS=25 python -u _compute_transfer_evolvenu_fox.py
