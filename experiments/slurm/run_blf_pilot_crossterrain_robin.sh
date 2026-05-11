#!/bin/bash
#SBATCH --job-name=blf_xterrain
#SBATCH --partition=main
#SBATCH --time=1:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=120
#SBATCH --mem-per-cpu=2G
#SBATCH --output=slurm/logs/blf_pilot_crossterrain/%j.log
#SBATCH --error=slurm/logs/blf_pilot_crossterrain/%j.err

cd ~/revolve2/experiments
mkdir -p slurm/logs/blf_pilot_crossterrain
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate revolve2

export N_WORKERS=120

python -u _compute_blf_pilot_crossterrain.py
echo "rc=$?"
