#!/bin/bash
#SBATCH --job-name=spider_contact
#SBATCH --account=ec29
#SBATCH --time=5:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=11
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-10
#SBATCH --output=logs/slurm-%A_%a.out
#SBATCH --error=logs/slurm-%A_%a.err

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create logs directory if it doesn't exist
mkdir -p logs

# Set run ID
RUN_ID=$SLURM_ARRAY_TASK_ID

echo "=========================================="
echo "Starting contact penalty evolution"
echo "Array Task ID: $SLURM_ARRAY_TASK_ID"
echo "Run ID: $RUN_ID"
echo "Node: $SLURMD_NODENAME"
echo "=========================================="

# Run the evolution with specified run ID
~/myenv/bin/python3 main.py

echo "=========================================="
echo "Evolution completed"
echo "Run ID: $RUN_ID"
echo "==========================================="
