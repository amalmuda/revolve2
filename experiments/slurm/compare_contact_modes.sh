#!/bin/bash
#SBATCH --job-name=compare_contact
#SBATCH --account=ec29
#SBATCH --time=5:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=11
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-20
#SBATCH --output=slurm/logs_%A/slurm-%A_%a.out
#SBATCH --error=slurm/logs_%A/slurm-%A_%a.err

# ============================================
# CONTACT MODE COMPARISON EXPERIMENT
# ============================================
# This script runs 10 replicates each of:
# - Binary mode (tasks 1-10)
# - Counting mode (tasks 11-20)
#
# Results will be saved separately for comparison:
# - results/binary_MODE_NAME/
# - results/counting_MODE_NAME/
# ============================================

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create logs directory for this job (SLURM will use this)
mkdir -p slurm/logs_${SLURM_ARRAY_JOB_ID}

# Determine contact mode and run ID based on array task ID
if [ $SLURM_ARRAY_TASK_ID -le 10 ]; then
    # Tasks 1-10: Binary mode
    CONTACT_MODE="binary"
    RUN_ID=$SLURM_ARRAY_TASK_ID
    EXPERIMENT_NAME="binary_${SLURM_ARRAY_JOB_ID}"
else
    # Tasks 11-20: Counting mode
    CONTACT_MODE="counting"
    RUN_ID=$((SLURM_ARRAY_TASK_ID - 10))
    EXPERIMENT_NAME="counting_${SLURM_ARRAY_JOB_ID}"
fi

# Export variables for the Python script
export EXPERIMENT_NAME
export RUN_ID
export CONTACT_MODE

echo "=========================================="
echo "Contact Mode Comparison Experiment"
echo "Job ID: $SLURM_ARRAY_JOB_ID"
echo "Array Task ID: $SLURM_ARRAY_TASK_ID"
echo "Contact Mode: $CONTACT_MODE"
echo "Run ID: $RUN_ID"
echo "Experiment name: $EXPERIMENT_NAME"
echo "Node: $SLURMD_NODENAME"
echo "Results: results/${EXPERIMENT_NAME}/"
echo "SLURM logs: slurm/logs_${SLURM_ARRAY_JOB_ID}/"
echo "=========================================="

# Run the evolution (CONTACT_MODE env var overrides config.py)
~/myenv/bin/python3 main.py

echo "=========================================="
echo "Evolution completed"
echo "Contact Mode: $CONTACT_MODE"
echo "Run ID: $RUN_ID"
echo "=========================================="
