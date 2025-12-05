#!/bin/bash
#SBATCH --job-name=spider_contact
#SBATCH --account=ec29
#SBATCH --time=5:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=11
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-10
#SBATCH --output=slurm/logs_%A/slurm-%A_%a.out
#SBATCH --error=slurm/logs_%A/slurm-%A_%a.err

# ============================================
# EXPERIMENT CONFIGURATION
# ============================================
# Set custom experiment name here (optional)
# If not set, will use SLURM job ID as experiment identifier
# Examples:
#   export EXPERIMENT_NAME="contact_penalty"
#   export EXPERIMENT_NAME="exp_12_no_penalty"
#   export EXPERIMENT_NAME="baseline_run"

# Uncomment the line below to set a custom experiment name:
# export EXPERIMENT_NAME="contact_penalty"

# If no custom name is set, use job ID
if [ -z "$EXPERIMENT_NAME" ]; then
    export EXPERIMENT_NAME="exp_${SLURM_ARRAY_JOB_ID}"
fi
# ============================================

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create logs directory for this job (SLURM will use this)
mkdir -p slurm/logs_${SLURM_ARRAY_JOB_ID}

# If using a custom experiment name, create a friendly symlink
if [ "$EXPERIMENT_NAME" != "exp_${SLURM_ARRAY_JOB_ID}" ]; then
    # Remove old symlink if it exists
    rm -f slurm/logs_${EXPERIMENT_NAME}
    # Create symlink with experiment name pointing to job ID directory
    ln -s logs_${SLURM_ARRAY_JOB_ID} slurm/logs_${EXPERIMENT_NAME}
fi

# Set run ID
RUN_ID=$SLURM_ARRAY_TASK_ID

echo "=========================================="
echo "Starting evolution experiment"
echo "Experiment name: $EXPERIMENT_NAME"
echo "Job ID: $SLURM_ARRAY_JOB_ID"
echo "Array Task ID: $SLURM_ARRAY_TASK_ID"
echo "Run ID: $RUN_ID"
echo "Node: $SLURMD_NODENAME"
echo "Results: results/${EXPERIMENT_NAME}/"
echo "SLURM logs: slurm/logs_${SLURM_ARRAY_JOB_ID}/"
if [ "$EXPERIMENT_NAME" != "exp_${SLURM_ARRAY_JOB_ID}" ]; then
    echo "            (also: slurm/logs_${EXPERIMENT_NAME}/)"
fi
echo "=========================================="

# Run the evolution with specified run ID
~/myenv/bin/python3 main.py

echo "=========================================="
echo "Evolution completed"
echo "Run ID: $RUN_ID"
echo "==========================================="
