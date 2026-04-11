#!/bin/bash
#SBATCH --job-name=morpho_pilot
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-3
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# MORPHOLOGICAL EVOLUTION PILOT
# 3 tasks: uncoupled, neighbor, blf
# Outer: pop 10, gens 10
# Inner: CMA-ES pop 5, gens 10
# Sim: 10s, lambda=2
# ============================================

COUPLINGS=("uncoupled" "neighbor" "blf")
COUPLING=${COUPLINGS[$((SLURM_ARRAY_TASK_ID - 1))]}

cd ~/revolve2/experiments

LOG_DIR="slurm/logs/morpho_pilot"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/${COUPLING}.log" 2>&1

echo "=========================================="
echo "MORPHOLOGICAL EVOLUTION PILOT"
echo "=========================================="
echo "  Coupling:  $COUPLING"
echo "  Task ID:   $SLURM_ARRAY_TASK_ID"
echo "  Node:      $SLURMD_NODENAME"
echo "=========================================="

source ~/myenv/bin/activate

python -u morpho_evolution.py \
    --coupling $COUPLING \
    --seed 42 \
    --results-dir results/morpho_pilot \
    --outer-pop 10 \
    --outer-gens 10 \
    --inner-pop 5 \
    --inner-gens 10 \
    --sim-time 10.0 \
    --lambda 2.0 \
    --max-modules 20 \
    --min-hinges 2

echo "=========================================="
echo "COMPLETE: $COUPLING"
echo "=========================================="
