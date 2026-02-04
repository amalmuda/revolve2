#!/bin/bash
#SBATCH --job-name=spider_offset
#SBATCH --account=ec29
#SBATCH --time=24:00:00
#SBATCH --mem-per-cpu=4G
#SBATCH --cpus-per-task=25
#SBATCH --array=1-10

# ============================================
# SPIDER ODE_CPG_OFFSET NEIGHBOR TEST
# Tests the hypothesis that adding offset
# parameters to Revolve2 CPG reduces dragging
# 10 runs, lambda=0 (pure distance fitness)
# ============================================

set -e

# Setup environment
cd ~/revolve2/experiments
source ~/myenv/bin/activate

# Configuration
ROBOT="spider"
CONTROLLER="ode_cpg_offset"
COUPLING="neighbor"
LAMBDA=0
PENALTY="dragging"
GENERATIONS=300
POPULATION=25
SIM_TIME=30

# Use array task ID as run number and seed
RUN_NUM=$SLURM_ARRAY_TASK_ID
SEED=$((42 + SLURM_ARRAY_TASK_ID))

# Results directory
RESULTS_DIR="results/offset_test/${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}_${PENALTY}"

echo "=========================================="
echo "SPIDER ODE_CPG_OFFSET NEIGHBOR TEST"
echo "Testing offset hypothesis for dragging"
echo "=========================================="
echo "Task ID: $SLURM_ARRAY_TASK_ID / 10"
echo "Robot: $ROBOT"
echo "Controller: $CONTROLLER"
echo "Coupling: $COUPLING"
echo "Lambda: $LAMBDA"
echo "Seed: $SEED"
echo "Results: $RESULTS_DIR"
echo "=========================================="

python evolve_comparison.py \
    --robot $ROBOT \
    --controller $CONTROLLER \
    --coupling $COUPLING \
    --lambda $LAMBDA \
    --penalty-type $PENALTY \
    --sim-time $SIM_TIME \
    --generations $GENERATIONS \
    --population $POPULATION \
    --workers 25 \
    --seed $SEED \
    --run-num $RUN_NUM \
    --results-dir $RESULTS_DIR

echo "Run $RUN_NUM completed!"
