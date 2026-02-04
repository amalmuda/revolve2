#!/bin/bash
#SBATCH --job-name=spider_offset
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-10
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# SPIDER ODE_CPG_OFFSET NEIGHBOR TEST
# Tests the hypothesis that adding offset
# parameters to Revolve2 CPG reduces dragging
# 10 runs, lambda=0 (pure distance fitness)
# ============================================

# Configuration
ROBOT="spider"
CONTROLLER="ode_cpg_offset"
COUPLING="neighbor"
LAMBDA=0
PENALTY="dragging"
GENERATIONS=300
POPULATION=25
SIM_TIME=30
WORKERS=25

# Use array task ID as run number
RUN_NUM=$SLURM_ARRAY_TASK_ID
TASK_ID=$SLURM_ARRAY_TASK_ID

# Generate random seed
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

# Navigate to experiments directory
cd ~/revolve2/experiments

# Results directory
RESULTS_DIR="results/offset_test/${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}_${PENALTY}"

# Create log directory
EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}"
LOG_DIR="slurm/logs/offset_test/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect output to log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "SPIDER ODE_CPG_OFFSET NEIGHBOR TEST"
echo "Testing offset hypothesis for dragging"
echo "=========================================="
echo "Task ID: $TASK_ID / 10"
echo ""
echo "Configuration:"
echo "  Robot:       $ROBOT"
echo "  Controller:  $CONTROLLER"
echo "  Coupling:    $COUPLING"
echo "  Lambda:      $LAMBDA"
echo "  Run:         $RUN_NUM / 10"
echo "  Seed:        $SEED"
echo ""
echo "Output:        $RESULTS_DIR"
echo ""
echo "Job ID: $SLURM_ARRAY_JOB_ID"
echo "Node: $SLURMD_NODENAME"
echo "=========================================="

# Activate virtual environment
source ~/myenv/bin/activate

# Run experiment
python evolve_comparison.py \
    --robot $ROBOT \
    --controller $CONTROLLER \
    --coupling $COUPLING \
    --lambda $LAMBDA \
    --penalty-type $PENALTY \
    --sim-time $SIM_TIME \
    --generations $GENERATIONS \
    --population $POPULATION \
    --workers $WORKERS \
    --seed $SEED \
    --run-num $RUN_NUM \
    --results-dir $RESULTS_DIR

echo "=========================================="
echo "Completed: ${EXPERIMENT_NAME} Run $RUN_NUM"
echo "=========================================="
