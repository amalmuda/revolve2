#!/bin/bash
#SBATCH --job-name=kargar_f2
#SBATCH --account=ec29
#SBATCH --time=3:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-270
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# KARGAR ET AL. (2021) F2 FITNESS FUNCTION
# F2 = 0.5 * speed_y (cm/s) + 0.5 * (1/contacts)
# Pop 25, 300 generations, bounds [-1,1]
# Uniform initial state (sqrt(2)/2)
# ============================================
# 3 robots × 3 couplings × 30 seeds = 270
# ============================================

ROBOTS=("spider" "gecko" "gecko_spider")
COUPLINGS=("uncoupled" "neighbor" "blf")
NUM_SEEDS=30

CONTROLLER="ode_cpg"
PENALTY_TYPE="kargar_f2"
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

RESULTS_DIR="results/kargar_f2_experiments"

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

SEED_IDX=$((TASK_IDX % NUM_SEEDS))
REMAINDER=$((TASK_IDX / NUM_SEEDS))
COUPLING_IDX=$((REMAINDER % 3))
ROBOT_IDX=$((REMAINDER / 3))

ROBOT=${ROBOTS[$ROBOT_IDX]}
COUPLING=${COUPLINGS[$COUPLING_IDX]}
RUN_NUM=$((SEED_IDX + 1))

SEED=$((TASK_ID * 10000 + RANDOM % 10000))

cd ~/revolve2/experiments

EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda0_${PENALTY_TYPE}"
LOG_DIR="slurm/logs/kargar_f2_experiments/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "KARGAR ET AL. (2021) F2 FITNESS"
echo "=========================================="
echo "Task ID: $TASK_ID / 270"
echo "  Robot:       $ROBOT"
echo "  Coupling:    $COUPLING"
echo "  Penalty:     $PENALTY_TYPE"
echo "  Run:         $RUN_NUM / $NUM_SEEDS"
echo "  Seed:        $SEED"
echo "  Population:  $POPULATION"
echo "  Workers:     $WORKERS"
echo "=========================================="

source ~/myenv/bin/activate

python evolve_comparison.py \
    --robot $ROBOT \
    --controller $CONTROLLER \
    --coupling $COUPLING \
    --lambda 0 \
    --penalty-type $PENALTY_TYPE \
    --sim-time $SIM_TIME \
    --generations $GENERATIONS \
    --population $POPULATION \
    --workers $WORKERS \
    --seed $SEED \
    --run-num $RUN_NUM \
    --results-dir $RESULTS_DIR

echo "Completed: ${EXPERIMENT_NAME} Run $RUN_NUM"
