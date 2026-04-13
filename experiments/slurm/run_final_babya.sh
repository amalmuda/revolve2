#!/bin/bash
#SBATCH --job-name=fin_babya
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-180%30
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# FINAL EXPERIMENT: BABYA
# 3 couplings x 2 lambdas x 30 seeds = 180 tasks
# %30 = max 30 concurrent (5 seeds per config)
# ============================================

ROBOT="babya"
COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 2)

CONTROLLER="ode_cpg"
PENALTY_TYPE="dragging"
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

RESULTS_DIR="results/final_v2"

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

NUM_CONFIGS=6  # 3 couplings x 2 lambdas

CONFIG_IDX=$((TASK_IDX % NUM_CONFIGS))       # 0-5
SEED_IDX=$((TASK_IDX / NUM_CONFIGS))          # 0-29

LAMBDA_IDX=$((CONFIG_IDX % 2))
COUPLING_IDX=$((CONFIG_IDX / 2))

COUPLING=${COUPLINGS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))  # 1-30

SEED=$((TASK_ID * 10000 + RANDOM % 10000))

cd ~/masteroppgave/revolve2/experiments

EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}_${PENALTY_TYPE}"
LOG_DIR="slurm/logs/final_v2/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "FINAL EXPERIMENT"
echo "=========================================="
echo "Task ID:      $TASK_ID / 180"
echo "  Robot:      $ROBOT"
echo "  Coupling:   $COUPLING"
echo "  Lambda:     $LAMBDA"
echo "  Run:        $RUN_NUM / 30"
echo "  Seed:       $SEED"
echo "  Population: $POPULATION"
echo "  Workers:    $WORKERS"
echo "=========================================="

source ~/myenv/bin/activate

python evolve_comparison.py \
    --robot $ROBOT \
    --controller $CONTROLLER \
    --coupling $COUPLING \
    --lambda $LAMBDA \
    --penalty-type $PENALTY_TYPE \
    --sim-time $SIM_TIME \
    --generations $GENERATIONS \
    --population $POPULATION \
    --workers $WORKERS \
    --seed $SEED \
    --run-num $RUN_NUM \
    --results-dir $RESULTS_DIR

echo "Completed: ${EXPERIMENT_NAME} Run $RUN_NUM"
