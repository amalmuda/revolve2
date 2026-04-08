#!/bin/bash
#SBATCH --job-name=big_robots
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-720
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# BIG ROBOTS EXPERIMENTS
# big_spider (12 hinges, 4 legs x 3 hinges)
# big_gecko (12 hinges, 4 legs x 2 hinges + 4 spine)
# Pop 25, 300 generations, bounds [-1,1]
# Uniform initial state (sqrt(2)/2)
# ============================================
# 2 robots x 3 couplings x 4 lambdas x 30 seeds = 720
# Round-robin batching: 5 seeds of each config first,
# then next 5, etc. (6 batches of 5 = 30 seeds per config)
# After every 120 tasks: +5 seeds for every config
# ============================================

ROBOTS=("big_spider" "big_gecko")
COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 1 2 3)
NUM_SEEDS=30
SEEDS_PER_BATCH=5

CONTROLLER="ode_cpg"
PENALTY_TYPE="dragging"
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

RESULTS_DIR="results/big_robots_experiments"

# 24 configs (2 robots x 3 couplings x 4 lambdas)
# 6 batches of 5 seeds each = 30 seeds per config
# Tasks 1-120: batch 0 (seeds 1-5 of each config)
# Tasks 121-240: batch 1 (seeds 6-10 of each config)
# Tasks 241-360: batch 2 (seeds 11-15 of each config)
# Tasks 361-480: batch 3 (seeds 16-20 of each config)
# Tasks 481-600: batch 4 (seeds 21-25 of each config)
# Tasks 601-720: batch 5 (seeds 26-30 of each config)

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

NUM_CONFIGS=24  # 2 * 3 * 4
TASKS_PER_BATCH=$((NUM_CONFIGS * SEEDS_PER_BATCH))  # 120

BATCH=$((TASK_IDX / TASKS_PER_BATCH))                    # 0-5
WITHIN_BATCH=$((TASK_IDX % TASKS_PER_BATCH))             # 0-119
CONFIG_IDX=$((WITHIN_BATCH / SEEDS_PER_BATCH))           # 0-23
SEED_IN_BATCH=$((WITHIN_BATCH % SEEDS_PER_BATCH))        # 0-4
SEED_IDX=$((BATCH * SEEDS_PER_BATCH + SEED_IN_BATCH))    # 0-29

LAMBDA_IDX=$((CONFIG_IDX % 4))
REMAINDER=$((CONFIG_IDX / 4))
COUPLING_IDX=$((REMAINDER % 3))
ROBOT_IDX=$((REMAINDER / 3))

ROBOT=${ROBOTS[$ROBOT_IDX]}
COUPLING=${COUPLINGS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

SEED=$((TASK_ID * 10000 + RANDOM % 10000))

cd ~/revolve2/experiments

EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}_${PENALTY_TYPE}"
LOG_DIR="slurm/logs/big_robots_experiments/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "BIG ROBOTS EXPERIMENT"
echo "=========================================="
echo "Task ID:      $TASK_ID / 720"
echo "Batch:        $BATCH (seeds $((BATCH*5+1))-$((BATCH*5+5)))"
echo "  Robot:      $ROBOT"
echo "  Coupling:   $COUPLING"
echo "  Lambda:     $LAMBDA"
echo "  Run:        $RUN_NUM / $NUM_SEEDS"
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
