#!/bin/bash
#SBATCH --job-name=b2_gs
#SBATCH --account=ec29
#SBATCH --time=3:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=50
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-360
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# BOUNDS [-2,2] POP 50 - GECKO-SPIDER ONLY
# ============================================
# 1 robot × 3 couplings × 4 lambdas × 30 seeds = 360
# ============================================

COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 1 2 3)
NUM_SEEDS=30

CONTROLLER="ode_cpg"
SIM_TIME=30
GENERATIONS=300
POPULATION=50
WORKERS=50
ROBOT="gecko_spider"
BOUNDS=2.0

RESULTS_DIR="results/bounds2_experiments"

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

SEED_IDX=$((TASK_IDX % NUM_SEEDS))
REMAINDER=$((TASK_IDX / NUM_SEEDS))
LAMBDA_IDX=$((REMAINDER % 4))
COUPLING_IDX=$((REMAINDER / 4))

COUPLING=${COUPLINGS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

SEED=$((TASK_ID * 10000 + RANDOM % 10000))

cd ~/revolve2/experiments

EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}"
LOG_DIR="slurm/logs/bounds2_experiments/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "BOUNDS [-2,2] POP 50 - GECKO-SPIDER"
echo "=========================================="
echo "Task ID: $TASK_ID / 360"
echo "  Robot:       $ROBOT"
echo "  Coupling:    $COUPLING"
echo "  Lambda:      $LAMBDA"
echo "  Run:         $RUN_NUM / $NUM_SEEDS"
echo "  Seed:        $SEED"
echo "  Population:  $POPULATION"
echo "  Workers:     $WORKERS"
echo "  Bounds:      [-$BOUNDS, $BOUNDS]"
echo "=========================================="

source ~/myenv/bin/activate

python evolve_comparison.py \
    --robot $ROBOT \
    --controller $CONTROLLER \
    --coupling $COUPLING \
    --lambda $LAMBDA \
    --sim-time $SIM_TIME \
    --generations $GENERATIONS \
    --population $POPULATION \
    --workers $WORKERS \
    --seed $SEED \
    --run-num $RUN_NUM \
    --results-dir $RESULTS_DIR \
    --bounds $BOUNDS

echo "Completed: ${EXPERIMENT_NAME} Run $RUN_NUM"
