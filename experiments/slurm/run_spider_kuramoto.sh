#!/bin/bash
#SBATCH --job-name=spider_kuramoto
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=30
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-180
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================================
# SPIDER x KURAMOTO CPG x 3 COUPLINGS x 2 LAMBDAS x 30 SEEDS
# ============================================================
# Fixed natural frequency Hz = 0.2 (decoupled from coupling strength).
# Params evolved per individual = 2*n_hinges + 2*n_couplings.
# Spider: 16 (uncoupled) / 24 (neighbor) / 36 (blf) params.
# 3 couplings x 2 lambdas x 30 seeds = 180 tasks
# ============================================================

ROBOT="spider"
CONTROLLER="kuramoto"
COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 2)
NUM_SEEDS=30

HZ=0.2
SIM_TIME=30
GENERATIONS=300
POPULATION=30
WORKERS=30

RESULTS_DIR="results/spider_kuramoto_hz0.2"

# Layout: NUM_SEEDS x len(LAMBDAS) x len(COUPLINGS) = 30 x 2 x 3 = 180
TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

SEED_IDX=$((TASK_IDX % NUM_SEEDS))
REMAINDER=$((TASK_IDX / NUM_SEEDS))
LAMBDA_IDX=$((REMAINDER % 2))
COUPLING_IDX=$((REMAINDER / 2))

COUPLING=${COUPLINGS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

SEED=$((TASK_ID * 10000 + RANDOM % 10000))

cd ~/revolve2/experiments

EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}_hz${HZ}"
LOG_DIR="slurm/logs/spider_kuramoto/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "SPIDER KURAMOTO - ${EXPERIMENT_NAME}"
echo "=========================================="
echo "  Task ID:     $TASK_ID / 180"
echo "  Robot:       $ROBOT"
echo "  Controller:  $CONTROLLER"
echo "  Coupling:    $COUPLING"
echo "  Lambda:      $LAMBDA"
echo "  Hz:          $HZ"
echo "  Run:         $RUN_NUM / $NUM_SEEDS"
echo "  Seed:        $SEED"
echo "  Population:  $POPULATION"
echo "  Generations: $GENERATIONS"
echo "  Sim time:    ${SIM_TIME}s"
echo "  Workers:     $WORKERS"
echo "=========================================="

source ~/myenv/bin/activate

python -u evolve_kuramoto.py \
    --robot $ROBOT \
    --coupling $COUPLING \
    --lambda $LAMBDA \
    --hz $HZ \
    --sim-time $SIM_TIME \
    --generations $GENERATIONS \
    --population $POPULATION \
    --workers $WORKERS \
    --seed $SEED \
    --run-num $RUN_NUM \
    --results-dir $RESULTS_DIR

echo "Completed: ${EXPERIMENT_NAME} Run $RUN_NUM"
