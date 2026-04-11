#!/bin/bash
#SBATCH --job-name=zappa_blf_l2
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-5
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# ZAPPA x BLF x LAMBDA 2 x 5 RUNS
# ============================================

ROBOT="zappa"
COUPLING="blf"
LAMBDA=2
CONTROLLER="ode_cpg"
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

RESULTS_DIR="results/zappa_blf_l2"

RUN_NUM=$SLURM_ARRAY_TASK_ID
SEED=$((SLURM_ARRAY_TASK_ID * 10000 + RANDOM % 10000))

cd ~/revolve2/experiments

EXPERIMENT_NAME="${ROBOT}_${COUPLING}_lambda${LAMBDA}"
LOG_DIR="slurm/logs/zappa_blf_l2/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "ZAPPA BLF LAMBDA 2 - Run $RUN_NUM / 5"
echo "=========================================="
echo "  Robot:       $ROBOT"
echo "  Coupling:    $COUPLING"
echo "  Lambda:      $LAMBDA"
echo "  Seed:        $SEED"
echo "=========================================="

source ~/myenv/bin/activate

python -u evolve_comparison.py \
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
    --results-dir $RESULTS_DIR

echo "Completed: ${EXPERIMENT_NAME} Run $RUN_NUM"
