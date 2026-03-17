#!/bin/bash
#SBATCH --job-name=cpg_gs2
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-50
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# GECKO_SPIDER EXPERIMENTS V2
# ============================================
# Missing configs from the full matrix:
#
#   Jobs 1-10:   ode_cpg uncoupled lambda=0
#   Jobs 11-20:  ode_cpg uncoupled lambda=1
#   Jobs 21-30:  ode_cpg uncoupled lambda=3
#   Jobs 31-40:  ode_cpg neighbor  lambda=3
#   Jobs 41-50:  ode_cpg blf       lambda=3
#
# Total: 5 × 10 = 50 jobs
# ============================================

ROBOT="gecko_spider"
CONTROLLER="ode_cpg"
NUM_SEEDS=10

SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

RESULTS_DIR="results/gecko_spider_v1"

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

# Map task to config
CONFIG_IDX=$((TASK_IDX / NUM_SEEDS))
SEED_IDX=$((TASK_IDX % NUM_SEEDS))
RUN_NUM=$((SEED_IDX + 1))

case $CONFIG_IDX in
    0) COUPLING="uncoupled"; LAMBDA=0 ;;
    1) COUPLING="uncoupled"; LAMBDA=1 ;;
    2) COUPLING="uncoupled"; LAMBDA=3 ;;
    3) COUPLING="neighbor";  LAMBDA=3 ;;
    4) COUPLING="blf";       LAMBDA=3 ;;
esac

# Generate random seed
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create log directory
EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}"
LOG_DIR="slurm/logs/gecko_spider_v1/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect output to log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "GECKO_SPIDER EXPERIMENT V2"
echo "=========================================="
echo "Task ID: $TASK_ID / 50"
echo ""
echo "Configuration:"
echo "  Robot:       $ROBOT"
echo "  Controller:  $CONTROLLER"
echo "  Coupling:    $COUPLING"
echo "  Lambda:      $LAMBDA"
echo "  Run:         $RUN_NUM / $NUM_SEEDS"
echo "  Seed:        $SEED"
echo ""
echo "Settings:"
echo "  Sim Time:    ${SIM_TIME}s"
echo "  Generations: $GENERATIONS"
echo "  Population:  $POPULATION"
echo "  Workers:     $WORKERS"
echo "  Frequency:   0.2 Hz"
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
