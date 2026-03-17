#!/bin/bash
#SBATCH --job-name=cpg_gs
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-40
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# GECKO_SPIDER EXPERIMENTS
# ============================================
# Spider/gecko hybrid morphology (10 active hinges)
#
# Comparing:
#   - Controllers: ode_cpg, sine (2)
#   - Coupling: neighbor only (1)
#   - Penalty: lambda=0, 1 (2)
#   - Seeds: 10 per configuration
#
# Job breakdown:
#   Jobs 1-20:   ode_cpg neighbor (2 lambdas × 10 seeds)
#   Jobs 21-40:  sine neighbor    (2 lambdas × 10 seeds)
#
# Total: 2 × 1 × 2 × 10 = 40 jobs
#
# Settings:
#   - Simulation time: 30s
#   - Generations: 300
#   - Population: 25
#   - Workers: 25
#   - Frequency: 0.2 Hz
# ============================================

ROBOT="gecko_spider"
CONTROLLERS=("ode_cpg" "sine")
COUPLING="neighbor"
LAMBDAS=(0 1)
NUM_SEEDS=10

SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

RESULTS_DIR="results/gecko_spider_v1"

JOBS_PER_CONTROLLER=$((2 * NUM_SEEDS))  # 2 lambdas × 10 seeds = 20

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

# Determine controller
if [ $TASK_IDX -lt $JOBS_PER_CONTROLLER ]; then
    CONTROLLER="ode_cpg"
    LOCAL_IDX=$TASK_IDX
else
    CONTROLLER="sine"
    LOCAL_IDX=$((TASK_IDX - JOBS_PER_CONTROLLER))
fi

# Calculate lambda and seed
SEED_IDX=$((LOCAL_IDX % NUM_SEEDS))
LAMBDA_IDX=$((LOCAL_IDX / NUM_SEEDS))

LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

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
echo "GECKO_SPIDER EXPERIMENT"
echo "=========================================="
echo "Task ID: $TASK_ID / 40"
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
