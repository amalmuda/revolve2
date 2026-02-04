#!/bin/bash
#SBATCH --job-name=uncoupled_bounded
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
# UNCOUPLED_BOUNDED EXPERIMENTS
# ============================================
# Testing: BLF amplitude bounds WITHOUT coupling
#
# Comparing against existing results:
#   - sine_uncoupled: no bounds, no coupling (best distance)
#   - sine_blf_bounded: BLF bounds + BLF coupling
#
# This experiment:
#   - sine_uncoupled_bounded: BLF bounds, NO coupling
#
# Configuration:
#   - Robots: gecko, spider (2)
#   - Controller: sine only
#   - Coupling: uncoupled_bounded (1)
#   - Penalty: lambda=0, lambda=1 (2)
#   - Seeds: 10 per configuration
#
# Total: 2 × 2 × 10 = 40 jobs
#
# Settings:
#   - Simulation time: 30s
#   - Generations: 300
#   - Population: 25
#   - Workers: 25
#   - Frequency: 0.2 Hz (Bonardi et al.)
# ============================================

# Configuration
ROBOTS=("gecko" "spider")
LAMBDAS=(0 1)
NUM_SEEDS=10
COUPLING="uncoupled_bounded"
CONTROLLER="sine"

# Fixed parameters
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

# Results directory
RESULTS_DIR="results/comparison_v3"

# Calculate indices from task ID
TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

# Layout: 20 jobs per robot (2 lambdas × 10 seeds)
# Jobs 1-20:  gecko
# Jobs 21-40: spider
JOBS_PER_ROBOT=$((2 * NUM_SEEDS))  # 20

if [ $TASK_IDX -lt $JOBS_PER_ROBOT ]; then
    ROBOT="gecko"
    ROBOT_LOCAL_IDX=$TASK_IDX
else
    ROBOT="spider"
    ROBOT_LOCAL_IDX=$((TASK_IDX - JOBS_PER_ROBOT))
fi

# Layout within robot: seeds × lambdas
SEED_IDX=$((ROBOT_LOCAL_IDX % NUM_SEEDS))
LAMBDA_IDX=$((ROBOT_LOCAL_IDX / NUM_SEEDS))

LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

# Generate random seed
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create log directory
EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}"
LOG_DIR="slurm/logs/comparison/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect output to log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "UNCOUPLED_BOUNDED EXPERIMENT"
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
