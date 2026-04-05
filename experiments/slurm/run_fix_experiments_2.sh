#!/bin/bash
#SBATCH --job-name=fix_exp
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1001-1080
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# FIX EXPERIMENTS (initial_state = sqrt(2)*0.5)
# ============================================
# Comparing:
#   - Robots: gecko, spider, gecko_spider (3)
#   - Controller: ode_cpg (standard revolve2 CPG)
#   - Coupling: uncoupled, neighbor, blf (3)
#   - Penalty: lambda=0, 1, 2, 3 (4)
#   - Seeds: 30 per configuration
#
# Job breakdown:
#   3 robots × 3 couplings × 4 lambdas × 30 seeds = 1080
#
# Layout:
#   Jobs 1-360:    gecko        (3 couplings × 4 lambdas × 30 seeds)
#   Jobs 361-720:  spider       (3 couplings × 4 lambdas × 30 seeds)
#   Jobs 721-1080: gecko_spider (3 couplings × 4 lambdas × 30 seeds)
#
# Settings:
#   - Simulation time: 30s
#   - Generations: 300
#   - Population: 25
#   - Workers: 25
# ============================================

# Configuration arrays
ROBOTS=("gecko" "spider" "gecko_spider")
COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 1 2 3)
NUM_SEEDS=30

# Fixed parameters
CONTROLLER="ode_cpg"
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

# Results directory
RESULTS_DIR="results/final_experiments"

# Job counts
JOBS_PER_ROBOT=$((3 * 4 * NUM_SEEDS))  # 3 couplings × 4 lambdas × 10 seeds = 120

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

# Determine robot
ROBOT_IDX=$((TASK_IDX / JOBS_PER_ROBOT))
ROBOT=${ROBOTS[$ROBOT_IDX]}
ROBOT_LOCAL_IDX=$((TASK_IDX % JOBS_PER_ROBOT))

# Calculate coupling, lambda, seed from robot-local index
# Layout: seeds × lambdas × couplings
SEED_IDX=$((ROBOT_LOCAL_IDX % NUM_SEEDS))
REMAINDER=$((ROBOT_LOCAL_IDX / NUM_SEEDS))
LAMBDA_IDX=$((REMAINDER % 4))
COUPLING_IDX=$((REMAINDER / 4))

COUPLING=${COUPLINGS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

# Generate random seed
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create log directory
EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}"
LOG_DIR="slurm/logs/final_experiments/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect output to log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "FIX EXPERIMENT (initial_state=sqrt(2)*0.5)"
echo "=========================================="
echo "Task ID: $TASK_ID / 1080"
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
