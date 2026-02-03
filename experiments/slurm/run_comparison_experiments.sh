#!/bin/bash
#SBATCH --job-name=cpg_comparison
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-240
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# CPG COMPARISON EXPERIMENTS
# ============================================
# Comparing:
#   - Robots: spider, gecko (2)
#   - Controllers: ode_cpg, sine (2)
#   - Coupling: uncoupled, neighbor, blf (3)
#   - Penalty: lambda=0, lambda=1 (2)
#   - Seeds: 10 per configuration
#
# Total: 2 x 2 x 3 x 2 x 10 = 240 jobs
#
# Settings:
#   - Simulation time: 30s
#   - Generations: 300
#   - Population: 25
#   - Workers: 25
# ============================================

# Configuration arrays
ROBOTS=("spider" "gecko")
CONTROLLERS=("ode_cpg" "sine")
COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 1)
NUM_SEEDS=10

# Fixed parameters
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

# Results directory
RESULTS_DIR="results/comparison"

# Calculate indices from task ID (1-based)
# Layout: seeds x lambdas x couplings x controllers x robots
# = 10 x 2 x 3 x 2 x 2 = 240
TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

SEED_IDX=$((TASK_IDX % NUM_SEEDS))
REMAINDER=$((TASK_IDX / NUM_SEEDS))

LAMBDA_IDX=$((REMAINDER % 2))
REMAINDER=$((REMAINDER / 2))

COUPLING_IDX=$((REMAINDER % 3))
REMAINDER=$((REMAINDER / 3))

CONTROLLER_IDX=$((REMAINDER % 2))
ROBOT_IDX=$((REMAINDER / 2))

# Get actual values
ROBOT=${ROBOTS[$ROBOT_IDX]}
CONTROLLER=${CONTROLLERS[$CONTROLLER_IDX]}
COUPLING=${COUPLINGS[$COUPLING_IDX]}
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
echo "CPG COMPARISON EXPERIMENT"
echo "=========================================="
echo "Task ID: $TASK_ID / 240"
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
echo "Completed: ${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA} Run $RUN_NUM"
echo "=========================================="
