#!/bin/bash
#SBATCH --job-name=cpg_v5
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-180
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# V5 EXPERIMENTS - Per-joint evolved initial states
# ============================================
# Changes from v3:
#   - Controller: ode_cpg_init (evolves per-joint initial states)
#   - Initial state: per-joint [s1..s8] evolved in [0.01, 1.0]
#     instead of uniform 0.707 for all
#
# Comparing:
#   - Robots: gecko, spider (2)
#   - ODE-CPG: uncoupled, neighbor, blf (3 couplings)
#   - Penalty: lambda=0, 1, 3 (3)
#   - Seeds: 10 per configuration
#
# Job breakdown:
#   - ode_cpg_init: 2 robots × 3 couplings × 3 lambdas × 10 seeds = 180
#
# Layout:
#   Jobs 1-90:    gecko ode_cpg_init  (3 couplings × 3 lambdas × 10 seeds)
#   Jobs 91-180:  spider ode_cpg_init (3 couplings × 3 lambdas × 10 seeds)
#
# Settings:
#   - Simulation time: 30s
#   - Generations: 300
#   - Population: 25
#   - Workers: 25
#   - Frequency: 0.2 Hz (Bonardi et al.)
# ============================================

# Configuration arrays
ROBOTS=("gecko" "spider")
CPG_COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 1 3)
NUM_SEEDS=10

# Fixed parameters
CONTROLLER="ode_cpg_init"
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

# Results directory
RESULTS_DIR="results/comparison_v5"

# Job counts per robot
JOBS_PER_ROBOT=$((3 * 3 * NUM_SEEDS))    # 3 couplings × 3 lambdas × 10 seeds = 90

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

# Determine robot
if [ $TASK_IDX -lt $JOBS_PER_ROBOT ]; then
    ROBOT="gecko"
    ROBOT_LOCAL_IDX=$TASK_IDX
else
    ROBOT="spider"
    ROBOT_LOCAL_IDX=$((TASK_IDX - JOBS_PER_ROBOT))
fi

# Calculate coupling, lambda, seed from robot-local index
# Layout: seeds × lambdas × couplings
SEED_IDX=$((ROBOT_LOCAL_IDX % NUM_SEEDS))
REMAINDER=$((ROBOT_LOCAL_IDX / NUM_SEEDS))
LAMBDA_IDX=$((REMAINDER % 3))
COUPLING_IDX=$((REMAINDER / 3))

COUPLING=${CPG_COUPLINGS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

# Generate random seed
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create log directory
EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}"
LOG_DIR="slurm/logs/v5/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect output to log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "V5 EXPERIMENT - Per-joint initial states"
echo "=========================================="
echo "Task ID: $TASK_ID / 180"
echo ""
echo "Configuration:"
echo "  Robot:       $ROBOT"
echo "  Controller:  $CONTROLLER"
echo "  Coupling:    $COUPLING"
echo "  Lambda:      $LAMBDA"
echo "  Run:         $RUN_NUM / $NUM_SEEDS"
echo "  Seed:        $SEED"
echo ""
echo "Changes from v3:"
echo "  Per-joint initial states evolved in [0.01, 1.0]"
echo "  (v3 used uniform 0.707 for all joints)"
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
