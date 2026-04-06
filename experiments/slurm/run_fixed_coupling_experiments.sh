#!/bin/bash
#SBATCH --job-name=fixed_cpg
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
# FIXED COUPLING EXPERIMENTS
# ============================================
# Testing topology vs parameter count:
# All coupling weights fixed to 1.0, only internal weights optimized.
# Same number of parameters (N_hinges) for all topologies.
#
# Comparing:
#   - Robots: spider (8 hinges), gecko (6 hinges) (2)
#   - Coupling topology: uncoupled, neighbor, blf (3)
#   - Lambda: 0, 1, 2, 3 (4)
#   - Seeds: 10 per configuration
#
# Total: 2 × 3 × 4 × 10 = 240 jobs
#
# Layout (seeds × lambdas × couplings × robots):
#   Jobs 1-120:   spider (3 couplings × 4 lambdas × 10 seeds)
#   Jobs 121-240: gecko  (3 couplings × 4 lambdas × 10 seeds)
#
# Settings:
#   - Simulation time: 30s
#   - Generations: 300
#   - Population: 25
#   - Workers: 25
#   - Fixed coupling weight: 1.0
# ============================================

# Configuration arrays
ROBOTS=("spider" "gecko")
COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 1 2 3)
NUM_SEEDS=10
FIXED_COUPLING=1.0

# Fixed parameters
CONTROLLER="ode_cpg"
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
RESULTS_DIR="results/fixed_coupling_experiments"

# Jobs per robot: 3 couplings × 4 lambdas × 10 seeds = 120
JOBS_PER_ROBOT=120

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

# Determine robot
if [ $TASK_IDX -lt $JOBS_PER_ROBOT ]; then
    ROBOT="spider"
    LOCAL_IDX=$TASK_IDX
else
    ROBOT="gecko"
    LOCAL_IDX=$((TASK_IDX - JOBS_PER_ROBOT))
fi

# Calculate coupling, lambda, seed from local index
# Layout: seeds × lambdas × couplings
SEED_IDX=$((LOCAL_IDX % NUM_SEEDS))
REMAINDER=$((LOCAL_IDX / NUM_SEEDS))
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
EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}_fixed${FIXED_COUPLING}"
LOG_DIR="slurm/logs/fixed_coupling/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect output to log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "FIXED COUPLING EXPERIMENT"
echo "Only internal weights optimized, coupling fixed at ${FIXED_COUPLING}"
echo "=========================================="
echo "Task ID: $TASK_ID / 240"
echo ""
echo "Configuration:"
echo "  Robot:           $ROBOT"
echo "  Controller:      $CONTROLLER"
echo "  Coupling:        $COUPLING"
echo "  Lambda:          $LAMBDA"
echo "  Fixed coupling:  $FIXED_COUPLING"
echo "  Run:             $RUN_NUM / $NUM_SEEDS"
echo "  Seed:            $SEED"
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
    --results-dir $RESULTS_DIR \
    --fixed-coupling $FIXED_COUPLING

echo "=========================================="
echo "Completed: ${EXPERIMENT_NAME} Run $RUN_NUM"
echo "=========================================="
