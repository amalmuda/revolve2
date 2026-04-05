#!/bin/bash
#SBATCH --job-name=hydra_cpg
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-30
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# HYDRA EXPERIMENT
# ============================================
# Comparing:
#   - Robot: hydra (1)
#   - Controller: ode_cpg (1)
#   - Coupling: uncoupled, neighbor, blf (3)
#   - Penalty: lambda=0, lambda=2 (2)
#   - Seeds: 5 per configuration
#
# Total: 1 × 3 × 2 × 5 = 30 jobs
#
# Layout (seeds × lambdas × couplings):
#   Jobs 1-10:  uncoupled (5 seeds × 2 lambdas)
#   Jobs 11-20: neighbor  (5 seeds × 2 lambdas)
#   Jobs 21-30: blf       (5 seeds × 2 lambdas)
#
# Settings:
#   - Simulation time: 30s
#   - Generations: 150
#   - Population: 25
#   - Workers: 25
# ============================================

# Configuration arrays
COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 2)
NUM_SEEDS=5

# Fixed parameters
ROBOT="hydra"
CONTROLLER="ode_cpg"
SIM_TIME=30
GENERATIONS=150
POPULATION=25
WORKERS=25
RESULTS_DIR="results/hydra_experiments"

# Calculate indices from task ID (1-based)
# Layout: seeds × lambdas × couplings
TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

SEED_IDX=$((TASK_IDX % NUM_SEEDS))
REMAINDER=$((TASK_IDX / NUM_SEEDS))
LAMBDA_IDX=$((REMAINDER % 2))
COUPLING_IDX=$((REMAINDER / 2))

# Get actual values
COUPLING=${COUPLINGS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

# Generate random seed
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create log directory
EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}"
LOG_DIR="slurm/logs/hydra/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect output to log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "HYDRA EXPERIMENT"
echo "fitness = distance * (1 - contact)^lambda"
echo "=========================================="
echo "Task ID: $TASK_ID / 30"
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
