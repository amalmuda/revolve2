#!/bin/bash
#SBATCH --job-name=bounds2_exp
#SBATCH --account=ec29
#SBATCH --time=3:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=50
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-720
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# BOUNDS [-2, 2] EXPERIMENTS
# ============================================
# Same as pop50 but with wider parameter bounds
# Robots: gecko, spider (2)
# Couplings: uncoupled, neighbor, blf (3)
# Lambdas: 0, 1, 2, 3 (4)
# Seeds: 30
# Total: 2 × 3 × 4 × 30 = 720
# ============================================

ROBOTS=("gecko" "spider")
COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 1 2 3)
NUM_SEEDS=30

CONTROLLER="ode_cpg"
SIM_TIME=30
GENERATIONS=300
POPULATION=50
WORKERS=50
BOUNDS=2.0

RESULTS_DIR="results/bounds2_experiments"

JOBS_PER_ROBOT=$((3 * 4 * NUM_SEEDS))

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

ROBOT_IDX=$((TASK_IDX / JOBS_PER_ROBOT))
ROBOT=${ROBOTS[$ROBOT_IDX]}
ROBOT_LOCAL_IDX=$((TASK_IDX % JOBS_PER_ROBOT))

SEED_IDX=$((ROBOT_LOCAL_IDX % NUM_SEEDS))
REMAINDER=$((ROBOT_LOCAL_IDX / NUM_SEEDS))
LAMBDA_IDX=$((REMAINDER % 4))
COUPLING_IDX=$((REMAINDER / 4))

COUPLING=${COUPLINGS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

SEED=$((TASK_ID * 10000 + RANDOM % 10000))

cd ~/revolve2/experiments

EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}_b${BOUNDS}"
LOG_DIR="slurm/logs/bounds2_experiments/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "BOUNDS [-2, 2] EXPERIMENT"
echo "=========================================="
echo "Task ID: $TASK_ID / 720"
echo "Robot: $ROBOT | Coupling: $COUPLING | Lambda: $LAMBDA | Run: $RUN_NUM"
echo "Population: $POPULATION | Bounds: [-${BOUNDS}, ${BOUNDS}]"
echo "=========================================="

source ~/myenv/bin/activate

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
    --bounds $BOUNDS

echo "Completed: ${EXPERIMENT_NAME} Run $RUN_NUM"
