#!/bin/bash
#SBATCH --job-name=tripod_bounds1_pop25
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-525
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# TRIPOD (3-LEGGED SPIDER) EXPERIMENT - POPULATION 25
# fitness = distance * (1 - contact)^lambda
# 30s simulation, no warmup
# 1 bounds x 21 lambdas x 25 seeds = 525 jobs
# ============================================

# EXPERIMENT FOLDER NAME - unique folder for tripod results
RESULTS_DIR="results/tripod_v1_bounds1_pop25_25seeds"

# Configuration arrays
LAMBDAS=(0 0.25 0.5 0.75 1 1.25 1.5 1.75 2 2.25 2.5 2.75 3 3.25 3.5 3.75 4 4.25 4.5 4.75 5)
NUM_SEEDS=25

# Fixed parameters
ROBOT="tripod"
CONTACT_METRIC="m1"
FITNESS_FORMULA="power"
FRICTION=1.0
SIM_TIME=30
GENERATIONS=500
WORKERS=25
POPULATION=25
B_MAX=1
B_MIN=-1

# Calculate indices from task ID (1-based)
# Layout: 25 seeds x 21 lambdas = 525
TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

SEED_IDX=$((TASK_IDX % NUM_SEEDS))
LAMBDA_IDX=$((TASK_IDX / NUM_SEEDS))

# Get actual values
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

# Generate random seed
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create organized log directory (in slurm/logs)
EXPERIMENT_NAME="${ROBOT}_${CONTACT_METRIC}_${FITNESS_FORMULA}_bounds${B_MAX}_lambda${LAMBDA//./_}"
LOG_DIR="slurm/logs/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect all output to organized log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "TRIPOD (3-LEGGED SPIDER) EXPERIMENT - POPULATION 25"
echo "fitness = distance * (1 - contact)^lambda"
echo "30s simulation, no warmup"
echo "=========================================="
echo "Task ID: $TASK_ID / 525"
echo "Lambda: $LAMBDA"
echo "Bounds: [$B_MIN, $B_MAX]"
echo "Population: $POPULATION"
echo "Run: $RUN_NUM / $NUM_SEEDS"
echo "Seed: $SEED"
echo "Results Dir: $RESULTS_DIR"
echo ""
echo "Robot: $ROBOT"
echo "Friction: $FRICTION"
echo "Generations: $GENERATIONS"
echo ""
echo "Job ID: $SLURM_ARRAY_JOB_ID"
echo "Node: $SLURMD_NODENAME"
echo "=========================================="

# Activate virtual environment and run
source ~/myenv/bin/activate

python evolve.py \
    --robot $ROBOT \
    --contact-metric $CONTACT_METRIC \
    --fitness-formula $FITNESS_FORMULA \
    --lambda $LAMBDA \
    --friction $FRICTION \
    --bounds $B_MIN $B_MAX \
    --sim-time $SIM_TIME \
    --generations $GENERATIONS \
    --workers $WORKERS \
    --population $POPULATION \
    --seed $SEED \
    --run-num $RUN_NUM \
    --results-dir $RESULTS_DIR

echo "=========================================="
echo "Completed: Bounds=[$B_MIN,$B_MAX], Lambda=$LAMBDA, Run=$RUN_NUM"
echo "=========================================="
