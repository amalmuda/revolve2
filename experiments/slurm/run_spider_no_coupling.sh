#!/bin/bash
#SBATCH --job-name=spider_no_coupling
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-15
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# SPIDER NO-COUPLING EXPERIMENT
# CPG with only internal weights (8 params)
# No external coupling between hinges
# fitness = distance * (1 - contact_m1)^λ
# 30s simulation, no warmup
# 3 lambdas x 5 seeds = 15 jobs
# ============================================

# EXPERIMENT FOLDER NAME
RESULTS_DIR="results/spider_no_coupling"

# Configuration arrays
LAMBDAS=(0 2 4)
NUM_SEEDS=5

# Fixed parameters
ROBOT="spider"
CONTACT_METRIC="m1"
FITNESS_FORMULA="power"
FRICTION=1.0
SIM_TIME=30
GENERATIONS=250
WORKERS=25
POPULATION=25
B_MAX=1
B_MIN=-1

# Calculate indices from task ID (1-based)
# Layout: 5 seeds x 3 lambdas = 15
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
EXPERIMENT_NAME="${ROBOT}_no_coupling_${FITNESS_FORMULA}_bounds${B_MAX}_lambda${LAMBDA//./_}"
LOG_DIR="slurm/logs/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect all output to organized log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "SPIDER NO-COUPLING EXPERIMENT"
echo "CPG with only internal weights (8 params)"
echo "No external coupling between hinges"
echo "fitness = distance * (1 - contact_m1)^λ"
echo "30s simulation, no warmup"
echo "=========================================="
echo "Task ID: $TASK_ID / 15"
echo "Lambda: $LAMBDA"
echo "Bounds: [$B_MIN, $B_MAX]"
echo "Population: $POPULATION"
echo "Run: $RUN_NUM / $NUM_SEEDS"
echo "Seed: $SEED"
echo "Results Dir: $RESULTS_DIR"
echo ""
echo "Robot: $ROBOT"
echo "Contact Metric: $CONTACT_METRIC"
echo "Friction: $FRICTION"
echo "Generations: $GENERATIONS"
echo "NO COUPLING: True (8 internal params only)"
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
    --results-dir $RESULTS_DIR \
    --no-coupling

echo "=========================================="
echo "Completed: Lambda=$LAMBDA, Run=$RUN_NUM"
echo "=========================================="
