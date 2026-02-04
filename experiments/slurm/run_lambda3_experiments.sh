#!/bin/bash
#SBATCH --job-name=lambda3
#SBATCH --account=ec29
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-160
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# LAMBDA=3 EXPERIMENTS
# ============================================
# Testing stronger contact penalty (lambda=3)
#
# Configuration:
#   - Robots: gecko, spider (2)
#   - Controllers: ode_cpg, sine
#   - Coupling for ode_cpg: uncoupled, neighbor, blf (3)
#   - Coupling for sine: uncoupled, neighbor, blf, blf_bounded, uncoupled_bounded (5)
#   - Penalty: lambda=3 only
#   - Seeds: 10 per configuration
#
# Job breakdown:
#   - ode_cpg: 2 robots × 3 couplings × 10 seeds = 60
#   - sine:    2 robots × 5 couplings × 10 seeds = 100
#   - Total: 160 jobs
#
# Layout:
#   Jobs 1-30:    gecko ode_cpg (3 couplings × 10 seeds)
#   Jobs 31-60:   spider ode_cpg (3 couplings × 10 seeds)
#   Jobs 61-110:  gecko sine (5 couplings × 10 seeds)
#   Jobs 111-160: spider sine (5 couplings × 10 seeds)
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
SINE_COUPLINGS=("uncoupled" "neighbor" "blf" "blf_bounded" "uncoupled_bounded")
LAMBDA=3
NUM_SEEDS=10

# Fixed parameters
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

# Results directory
RESULTS_DIR="results/comparison_v3"

# Job counts per robot
CPG_JOBS_PER_ROBOT=$((3 * NUM_SEEDS))   # 3 couplings × 10 seeds = 30
SINE_JOBS_PER_ROBOT=$((5 * NUM_SEEDS))  # 5 couplings × 10 seeds = 50
TOTAL_CPG_JOBS=$((CPG_JOBS_PER_ROBOT * 2))  # 60
TOTAL_SINE_JOBS=$((SINE_JOBS_PER_ROBOT * 2)) # 100

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

# Determine controller and calculate local index
if [ $TASK_ID -le $TOTAL_CPG_JOBS ]; then
    # ode_cpg jobs (1-60)
    CONTROLLER="ode_cpg"
    LOCAL_IDX=$TASK_IDX

    # Determine robot
    if [ $LOCAL_IDX -lt $CPG_JOBS_PER_ROBOT ]; then
        ROBOT="gecko"
        ROBOT_LOCAL_IDX=$LOCAL_IDX
    else
        ROBOT="spider"
        ROBOT_LOCAL_IDX=$((LOCAL_IDX - CPG_JOBS_PER_ROBOT))
    fi

    # Calculate coupling, seed from robot-local index
    # Layout: seeds × couplings
    SEED_IDX=$((ROBOT_LOCAL_IDX % NUM_SEEDS))
    COUPLING_IDX=$((ROBOT_LOCAL_IDX / NUM_SEEDS))

    COUPLING=${CPG_COUPLINGS[$COUPLING_IDX]}
else
    # sine jobs (61-160)
    CONTROLLER="sine"
    LOCAL_IDX=$((TASK_IDX - TOTAL_CPG_JOBS))

    # Determine robot
    if [ $LOCAL_IDX -lt $SINE_JOBS_PER_ROBOT ]; then
        ROBOT="gecko"
        ROBOT_LOCAL_IDX=$LOCAL_IDX
    else
        ROBOT="spider"
        ROBOT_LOCAL_IDX=$((LOCAL_IDX - SINE_JOBS_PER_ROBOT))
    fi

    # Calculate coupling, seed from robot-local index
    # Layout: seeds × couplings
    SEED_IDX=$((ROBOT_LOCAL_IDX % NUM_SEEDS))
    COUPLING_IDX=$((ROBOT_LOCAL_IDX / NUM_SEEDS))

    COUPLING=${SINE_COUPLINGS[$COUPLING_IDX]}
fi

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
echo "LAMBDA=3 EXPERIMENT"
echo "=========================================="
echo "Task ID: $TASK_ID / 160"
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
