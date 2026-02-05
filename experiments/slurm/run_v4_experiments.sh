#!/bin/bash
#SBATCH --job-name=cpg_v4
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
# V4 EXPERIMENTS - Parameter sensitivity
# ============================================
# Changes from v3:
#   - ODE-CPG initial state: 0.707 -> 1.0
#   - Sine amplitude bound: [0, 1.0] -> [0, 0.5]
#
# Comparing:
#   - Robots: gecko, spider (2)
#   - ODE-CPG: uncoupled, neighbor, blf (3 couplings)
#   - Sine: uncoupled only (1 coupling)
#   - Penalty: lambda=0, 1, 3 (3)
#   - Seeds: 10 per configuration
#
# Job breakdown:
#   - ode_cpg: 2 robots × 3 couplings × 3 lambdas × 10 seeds = 180
#   - sine:    2 robots × 1 coupling  × 3 lambdas × 10 seeds =  60
#   - Total: 240 jobs
#
# Layout:
#   Jobs 1-90:    gecko ode_cpg  (3 couplings × 3 lambdas × 10 seeds)
#   Jobs 91-180:  spider ode_cpg (3 couplings × 3 lambdas × 10 seeds)
#   Jobs 181-210: gecko sine uncoupled (3 lambdas × 10 seeds)
#   Jobs 211-240: spider sine uncoupled (3 lambdas × 10 seeds)
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
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

# Results directory
RESULTS_DIR="results/comparison_v4"

# Job counts per robot
CPG_JOBS_PER_ROBOT=$((3 * 3 * NUM_SEEDS))    # 3 couplings × 3 lambdas × 10 seeds = 90
SINE_JOBS_PER_ROBOT=$((1 * 3 * NUM_SEEDS))   # 1 coupling × 3 lambdas × 10 seeds = 30
TOTAL_CPG_JOBS=$((CPG_JOBS_PER_ROBOT * 2))   # 180
TOTAL_SINE_JOBS=$((SINE_JOBS_PER_ROBOT * 2)) # 60

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

# Determine controller and calculate local index
if [ $TASK_ID -le $TOTAL_CPG_JOBS ]; then
    # ode_cpg jobs (1-180)
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

    # Calculate coupling, lambda, seed from robot-local index
    # Layout: seeds × lambdas × couplings
    SEED_IDX=$((ROBOT_LOCAL_IDX % NUM_SEEDS))
    REMAINDER=$((ROBOT_LOCAL_IDX / NUM_SEEDS))
    LAMBDA_IDX=$((REMAINDER % 3))
    COUPLING_IDX=$((REMAINDER / 3))

    COUPLING=${CPG_COUPLINGS[$COUPLING_IDX]}
else
    # sine jobs (181-240)
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

    # Calculate lambda, seed from robot-local index
    # Layout: seeds × lambdas (only uncoupled coupling)
    SEED_IDX=$((ROBOT_LOCAL_IDX % NUM_SEEDS))
    LAMBDA_IDX=$((ROBOT_LOCAL_IDX / NUM_SEEDS))

    COUPLING="uncoupled"
fi

LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

# Generate random seed
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create log directory
EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}"
LOG_DIR="slurm/logs/v4/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"

# Redirect output to log file
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "=========================================="
echo "V4 EXPERIMENT - Parameter sensitivity"
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
echo "Changes from v3:"
echo "  ODE-CPG initial state: 1.0 (was 0.707)"
echo "  Sine amplitude bound:  0.5 (was 1.0)"
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
