#!/bin/bash
#SBATCH --job-name=random_params
#SBATCH --account=ec29
#SBATCH --time=1:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=32
#SBATCH --mem-per-cpu=2G
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# RANDOM PARAMETER EXPERIMENT
# ============================================
# For each config (robot × controller × coupling), generate 200
# random parameter vectors in [-1,1], simulate 30s with λ=0,
# record distance and dragging.
#
# Tests whether coupling topology makes it easier to find good
# gaits even with random (unevolved) weights.
#
# Configs (8 total):
#   Gecko:  Uncoupled CPG (6p), Neighbour CPG (9p), BLF CPG (17p), Sine (18p)
#   Spider: Uncoupled CPG (8p), Neighbour CPG (12p), BLF CPG (18p), Sine (24p)
#
# Total: 8 × 200 = 1600 simulations
# With 32 workers, ~10 minutes expected runtime.
#
# NOT an array job — Python script handles all configs internally
# with multiprocessing.Pool.
# ============================================

SAMPLES=200
WORKERS=32
SIM_TIME=30
SEED=42

# Navigate to experiments directory
cd ~/revolve2/experiments

# Create log directory
LOG_DIR="slurm/logs/random_params"
mkdir -p "$LOG_DIR"

# Redirect all output to log file
exec > "${LOG_DIR}/random_params_${SLURM_JOB_ID}.log" 2>&1

echo "=========================================="
echo "RANDOM PARAMETER EXPERIMENT"
echo "=========================================="
echo ""
echo "Settings:"
echo "  Samples:     $SAMPLES per config"
echo "  Workers:     $WORKERS"
echo "  Sim Time:    ${SIM_TIME}s"
echo "  Seed:        $SEED"
echo "  Total sims:  $((SAMPLES * 8))"
echo ""
echo "Job ID:  $SLURM_JOB_ID"
echo "Node:    $SLURMD_NODENAME"
echo "CPUs:    $SLURM_CPUS_PER_TASK"
echo "Started: $(date)"
echo "=========================================="

# Activate virtual environment
source ~/myenv/bin/activate

# Verify setup
echo ""
echo "Python: $(which python)"
echo "Python version: $(python --version 2>&1)"
echo ""

# Verify script exists
if [ ! -f "random_params_experiment.py" ]; then
    echo "ERROR: random_params_experiment.py not found in $(pwd)"
    echo "Did you forget to push/pull the code?"
    exit 1
fi

# Run experiment
echo "Starting experiment at $(date)"
echo ""

python random_params_experiment.py \
    --samples $SAMPLES \
    --workers $WORKERS \
    --sim-time $SIM_TIME \
    --seed $SEED

EXIT_CODE=$?

echo ""
echo "=========================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo "COMPLETED SUCCESSFULLY at $(date)"
    # Copy results to log dir for easy access
    if [ -f "random_params_results.npz" ]; then
        cp random_params_results.npz "${LOG_DIR}/"
        echo "Results copied to ${LOG_DIR}/random_params_results.npz"
    fi
else
    echo "FAILED with exit code $EXIT_CODE at $(date)"
fi
echo "=========================================="

exit $EXIT_CODE
