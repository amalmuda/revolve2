#!/bin/bash
#SBATCH --job-name=evolvew
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-120
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# BLF + evolve-w (per-edge coupling weights), lambda=0, drag skipped during evolution.
# 6 robots x 2 terrains x 10 seeds = 120 tasks.
# Layout: robot 0..5 (each gets 20 tasks), within robot: 0..9 = flat, 10..19 = rugged.

ROBOTS=(spider gecko babya queen insect ege2)
TERRAINS=(flat rugged)
NUM_SEEDS=10

NU=0.5
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
LAMBDA=0
COUPLING="blf"

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

ROBOT_IDX=$((TASK_IDX / 20))
INNER=$((TASK_IDX % 20))
TERR_IDX=$((INNER / NUM_SEEDS))
SEED_IDX=$((INNER % NUM_SEEDS))

ROBOT=${ROBOTS[$ROBOT_IDX]}
TERRAIN=${TERRAINS[$TERR_IDX]}
RUN_NUM=$((SEED_IDX + 1))
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

if [ "$TERRAIN" = "rugged" ]; then
    TERRAIN_FLAG="--terrain rugged"
    RESULTS_DIR="results/${ROBOT}_bonardi_evolvew_rugged_nu0.5_w1"
else
    TERRAIN_FLAG="--terrain flat"
    RESULTS_DIR="results/${ROBOT}_bonardi_evolvew_flat_nu0.5_w1"
fi

cd ~/revolve2/experiments
EXPERIMENT_NAME="${ROBOT}_bonardi_w_${COUPLING}_lambda${LAMBDA}_nu${NU}_w${W}"
LOG_DIR="slurm/logs/evolvew_thesis/${ROBOT}_${TERRAIN}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "${ROBOT} ${TERRAIN} evolve-w blf lambda=0 (no-drag) run ${RUN_NUM} seed ${SEED}"

source ~/myenv/bin/activate

python -u evolve_bonardi.py \
    --robot "$ROBOT" --coupling "$COUPLING" --lambda "$LAMBDA" \
    --nu "$NU" --w "$W" --sim-time "$SIM_TIME" \
    --generations "$GENERATIONS" --population "$POPULATION" --workers "$WORKERS" \
    --seed "$SEED" --run-num "$RUN_NUM" --results-dir "$RESULTS_DIR" \
    $TERRAIN_FLAG --skip-drag --evolve-w

echo "Done"
