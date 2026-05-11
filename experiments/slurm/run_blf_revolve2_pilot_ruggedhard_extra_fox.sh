#!/bin/bash
#SBATCH --job-name=blf_rh_x
#SBATCH --account=ec29
#SBATCH --time=2:30:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-600%24
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Hard-rugged extension: revolve2 BLF, runs 6-30 (25 extra seeds per cell), rugged_hard.
# 6 robots x 4 fitness x 25 seeds = 600 tasks.

ROBOTS=(spider gecko babya queen insect ege2)
FITNESSES=(f1 f2 f3 f4)
NUM_SEEDS=25

SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

# 6 robots × 100 tasks per robot (4 fits × 25 seeds)
ROBOT_IDX=$((TASK_IDX / 100))
INNER=$((TASK_IDX % 100))
FIT_IDX=$((INNER / NUM_SEEDS))
SEED_IDX=$((INNER % NUM_SEEDS))

ROBOT=${ROBOTS[$ROBOT_IDX]}
FITNESS=${FITNESSES[$FIT_IDX]}
RUN_NUM=$((SEED_IDX + 1 + 5))  # runs 6..30
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

NFS_RESULTS_DIR="results/revolve2_blf_pilot"

cd ~/revolve2/experiments

LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/revolve2_blf_pilot"
mkdir -p "$LOCAL_RESULTS_DIR"

LOG_DIR="slurm/logs/revolve2_blf_pilot_ruggedhard/${ROBOT}_${FITNESS}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "${ROBOT} rugged_hard fitness=${FITNESS} run ${RUN_NUM} seed ${SEED}"

source ~/myenv/bin/activate

python -u evolve_blf_revolve2_pilot.py \
    --robot "$ROBOT" --fitness "$FITNESS" --terrain rugged_hard \
    --sim-time "$SIM_TIME" --generations "$GENERATIONS" \
    --population "$POPULATION" --workers "$WORKERS" \
    --seed "$SEED" --run-num "$RUN_NUM" --results-dir "$LOCAL_RESULTS_DIR"
PYRC=$?
echo "evolve rc=$PYRC"

for src_dir in "$LOCAL_RESULTS_DIR"/*/; do
    sub=$(basename "$src_dir")
    dst_dir="$NFS_RESULTS_DIR/$sub"
    mkdir -p "$dst_dir"
    cp "$src_dir"/best_params_run_${RUN_NUM}.npy        "$dst_dir/" 2>/dev/null || echo "WARN: no npy"
    cp "$src_dir"/convergence_run_${RUN_NUM}.csv        "$dst_dir/" 2>/dev/null || echo "WARN: no csv"
    echo "copied to $dst_dir:"
    ls -la "$dst_dir"/*run_${RUN_NUM}.* 2>/dev/null || true
done

rm -rf "$LOCAL_TMP/results"

echo "Done"
exit $PYRC
