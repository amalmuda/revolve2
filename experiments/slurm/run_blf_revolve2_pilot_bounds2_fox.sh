#!/bin/bash
#SBATCH --job-name=blf_b2
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-120%24
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Bounds-widening pilot: revolve2 BLF, [-2, 2] parameter bounds, flat terrain.
# 6 robots x 4 fitness x 5 seeds = 120 tasks.

ROBOTS=(spider gecko babya queen insect ege2)
FITNESSES=(f1 f2 f3 f4)
NUM_SEEDS=5

SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
BOUNDS_HALF=2.0

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

ROBOT_IDX=$((TASK_IDX / 20))
INNER=$((TASK_IDX % 20))
FIT_IDX=$((INNER / NUM_SEEDS))
SEED_IDX=$((INNER % NUM_SEEDS))

ROBOT=${ROBOTS[$ROBOT_IDX]}
FITNESS=${FITNESSES[$FIT_IDX]}
RUN_NUM=$((SEED_IDX + 1))
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

NFS_RESULTS_DIR="results/revolve2_blf_pilot_bounds2"

cd ~/revolve2/experiments

LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/revolve2_blf_pilot_bounds2"
mkdir -p "$LOCAL_RESULTS_DIR"

LOG_DIR="slurm/logs/revolve2_blf_pilot_bounds2/${ROBOT}_${FITNESS}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "${ROBOT} flat fitness=${FITNESS} bounds=[-${BOUNDS_HALF},+${BOUNDS_HALF}] run ${RUN_NUM} seed ${SEED}"

source ~/myenv/bin/activate

python -u evolve_blf_revolve2_pilot.py \
    --robot "$ROBOT" --fitness "$FITNESS" --terrain flat \
    --bounds-half "$BOUNDS_HALF" --sim-time "$SIM_TIME" \
    --generations "$GENERATIONS" --population "$POPULATION" --workers "$WORKERS" \
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
