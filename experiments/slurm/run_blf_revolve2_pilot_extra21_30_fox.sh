#!/bin/bash
#SBATCH --job-name=blf_p_x30
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-240%24
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Pilot extension: revolve2 BLF, runs 21-30 (10 extra seeds per cell), flat terrain.
# 6 robots x 4 fitness x 10 seeds = 240 tasks.
# Layout: robot 0..5 (40 each); within robot: fitness 0..3 (10 each).

ROBOTS=(spider gecko babya queen insect ege2)
FITNESSES=(f1 f2 f3 f4)
NUM_SEEDS=10

SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

ROBOT_IDX=$((TASK_IDX / 40))
INNER=$((TASK_IDX % 40))
FIT_IDX=$((INNER / NUM_SEEDS))
SEED_IDX=$((INNER % NUM_SEEDS))

ROBOT=${ROBOTS[$ROBOT_IDX]}
FITNESS=${FITNESSES[$FIT_IDX]}
RUN_NUM=$((SEED_IDX + 1 + 20))
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

NFS_RESULTS_DIR="results/revolve2_blf_pilot"

cd ~/revolve2/experiments

LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/revolve2_blf_pilot"
mkdir -p "$LOCAL_RESULTS_DIR"

LOG_DIR="slurm/logs/revolve2_blf_pilot/${ROBOT}_${FITNESS}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "${ROBOT} flat fitness=${FITNESS} run ${RUN_NUM} seed ${SEED}"

source ~/myenv/bin/activate

python -u evolve_blf_revolve2_pilot.py \
    --robot "$ROBOT" --fitness "$FITNESS" --sim-time "$SIM_TIME" \
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
