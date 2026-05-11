#!/bin/bash
#SBATCH --job-name=neigh_p
#SBATCH --account=ec29
#SBATCH --time=2:30:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-240%24
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Pilot: revolve2 CPG with NEIGHBOR coupling, flat + rugged_hard.
# 6 robots x 4 fitness x 5 seeds x 2 terrains = 240 tasks.
# Layout: terrain 0..1 (120 each); robot 0..5 (20 each); fit 0..3 (5 each).

ROBOTS=(spider gecko babya queen insect ege2)
FITNESSES=(f1 f2 f3 f4)
TERRAINS=(flat rugged_hard)
NUM_SEEDS=5

SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

TERR_IDX=$((TASK_IDX / 120))
INNER1=$((TASK_IDX % 120))
ROBOT_IDX=$((INNER1 / 20))
INNER2=$((INNER1 % 20))
FIT_IDX=$((INNER2 / NUM_SEEDS))
SEED_IDX=$((INNER2 % NUM_SEEDS))

ROBOT=${ROBOTS[$ROBOT_IDX]}
FITNESS=${FITNESSES[$FIT_IDX]}
TERRAIN=${TERRAINS[$TERR_IDX]}
RUN_NUM=$((SEED_IDX + 1))
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

NFS_RESULTS_DIR="results/revolve2_neighbor_pilot"

cd ~/revolve2/experiments

LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/revolve2_neighbor_pilot"
mkdir -p "$LOCAL_RESULTS_DIR"

LOG_DIR="slurm/logs/revolve2_neighbor_pilot/${ROBOT}_${TERRAIN}_${FITNESS}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "${ROBOT} ${TERRAIN} fitness=${FITNESS} coupling=neighbor run ${RUN_NUM} seed ${SEED}"

source ~/myenv/bin/activate

python -u evolve_blf_revolve2_pilot.py \
    --robot "$ROBOT" --fitness "$FITNESS" --terrain "$TERRAIN" \
    --coupling neighbor \
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
