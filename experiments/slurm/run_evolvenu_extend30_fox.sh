#!/bin/bash
#SBATCH --job-name=evnu_x30
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-480%24
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Extend evolve_nu from 10 to 30 seeds per cell.
# Adds runs 11..30 for each (robot, terrain, controller) combination.
#
# 6 robots x 2 terrains x 2 conditions x 20 NEW seeds = 480 tasks.
# Layout: robot 0..5 (80 each); within robot: terrain 0..1 (40 each);
# within terrain: condition 0..1 (20 each); within condition: seed_offset 0..19.
# Run number = 11 + seed_offset (so 11..30).
#
# Same local-tmp + copy-back pattern as the original evolvenu wrapper.

ROBOTS=(spider gecko babya queen insect ege2)
TERRAINS=(flat rugged)
NUM_NEW_SEEDS=20

NU=0.5
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
LAMBDA=0

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

ROBOT_IDX=$((TASK_IDX / 80))
INNER1=$((TASK_IDX % 80))
TERR_IDX=$((INNER1 / 40))
INNER2=$((INNER1 % 40))
COND_IDX=$((INNER2 / NUM_NEW_SEEDS))
SEED_OFFSET=$((INNER2 % NUM_NEW_SEEDS))

ROBOT=${ROBOTS[$ROBOT_IDX]}
TERRAIN=${TERRAINS[$TERR_IDX]}
RUN_NUM=$((11 + SEED_OFFSET))      # runs 11..30
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

case $COND_IDX in
    0) COUPLING="uncoupled"; PHI_FLAG="--evolve-phi0"; COND_LABEL="phi_uncoupled" ;;
    1) COUPLING="blf";       PHI_FLAG="";              COND_LABEL="base_blf"      ;;
esac

if [ "$TERRAIN" = "rugged" ]; then
    TERRAIN_FLAG="--terrain rugged"
else
    TERRAIN_FLAG="--terrain flat"
fi

NFS_RESULTS_DIR="results/${ROBOT}_bonardi_evolvenu_${TERRAIN}_nu0.5_w1"

cd ~/revolve2/experiments

LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/${ROBOT}_bonardi_evolvenu_${TERRAIN}_nu0.5_w1"
mkdir -p "$LOCAL_RESULTS_DIR"

LOG_DIR="slurm/logs/evolvenu_thesis/${ROBOT}_${TERRAIN}_${COND_LABEL}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "${ROBOT} ${TERRAIN} ${COND_LABEL} +evolve_nu lambda=0 (no-drag) run ${RUN_NUM} seed ${SEED}"
echo "  local_dir=${LOCAL_RESULTS_DIR}"
echo "  nfs_dir=${NFS_RESULTS_DIR}"

source ~/myenv/bin/activate

python -u evolve_bonardi.py \
    --robot "$ROBOT" --coupling "$COUPLING" --lambda "$LAMBDA" \
    --nu "$NU" --w "$W" --sim-time "$SIM_TIME" \
    --generations "$GENERATIONS" --population "$POPULATION" --workers "$WORKERS" \
    --seed "$SEED" --run-num "$RUN_NUM" --results-dir "$LOCAL_RESULTS_DIR" \
    $TERRAIN_FLAG --skip-drag --evolve-nu \
    $PHI_FLAG
PYRC=$?
echo "evolve_bonardi rc=$PYRC"

echo "local outputs:"
find "$LOCAL_RESULTS_DIR" -name "best_params_run_${RUN_NUM}.npy" -o -name "run_${RUN_NUM}.sqlite" || true

# Glob each subdir under LOCAL_RESULTS_DIR (auto-named by evolve_bonardi.py)
for src_dir in "$LOCAL_RESULTS_DIR"/*/; do
    sub=$(basename "$src_dir")
    dst_dir="$NFS_RESULTS_DIR/$sub"
    mkdir -p "$dst_dir"
    cp "$src_dir"/best_params_run_${RUN_NUM}.npy "$dst_dir/" 2>/dev/null || echo "WARN: no npy in $src_dir"
    cp "$src_dir"/run_${RUN_NUM}.sqlite        "$dst_dir/" 2>/dev/null || echo "WARN: no sqlite in $src_dir"
    echo "copied to $dst_dir:"; ls -la "$dst_dir"/*run_${RUN_NUM}.* 2>/dev/null || true
done

rm -rf "$LOCAL_TMP/results"

echo "Done"
exit $PYRC
