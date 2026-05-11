#!/bin/bash
#SBATCH --job-name=v_rugged
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-360%24
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Same Experiment 1 design (phi/uncoupled, base/blf; fixed w=1, fixed nu=0.5)
# but on a MORE rugged terrain: same Perlin pattern, max height doubled
# (10 cm peak vs 5 cm in the original rugged batch).
#
# 6 robots x 2 conditions x 30 seeds = 360 tasks.
# Conditions: 0 = phi/uncoupled (--evolve-phi0), 1 = base/blf.
# Index: robot 0..5 (60 tasks each); within robot: condition 0..1 (30 each).
#
# lambda=0, --skip-drag (drag measured post-hoc).
# Writes evolution outputs to LOCAL node tmp to avoid SQLite NFS contention,
# then copies final npy + sqlite to NFS at end of each run.
# Concurrency capped at 24.

ROBOTS=(spider gecko babya queen insect ege2)
NUM_SEEDS=30

NU=0.5
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
LAMBDA=0

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

ROBOT_IDX=$((TASK_IDX / 60))
INNER=$((TASK_IDX % 60))
COND_IDX=$((INNER / NUM_SEEDS))
SEED_IDX=$((INNER % NUM_SEEDS))

ROBOT=${ROBOTS[$ROBOT_IDX]}
RUN_NUM=$((SEED_IDX + 1))
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

case $COND_IDX in
    0) COUPLING="uncoupled"; PHI_FLAG="--evolve-phi0"; COND_LABEL="phi_uncoupled" ;;
    1) COUPLING="blf";       PHI_FLAG="";              COND_LABEL="base_blf"      ;;
esac

# Auto-generated experiment_name from evolve_bonardi.py:
#   phi/uncoupled -> {robot}_bonardi_phi_uncoupled_lambda0_nu0.5_w1.0
#   base/blf      -> {robot}_bonardi_base_blf_lambda0_nu0.5_w1.0
NFS_RESULTS_DIR="results/${ROBOT}_bonardi_very_rugged_nu0.5_w1"

cd ~/revolve2/experiments

# Local node tmp for evolution writes (no NFS contention)
LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/${ROBOT}_bonardi_very_rugged_nu0.5_w1"
mkdir -p "$LOCAL_RESULTS_DIR"

LOG_DIR="slurm/logs/very_rugged_thesis/${ROBOT}_${COND_LABEL}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "${ROBOT} very_rugged ${COND_LABEL} lambda=0 (no-drag) run ${RUN_NUM} seed ${SEED}"
echo "  local_dir=${LOCAL_RESULTS_DIR}"
echo "  nfs_dir=${NFS_RESULTS_DIR}"

source ~/myenv/bin/activate

set -e
python -u evolve_bonardi.py \
    --robot "$ROBOT" --coupling "$COUPLING" --lambda "$LAMBDA" \
    --nu "$NU" --w "$W" --sim-time "$SIM_TIME" \
    --generations "$GENERATIONS" --population "$POPULATION" --workers "$WORKERS" \
    --seed "$SEED" --run-num "$RUN_NUM" --results-dir "$LOCAL_RESULTS_DIR" \
    --terrain very_rugged --skip-drag \
    $PHI_FLAG
PYRC=$?
set +e

echo "evolve_bonardi rc=$PYRC"
echo "local outputs:"
find "$LOCAL_RESULTS_DIR" -name "best_params_run_${RUN_NUM}.npy" -o -name "run_${RUN_NUM}.sqlite" || true

# Copy back. Experiment subdir name is auto-generated; mirror it under NFS.
for src_dir in "$LOCAL_RESULTS_DIR"/*/; do
    sub=$(basename "$src_dir")
    dst_dir="$NFS_RESULTS_DIR/$sub"
    mkdir -p "$dst_dir"
    cp "$src_dir"/best_params_run_${RUN_NUM}.npy "$dst_dir/" 2>/dev/null || echo "WARN: no npy in $src_dir"
    cp "$src_dir"/run_${RUN_NUM}.sqlite        "$dst_dir/" 2>/dev/null || echo "WARN: no sqlite in $src_dir"
    echo "copied to $dst_dir:"; ls -la "$dst_dir"/*run_${RUN_NUM}.* 2>/dev/null || true
done

# Also copy the terrain heights file once (only by task 1 for each robot — others will see it)
if [ -f "$LOCAL_RESULTS_DIR/terrain_heights.npy" ] && [ ! -f "$NFS_RESULTS_DIR/terrain_heights.npy" ]; then
    cp "$LOCAL_RESULTS_DIR/terrain_heights.npy" "$NFS_RESULTS_DIR/" 2>/dev/null || true
fi

rm -rf "$LOCAL_TMP/results"

echo "Done"
exit $PYRC
