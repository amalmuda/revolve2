#!/bin/bash
#SBATCH --job-name=thesis500
#SBATCH --account=ec29
#SBATCH --time=3:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-720%24
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Re-run of Experiment 1 design (phi/uncoupled, base/blf; fixed w=1, fixed nu=0.5)
# at 500 generations and 30 seeds, both terrains, all 6 morphologies.
# Per-gen convergence is written by python to convergence_run_N.csv (NFS-safe).
#
# 6 robots x 2 terrains x 2 conditions x 30 seeds = 720 tasks.
# Layout: robot 0..5 (120 each); within robot: terrain 0..1 (60 each);
# within terrain: condition 0..1 (30 each).
#
# lambda=0, --skip-drag (drag measured post-hoc).
# Local-tmp + copy-back for npy + sqlite + convergence CSV.

ROBOTS=(spider gecko babya queen insect ege2)
TERRAINS=(flat rugged)
NUM_SEEDS=30

NU=0.5
W=1
SIM_TIME=30
GENERATIONS=500
POPULATION=25
WORKERS=25
LAMBDA=0

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

ROBOT_IDX=$((TASK_IDX / 120))
INNER1=$((TASK_IDX % 120))
TERR_IDX=$((INNER1 / 60))
INNER2=$((INNER1 % 60))
COND_IDX=$((INNER2 / NUM_SEEDS))
SEED_IDX=$((INNER2 % NUM_SEEDS))

ROBOT=${ROBOTS[$ROBOT_IDX]}
TERRAIN=${TERRAINS[$TERR_IDX]}
RUN_NUM=$((SEED_IDX + 1))
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

NFS_RESULTS_DIR="results/${ROBOT}_bonardi_500gen_thesis_${TERRAIN}_nu0.5_w1"

cd ~/revolve2/experiments

LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/${ROBOT}_bonardi_500gen_thesis_${TERRAIN}_nu0.5_w1"
mkdir -p "$LOCAL_RESULTS_DIR"

LOG_DIR="slurm/logs/thesis_500gen/${ROBOT}_${TERRAIN}_${COND_LABEL}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "${ROBOT} ${TERRAIN} ${COND_LABEL} 500gen lambda=0 (no-drag) run ${RUN_NUM} seed ${SEED}"
echo "  local_dir=${LOCAL_RESULTS_DIR}"
echo "  nfs_dir=${NFS_RESULTS_DIR}"

source ~/myenv/bin/activate

python -u evolve_bonardi.py \
    --robot "$ROBOT" --coupling "$COUPLING" --lambda "$LAMBDA" \
    --nu "$NU" --w "$W" --sim-time "$SIM_TIME" \
    --generations "$GENERATIONS" --population "$POPULATION" --workers "$WORKERS" \
    --seed "$SEED" --run-num "$RUN_NUM" --results-dir "$LOCAL_RESULTS_DIR" \
    $TERRAIN_FLAG --skip-drag \
    $PHI_FLAG
PYRC=$?
echo "evolve_bonardi rc=$PYRC"

# Glob each subdir under LOCAL_RESULTS_DIR (auto-named by evolve_bonardi.py)
for src_dir in "$LOCAL_RESULTS_DIR"/*/; do
    sub=$(basename "$src_dir")
    dst_dir="$NFS_RESULTS_DIR/$sub"
    mkdir -p "$dst_dir"
    cp "$src_dir"/best_params_run_${RUN_NUM}.npy        "$dst_dir/" 2>/dev/null || echo "WARN: no npy"
    cp "$src_dir"/run_${RUN_NUM}.sqlite                 "$dst_dir/" 2>/dev/null || echo "WARN: no sqlite"
    cp "$src_dir"/convergence_run_${RUN_NUM}.csv        "$dst_dir/" 2>/dev/null || echo "WARN: no convergence csv"
    echo "copied to $dst_dir:"
    ls -la "$dst_dir"/*run_${RUN_NUM}.* 2>/dev/null || true
done

rm -rf "$LOCAL_TMP/results"

echo "Done"
exit $PYRC
