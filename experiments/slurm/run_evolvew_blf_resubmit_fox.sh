#!/bin/bash
#SBATCH --job-name=evolvew_r
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-67%24
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Resubmit ONLY the runs missing best_params_run_*.npy.
# Reads slurm/missing_evolvew_runs.txt (one "robot terrain seed" per line).
# Writes SQLite + outputs to LOCAL node tmp during evolution to avoid NFS
# contention, then copies back to the shared results dir on completion.
# %24 limits concurrency to 24 tasks at a time to further reduce contention.

NU=0.5
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
LAMBDA=0
COUPLING="blf"

cd ~/revolve2/experiments
MANIFEST="slurm/missing_evolvew_runs.txt"

TASK_ID=$SLURM_ARRAY_TASK_ID
LINE=$(sed -n "${TASK_ID}p" "$MANIFEST")
if [ -z "$LINE" ]; then
    echo "No manifest line for task $TASK_ID" >&2
    exit 1
fi

ROBOT=$(echo "$LINE" | awk '{print $1}')
TERRAIN=$(echo "$LINE" | awk '{print $2}')
RUN_NUM=$(echo "$LINE" | awk '{print $3}')
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

if [ "$TERRAIN" = "rugged" ]; then
    TERRAIN_FLAG="--terrain rugged"
else
    TERRAIN_FLAG="--terrain flat"
fi

EXPERIMENT_NAME="${ROBOT}_bonardi_w_${COUPLING}_lambda${LAMBDA}_nu${NU}_w${W}"
NFS_RESULTS_DIR="results/${ROBOT}_bonardi_evolvew_${TERRAIN}_nu0.5_w1"
NFS_FULL_DIR="${NFS_RESULTS_DIR}/${EXPERIMENT_NAME}"

# Local node tmp for evolution writes (no NFS contention)
LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/${ROBOT}_bonardi_evolvew_${TERRAIN}_nu0.5_w1"
LOCAL_FULL_DIR="${LOCAL_RESULTS_DIR}/${EXPERIMENT_NAME}"
mkdir -p "$LOCAL_FULL_DIR"

LOG_DIR="slurm/logs/evolvew_thesis/${ROBOT}_${TERRAIN}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "RESUBMIT ${ROBOT} ${TERRAIN} evolve-w blf lambda=0 run ${RUN_NUM} seed ${SEED}"
echo "  local_dir=${LOCAL_FULL_DIR}"
echo "  nfs_dir=${NFS_FULL_DIR}"

source ~/myenv/bin/activate

set -e
python -u evolve_bonardi.py \
    --robot "$ROBOT" --coupling "$COUPLING" --lambda "$LAMBDA" \
    --nu "$NU" --w "$W" --sim-time "$SIM_TIME" \
    --generations "$GENERATIONS" --population "$POPULATION" --workers "$WORKERS" \
    --seed "$SEED" --run-num "$RUN_NUM" --results-dir "$LOCAL_RESULTS_DIR" \
    $TERRAIN_FLAG --skip-drag --evolve-w
PYRC=$?
set +e

echo "evolve_bonardi rc=$PYRC"
echo "local outputs:"; ls -la "$LOCAL_FULL_DIR" || true

mkdir -p "$NFS_FULL_DIR"
cp "$LOCAL_FULL_DIR"/best_params_run_${RUN_NUM}.npy "$NFS_FULL_DIR/" || echo "WARN: no npy"
cp "$LOCAL_FULL_DIR"/run_${RUN_NUM}.sqlite "$NFS_FULL_DIR/" || echo "WARN: no sqlite"
echo "copied:"; ls -la "$NFS_FULL_DIR"/*run_${RUN_NUM}.* || true

rm -rf "$LOCAL_TMP/results"

echo "Done"
exit $PYRC
