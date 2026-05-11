#!/bin/bash
#SBATCH --job-name=evnu_fix
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-2
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Rerun the 2 failed tasks from job 3452597:
#   task 1: queen, flat, base/blf,    run 8
#   task 2: ege2,  rugged, phi/uncoupled, run 9
# Same local-tmp + copy-back pattern as the original evolvenu wrapper.

NU=0.5
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
LAMBDA=0

case $SLURM_ARRAY_TASK_ID in
    1)
        ROBOT="queen"; TERRAIN="flat"; COUPLING="blf"
        PHI_FLAG=""; COND_LABEL="base_blf"
        RUN_NUM=8
        ;;
    2)
        ROBOT="ege2"; TERRAIN="rugged"; COUPLING="uncoupled"
        PHI_FLAG="--evolve-phi0"; COND_LABEL="phi_uncoupled"
        RUN_NUM=9
        ;;
    *)
        echo "Bad task id: $SLURM_ARRAY_TASK_ID" >&2; exit 1 ;;
esac

SEED=$((SLURM_ARRAY_TASK_ID * 90000 + RANDOM % 10000))

if [ "$TERRAIN" = "rugged" ]; then
    TERRAIN_FLAG="--terrain rugged"
else
    TERRAIN_FLAG="--terrain flat"
fi

NFS_RESULTS_DIR="results/${ROBOT}_bonardi_evolvenu_${TERRAIN}_nu0.5_w1"

cd ~/revolve2/experiments

LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${SLURM_ARRAY_TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/${ROBOT}_bonardi_evolvenu_${TERRAIN}_nu0.5_w1"
mkdir -p "$LOCAL_RESULTS_DIR"

LOG_DIR="slurm/logs/evolvenu_thesis/${ROBOT}_${TERRAIN}_${COND_LABEL}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "RERUN ${ROBOT} ${TERRAIN} ${COND_LABEL} +evolve_nu lambda=0 (no-drag) run ${RUN_NUM} seed ${SEED}"

source ~/myenv/bin/activate

# Don't fail-fast: NFS hiccup at end shouldn't lose the npy from local tmp.
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
