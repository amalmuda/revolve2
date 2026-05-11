#!/bin/bash
#SBATCH --job-name=phi_vs_blf
#SBATCH --account=ec29
#SBATCH --time=3:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-480%24
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Pilot: Bonardi CPG, phi/uncoupled vs base/BLF, 4 fitness functions, flat terrain.
# 6 robots x 2 conditions x 4 fitness x 10 seeds = 480 tasks.
# Layout: robot 0..5 (80 each); within robot: condition 0..1 (40 each);
# within condition: fitness 0..3 (10 each).

ROBOTS=(spider gecko babya queen insect ege2)
FITNESSES=(f1 f2 f3 f4)
NUM_SEEDS=10

SIM_TIME=30
GENERATIONS=500
POPULATION=25
WORKERS=25

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

ROBOT_IDX=$((TASK_IDX / 80))
INNER1=$((TASK_IDX % 80))
COND_IDX=$((INNER1 / 40))
INNER2=$((INNER1 % 40))
FIT_IDX=$((INNER2 / NUM_SEEDS))
SEED_IDX=$((INNER2 % NUM_SEEDS))

ROBOT=${ROBOTS[$ROBOT_IDX]}
FITNESS=${FITNESSES[$FIT_IDX]}
RUN_NUM=$((SEED_IDX + 1))
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

case $COND_IDX in
    0) COUPLING="uncoupled"; PHI_FLAG="--evolve-phi0"; COND_LABEL="phi_uncoupled" ;;
    1) COUPLING="blf";       PHI_FLAG="";              COND_LABEL="base_blf"      ;;
esac

NFS_RESULTS_DIR="results/bonardi_phi_vs_blf_500gen"

cd ~/revolve2/experiments

LOCAL_TMP="${SLURM_TMPDIR:-/tmp/slurm_${SLURM_JOB_ID}_${TASK_ID}}"
mkdir -p "$LOCAL_TMP"
LOCAL_RESULTS_DIR="$LOCAL_TMP/results/bonardi_phi_vs_blf_500gen"
mkdir -p "$LOCAL_RESULTS_DIR"

LOG_DIR="slurm/logs/phi_vs_blf_500gen/${ROBOT}_${COND_LABEL}_${FITNESS}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "${ROBOT} ${COND_LABEL} fitness=${FITNESS} run ${RUN_NUM} seed ${SEED}"

source ~/myenv/bin/activate

python -u evolve_blf_bonardi_pilot.py \
    --robot "$ROBOT" --fitness "$FITNESS" --terrain flat \
    --coupling "$COUPLING" $PHI_FLAG \
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
