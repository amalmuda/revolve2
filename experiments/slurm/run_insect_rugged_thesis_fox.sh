#!/bin/bash
#SBATCH --job-name=ins_rug_t
#SBATCH --account=ec29
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-60
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Spider on rugged terrain, lambda=0, drag detection skipped during evolution.
# 2 conditions x 30 seeds = 60 tasks
# Conditions: 0=phi/uncoupled, 1=base/blf

ROBOT="insect"
NUM_SEEDS=30
NU=0.5
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
LAMBDA=0
RESULTS_DIR="results/insect_bonardi_rugged_nu0.5_w1"

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))
SEED_IDX=$((TASK_IDX % NUM_SEEDS))
COND_IDX=$(( TASK_IDX / NUM_SEEDS ))

RUN_NUM=$((SEED_IDX + 1))
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

case $COND_IDX in
    0) COUPLING="uncoupled"; PHI_FLAG="--evolve-phi0"; VARIANT="phi"  ;;
    1) COUPLING="blf";       PHI_FLAG="";              VARIANT="base" ;;
esac

cd ~/revolve2/experiments
EXPERIMENT_NAME="${ROBOT}_bonardi_${VARIANT}_${COUPLING}_lambda${LAMBDA}_nu${NU}_w${W}"
LOG_DIR="slurm/logs/insect_rugged_thesis/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "$EXPERIMENT_NAME (rugged, no-drag) run $RUN_NUM seed $SEED"

source ~/myenv/bin/activate

python -u evolve_bonardi.py \
    --robot $ROBOT --coupling $COUPLING --lambda $LAMBDA \
    --nu $NU --w $W --sim-time $SIM_TIME \
    --generations $GENERATIONS --population $POPULATION --workers $WORKERS \
    --seed $SEED --run-num $RUN_NUM --results-dir $RESULTS_DIR \
    --terrain rugged --skip-drag \
    $PHI_FLAG

echo "Done"
