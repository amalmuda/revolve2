#!/bin/bash
#SBATCH --job-name=ins_full_05
#SBATCH --account=ec29
#SBATCH --time=1:30:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-120
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Spider Bonardi 0.5 Hz, full grid:
# 6 conditions x 2 lambdas x 30 seeds = 360
# Conditions: 0=base_unc, 1=base_neigh, 2=base_blf, 3=phi_unc, 4=phi_neigh, 5=phi_blf

ROBOT="insect"
NUM_SEEDS=10
NU=0.5
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
RESULTS_DIR="results/insect_bonardi_nu0.5_w1"

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))
SEED_IDX=$((TASK_IDX % NUM_SEEDS))
LAMBDA_IDX=$(( (TASK_IDX / NUM_SEEDS) % 2 ))
COND_IDX=$(( TASK_IDX / (NUM_SEEDS * 2) ))

LAMBDAS=(0 1)
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 16))
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

case $COND_IDX in
    0) COUPLING="uncoupled"; PHI_FLAG="";              VARIANT="base" ;;
    1) COUPLING="neighbor";  PHI_FLAG="";              VARIANT="base" ;;
    2) COUPLING="blf";       PHI_FLAG="";              VARIANT="base" ;;
    3) COUPLING="uncoupled"; PHI_FLAG="--evolve-phi0"; VARIANT="phi"  ;;
    4) COUPLING="neighbor";  PHI_FLAG="--evolve-phi0"; VARIANT="phi"  ;;
    5) COUPLING="blf";       PHI_FLAG="--evolve-phi0"; VARIANT="phi"  ;;
esac

cd ~/revolve2/experiments
EXPERIMENT_NAME="${ROBOT}_bonardi_${VARIANT}_${COUPLING}_lambda${LAMBDA}_nu${NU}_w${W}"
LOG_DIR="slurm/logs/insect_bonardi_full_05hz/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "$EXPERIMENT_NAME run $RUN_NUM seed $SEED"

source ~/myenv/bin/activate

python -u evolve_bonardi.py \
    --robot $ROBOT --coupling $COUPLING --lambda $LAMBDA \
    --nu $NU --w $W --sim-time $SIM_TIME \
    --generations $GENERATIONS --population $POPULATION --workers $WORKERS \
    --seed $SEED --run-num $RUN_NUM --results-dir $RESULTS_DIR \
    $PHI_FLAG

echo "Done"
