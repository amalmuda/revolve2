#!/bin/bash
#SBATCH --job-name=spider_bon_phiw
#SBATCH --partition=main
#SBATCH --time=4:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-60
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Bonardi variant: + evolved phi0 AND + evolved w_ij
# Spider counts: uncoupled=24, neighbor=32, blf=44.

ROBOT="spider"
CONTROLLER="bonardi_phi_w"
COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 2)
NUM_SEEDS=10

NU=1
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25

RESULTS_DIR="results/spider_bonardi_phi_w_nu1"

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))
SEED_IDX=$((TASK_IDX % NUM_SEEDS))
REMAINDER=$((TASK_IDX / NUM_SEEDS))
LAMBDA_IDX=$((REMAINDER % 2))
COUPLING_IDX=$((REMAINDER / 2))
COUPLING=${COUPLINGS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))
SEED=$((TASK_ID * 10000 + RANDOM % 10000))

cd ~/revolve2/experiments
EXPERIMENT_NAME="${ROBOT}_${CONTROLLER}_${COUPLING}_lambda${LAMBDA}_nu${NU}"
LOG_DIR="slurm/logs/spider_bonardi_phi_w/${EXPERIMENT_NAME}"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "SPIDER BONARDI+phi+w - $EXPERIMENT_NAME run $RUN_NUM seed $SEED"

source /opt/anaconda3/etc/profile.d/conda.sh
conda activate revolve2

python -u evolve_bonardi.py \
    --robot $ROBOT --coupling $COUPLING --lambda $LAMBDA \
    --nu $NU --w $W --sim-time $SIM_TIME \
    --generations $GENERATIONS --population $POPULATION --workers $WORKERS \
    --seed $SEED --run-num $RUN_NUM --results-dir $RESULTS_DIR \
    --evolve-phi0 --evolve-w

echo "Done: $EXPERIMENT_NAME Run $RUN_NUM"
