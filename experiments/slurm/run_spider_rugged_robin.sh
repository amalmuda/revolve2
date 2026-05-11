#!/bin/bash
#SBATCH --job-name=spi_rug
#SBATCH --partition=main
#SBATCH --time=2:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-10
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Spider on rugged terrain, lambda=0, 5 seeds each:
#  tasks  1- 5 = phi/uncoupled (run_2..6, run_1 already exists)
#  tasks  6-10 = base/blf      (run_2..6)

ROBOT="spider"
NU=0.5
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
RESULTS_DIR="results/spider_bonardi_rugged_nu0.5_w1"

LAMBDA=0
TASK_IDX=$((SLURM_ARRAY_TASK_ID - 1))
COND_IDX=$((TASK_IDX / 5))
SEED_IDX=$((TASK_IDX % 5))
RUN_NUM=$((SEED_IDX + 2))   # run_2..6 (skip run_1, already done)
SEED=$((SLURM_ARRAY_TASK_ID * 13371 + RANDOM % 9999))

case $COND_IDX in
    0) COUPLING="uncoupled"; PHI_FLAG="--evolve-phi0"; VARIANT="phi"  ;;
    1) COUPLING="blf";       PHI_FLAG="";              VARIANT="base" ;;
esac

cd ~/revolve2/experiments
EXPERIMENT_NAME="${ROBOT}_bonardi_${VARIANT}_${COUPLING}_lambda${LAMBDA}_nu${NU}_w${W}"
LOG_DIR="slurm/logs/spider_rugged"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/${VARIANT}_${COUPLING}_run${RUN_NUM}.log" 2>&1

echo "$EXPERIMENT_NAME (rugged) seed $SEED"

source /opt/anaconda3/etc/profile.d/conda.sh
conda activate revolve2

python -u evolve_bonardi.py \
    --robot $ROBOT --coupling $COUPLING --lambda $LAMBDA \
    --nu $NU --w $W --sim-time $SIM_TIME \
    --generations $GENERATIONS --population $POPULATION --workers $WORKERS \
    --seed $SEED --run-num $RUN_NUM --results-dir $RESULTS_DIR \
    --terrain rugged --skip-drag \
    $PHI_FLAG

echo "Done"
