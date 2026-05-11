#!/bin/bash
#SBATCH --job-name=spi_repl
#SBATCH --partition=main
#SBATCH --time=1:30:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=25
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-1
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Single replacement run for the corrupted spider phi/blf l=1 seed in the backup.
# Saved as run_99 so it doesn't collide with robin's existing 1-5.

ROBOT="spider"
NU=0.5
W=1
SIM_TIME=30
GENERATIONS=300
POPULATION=25
WORKERS=25
RESULTS_DIR="results/spider_bonardi_replacement_nu0.5_w1"

LAMBDA=1
COUPLING="blf"
PHI_FLAG="--evolve-phi0"
VARIANT="phi"
RUN_NUM=99
SEED=$((RANDOM * 31 + 9999))

cd ~/revolve2/experiments
EXPERIMENT_NAME="${ROBOT}_bonardi_${VARIANT}_${COUPLING}_lambda${LAMBDA}_nu${NU}_w${W}"
LOG_DIR="slurm/logs/spider_replacement"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/run_${RUN_NUM}.log" 2>&1

echo "$EXPERIMENT_NAME run $RUN_NUM seed $SEED"

source /opt/anaconda3/etc/profile.d/conda.sh
conda activate revolve2

python -u evolve_bonardi.py \
    --robot $ROBOT --coupling $COUPLING --lambda $LAMBDA \
    --nu $NU --w $W --sim-time $SIM_TIME \
    --generations $GENERATIONS --population $POPULATION --workers $WORKERS \
    --seed $SEED --run-num $RUN_NUM --results-dir $RESULTS_DIR \
    $PHI_FLAG

echo "Done"
