#!/bin/bash
#SBATCH --job-name=balance
#SBATCH --account=ec29
#SBATCH --time=1:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --mem-per-cpu=4G
#SBATCH --array=1-720
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# ============================================
# COMPUTE BALANCE METRIC (Kargar et al. 2021)
# Re-simulate best individual from each run
# 2 robots × 3 couplings × 4 lambdas × 30 seeds = 720
# ============================================

ROBOTS=("spider" "gecko")
COUPLINGS=("uncoupled" "neighbor" "blf")
COUPLING_LABELS=("No_coupling" "Neighbour" "Structured")
LAMBDAS=(0 1 2 3)
NUM_SEEDS=30

TASK_ID=$SLURM_ARRAY_TASK_ID
TASK_IDX=$((TASK_ID - 1))

SEED_IDX=$((TASK_IDX % NUM_SEEDS))
REMAINDER=$((TASK_IDX / NUM_SEEDS))
LAMBDA_IDX=$((REMAINDER % 4))
REMAINDER2=$((REMAINDER / 4))
COUPLING_IDX=$((REMAINDER2 % 3))
ROBOT_IDX=$((REMAINDER2 / 3))

ROBOT=${ROBOTS[$ROBOT_IDX]}
COUPLING=${COUPLINGS[$COUPLING_IDX]}
COUPLING_LABEL=${COUPLING_LABELS[$COUPLING_IDX]}
LAMBDA=${LAMBDAS[$LAMBDA_IDX]}
RUN_NUM=$((SEED_IDX + 1))

cd ~/revolve2/experiments

LOG_DIR="slurm/logs/balance"
mkdir -p "$LOG_DIR"
exec > "${LOG_DIR}/task_${TASK_ID}.log" 2>&1

PARAMS_PATH="results/final_experiments/${ROBOT}_ode_cpg_${COUPLING}_lambda${LAMBDA}_dragging/best_params_run_${RUN_NUM}.npy"
OUTPUT_DIR="results/balance_results"
mkdir -p "$OUTPUT_DIR"
OUTPUT_FILE="${OUTPUT_DIR}/balance_${TASK_ID}.txt"

echo "Task $TASK_ID: $ROBOT $COUPLING_LABEL lambda=$LAMBDA run=$RUN_NUM"

source ~/myenv/bin/activate

python3 -c "
from compute_balance import compute_balance
b, d, dr = compute_balance('${ROBOT}', '${COUPLING}', '${PARAMS_PATH}')
with open('${OUTPUT_FILE}', 'w') as f:
    f.write('${ROBOT}\t${COUPLING_LABEL}\t${LAMBDA}\t${RUN_NUM}\t%.6f\t%.4f\t%.4f\n' % (b, d, dr))
print('Balance=%.4f Distance=%.2f Dragging=%.1f%%' % (b, d, dr*100))
"

echo "Done: $OUTPUT_FILE"
