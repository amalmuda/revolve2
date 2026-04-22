#!/bin/bash
# Run gecko Kuramoto experiment locally.
# 3 couplings x 2 lambdas x 5 seeds = 30 runs, sequentially.
set -e

cd "$(dirname "$0")"
source ~/masteroppgave/.venv/bin/activate

COUPLINGS=("uncoupled" "neighbor" "blf")
LAMBDAS=(0 2)
SEEDS=(1 2 3 4 5)

HZ=0.2
SIM_TIME=30
GENERATIONS=300
POPULATION=30
WORKERS=15
RESULTS_DIR="results/gecko_kuramoto_hz0.2_local"

LOG_DIR="logs/gecko_kuramoto_local"
mkdir -p "$LOG_DIR"

TOTAL=0
for _ in "${COUPLINGS[@]}"; do for _ in "${LAMBDAS[@]}"; do for _ in "${SEEDS[@]}"; do TOTAL=$((TOTAL+1)); done; done; done

echo "Starting $TOTAL runs at $(date)" | tee "$LOG_DIR/master.log"

COUNT=0
for COUPLING in "${COUPLINGS[@]}"; do
    for LAMBDA in "${LAMBDAS[@]}"; do
        for SEED in "${SEEDS[@]}"; do
            COUNT=$((COUNT+1))
            NAME="${COUPLING}_lambda${LAMBDA}_run${SEED}"
            RUN_LOG="$LOG_DIR/${NAME}.log"
            echo "[$COUNT/$TOTAL] $NAME starting at $(date)" | tee -a "$LOG_DIR/master.log"

            python -u evolve_kuramoto.py \
                --robot gecko \
                --coupling "$COUPLING" \
                --lambda "$LAMBDA" \
                --hz "$HZ" \
                --sim-time "$SIM_TIME" \
                --generations "$GENERATIONS" \
                --population "$POPULATION" \
                --workers "$WORKERS" \
                --seed "$((SEED * 10000 + RANDOM % 10000))" \
                --run-num "$SEED" \
                --results-dir "$RESULTS_DIR" \
                > "$RUN_LOG" 2>&1

            echo "[$COUNT/$TOTAL] $NAME done at $(date)" | tee -a "$LOG_DIR/master.log"
        done
    done
done

echo "All $TOTAL runs finished at $(date)" | tee -a "$LOG_DIR/master.log"
