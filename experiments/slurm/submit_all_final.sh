#!/bin/bash
# ============================================
# Submit all 9 final experiments
# 9 robots x 3 couplings x 2 lambdas x 30 seeds
# = 1,620 total runs (180 tasks per robot)
# Max 30 concurrent tasks per robot (5 seeds per config)
# ============================================

cd ~/masteroppgave/revolve2/experiments

echo "Submitting final experiments..."
echo ""

for robot in spider gecko babya babyb ant queen park insect snake; do
    JOB_ID=$(sbatch --parsable slurm/run_final_${robot}.sh)
    echo "  ${robot}: submitted (job ${JOB_ID}, 180 tasks)"
done

echo ""
echo "All 9 robots submitted. Total: 1,620 runs."
echo "Results will be in: results/final_v2/"
echo ""
echo "Monitor with: squeue -u \$USER"
