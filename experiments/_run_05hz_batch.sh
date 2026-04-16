#!/bin/bash
# Run 8 polar spider experiments at 0.5 Hz
set -e
source ~/masteroppgave/.venv/bin/activate
cd ~/masteroppgave/revolve2/experiments

for COUPLING in neighbor blf; do
  for DIR in xy y; do
    for LAMBDA in 0 1; do
      DIR_TAG="xy"
      if [ "$DIR" = "y" ]; then DIR_TAG="directed"; fi
      TAG="${COUPLING}_${DIR_TAG}_lam${LAMBDA}_05hz"
      echo "=========================================="
      echo "=== Running: ${TAG} ==="
      echo "=========================================="
      HZ=0.5 LAMBDA="$LAMBDA" COUPLING="$COUPLING" DIRECTED="$DIR" \
        python _evolve_polar_spider.py 2>&1 | tee "polar_spider_${TAG}.log"
    done
  done
done
echo "All 8 0.5Hz experiments done."
