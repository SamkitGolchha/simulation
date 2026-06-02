#!/bin/bash
# Launch 5 cuboid 20N seeds in parallel via nohup. Survives shell exit.
# Logs: output/20N_force/logs/sim_<seed>.log
set -e
cd "$(dirname "$0")/.."

PYTHON="/Users/samkit/anaconda3/envs/chrono/bin/python"
DYLD="/Users/samkit/anaconda3/envs/chrono/lib/libomp.dylib"
LABEL="20N_force"
LOG_DIR="output/${LABEL}/logs"
mkdir -p "${LOG_DIR}"

for SEED in 1 2 3 4 5; do
    LOG="${LOG_DIR}/sim_${SEED}.log"
    OMP_NUM_THREADS=1 DYLD_INSERT_LIBRARIES="${DYLD}" \
      nohup "${PYTHON}" scripts/run_seed.py --seed "${SEED}" \
      > "${LOG}" 2>&1 &
    PID=$!
    disown
    echo "Launched seed ${SEED} (PID ${PID}) -> ${LOG}"
done
echo "All 5 seeds detached. Tail logs to monitor."
