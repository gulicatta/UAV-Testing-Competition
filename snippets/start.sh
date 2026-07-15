#!/bin/bash
# Single container for the whole budget: the generator keeps state and archive
TOTAL_BUDGET=${1:-20}
MISSION_ARG=${2:-mission1}
HEADLESS=${3:-true}
HOST_UID=$(id -u)
HOST_GID=$(id -g)

# accepts a short name ("mission3") or a full path; the file must be in
# case_studies/ with a nominal .plan and .ulg (used for adaptive init, or
# obtained with 1 bootstrap sim if absent)
case "$MISSION_ARG" in
  *.yaml) MISSION_FILE="$MISSION_ARG" ;;
  */*)    MISSION_FILE="${MISSION_ARG}.yaml" ;;
  *)      MISSION_FILE="case_studies/${MISSION_ARG}.yaml" ;;
esac
MISSION_NAME=$(basename "$MISSION_FILE" .yaml)
# Self-descriptive run folder: mission-generator[-sSEED]-date
RUN_DIR="./generated_tests/${MISSION_NAME}-${GENERATOR:-es}${SEED:+-s${SEED}}-$(date +%d-%m-%H-%M-%S)"
GEN_LABEL="(1+1)-ES"
[ "${GENERATOR:-es}" = "random" ] && GEN_LABEL="Random baseline"
[ "${ADAPTIVE:-1}" = "0" ] && GEN_LABEL="$GEN_LABEL [fixed box]"

echo "================================================"
echo "  UAV Test Generator — $GEN_LABEL"
echo "  Mission: $MISSION_NAME"
echo "  Budget: $TOTAL_BUDGET simulations"
echo "  N_RUNS: ${N_RUNS:-1}"
echo "  Initial seeds: ${INIT_SEEDS:-5}"
echo "  Restart after ${STAGNATION_LIMIT:-15} stagnant gen"
echo "  DTW diversity threshold: ${DTW_THRESHOLD:-1.5} m | Archive: ${ARCHIVE_CAP:-20} slots"
[ -n "${SEED:-}" ] && echo "  Seed: $SEED"
echo "  Delivery: $RUN_DIR/consegna/  (full history in all_fails/)"
echo "================================================"

# Clean up zombie containers from previous runs/crashes
docker ps -aq --filter ancestor=skhatiri/aerialist | xargs -r docker rm -f 2>/dev/null

mkdir -p ~/UAV-Testing-Competition/snippets/generated_tests

if [ "$HEADLESS" = "false" ]; then
    xhost +local:docker
    DOCKER_DISPLAY="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
    HEADLESS_SED="sed -i 's/headless: true/headless: false/' $MISSION_FILE &&"
else
    DOCKER_DISPLAY=""
    HEADLESS_SED=""
fi

# A SINGLE run: the ES keeps going uninterrupted, no reset. --rm cleans up at the end.
CMD="cd /src/aerialist && \
  cp /workspace/testcase.py . && \
  cp /workspace/config.py . && \
  cp /workspace/geometry.py . && \
  cp /workspace/models.py . && \
  cp /workspace/es_generator.py . && \
  cp /workspace/random_generator.py . && \
  cp /workspace/cli.py . && \
  cp /workspace/plot_official.py . && \
  cp -r /workspace/case_studies . && \
  $HEADLESS_SED \
  ( AGENT=local N_RUNS=${N_RUNS:-1} TEST_TIMEOUT=${TEST_TIMEOUT:-500} INIT_SEEDS=${INIT_SEEDS:-5} STAGNATION_LIMIT=${STAGNATION_LIMIT:-15} PYTHONUNBUFFERED=1 \
      python3 -u cli.py generate $MISSION_FILE $TOTAL_BUDGET ; rc=\$? ; \
    chown -R $HOST_UID:$HOST_GID generated_tests 2>> docker_permissions_error.log || true ; \
    exit \$rc )"

# auto-resume: if the container crashes it restarts and resumes from the
# checkpoint (global budget, only reruns the interrupted test). -it is
# needed by local PX4/Gazebo. Ctrl+C: the trap stops the restart loop
# (Aerialist needs a couple of Ctrl+C to close the container).
STOP=0
trap 'STOP=1' INT TERM
MAX_RESTARTS=${MAX_RESTARTS:-12}
RESTARTS=0
while [ "$STOP" = "0" ] ; do
  docker run --rm -it $DOCKER_DISPLAY \
    -e TESTS_FOLDER="$RUN_DIR/" \
    -e ADAPTIVE=${ADAPTIVE:-1} \
    -e SEED="${SEED:-}" \
    -e N_RUNS=${N_RUNS:-1} \
    -e TEST_TIMEOUT=${TEST_TIMEOUT:-500} \
    -e GENERATOR="${GENERATOR:-es}" \
    -e INIT_SEEDS=${INIT_SEEDS:-5} \
    -e STAGNATION_LIMIT=${STAGNATION_LIMIT:-15} \
    -e DTW_THRESHOLD=${DTW_THRESHOLD:-1.5} \
    -e TRAJ_N=${TRAJ_N:-100} \
    -e ARCHIVE_CAP=${ARCHIVE_CAP:-20} \
    -e TRAP_PROB=${TRAP_PROB:-0.7} \
    -e INTENSIFY_PROB=${INTENSIFY_PROB:-0.4} \
    -e CONFIRM_BELOW=${CONFIRM_BELOW:-1.0} \
    -e CONFIRM_EXTRA_RUNS=${CONFIRM_EXTRA_RUNS:-1} \
    -v ~/UAV-Testing-Competition/snippets:/workspace \
    -v ~/UAV-Testing-Competition/snippets/generated_tests:/src/aerialist/generated_tests \
    skhatiri/aerialist bash -lc "$CMD"
  EXIT_CODE=$?
  if [ "$STOP" = "1" ]; then
    echo -e "\nStopped by the user (no restart)."
    break
  fi
  if [ $EXIT_CODE -eq 0 ]; then
    echo -e "\n Done (budget exhausted)."
    break
  fi
  RESTARTS=$((RESTARTS + 1))
  if [ $RESTARTS -ge $MAX_RESTARTS ]; then
    echo -e "\n Reached the limit of $MAX_RESTARTS restarts. Stopping (fails found so far remain saved)."
    break
  fi
  echo -e "\n Crash detected (exit $EXIT_CODE). Restarting and resuming from checkpoint... ($RESTARTS/$MAX_RESTARTS)"
  sleep 3
done

echo "   Delivery in: ~/UAV-Testing-Competition/snippets/generated_tests/$(basename $RUN_DIR)/consegna/"
echo "   (full fail history in .../$(basename $RUN_DIR)/all_fails/)"
