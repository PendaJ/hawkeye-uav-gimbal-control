#!/usr/bin/env bash
# Hawkeye: run/resume a Gazebo counterpart of manuscript Exp. 2,3,4,5 or 7.
# Uses the SAME metric implementation, 15 s lateral scenario, fixed P90
# references, random seed, parameter bounds and 20 x 11 budget as Gazebo Exp. 6.
# Does not change the optimization algorithm, simulator, or controller.
set -eo pipefail

WS="${HAWKEYE_WS:-$HOME/hawkeye_x3_ws}"
WORKFLOW="$WS/hawkeye_workflow"
OPT="$WS/hawkeye_npso_gazebo/npso_gazebo.py"
TRIAL="$WS/hawkeye_npso_gazebo/npso_trial.py"
CTRL="$HOME/Downloads/hawkeye_gazebo_sspd_controller_safe.py"
QOS="$WS/rosbags/qos_override.yaml"
NORMALIZERS="$WS/rosbags/hawkeye_workflow/normalizers_lateral_15s.json"
BASE="$WS/paper_results"
SEED=42
DURATION=15
PARTICLES=20
ROUNDS=11

usage() {
  echo "Usage: $0 EXP_NUMBER" >&2
  echo "EXP_NUMBER: 2 (MAE), 3 (MAE+Vol), 4 (MAE+ZC), 5 (MAE+Acc), 7 (reweighted)" >&2
  exit 2
}
[[ $# == 1 ]] || usage
case "$1" in
  2) EXP=2; LABEL=MAE;           WEIGHTS=(1 0 0 0) ;;
  3) EXP=3; LABEL=MAE_Volatility; WEIGHTS=(1 1 0 0) ;;
  4) EXP=4; LABEL=MAE_ZC;         WEIGHTS=(1 0 0 1) ;;
  5) EXP=5; LABEL=MAE_Acceleration;WEIGHTS=(1 0 1 0) ;;
  7) EXP=7; LABEL=Reweighted;     WEIGHTS=(1 0.5 0.5 1) ;;
  *) usage ;;
esac

for path in "$WORKFLOW/hawkeye" "$OPT" "$TRIAL" "$CTRL" "$QOS" "$NORMALIZERS" "$WS/.venv_yolo/bin/python"; do
  if [[ ! -f "$path" ]]; then
    echo "Missing prerequisite: $path" >&2
    exit 1
  fi
done

# ROS setup scripts expect some shell variables not to be defined.
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

STATUS="$($WORKFLOW/hawkeye status)"
echo "$STATUS"
if ! grep -q 'Mode: lateral' <<< "$STATUS"; then
  echo 'The active Hawkeye world must be lateral.' >&2
  exit 1
fi
for SERVICE in gazebo bridge adapter tracker; do
  if ! grep -Eq "^${SERVICE}[[:space:]]+RUNNING" <<< "$STATUS"; then
    echo "Hawkeye service $SERVICE is not running." >&2
    exit 1
  fi
done

OUT="$BASE/Gazebo_Exp${EXP}_${LABEL}"
mkdir -p "$OUT"
LOCK="$WS/rosbags/hawkeye_workflow/trial.lock"
if [[ -e "$LOCK" ]]; then
  echo "Another workflow trial/optimizer holds: $LOCK" >&2
  echo 'Do not delete an active lock.' >&2
  exit 1
fi
printf '%s\n' "$$ Gazebo_Exp${EXP}_${LABEL}" > "$LOCK"
trap 'rm -f "$LOCK"' EXIT

# Preserve paper-relevant metadata and executable code alongside the data.
cp -p "$NORMALIZERS" "$OUT/normalizers_lateral_15s.json"
cp -p "$OPT" "$OUT/npso_gazebo_source.py"
cp -p "$TRIAL" "$OUT/npso_trial_source.py"
sha256sum "$OPT" "$TRIAL" "$CTRL" "$QOS" "$NORMALIZERS" > "$OUT/sha256_sources.txt"
cat > "$OUT/experiment_definition.txt" <<EOF
Gazebo counterpart to manuscript Experiment $EXP ($LABEL)
Weights (MAE, volatility, second difference, zero crossings): ${WEIGHTS[*]}
NPSO: $PARTICLES particles x $ROUNDS rounds = $((PARTICLES * ROUNDS)) evaluations
Seed: $SEED
World mode: lateral
Duration: $DURATION s per candidate
Initial target pixel offsets: (80, 28), tolerance 5 px
Control: Hawkeye 2-DoF observer-based SS-PD; GPU YOLO+ByteTrack
Metrics: EXACTLY the same implementation as prior Gazebo Exp. 6
Normalization: same fixed pilot reference_constants as Gazebo Exp. 6
Caveat: not a scene-identical replication of the Isaac Sim setup.
EOF

# Restarting the script after a Gazebo/YOLO interruption resumes its checkpoint.
mapfile -t FOUND < <(find "$OUT" -mindepth 1 -maxdepth 1 -type d -name "npso_seed_${SEED}_*" | sort)
if (( ${#FOUND[@]} > 1 )); then
  echo "Multiple sessions found in $OUT. Choose/resolve the desired session manually." >&2
  exit 1
fi
RESUME=()
if (( ${#FOUND[@]} == 1 )); then
  RUN="${FOUND[0]}"
  if [[ -f "$RUN/best_parameters.json" ]]; then
    echo "Experiment $EXP already complete: $RUN"
    cat "$RUN/best_parameters.json"
    exit 0
  fi
  if [[ ! -f "$RUN/checkpoint.json" ]]; then
    echo "Session exists but no checkpoint: $RUN" >&2
    echo 'Inspect its logs before making a new session. No data were overwritten.' >&2
    exit 1
  fi
  RESUME=(--resume "$RUN")
  echo "Resuming Experiment $EXP from $RUN"
else
  echo "Starting new Gazebo Experiment $EXP ($LABEL)"
fi

LOG="$OUT/console_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to $LOG"
"$WS/.venv_yolo/bin/python" -u "$OPT" \
  --mode optimize --world-mode lateral --repro \
  --duration "$DURATION" \
  --normalizers "$NORMALIZERS" \
  --particles "$PARTICLES" --rounds "$ROUNDS" \
  --seed "$SEED" \
  --weights "${WEIGHTS[@]}" \
  --controller "$CTRL" --qos "$QOS" \
  --output "$OUT" \
  "${RESUME[@]}" 2>&1 | tee "$LOG"

mapfile -t DONE < <(find "$OUT" -mindepth 2 -maxdepth 2 -name best_parameters.json -type f | sort)
if (( ${#DONE[@]} == 1 )); then
  cp -p "${DONE[0]}" "$OUT/best_parameters.json"
  cp -p "$(dirname "${DONE[0]}")/evaluations.csv" "$OUT/evaluations.csv"
  echo "RESULTS SAVED: $OUT"
  echo "Individual ROS bags and full checkpoints are in $(dirname "${DONE[0]}")"
else
  echo 'Optimizer returned success, but the final best_parameters.json was not found uniquely.' >&2
  exit 1
fi
