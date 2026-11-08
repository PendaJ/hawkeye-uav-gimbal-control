#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/hawkeye_x3_ws"
WF="$WS/hawkeye_workflow"
OUT="$WS/paper_results/Gazebo_Exp6_Repeatability"
BASE="$WS/rosbags/hawkeye_workflow"

mkdir -p "$OUT"
cd "$WF"

STATUS="$(./hawkeye status)"
echo "$STATUS"

if ! grep -q "Mode: lateral" <<< "$STATUS"; then
    echo "ERROR: Start Gazebo in lateral mode."
    exit 1
fi

for service in gazebo bridge adapter tracker; do
    if ! grep -Eq "^${service}[[:space:]]+RUNNING" <<< "$STATUS"; then
        echo "ERROR: $service is not running."
        exit 1
    fi
done

cat > "$OUT/configuration.txt" <<EOF
Experiment: 6
Purpose: Fixed-gain closed-loop repeatability
Scenario: Gazebo lateral carrier motion
Seed: 42
Valid trials requested: 10
Controller duration: 15 s
Initial error target: (80, 28) px
Position tolerance: 5 px
Kp: 7.177391
Kd: 0.061814
Observer bandwidth: 699.944368
EOF

for i in $(seq 1 10); do
    LABEL=$(printf "run_%02d" "$i")
    DEST="$OUT/$LABEL"
    LOG="$OUT/${LABEL}.log"

    if [[ -f "$DEST/analysis/summary.json" ]]; then
        echo "Skipping completed $LABEL"
        continue
    fi

    echo
    echo "=================================="
    echo "EXP. 6 REPEATABILITY: $LABEL / 10"
    echo "=================================="

    if ./hawkeye trial custom 15 \
        --kp 7.177391 \
        --kd 0.061814 \
        --wo 699.944368 \
        --initial-x 80 \
        --initial-y 28 \
        --tolerance 5 2>&1 | tee "$LOG"; then
        :
    else
        echo "Trial failed. Inspect $LOG"
        exit 1
    fi

    if ! grep -q "world lateral" "$LOG"; then
        echo "ERROR: Trial did not confirm lateral mode."
        exit 1
    fi

    if ! grep -q "TRIAL PASS" "$LOG"; then
        echo "ERROR: Trial was not accepted."
        exit 1
    fi

    SRC=$(sed -n 's/^Recorded: //p' "$LOG" | tail -1)

    if [[ -z "$SRC" || ! -f "$SRC/analysis/summary.json" ||
          ! -f "$SRC/analysis/tracking.csv" ]]; then
        echo "ERROR: Missing trial measurements."
        exit 1
    fi

    if [[ -e "$DEST" ]]; then
        echo "ERROR: Destination exists: $DEST"
        exit 1
    fi

    cp -a "$SRC" "$DEST"

    echo "Saved: $DEST"
done

echo
echo "Repeatability trials completed."
echo "Results: $OUT"
