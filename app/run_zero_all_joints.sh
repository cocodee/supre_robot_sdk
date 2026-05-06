#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

CONFIG="${CONFIG:-examples/robot_config_repeat.yaml}"
DURATION="${DURATION:-10}"
FREQUENCY="${FREQUENCY:-30}"
MAX_ABS_POSITION="${MAX_ABS_POSITION:-360}"
MAX_STEP="${MAX_STEP:-2}"

echo "This will connect to robot hardware and slowly command every joint to 0."
echo "Config: ${CONFIG}"
echo "Duration: ${DURATION}s, frequency: ${FREQUENCY}Hz, max step: ${MAX_STEP}"

if [[ "${AUTO_CONFIRM:-0}" != "1" ]]; then
  read -r -p "Type ZERO to continue: " CONFIRMATION
  if [[ "$CONFIRMATION" != "ZERO" ]]; then
    echo "Aborted."
    exit 1
  fi
fi

export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

python3 app/zero_all_joints.py \
  --config "$CONFIG" \
  --duration "$DURATION" \
  --frequency "$FREQUENCY" \
  --max-abs-position "$MAX_ABS_POSITION" \
  --max-step "$MAX_STEP"
