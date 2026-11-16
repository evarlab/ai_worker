#!/usr/bin/env bash
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
for argument in "$@"; do
    if [[ "$argument" == '--help' || "$argument" == '-h' ]]; then
        exec /usr/bin/python3 -c 'from ffw_teleop.calibrate_vr import main; main(["--help"])'
    fi
done
"$REPO_DIR/stop_teleop.sh"
CALIBRATION="$HOME/.config/ai_worker/vr_motion_calibration.yaml"
LOG_DIR="$HOME/.local/state/ai_worker/calibration"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/terminal-$(date +%Y%m%d-%H%M%S)-$$.log"
echo "ターミナルログ: $LOG_FILE"
/usr/bin/python3 -u -c 'import sys; from ffw_teleop.calibrate_vr import main; sys.exit(main())' \
    --output "$CALIBRATION" "$@" 2>&1 | tee "$LOG_FILE"
ROBOT="${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}"
SOCKET="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}"
scp -o ControlPath="$SOCKET" "$CALIBRATION" "$ROBOT:/tmp/vr-motion-calibration.yaml"
ssh -S "$SOCKET" "$ROBOT" \
    'docker exec ai_worker mkdir -p /root/.config/ai_worker && docker cp /tmp/vr-motion-calibration.yaml ai_worker:/root/.config/ai_worker/vr_motion_calibration.yaml'
echo '実機側にも保存しました。追従は停止しています。次は ./preview_vr.sh'
