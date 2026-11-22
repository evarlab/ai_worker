#!/usr/bin/env bash
# Stop following, then slowly return the arms. Following remains off.
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
if [[ "${1:-}" == '--help' ]]; then
    echo '両腕の追従を停止し、URDF初期姿勢へ15秒以上・最大0.15rad/sで移動します。停止はstop_teleop.sh。'
    exit 0
fi
bash "$REPO_DIR/stop_teleop.sh"
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
OPERATION_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/ai_worker"
exec 8>"$OPERATION_DIR/resume.lock"
flock -n 8 || { echo '開始・移動処理がまだ実行中です。再度操作してください。'; exit 1; }
export AI_WORKER_OPERATION_FILE="$OPERATION_DIR/stop-generation"
export AI_WORKER_OPERATION_TOKEN="$(cat "$AI_WORKER_OPERATION_FILE" 2>/dev/null || true)"
exec /usr/bin/python3 -m ffw_teleop.initialize_arms --minimum-duration 15 --maximum-speed 0.15 "$@"
