#!/usr/bin/env bash
# Read-only head-camera observer. Sends no motor commands.
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SSH_HOST="${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}"
SSH_SOCKET="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}"
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: ./observe_robot_board.sh [robot_board arguments]'
  echo '頭部ZED画像・CameraInfo・実機URDFから0〜8のボードを検出。モータ指令は送らない。'
  exit 0
fi
bundle=$(mktemp /tmp/ai-worker-board-XXXXXX.tar)
trap 'rm -f "$bundle"' EXIT
tar -C "$REPO_DIR/ffw_teleop" -cf "$bundle" \
  ffw_teleop/__init__.py ffw_teleop/robot_board.py ffw_teleop/human_demo_robot.py \
  ffw_teleop/human_demo_data.py ffw_teleop/human_demo_calibration.py \
  ffw_teleop/human_demo_aruco.py config/human_demo_workspace.json
ssh_opts=(-S "$SSH_SOCKET" -o ControlMaster=auto -o ControlPersist=600 -o ConnectTimeout=5)
# Only a temporary runtime is staged; installed packages and running controllers are untouched.
ssh "${ssh_opts[@]}" "$SSH_HOST" 'docker exec -i ai_worker bash -c "mkdir -p /tmp/human-board-runtime; tar -xf - -C /tmp/human-board-runtime"' < "$bundle"
arguments=''
if [[ $# -gt 0 ]]; then printf -v arguments '%q ' "$@"; fi
inner="source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; export PYTHONPATH=/tmp/human-board-runtime:\${PYTHONPATH:-}; exec python3 -u -m ffw_teleop.robot_board --workspace /tmp/human-board-runtime/config/human_demo_workspace.json $arguments"
printf -v remote 'docker exec -it ai_worker bash -lc %q' "$inner"
exec ssh -tt "${ssh_opts[@]}" -o ServerAliveInterval=2 -o ServerAliveCountMax=2 "$SSH_HOST" "$remote"
