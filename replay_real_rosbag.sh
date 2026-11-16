#!/usr/bin/env bash
# Session path is inside the AI Worker container, as printed by record_rosbag.sh.
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
if [[ $# -eq 0 ]]; then
    echo "使用方法: $0 /workspace/contactgrove_sharpa/recordings/SESSION [--rate 0.5]"
    exit 1
fi
printf -v arguments '%q ' "$@"
inner="source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; source /workspace/contactgrove_sharpa/install/setup.bash; exec /usr/bin/python3 -m ffw_teleop.replay_rosbag --real $arguments"
printf -v remote 'docker exec -it ai_worker bash -lc %q' "$inner"
exec ssh -tt -S "${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}" \
    -o ConnectTimeout=3 -o ServerAliveInterval=2 -o ServerAliveCountMax=2 \
    "${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}" "$remote"
