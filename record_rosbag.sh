#!/usr/bin/env bash
# Recording is independent of lift/VR control. Default storage is on AI Worker.
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
if [[ "${1:-}" == '--local' ]]; then
    shift
    source /opt/ros/jazzy/setup.bash
    export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
    exec /usr/bin/python3 -m ffw_teleop.record_rosbag "$@"
fi
if [[ "${1:-}" == '--help' || "${1:-}" == '-h' ]]; then
    source /opt/ros/jazzy/setup.bash
    export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
    /usr/bin/python3 -m ffw_teleop.record_rosbag --help
    echo '通常は実機内に記録。--local を先頭に指定するとこのPCで記録。'
    exit 0
fi
printf -v arguments '%q ' "$@"
inner="source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; source /workspace/contactgrove_sharpa/install/setup.bash; exec /usr/bin/python3 -m ffw_teleop.record_rosbag $arguments"
printf -v remote 'docker exec -it ai_worker bash -lc %q' "$inner"
exec ssh -tt -S "${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}" \
    -o ConnectTimeout=3 -o ServerAliveInterval=2 -o ServerAliveCountMax=2 \
    "${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}" "$remote"
