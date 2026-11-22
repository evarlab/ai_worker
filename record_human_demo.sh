#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${AI_WORKER_HUMAN_VENV:-$HOME/.local/share/ai_worker/human-demo-venv}"
if [[ "${1:-}" == setup ]]; then
    /usr/bin/python3 -m venv --system-site-packages "$VENV"
    exec "$VENV/bin/pip" install -r "$REPO_DIR/ffw_teleop/human_demo_requirements.txt"
fi
if [[ ! -x "$VENV/bin/python" ]]; then echo '先に ./record_human_demo.sh setup を実行してください。'; exit 1; fi
source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
exec "$VENV/bin/python" -u -m ffw_teleop.human_demo "$@"
