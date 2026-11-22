#!/usr/bin/env bash
# Local Quest diagnostics only: no robot connection or actuator publishers.
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${AI_WORKER_HUMAN_VENV:-$HOME/.local/share/ai_worker/human-demo-venv}"
source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
exec "$VENV/bin/python" -u -m ffw_teleop.quest_check "$@"
