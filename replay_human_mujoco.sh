#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
SIM_ROOT="${AI_WORKER_MUJOCO_ROOT:-$HOME/ffw_sharpa_sim}"
[[ -x "$SIM_ROOT/.venv/bin/python" ]] || { echo "MuJoCo環境がありません: $SIM_ROOT"; exit 1; }
export PYTHONPATH="$REPO_DIR/ffw_teleop:$SIM_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$SIM_ROOT/.venv/bin/python" -u -m ffw_teleop.human_demo_mujoco "$@"
