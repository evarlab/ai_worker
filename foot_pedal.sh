#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -u -m ffw_teleop.foot_pedal --repo "$REPO_DIR" "$@"
