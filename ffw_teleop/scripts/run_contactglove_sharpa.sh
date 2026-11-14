#!/usr/bin/env bash
# Background companion for the user-controlled contact_glove launch.
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
PROFILE="${1:-$HOME/.config/ai_worker/contactglove_sharpa.yaml}"
exec /usr/bin/python3 -c 'from ffw_teleop.contactglove_sharpa import main; main()' \
    --ros-args --params-file "$PROFILE"
