#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
if [[ $# -lt 1 ]]; then echo 'Usage: ./replay_human_sim.sh /path/to/sim_trajectory.npz [--loop]'; exit 1; fi
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=97
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
URDF="$HOME/.cache/ai_worker/vr_preview/robot.urdf"
[[ -f "$URDF" ]] || { echo "モデルがありません: $URDF"; exit 1; }
pids=()
cleanup() { trap - EXIT INT TERM; for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT
trap 'exit 130' INT TERM
ros2 run robot_state_publisher robot_state_publisher "$URDF" &
pids+=("$!")
rviz2 -d "$REPO_DIR/ffw_teleop/config/sharpa_vr_preview.rviz" &
pids+=("$!")
/usr/bin/python3 -m ffw_teleop.human_demo_sim play "$@" &
pids+=("$!")
wait "${pids[-1]}"
