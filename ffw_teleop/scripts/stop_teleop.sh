#!/usr/bin/env bash
# Stop both command gates; SIGINT publishes measured holds while ROS is still alive.
set -uo pipefail
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != '--disable-hands' ) ]]; then
    echo "usage: $0 [--disable-hands]" >&2
    exit 2
fi
ROBOT="${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}"
SOCKET="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}"
local_status=0
remote_status=0
systemctl --user stop contactglove-sharpa.service &
local_stop_pid=$!
ssh -S "$SOCKET" -o ControlMaster=auto -o ControlPersist=600 -o ConnectTimeout=3 \
    "$ROBOT" 'docker exec ai_worker bash -c '\''pkill -INT -f "[l]ib/ffw_teleop/sharpa_vr_bridge|[c]heck_arm_joint.py" || true
for attempt in {1..20}; do
    pgrep -f "[l]ib/ffw_teleop/sharpa_vr_bridge|[c]heck_arm_joint.py" >/dev/null || exit 0
    sleep 0.05
done
exit 1'\''' || remote_status=$?
wait "$local_stop_pid" || local_status=$?
if [[ "$local_status" -ne 0 || "$remote_status" -ne 0 ]]; then
    echo "Stop could not be confirmed: PC=$local_status robot=$remote_status" >&2
    exit 1
fi
echo 'Arm and finger following stopped; measured poses held. Motor power state is unchanged.'
if [[ "${1:-}" == '--disable-hands' ]]; then
    ssh -S "$SOCKET" -o ConnectTimeout=3 "$ROBOT" \
        'docker exec ai_worker bash -lc '\''source /opt/ros/jazzy/setup.bash
ros2 control switch_controllers --deactivate hand_l_controller hand_r_controller
ros2 control set_hardware_component_state sharpa_left_hand inactive
ros2 control set_hardware_component_state sharpa_right_hand inactive'\''' || exit 1
    echo 'Both hand motors disabled.'
fi
