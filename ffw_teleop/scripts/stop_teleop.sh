#!/usr/bin/env bash
# Stop both command gates; SIGINT publishes measured holds while ROS is still alive.
set -uo pipefail
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != '--disable-hands' ) ]]; then
    echo "usage: $0 [--disable-hands]" >&2
    exit 2
fi
ROBOT="${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}"
SOCKET="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}"
operation_dir="${XDG_CACHE_HOME:-$HOME/.cache}/ai_worker"
mkdir -p "$operation_dir"
date +%s%N > "$operation_dir/stop-generation"
operation_pid=$(cat "$operation_dir/resume.pid" 2>/dev/null || true)
if [[ "$operation_pid" =~ ^[0-9]+$ && -r "/proc/$operation_pid/cmdline" ]]; then
    operation_command=$(tr '\0' ' ' < "/proc/$operation_pid/cmdline")
    if [[ "$operation_command" == *resume_teleop.sh* ]]; then
        operation_group=$(ps -o pgid= -p "$operation_pid" | tr -d ' ')
        if [[ "$operation_group" == "$operation_pid" ]]; then
            kill -INT -- "-$operation_pid" 2>/dev/null || true
        else
            kill -TERM "$operation_pid" 2>/dev/null || true
        fi
    fi
fi
pkill -INT -f '[f]fw_teleop.initialize_arms' || true
local_status=0
remote_status=0
systemctl --user stop contactglove-sharpa.service &
local_stop_pid=$!
timeout --foreground 6 ssh -S "$SOCKET" -o ControlMaster=auto -o ControlPersist=600 -o ConnectTimeout=3 \
    -o ServerAliveInterval=2 -o ServerAliveCountMax=2 \
    "$ROBOT" 'docker exec ai_worker bash -c '\''pkill -INT -f "[l]ib/ffw_teleop/sharpa_vr_bridge|[c]heck_arm_joint.py|[a]i-worker-hand-recovery" || true
for attempt in {1..60}; do
    pgrep -f "[l]ib/ffw_teleop/sharpa_vr_bridge|[c]heck_arm_joint.py|[a]i-worker-hand-recovery" >/dev/null || exit 0
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
    timeout --foreground 15 ssh -S "$SOCKET" -o ConnectTimeout=3 "$ROBOT" \
        'docker exec ai_worker bash -lc '\''source /opt/ros/jazzy/setup.bash
ros2 control switch_controllers --deactivate hand_l_controller hand_r_controller
ros2 control set_hardware_component_state sharpa_left_hand inactive
ros2 control set_hardware_component_state sharpa_right_hand inactive'\''' || exit 1
    echo 'Both hand motors disabled.'
fi
