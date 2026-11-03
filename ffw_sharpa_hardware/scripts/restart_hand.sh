#!/usr/bin/env bash
# Re-enables a Sharpa hand after a fault, lost feedback or a disable from outside ROS, while the
# rest of the robot keeps running (run where `ros2` reaches the controller manager, e.g. in the
# ai_worker container):
#
#   bash restart_hand.sh left|right
#
# Order matters: the hand's trajectory controller is stopped first, so it does not resend its old
# set-point; the hardware component is cycled (disable, then enable holding the measured pose);
# the controller then starts from the measured pose. Fix the fault itself first (Sharpa Pilot).
set -euo pipefail

case "${1:-}" in
    left) controller=hand_l_controller ;;
    right) controller=hand_r_controller ;;
    *) echo "usage: $0 left|right" >&2; exit 2 ;;
esac
component="sharpa_${1}_hand"

ros2 control switch_controllers --deactivate "$controller"
ros2 control set_hardware_component_state "$component" inactive
ros2 control set_hardware_component_state "$component" active
ros2 control switch_controllers --activate "$controller"
ros2 control list_controllers | grep -E "^$controller "
