#!/usr/bin/env bash
# Resume the stopped command gates; do not reboot hardware or the Quest server.
set -eo pipefail
if [[ "${1:-}" == '--help' || "${1:-}" == '-h' ]]; then
    echo 'usage: ./resume_teleop.sh'
    echo '既存の実機bringup・Cyclo・Vuer・contact_gloveを使い、腕と指の追従を再開します。'
    exit 0
fi
if [[ $# -ne 0 ]]; then echo 'usage: ./resume_teleop.sh' >&2; exit 2; fi
source /opt/ros/jazzy/setup.bash
ROBOT="${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}"
SOCKET="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}"
ssh -S "$SOCKET" -o ConnectTimeout=3 "$ROBOT" 'docker exec ai_worker bash -lc '\''source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/contactgrove_sharpa/install/setup.bash
if ! pgrep -f "[l]ib/ffw_teleop/sharpa_vr_bridge" >/dev/null; then
    nohup ros2 run ffw_teleop sharpa_vr_bridge --ros-args \
        --params-file /workspace/contactgrove_sharpa/install/ffw_teleop/share/ffw_teleop/config/sharpa_vr.yaml \
        -p dry_run:=false > /workspace/contactgrove_sharpa/vr-resume-current.log 2>&1 < /dev/null &
fi'\'''
python3 - <<'PY'
import rclpy
from std_srvs.srv import SetBool
rclpy.init(domain_id=0)
node = rclpy.create_node('resume_teleop_once')
client = node.create_client(SetBool, '/sharpa_vr_bridge/enable')
success = False
try:
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError('腕ブリッジのサービスがありません。起動手順を確認してください')
    future = client.call_async(SetBool.Request(data=True))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done():
        raise RuntimeError('腕の開始応答がありません')
    result = future.result()
    print(result.message, flush=True)
    success = result.success
finally:
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if success else 1)
PY
systemctl --user start contactglove-sharpa.service
python3 - <<'PY'
import rclpy
from std_srvs.srv import SetBool
rclpy.init(domain_id=0)
node = rclpy.create_node('resume_fingers_once')
client = node.create_client(SetBool, '/contactglove_sharpa/enable')
success = False
try:
    if not client.wait_for_service(timeout_sec=8):
        raise RuntimeError('指ブリッジのサービスがありません')
    future = client.call_async(SetBool.Request(data=True))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done():
        raise RuntimeError('指の開始応答がありません')
    result = future.result()
    print('指: '+result.message, flush=True)
    success = result.success
finally:
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if success else 1)
PY
echo '腕・指の追従を有効にしました。停止: ./stop_teleop.sh'
