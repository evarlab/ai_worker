#!/usr/bin/env bash
# Resume the stopped command gates; do not reboot hardware or the Quest server.
set -eo pipefail
if [[ "${1:-}" == '--help' || "${1:-}" == '-h' ]]; then
    echo 'usage: ./resume_teleop.sh'
    echo '両腕をURDF初期姿勢へ移動し、到着時のVR基準で腕と指の追従を開始します。'
    exit 0
fi
if [[ $# -ne 0 ]]; then echo 'usage: ./resume_teleop.sh' >&2; exit 2; fi
source "$(dirname "${BASH_SOURCE[0]}")/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
source /opt/ros/jazzy/setup.bash
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
OPERATION_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/ai_worker"
mkdir -p "$OPERATION_DIR"
exec 8>"$OPERATION_DIR/resume.lock"
flock -n 8 || { echo '7番の開始処理が実行中です。中止は8番。' >&2; exit 1; }
export AI_WORKER_OPERATION_FILE="$OPERATION_DIR/stop-generation"
export AI_WORKER_OPERATION_TOKEN="$(cat "$AI_WORKER_OPERATION_FILE" 2>/dev/null || true)"
echo "$$" > "$OPERATION_DIR/resume.pid"
cleanup_operation() {
    rm -f "$OPERATION_DIR/resume.pid"
    if [[ "$(cat "$AI_WORKER_OPERATION_FILE" 2>/dev/null || true)" != "$AI_WORKER_OPERATION_TOKEN" ]]; then
        systemctl --user stop contactglove-sharpa.service || true
    fi
}
trap cleanup_operation EXIT
trap 'exit 130' INT TERM
systemctl --user stop contactglove-sharpa.service
ROBOT="${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}"
SOCKET="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}"
ssh_status=0
timeout --foreground 8 ssh -S "$SOCKET" -o ConnectTimeout=3 \
    -o ServerAliveInterval=2 -o ServerAliveCountMax=2 "$ROBOT" 'docker exec ai_worker bash -lc '\''source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/contactgrove_sharpa/install/setup.bash
/usr/bin/python3 -m ffw_teleop.vr_processes'\''' || ssh_status=$?
if [[ "$ssh_status" -ne 0 ]]; then
    echo '再開できません: AI Workerへの接続、または5番の腕IKを確認してください。' >&2
    exit "$ssh_status"
fi
/usr/bin/python3 - <<'PY'
import rclpy
import time
from std_srvs.srv import SetBool
rclpy.init(domain_id=0)
node = rclpy.create_node('resume_teleop_once')
client = node.create_client(SetBool, '/sharpa_vr_bridge/enable')
success = False
try:
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError('腕ブリッジのサービスがありません。起動手順を確認してください')
    # Always discard the previous relative reference, including when already
    # following. A second press of Enter must mean "use this VR pose now".
    stop_future = client.call_async(SetBool.Request(data=False))
    rclpy.spin_until_future_complete(node, stop_future, timeout_sec=5)
    if not stop_future.done() or not stop_future.result().success:
        raise RuntimeError('基準を取り直すための保持切替に失敗しました')
finally:
    node.destroy_node()
    rclpy.try_shutdown()
PY
/usr/bin/python3 -m ffw_teleop.initialize_arms
/usr/bin/python3 - <<'PY'
import os
from pathlib import Path
import rclpy
import time
from std_srvs.srv import SetBool
rclpy.init(domain_id=0)
node = rclpy.create_node('start_vr_after_initial_pose')
client = node.create_client(SetBool, '/sharpa_vr_bridge/enable')
success = False
def check_cancelled():
    path = Path(os.environ['AI_WORKER_OPERATION_FILE'])
    token = path.read_text().strip() if path.exists() else ''
    if token != os.environ['AI_WORKER_OPERATION_TOKEN']:
        raise RuntimeError('8番で停止されました。追従は開始しません')
try:
    check_cancelled()
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('腕ブリッジが停止しています')
    # Give a restarted gate a short window to discover existing pose publishers.
    deadline = time.monotonic()+3.
    while True:
        check_cancelled()
        future = client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        if not future.done():
            raise RuntimeError('腕の開始応答がありません')
        result = future.result()
        if result.success or time.monotonic() >= deadline:
            break
        rclpy.spin_once(node, timeout_sec=.1)
    print(result.message, flush=True)
    success = result.success
    if success:
        print('今のVRの位置・向きを、到着したURDF初期姿勢へ合わせて登録しました。', flush=True)
finally:
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if success else 1)
PY
[[ "$(cat "$AI_WORKER_OPERATION_FILE" 2>/dev/null || true)" == "$AI_WORKER_OPERATION_TOKEN" ]] || exit 130
systemctl --user start contactglove-sharpa.service
/usr/bin/python3 - <<'PY'
import os
from pathlib import Path
import rclpy
from std_srvs.srv import SetBool
rclpy.init(domain_id=0)
node = rclpy.create_node('resume_fingers_once')
client = node.create_client(SetBool, '/contactglove_sharpa/enable')
success = False
try:
    path = Path(os.environ['AI_WORKER_OPERATION_FILE'])
    if (path.read_text().strip() if path.exists() else '') != os.environ['AI_WORKER_OPERATION_TOKEN']:
        raise RuntimeError('8番で停止されました。指の追従は開始しません')
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
