#!/usr/bin/env bash
# GUI runs on the PC; hand receive packets are relayed over AI Worker's LAN.
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
teleop_session_log "$@"
ROBOT="${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}"
SOCKET="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}"
command -v sharpa-pilot >/dev/null || { echo 'Install official sharpa-pilot first.'; exit 1; }
mkdir -p "${XDG_STATE_HOME:-$HOME/.local/state}/ai_worker"
exec 8>"${XDG_STATE_HOME:-$HOME/.local/state}/ai_worker/pilot.lock"
flock -n 8 || { echo 'Pilotは既に起動しています。既存の画面を使用してください。'; exit 1; }
peer=$(/usr/bin/python3 - <<'PY'
import socket
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.connect(('192.168.6.2',9))
print(s.getsockname()[0])
s.close()
PY
)
if [[ "$peer" != 192.168.6.* ]]; then echo 'Connect the PC to AI Worker wired LAN (192.168.6.x).'; exit 1; fi
if ssh -S "$SOCKET" -o ConnectTimeout=3 "$ROBOT" "docker exec ai_worker pgrep -f '^/opt/ros/jazzy/lib/controller_manager/ros2_control_node'"; then
    echo '2番の実機本体をCtrl+Cで終了してからPilotを起動してください。'; exit 1
fi
for hand in 192.168.10.10 192.168.10.20; do
    if ! ip -4 route get "$hand" | /usr/bin/grep -q 'via 192.168.6.2'; then
        echo 'Pilotの一時ルート設定が必要です。READMEのPilot導入手順を参照してください。'; exit 1
    fi
done
relay_log="${XDG_STATE_HOME:-$HOME/.local/state}/ai_worker/pilot-relay.log"
remote_script='/home/robotis/ai_worker/docker/workspace/contactgrove_sharpa/src/ai_worker/ffw_sharpa_hardware/scripts/pilot_udp_relay.py'
ssh -tt -S "$SOCKET" -o ConnectTimeout=3 -o ServerAliveInterval=2 -o ServerAliveCountMax=2 \
    "$ROBOT" "/usr/bin/python3 -u $remote_script --peer $peer" </dev/null >"$relay_log" 2>&1 &
relay_pid=$!
cleanup() { kill -TERM "$relay_pid" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
for attempt in {1..30}; do
    if /usr/bin/grep -q 'relay ready' "$relay_log"; then break; fi
    if ! kill -0 "$relay_pid" 2>/dev/null; then cat "$relay_log"; exit 1; fi
    sleep .1
done
/usr/bin/grep -q 'relay ready' "$relay_log" || { cat "$relay_log"; exit 1; }
sharpa-pilot "$@"
