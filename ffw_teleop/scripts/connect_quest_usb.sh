#!/usr/bin/env bash
# Open Vuer on Quest after SSH, HTTPS and USB authorization are ready.
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT="${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}"
SOCKET="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}"
WAIT=1
EXISTING_MASTER=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --wait) WAIT="$2"; shift 2 ;;
        --existing-master) EXISTING_MASTER=true; shift ;;
        --help|-h) echo 'usage: connect_quest_usb.sh [--wait SECONDS] [--existing-master] [robotis@HOST]'; exit 0 ;;
        *) ROBOT="$1"; shift ;;
    esac
done
[[ "$WAIT" =~ ^[1-9][0-9]*$ ]] || { echo 'wait must be a positive integer'; exit 2; }
ADB_BIN="${ADB_BIN:-$(command -v adb || true)}"
[[ -n "$ADB_BIN" ]] || ADB_BIN="$HOME/.local/share/ai_worker/android/platform-tools/adb"
[[ -x "$ADB_BIN" ]] || { echo 'Install Android platform-tools or set ADB_BIN.' >&2; exit 1; }
deadline=$((SECONDS+WAIT))
echo 'Quest自動接続: SSH接続・USB認証を待っています。'
until ssh -S "$SOCKET" -O check "$ROBOT" >/dev/null 2>&1; do
    if [[ "$EXISTING_MASTER" == false ]]; then
        ssh -M -S "$SOCKET" -o ControlPersist=600 -o ConnectTimeout=3 -fNT "$ROBOT"
    fi
    if (( SECONDS >= deadline )); then echo 'Quest自動接続: SSH接続を確認できませんでした。'; exit 1; fi
    sleep 1
done
# Reusing a master with an existing forward is expected. Test HTTPS below.
ssh -S "$SOCKET" -O forward -L 127.0.0.1:8012:127.0.0.1:8012 "$ROBOT" >/dev/null 2>&1 || true
until "$ADB_BIN" get-state >/dev/null 2>&1; do
    if (( SECONDS >= deadline )); then
        "$ADB_BIN" devices -l
        echo 'Quest自動接続: USB接続・ヘッドセット内のデバッグ許可を確認してください。'; exit 1
    fi
    sleep 1
done
echo 'Quest自動接続: VRサーバーのHTTPS応答を待っています。'
until curl -k -fsS --connect-timeout 1 --max-time 2 -o /dev/null https://127.0.0.1:8012/; do
    if (( SECONDS >= deadline )); then echo 'Quest自動接続: VRサーバーが応答しません。3番のエラーを確認してください。'; exit 1; fi
    sleep 1
done
"$ADB_BIN" reverse tcp:8012 tcp:8012
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
PYTHON_BIN="$HOME/.local/share/ai_worker/human-demo-venv/bin/python"
[[ -x "$PYTHON_BIN" ]] || PYTHON_BIN=/usr/bin/python3
exec "$PYTHON_BIN" -m ffw_teleop.quest_browser --adb "$ADB_BIN"
