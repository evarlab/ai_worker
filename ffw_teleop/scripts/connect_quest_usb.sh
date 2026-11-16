#!/usr/bin/env bash
# Forward the robot's official Vuer server to Quest Browser over this PC's USB cable.
set -euo pipefail
ROBOT="${1:-robotis@ffw-snpr48a1130.local}"
ADB_BIN="${ADB_BIN:-$(command -v adb || true)}"
if [[ -z "$ADB_BIN" ]]; then
    ADB_BIN="$HOME/.local/share/ai_worker/android/platform-tools/adb"
fi
[[ -x "$ADB_BIN" ]] || { echo 'Install Android platform-tools or set ADB_BIN.' >&2; exit 1; }
if ! "$ADB_BIN" get-state >/dev/null 2>&1; then
    "$ADB_BIN" devices -l
    echo 'Enable Developer Mode and USB debugging on Quest, and approve this PC in the headset.' >&2
    exit 1
fi
SOCKET="${XDG_RUNTIME_DIR:-/tmp}/ai-worker-vr-ssh-$UID"
if ! ssh -S "$SOCKET" -O check "$ROBOT" >/dev/null 2>&1; then
    ssh -M -S "$SOCKET" -o ControlPersist=600 -o ExitOnForwardFailure=yes \
        -fNT -L 127.0.0.1:8012:127.0.0.1:8012 "$ROBOT"
fi
"$ADB_BIN" reverse tcp:8012 tcp:8012
URL='https://127.0.0.1:8012?ws=wss://127.0.0.1:8012'
"$ADB_BIN" shell am start -a android.intent.action.VIEW -d "$URL"
echo "Quest Browser: $URL"
echo 'Accept the local certificate warning, then Enter VR at the operating position.'
