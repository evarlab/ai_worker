#!/usr/bin/env bash
# Open command-ready Terminator panes. Nothing starts before Enter in a pane.
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"
case "${1:-}" in
    divingstation|robot|vuer|glove|ik)
        source "$REPO_DIR/ffw_teleop/scripts/session_log.sh"
        teleop_session_log "$@" ;;
esac

usage() {
    echo 'usage: ./teleop_terminal.sh [--print-layout | --help]'
    echo 'Terminatorを9分割し、Enterで実行できるコマンドを表示します。'
    echo '画面を開くだけではSSH接続・ROS起動・モータ操作はしません。'
}

remote_run() {
    local container="$1" overlay="$2" quoted_args remote_command
    shift 2
    printf -v quoted_args '%q ' "$@"
    local inner="source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; source $overlay/setup.bash; exec $quoted_args"
    printf -v remote_command 'docker exec -it %q bash -lc %q' "$container" "$inner"
    exec ssh -tt -S "${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}" \
        -o ControlMaster=auto -o ControlPersist=600 -o ConnectTimeout=3 \
        -o ServerAliveInterval=2 -o ServerAliveCountMax=2 \
        "${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}" "$remote_command"
}

case "${1:-}" in
    --help|-h) usage; exit 0 ;;
    divingstation) exec divingstation ;;
    robot)
        remote_run ai_worker /workspace/contactgrove_sharpa/install \
            ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py \
            hands:=both init_position:=false hand_speed_coeff:=0.5 \
            launch_cameras:=false launch_lidar:=false ;;
    vuer)
        exec 9<&0
        remote_run robotis-applications /workspace/sharpa_vr/install \
            ros2 launch ffw_teleop vr_sharpa_publisher.launch.py <&9 &
        vuer_pid=$!
        exec 9<&-
        "$REPO_DIR/ffw_teleop/scripts/connect_quest_usb.sh" --wait 90 --existing-master &
        quest_pid=$!
        cleanup_vuer() {
            kill -TERM "$quest_pid" 2>/dev/null || true
            kill -INT "$vuer_pid" 2>/dev/null || true
        }
        trap cleanup_vuer EXIT
        trap 'exit 130' INT TERM HUP
        wait "$vuer_pid" ;;
    glove)
        source /opt/ros/jazzy/setup.bash
        exec ros2 launch contact_glove contact_glove.launch.xml ;;
    ik)
        remote_run ai_worker /workspace/contactgrove_sharpa/install \
            ros2 launch ffw_teleop sharpa_vr.launch.py dry_run:=false ;;
    --pane)
        role="${2:-}"
        case "$role" in
            divingstation) title='1 DivingStation'; hint='左右グローブを接続。指のcalibはアプリで行う。' ;;
            robot) title='2 実機本体'; hint='SSHパスワードは求められたら入力。初期位置への移動なし。' ;;
            vuer) title='3 VRサーバー'; hint='USB接続したQuestへURLを自動で開く。QuestでEnter VR。' ;;
            glove) title='4 グローブ入力'; hint='DivingStationで左右接続してからEnter。' ;;
            ik) title='5 腕IK'; hint='実機本体の起動後にEnter。追従開始は7番。' ;;
            calibrate) title='6 校正 / URDF（任意）'; hint='校正済みならCtrl+Uで ./preview_vr.sh に変更。実機前にはプレビューを閉じる。' ;;
            resume) title='7 初期姿勢へ移動 / 開始'; hint='Enterで実機の腕を下げたURDF初期姿勢へ移動。到着後、VR基準を登録して追従。中止は8番。' ;;
            stop) title='8 停止（姿勢保持）'; hint='Enterで腕・指の追従を止める。再開は7番。' ;;
            pedal) title='9 フットペダル'; hint='左: 7番の開始/停止、中央: 記録開始/保存終了、右: 初期姿勢へ低速移動。Enterで有効、Ctrl+Cで終了。' ;;
            *) echo "Unknown pane: $role" >&2; exit 2 ;;
        esac
        printf '\033]0;%s\007' "$title"
        printf '\n%s\n%s\n\n' "$title" "$hint"
        case "$role" in
            calibrate) preset='./calibrate_vr.sh' ;;
            resume) preset='./resume_teleop.sh' ;;
            stop) preset='./stop_teleop.sh' ;;
            pedal) preset='./foot_pedal.sh' ;;
            *) preset="./teleop_terminal.sh $role" ;;
        esac
        # Commands run in a child shell, so Ctrl+C returns to this input line.
        trap ':' INT
        while true; do
            read_status=0
            IFS= read -e -r -i "$preset" -p 'Enterで実行 > ' line || read_status=$?
            if [[ "$read_status" -ne 0 ]]; then
                [[ "$read_status" -gt 128 ]] && continue || exit 0
            fi
            [[ -n "$line" ]] || continue
            status=0
            log_dir="${XDG_STATE_HOME:-$HOME/.local/state}/ai_worker/terminal"
            mkdir -p "$log_dir"
            logfile="$log_dir/$role-$(date +%Y%m%d-%H%M%S)-$$.log"
            printf 'ログ保存先: %s\n' "$logfile"
            printf -v run_command 'bash -lc %q' "$line"
            AI_WORKER_SESSION_LOG_ACTIVE=1 script --quiet --flush --return \
                --command "$run_command" "$logfile" || status=$?
            printf '\n終了コード: %s（Enterで再実行、Ctrl+Uで入力消去）\n' "$status"
            if [[ "$role" == calibrate && "$status" == 0 && "$line" == './calibrate_vr.sh' ]]; then
                preset='./preview_vr.sh'
            fi
        done ;;
    ''|--print-layout) ;;
    *) usage >&2; exit 2 ;;
esac

layout_json=$(/usr/bin/python3 - "$REPO_DIR" <<'PY'
import json, shlex, sys
root = sys.argv[1]
def pane(role):
    return {'command': shlex.join(['bash', root+'/teleop_terminal.sh', '--pane', role])}
layout = {
    'profile': {'use_system_font': False, 'font': 'Monospace 10',
                'scrollback_lines': 5000, 'scroll_on_output': False},
    'layout': {'vertical': False, 'Sharpa teleop': [
        {'children': [pane(r) for r in ('divingstation', 'glove', 'resume')]},
        {'children': [pane(r) for r in ('robot', 'ik', 'stop')]},
        {'children': [pane(r) for r in ('vuer', 'calibrate', 'pedal')]},
    ]},
}
print(json.dumps(layout, ensure_ascii=False, indent=2))
PY
)
if [[ "${1:-}" == '--print-layout' ]]; then
    printf '%s\n' "$layout_json"
    exit 0
fi
command -v terminator >/dev/null || { echo 'terminatorが必要です' >&2; exit 1; }
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/ai_worker"
mkdir -p "$CACHE_DIR"
layout_file=$(mktemp "$CACHE_DIR/teleop-terminal-XXXXXX.json")
printf '%s\n' "$layout_json" > "$layout_file"
trap 'rm -f -- "$layout_file"' EXIT
terminator --no-dbus --maximise --title='Sharpa teleop — Enter to run' --config-json "$layout_file"
