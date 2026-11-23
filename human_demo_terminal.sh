#!/usr/bin/env bash
# Human collector only: no SSH, robot bringup or robot action launchers.
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"
if [[ "${1:-}" == '--pane' ]]; then
    role="${2:-}"
    auto=1
    case "$role" in
        server)
            title='1 人間収集サーバー（実機出力なし）'
            hint='Questで証明書承認→Enter XR。グローブSDK許可はDivingStationで承認。切り替え状態はこの端末へ表示。校正: board-start→頭を上下左右へ回転→board-finish / mount-calibrate。'
            adb_bin="${AI_WORKER_ADB:-$HOME/.local/share/ai_worker/android/platform-tools/adb}"
            model=$("$adb_bin" shell getprop ro.product.model 2>/dev/null || true)
            camera_option='--aruco'
            if [[ "$model" == *'Quest Pro'* ]]; then camera_option='--allow-no-camera'; fi
            command="./record_human_demo.sh serve --usb --open-quest $camera_option --glove-source sdk --workspace ffw_teleop/config/human_demo_workspace.json" ;;
        pedal)
            title='2 人間収集フットペダル'
            hint='左: 頭の基準設定・追従ON/OFF | 中央: 記録開始/成功保存 | 右: 失敗保存。Ctrl+Cで追従・記録終了。'
            command='./human_demo_pedal.sh' ;;
        status)
            title='3 受信・追従・記録ステータス'
            hint='xr / glove_left / glove_right / camera_frameの受信経過秒数を確認。Quest 3は外向きカメラを選択。'
            command='./record_human_demo.sh watch' ;;
        sim)
            title='4 700mm机＋ArUco ID 0〜8（任意）'
            hint='Enterでシミュレーション場面のみ表示。実測校正ではありません。'
            command='./replay_human_mujoco.sh --scene-only'
            auto=0 ;;
        *) echo 'Unknown pane' >&2; exit 2 ;;
    esac
    printf '\033]0;%s\007\n%s\n%s\n\n' "$title" "$title" "$hint"
    trap ':' INT
    while true; do
        if [[ "$auto" == 1 ]]; then
            line="$command"
            auto=0
        else
            read_status=0
            IFS= read -e -r -i "$command" -p 'Enterで実行 > ' line || read_status=$?
            if [[ "$read_status" -ne 0 ]]; then
                [[ "$read_status" -gt 128 ]] && continue || exit 0
            fi
        fi
        [[ -n "$line" ]] || continue
        log_dir="${XDG_STATE_HOME:-$HOME/.local/state}/ai_worker/terminal"
        mkdir -p "$log_dir"
        logfile="$log_dir/human-$role-$(date +%Y%m%d-%H%M%S)-$$.log"
        printf '実行: %s\nログ: %s\n' "$line" "$logfile"
        printf -v run_command 'bash -lc %q' "$line"
        status=0
        AI_WORKER_SESSION_LOG_ACTIVE=1 script --quiet --flush --return \
            --command "$run_command" "$logfile" || status=$?
        printf '\n終了コード: %s（Enterで再実行）\n' "$status"
    done
fi
case "${1:-}" in
    ''|--print-layout) ;;
    --help|-h) echo 'Usage: ./human_demo_terminal.sh [--print-layout]'; exit 0 ;;
    *) exit 2 ;;
esac
layout_json=$(/usr/bin/python3 - "$REPO_DIR" <<'PY'
import json,shlex,sys
root=sys.argv[1]
def pane(role):return {'command':shlex.join(['bash',root+'/human_demo_terminal.sh','--pane',role])}
print(json.dumps({'profile':{'use_system_font':False,'font':'Monospace 10','scrollback_lines':5000},
 'layout':{'vertical':False,'Quest + ContactGlove human capture':[
  {'children':[pane('server'),pane('pedal')]},
  {'children':[pane('status'),pane('sim')]}]}},ensure_ascii=False))
PY
)
if [[ "${1:-}" == '--print-layout' ]]; then printf '%s\n' "$layout_json"; exit 0; fi
command -v terminator >/dev/null
cache_dir="${XDG_CACHE_HOME:-$HOME/.cache}/ai_worker"
mkdir -p "$cache_dir"
layout_file=$(mktemp "$cache_dir/human-demo-terminal-XXXXXX.json")
printf '%s\n' "$layout_json" > "$layout_file"
trap 'rm -f -- "$layout_file"' EXIT
terminator --no-dbus --maximise --title='Quest + ContactGlove — human capture' --config-json "$layout_file"
