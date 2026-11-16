#!/usr/bin/env bash
# Source from a launcher, then call teleop_session_log "$@".
teleop_session_log() {
    if [[ "${AI_WORKER_SESSION_LOG_ACTIVE:-}" == 1 ]]; then return; fi
    case "${1:-}" in --help|-h|--print-layout) return ;; esac
    local log_dir="${XDG_STATE_HOME:-$HOME/.local/state}/ai_worker/terminal"
    mkdir -p "$log_dir"
    local name="$(basename "$0" .sh)"
    local logfile="$log_dir/$name-$(date +%Y%m%d-%H%M%S)-$$.log"
    local command_line
    printf -v command_line '%q ' bash "$0" "$@"
    printf 'ログ保存先: %s\n' "$logfile"
    export AI_WORKER_SESSION_LOG_ACTIVE=1
    exec script --quiet --flush --return --command "$command_line" "$logfile"
}
