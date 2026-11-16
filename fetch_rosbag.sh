#!/usr/bin/env bash
set -eo pipefail
if [[ $# -lt 1 || ! "$1" =~ ^[a-zA-Z0-9_-]+$ ]]; then
    echo "使用方法: $0 SESSION名 [コピー先（既定 ./recordings）]"
    exit 1
fi
destination="${2:-./recordings}"
mkdir -p "$destination"
exec scp -r -o ControlPath="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}" \
    -o ConnectTimeout=3 -o ServerAliveInterval=2 -o ServerAliveCountMax=2 \
    "${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}:/home/robotis/ai_worker/docker/workspace/contactgrove_sharpa/recordings/$1" "$destination/"
