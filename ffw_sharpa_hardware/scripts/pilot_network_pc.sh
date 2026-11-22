#!/usr/bin/env bash
# Temporary host routes only; run on the Pilot PC with pkexec/sudo.
set -euo pipefail
[[ "$EUID" == 0 ]] || { echo 'root required'; exit 1; }
case "${1:-}" in
    on)
        ip route replace 192.168.10.10/32 via 192.168.6.2
        ip route replace 192.168.10.20/32 via 192.168.6.2 ;;
    off)
        ip route del 192.168.10.10/32 via 192.168.6.2 2>/dev/null || true
        ip route del 192.168.10.20/32 via 192.168.6.2 2>/dev/null || true ;;
    *) echo "usage: $0 on|off"; exit 2 ;;
esac
