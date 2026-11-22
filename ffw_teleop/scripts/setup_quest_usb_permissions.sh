#!/usr/bin/env bash
# Run through pkexec or sudo; affects only the Quest Pro USB rule.
set -euo pipefail
if [[ "$EUID" -ne 0 ]]; then echo '管理者権限が必要です。'; exit 1; fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
install -m 0644 "$SCRIPT_DIR/../udev/99-meta-quest.rules" /etc/udev/rules.d/99-meta-quest.rules
udevadm control --reload-rules
udevadm trigger --action=change --subsystem-match=usb --attr-match=idVendor=2833
udevadm settle --timeout=5
echo 'Quest Pro USB permissions configured.'
