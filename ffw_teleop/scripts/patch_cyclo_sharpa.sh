#!/usr/bin/env bash
# Keep Cyclo's collision model synchronized with externally commanded finger joints.
set -euo pipefail
CYCLO_ROOT="${1:?usage: patch_cyclo_sharpa.sh /path/to/cyclo_control}"
PATCH_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../patches" && pwd)/cyclo_vr_passive_feedback.patch"
if patch -d "$CYCLO_ROOT" -p1 --forward --dry-run < "$PATCH_FILE" >/dev/null 2>&1; then
    patch -d "$CYCLO_ROOT" -p1 --forward < "$PATCH_FILE"
elif patch -d "$CYCLO_ROOT" -p1 --reverse --dry-run < "$PATCH_FILE" >/dev/null 2>&1; then
    echo 'Cyclo passive-feedback patch already applied.'
else
    echo 'Cyclo source does not match this patch; no files changed.' >&2
    exit 1
fi
