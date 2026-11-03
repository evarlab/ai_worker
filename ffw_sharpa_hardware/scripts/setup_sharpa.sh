#!/usr/bin/env bash
# Fetches what Sharpa Wave support needs but this repository does not store (both git-ignored):
#
#   third_party/sharpa-wave-sdk      SharpaWave SDK 5.0.11, the vendor's release for this machine
#                                    (ARM .zip on aarch64, .deb on x86_64), checked against its
#                                    SHA-256. Proprietary: not redistributed, only extracted here.
#   third_party/sharpa-urdf-usd-xml  Official hand descriptions (Apache-2.0), sparse checkout of the
#                                    ROS 2 packages left_sharpa_wave and right_sharpa_wave.
#
# colcon then builds the hardware plugin against the SDK and finds the two description packages.
# Run it where the workspace is built: on the robot host (~/ai_worker is mounted into the
# ai_worker container) or inside the container. Needs curl, git, sha256sum and unzip (aarch64) or
# dpkg-deb (x86_64). Re-running is cheap: finished steps are skipped.
#
#   bash ffw_sharpa_hardware/scripts/setup_sharpa.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
THIRD="$REPO/third_party"
SDK_VERSION="5.0.11"
SDK_URL="https://github.com/sharpa-robotics/sharpa-wave-sdk/releases/download/v$SDK_VERSION"
SDK_ARM_SHA256="5b399354596858d85ea6a77904dba68e170ae18b3c6036141790a17585fdda9a"
SDK_AMD64_SHA256="271e3c6f32401b1c28828ec5658aeb360e34b37ddd8285ce5a67aebdf216bd31"
DESCRIPTION_URL="https://github.com/sharpa-robotics/sharpa-urdf-usd-xml.git"
DESCRIPTION_COMMIT="0d19cac602f46456b819e4b6a2c09a74982c9a3e"

install_sdk() {
    local dest="$THIRD/sharpa-wave-sdk" cache="$THIRD/.cache" file sha tmp root
    case "$(uname -m)" in
        aarch64) file="SharpaWaveSDK_${SDK_VERSION}_arm.zip" sha="$SDK_ARM_SHA256" ;;
        x86_64) file="sharpa-wave-sdk_${SDK_VERSION}_amd64.deb" sha="$SDK_AMD64_SHA256" ;;
        *) echo "No SharpaWave SDK release for $(uname -m)" >&2; exit 1 ;;
    esac
    if [[ "$(cat "$dest/.release" 2>/dev/null)" == "$file $sha" ]]; then
        echo "== SharpaWave SDK $SDK_VERSION already in $dest"
        return
    fi
    mkdir -p "$cache"
    if [[ ! -f "$cache/$file" ]] || ! echo "$sha  $cache/$file" | sha256sum -c --quiet >/dev/null 2>&1; then
        echo "== downloading SharpaWave SDK $SDK_VERSION ($file)"
        curl -fL --retry 3 -o "$cache/$file.part" "$SDK_URL/$file"
        echo "$sha  $cache/$file.part" | sha256sum -c --quiet  # stops here on a mismatch
        mv "$cache/$file.part" "$cache/$file"
    fi
    tmp="$(mktemp -d "$THIRD/.sdk.XXXXXX")"
    if [[ "$file" == *.zip ]]; then
        unzip -q "$cache/$file" -d "$tmp/sdk"
        root="$tmp/sdk"
    else
        dpkg-deb -x "$cache/$file" "$tmp/deb"
        root="$tmp/deb/opt/sharpa-wave-sdk"
    fi
    if [[ ! -f "$root/include/SharpaWaveSDK.h" || ! -f "$root/lib/libsharpa-wave-sdk.so" ]]; then
        rm -rf "$tmp"
        echo "Unexpected SDK layout in $file" >&2
        exit 1
    fi
    rm -rf "$dest"
    mv "$root" "$dest"
    rm -rf "$tmp"
    touch "$dest/COLCON_IGNORE"
    echo "$file $sha" > "$dest/.release"
    echo "== SharpaWave SDK $SDK_VERSION extracted to $dest"
}

install_descriptions() {
    local dest="$THIRD/sharpa-urdf-usd-xml"
    if [[ "$(git -C "$dest" rev-parse HEAD 2>/dev/null)" == "$DESCRIPTION_COMMIT" ]]; then
        echo "== Sharpa hand descriptions already at ${DESCRIPTION_COMMIT:0:7}"
        return
    fi
    echo "== fetching Sharpa hand descriptions (${DESCRIPTION_COMMIT:0:7}, two packages only)"
    rm -rf "$dest"
    git init -q "$dest"
    git -C "$dest" remote add origin "$DESCRIPTION_URL"
    git -C "$dest" sparse-checkout set --no-cone /LICENSE.txt /NOTICE.txt /README.md \
        /wave_01/left_sharpa_wave/ /wave_01/right_sharpa_wave/
    git -C "$dest" fetch -q --depth 1 --filter=blob:none origin "$DESCRIPTION_COMMIT"
    git -C "$dest" -c advice.detachedHead=false checkout -q FETCH_HEAD
    for side in left right; do
        [[ -f "$dest/wave_01/${side}_sharpa_wave/package.xml" ]] || {
            echo "Missing ${side}_sharpa_wave in the description checkout" >&2; exit 1; }
    done
}

mkdir -p "$THIRD"
install_sdk
install_descriptions
echo "Done. Build with: colcon build --symlink-install --packages-up-to ffw_bringup ffw_sharpa_hardware left_sharpa_wave right_sharpa_wave"
