#!/usr/bin/env bash
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$REPO_DIR/stop_teleop.sh"
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONPATH="$REPO_DIR/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
ROBOT="${AI_WORKER_SSH_HOST:-robotis@ffw-snpr48a1130.local}"
SOCKET="${AI_WORKER_SSH_SOCKET:-/tmp/contactglove-aiworker-ssh}"
CACHE="$HOME/.cache/ai_worker/vr_preview"
mkdir -p "$CACHE"
if [[ ! -f "$HOME/.config/ai_worker/vr_motion_calibration.yaml" ]]; then
    echo '先に ./calibrate_vr.sh を実行してください。' >&2
    exit 1
fi
if [[ ! -f "$CACHE/robot.urdf" || "${1:-}" == '--refresh-model' ]]; then
    ssh -S "$SOCKET" "$ROBOT" 'docker exec ai_worker bash -lc '\''source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/contactgrove_sharpa/install/setup.bash
python3 /workspace/contactgrove_sharpa/src/ai_worker/ffw_teleop/scripts/export_vr_preview_model.py'\'''
    ssh -S "$SOCKET" "$ROBOT" 'docker cp ai_worker:/tmp/vr-preview-model.tar.gz /tmp/vr-preview-model.tar.gz'
    scp -o ControlPath="$SOCKET" "$ROBOT:/tmp/vr-preview-model.tar.gz" "$CACHE/model.tar.gz"
    tar -xzf "$CACHE/model.tar.gz" -C "$CACHE"
    python3 -c 'import pathlib,sys; p=pathlib.Path(sys.argv[1]); p.write_text(p.read_text().replace("__PREVIEW_MESH_ROOT__", "file://"+str(p.parent)))' "$CACHE/robot.urdf"
fi
pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
    # Only the isolated preview domain is targeted; no physical processes restart.
    ssh -S "$SOCKET" -o ConnectTimeout=3 "$ROBOT" 'docker exec ai_worker pkill -INT -f "[s]harpa_vr.launch.py.*preview98"' >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
# The launch process group is identified by an otherwise unused launch argument.
ssh -S "$SOCKET" "$ROBOT" 'docker exec ai_worker bash -lc '\''source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/contactgrove_sharpa/install/setup.bash
export ROS_DOMAIN_ID=98
exec ros2 launch ffw_teleop sharpa_vr.launch.py dry_run:=true preserve_orientation_reference:=true preview98:=true'\''' > "$CACHE/solver.log" 2>&1 &
pids+=("$!")
ROS_DOMAIN_ID=98 ros2 run robot_state_publisher robot_state_publisher "$CACHE/robot.urdf" > "$CACHE/robot-state.log" 2>&1 &
pids+=("$!")
ROS_DOMAIN_ID=98 /usr/bin/python3 -c 'from ffw_teleop.contactglove_sharpa import main; main()' \
    --ros-args --params-file "$HOME/.config/ai_worker/contactglove_sharpa.yaml" \
    -p dry_run:=true -p auto_enable:=true > "$CACHE/glove.log" 2>&1 &
pids+=("$!")
/usr/bin/python3 -c 'from ffw_teleop.vr_preview import main; main()' \
    --domain 98 --palms "${VR_PREVIEW_PALMS:-inward}" --urdf "$CACHE/robot.urdf" > "$CACHE/relay.log" 2>&1 &
pids+=("$!")
echo "モデル専用プレビューを起動しました。実機への出力はありません。ログ: $CACHE"
echo 'RVizで方向・移動量・回転を確認してください。終了はこのターミナルでCtrl+C。'
ROS_DOMAIN_ID=98 rviz2 -d "$REPO_DIR/ffw_teleop/config/sharpa_vr_preview.rviz" &
pids+=("$!")
wait "${pids[-1]}"
