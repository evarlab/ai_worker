# 昇降操作、rosbag記録、再生

これらは独立した機能。昇降操作を起動しても記録は始まらず、記録を起動しても実機は動かない。各ターミナルの出力も既存の `~/.local/state/ai_worker/terminal/` に保存する。

## 昇降だけ操作する

本体と `lift_controller` が起動した状態で、手元PCから実行する。

```bash
./lift_keyboard.sh
```

↑で上昇、↓で下降、1入力1cm。押し続けると繰り返す。Spaceで現在位置保持、qまたはCtrl+Cで保持して終了する。移動幅の変更例は `./lift_keyboard.sh --step 0.005`。SG2のURDF範囲は−0.5〜0m。腕やハンドの指令は発行しない。

## 記録

本体、左右手元カメラ、頭のカメラを起動してから、別のターミナルで実行する。

```bash
./record_rosbag.sh
```

既存の実機起動が `launch_cameras:=false` の場合、実機コンテナ内のROS環境をsourceしたターミナルから別途カメラを起動する。

```bash
ros2 launch ffw_bringup camera.launch.py head_camera_type:=zed auto_assign_cameras:=false colorize_depth:=false
```

左右のRealSenseシリアル番号は既存の機体設定を使う。録画に必要なのは色付けされた深度画像ではなく、16UC1または32FC1の距離画像。配信が不足する場合、記録は不足項目を表示して終了する。カメラ実機での配信確認は別途必要。

保存先は実機コンテナ内 `/workspace/contactgrove_sharpa/recordings/SESSION`。ホストの `/home/robotis/ai_worker/docker/workspace/contactgrove_sharpa/recordings/SESSION` に永続保存される。Ctrl+Cで記録終了を待つ。録画の開始・終了だけで他の操作プロセスは停止しない。

MCAP形式で次を保存する。

- 実測関節位置・速度・effort、各コントローラの軌跡指令と状態、TF。
- 左右手元と頭のRGB、距離画像、CameraInfo。
- `/dynamic_joint_states` の `Present Current`。ドライバの元の値を保存し、アンペアへの換算はしない。
- URDFとメッシュのコピー、トピック・深度encoding・電流ソースを記載した `session.yaml`。

Sharpaの `joint_states.effort` はトルク値。Sharpaの電流値としては扱わない。Dynamixelもeffortだけで電流を判定せず、元のPresent Currentを保存する。トピック名が異なる場合は `ffw_teleop/config/rosbag_rgbd.yaml` を編集する。診断用の部分記録のみ、明示的に `--allow-missing` を使える。

## RVizで再生

実機からSESSION名のフォルダをコピーする。SESSION名は記録時に表示される日時と数字。

```bash
./fetch_rosbag.sh SESSION
./replay_rosbag.sh ./recordings/SESSION
```

RVizには記録したロボットと6画像を表示する。既定ROS_DOMAIN_ID=94で実機とは独立し、アクチュエータの指令トピックは再配信しない。`--rate 0.5`で半速再生。終了はRVizを閉じるかCtrl+C。実機の稼働やカメラ接続は不要。

## 実機を動かして再生

実機の本体コントローラを起動し、VR・グローブの操作をstop、昇降操作を終了してから実行する。コマンドにはコンテナ内の記録パスを指定する。

```bash
./replay_real_rosbag.sh /workspace/contactgrove_sharpa/recordings/SESSION --rate 0.5
```

再生対象は記録した実測関節角度。腕・ハンド・頭・昇降のうち、記録と実機の両方に存在する関節グループを再生する。台車は動かさない。現在の実測姿勢から最初の記録姿勢まで5秒で移動し、その後は記録間隔に従う。各コントローラのFollowJointTrajectory Actionへ送るため、実機に `control_msgs` と `controller_manager_msgs` が必要。

Ctrl+Cで各軌跡をキャンセルする。別の操作ノードが同じ軌跡トピックへ指令中、関節データ不足、URDF範囲外、フィードバック途絶、コントローラ異常の場合は再生を中断する。記録経路の衝突判定は再計算しないため、録画時と再生時の周囲や昇降位置を確認する。実機での再生検証は未実施。
