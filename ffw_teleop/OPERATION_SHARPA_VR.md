# Sharpa + ContactGlove + Quest 操作手順

作業PC: `/home/koki/ai_worker`。
保存ブランチ: `devel/vr_contactgrove_sharpa_20261116`。
詳細な導入・依存関係は[VR_SHARPA.md](VR_SHARPA.md)。

## 分割ターミナルで操作する

```bash
cd /home/koki/ai_worker
./teleop_terminal.sh
```

Terminatorが9分割（3列×3段）で開き、各ペインにコマンドが入力済みになる。
9番はフットペダル。Enterで`./foot_pedal.sh`を起動し、左で7番の開始／停止、中央でrosbagの開始／保存終了、右で初期姿勢への低速移動を行う。Ctrl+Cで終了する。
3番はVRサーバー起動後、USB認証済みQuestへteleop URLを自動で開く。初回の証明書警告の承認とEnter VRはQuest内で行う。USB未接続・未承認の場合は最大90秒待って理由を表示する。VR入力の配信は自動で有効になるが、実機追従の開始は引き続き7番。
開くだけでは何も起動しない。クリックしてEnterで実行する。
普段は1（DivingStation）→2（本体）→3（VR配信）→4（グローブ入力）→5（腕IK）の順に起動し、
QuestでEnter VRして姿勢を合わせてから7で開始する。止めるときは8でEnter。
SSHのパスワードは最初の接続時に入力する。以降は同じ接続を共有する。

6は任意の校正・URDF確認用。校正が成功すると次の入力は`./preview_vr.sh`になる。
入力済みコマンドはそのまま編集でき、Ctrl+Uで消せる。
前に校正済みなら6を使わず開始してよい。プレビューを使った場合は閉じてから実機を開始する。
各処理をCtrl+Cで終了すると、そのペインでEnterを押して再実行できる。
この画面は一つだけ開き、本体・VR配信・腕IKを二重起動しない。
Quest内のブラウザ操作は手動。ログが必要な処理は各ペインで見られる。

## 普段使うコマンド

PCのリポジトリ直下で実行する。

| 操作 | コマンド |
| --- | --- |
| 分割ターミナルを開く | `./teleop_terminal.sh` |
| 校正 | `./calibrate_vr.sh` |
| 同じVRセッションの記録から続行 | `./calibrate_vr.sh --resume-log <poses-....jsonl>` |
| URDFだけ動かす | `./preview_vr.sh` |
| 実機の腕・指を開始／再開 | `./resume_teleop.sh` |
| 実機の腕・指の追従を停止 | `./stop_teleop.sh` |
| 追従停止後にハンドを脱力 | `./stop_teleop.sh --disable-hands` |

`resume_teleop.sh`は下記の本体・VRサーバー・Cyclo・グローブ入力が起動済みの場合に使う。
ハードウェアやQuestは再起動しない。VR入力が古いと理由を表示して開始しない。
プレビューは閉じてから実機を再開する。
7番は実行のたびに追従を止め、実機の両腕をURDFプレビューの初期姿勢へ移動する。
腕を下げ、手のひらを太もも側に向ける姿勢。操作者も同じ姿勢で保持する。
移動は最低5秒、関節差に応じて長くする。既に初期姿勢なら移動は省略する。
実測値で到着を確認してから、その時のVRの位置・向きを初期姿勢に対応付け、腕と指の追従を開始する。
移動中も8番で中止できる。中止・入力断・未到着の場合は追従開始へ進まない。
古い開始姿勢を使い回さず、そこから先の移動・回転を反映する。
開始時にはQuestの水平な向きも取得し、両手共通の前・左右の座標軸を取り直す。
Questは毎回、体に対して同じ向きで首に掛ける。首の上下の傾きは除いて水平な向きだけ使う。
電源再投入やVR空間の向きの変更だけなら6方向の再校正は不要で、7番で再登録する。
頭部追跡データがない場合は理由を表示して開始しない。
向きはVuerの`CAMERA_MOVE`から取得し、全身トラッキングに依存しない。
カメラ情報がない場合だけBODY_MOVEの頭部姿勢を使う。
現在は`position_y_sign: 1.0`。直前の`-1.0`設定から、左右移動と前後軸・上下軸まわりの回転を反転した。
前後・上下移動、左右軸まわりの回転は直前の設定と同じ。

## 一から起動する

1. PCでDivingStationを起動し、左右グローブを接続する。取り付けや指の校正はこのアプリで行う。
2. AI Workerへ接続する。パスワードは保存していない。

   ```bash
   ssh -M -S /tmp/contactglove-aiworker-ssh -o ControlPersist=600 robotis@ffw-snpr48a1130.local
   ```

3. 実機のターミナルで本体を起動する。

   ```bash
   docker exec -it ai_worker bash
   source /opt/ros/jazzy/setup.bash
   source /root/ros2_ws/install/setup.bash
   source /workspace/contactgrove_sharpa/install/setup.bash
   ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py \
     hands:=both init_position:=false hand_speed_coeff:=0.5 \
     launch_cameras:=false launch_lidar:=false
   ```

4. 別の実機ターミナルでQuestのVRサーバーを起動する。

   ```bash
   docker exec -it robotis-applications bash
   source /opt/ros/jazzy/setup.bash
   source /root/ros2_ws/install/setup.bash
   source /workspace/sharpa_vr/install/setup.bash
   ros2 launch ffw_teleop vr_sharpa_publisher.launch.py
   ```

5. QuestをAI WorkerのWi-Fiへ接続し、ブラウザで次を開いてEnter VRする。

   ```text
   https://192.168.6.2:8012?ws=wss://192.168.6.2:8012
   ```

   サーバーを再起動した場合はページを再読み込みする。
   首掛けでは、装着検知やコントローラ追跡が途切れる問題が未解決。
   ボタン情報が届いていても手首姿勢が更新されないことがある。

6. PCの別ターミナルでグローブ入力を起動する。

   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 launch contact_glove contact_glove.launch.xml
   ```

## 校正してモデルで確認する

```bash
cd /home/koki/ai_worker
./calibrate_vr.sh
./preview_vr.sh
```

左右それぞれ、前・左・上へ動かし、ロール・ピッチ・ヨーで回す。
距離を揃える必要はない。倍率は1倍。Enter後5秒で記録する。
小さな手ぶれや移動中の手首回転では不成立にしない。
手首位置の補正が推定できない場合は0で保存し、方向の校正を続行する。
入力がない、移動が小さすぎる、3方向を区別できない場合は取り直す。

校正値は`~/.config/ai_worker/vr_motion_calibration.yaml`へ保存され、実機コンテナにもコピーされる。
ログは`~/.local/state/ai_worker/calibration/`。取り付け・VR原点が変わったら再校正する。

モデルの開始姿勢は両腕を下げ、手のひらを太もも側に向けた姿勢。
操作者もその姿勢で保持して開始する。手のひら正面なら
`VR_PREVIEW_PALMS=front ./preview_vr.sh`を使う。
プレビューはROS domain 98で、実機へは出力しない。終了はRVizを閉じるかターミナルでCtrl+C。

## 実機で開始・再開する

プレビューを終了し、操作者は両腕を下げて初期姿勢で保持する。
本体ターミナルとは別の`ai_worker`コンテナのターミナルで、腕の計算系を一度起動する。

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/contactgrove_sharpa/install/setup.bash
ros2 launch ffw_teleop sharpa_vr.launch.py dry_run:=false
```

PCで開始する。

```bash
cd /home/koki/ai_worker
./resume_teleop.sh
```

グリップを握る必要はない。7番では実機も初期姿勢へ移動してからVRの基準を取る。
指は保存済みのグローブ校正を使う。
物理非常停止と、古い／不正な入力・通信断での停止は残している。
実機側の入力断からの自動再開はしない。モデル確認だけは入力が安定すると自動再開する。

## 停止・全終了

```bash
./stop_teleop.sh
```

腕・指の追従を止め、取れる場合は実測姿勢を保持する。モータ電源は変更しない。
再開は`./resume_teleop.sh`。

全終了するときは、追従停止後に起動した各ターミナルのVRサーバー・腕計算系・
`contact_glove`をCtrl+Cで終了し、最後に本体bringupをCtrl+Cで終了する。
本体の終了ではハンド・腕のドライバも終了する。DivingStationも閉じる。

## 再開できないとき

| 表示・状態 | 対応 |
| --- | --- |
| `vr wrist pose ... old` / `missing` | QuestのEnter VR・コントローラ追跡を確認。古い姿勢では開始しない |
| `robot wrist pose ... missing` | 本体のjoint state配信とCycloを確認 |
| コントローラactiveなのに動かない | active表示だけでは駆動確認にならない。本体を停止し`init_position:=false`で起動し直す |
| 腕が未構成、通信エラー | モータ電源・非常停止・通信を確認してから本体を起動し直す |
| SSHで待ち続けていた | 再開8秒・停止6秒でタイムアウトするよう修正済み。有線LAN/Wi-Fi接続を確認 |

再開ではdomain 98のプレビューをdomain 0の実機ブリッジと区別して検出する。
停止で実機ブリッジを終了しても、7番が切り離したプロセスとして再起動する。
関節コントローラ自体がinactiveなら、7番はその理由を表示する。通信・電源を確認して2番を再起動する。

最新の確認では左右腕・指の実動作を確認したが、その後のVR手首姿勢の更新断は未解決。
現在の校正例では右方向のばらつきが最大約16度あり、右の手首位置補正は0。
実機の前にURDFで方向を確認する。

## 常時ログ

分割画面で実行した各コマンドの端末出力は、毎回別ファイルとして
`~/.local/state/ai_worker/terminal/`へ保存する。
単独実行した開始・停止・プレビューも保存する。校正ログは従来どおり
`~/.local/state/ai_worker/calibration/`に残る。過去のログは上書きしない。

VRブリッジは入力Pose・Questの向き・目標Pose・実測関節角・腕指令・
開始時の基準と座標軸・停止理由を`~/.local/state/ai_worker/teleop/`へ記録する。
Vuerも元のコントローラ行列と頭部姿勢を同じディレクトリへ記録する。
この2つは各実機コンテナ内のパス。ファイルは時刻・PID付きの`*.jsonl.gz`で、
連続データは20Hz、開始・停止などのイベントは都度記録する。
