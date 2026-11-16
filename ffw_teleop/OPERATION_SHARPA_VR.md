# Sharpa + ContactGlove + Quest 操作手順

作業PC: `/home/koki/ai_worker`。
保存ブランチ: `devel/vr_contactgrove_sharpa_20261116`。
詳細な導入・依存関係は[VR_SHARPA.md](VR_SHARPA.md)。

## 普段使うコマンド

PCのリポジトリ直下で実行する。

| 操作 | コマンド |
| --- | --- |
| 校正 | `./calibrate_vr.sh` |
| 同じVRセッションの記録から続行 | `./calibrate_vr.sh --resume-log <poses-....jsonl>` |
| URDFだけ動かす | `./preview_vr.sh` |
| 実機の腕・指を開始／再開 | `./resume_teleop.sh` |
| 実機の腕・指の追従を停止 | `./stop_teleop.sh` |
| 追従停止後にハンドを脱力 | `./stop_teleop.sh --disable-hands` |

`resume_teleop.sh`は下記の本体・VRサーバー・Cyclo・グローブ入力が起動済みの場合に使う。
ハードウェアやQuestは再起動しない。VR入力が古いと理由を表示して開始しない。
プレビューは閉じてから実機を再開する。実機は現在姿勢から始まり、モデルの下げ姿勢には移動しない。

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

プレビューを終了し、実機の現在姿勢にグローブの位置・向きを対応付ける。
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

グリップを握る必要はない。腕は開始時の実測姿勢、指は保存済みのグローブ校正を使う。
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

最新の確認では左右腕・指の実動作を確認したが、その後のVR手首姿勢の更新断は未解決。
現在の校正例では右方向のばらつきが最大約16度あり、右の手首位置補正は0。
実機の前にURDFで方向を確認する。
