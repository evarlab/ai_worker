# Quest 3 + ContactGlove 3 Pro + Sharpa Wave

保存ブランチ: `devel/vr_contactgrove_sharpa_20261116`（`devel/contactgrove_sharpa`を基に開発）。
[ROBOTIS公式VR手順](https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/operation_guide/vr_teleoperation)
のSG2コントローラ方式を使い、腕と指を同時に操作する。

```text
Quest 3 + 左右Touchコントローラ（グローブの甲に固定）
 → Quest Browser → Vuer → 手首Pose → SharpaVRBridge → Cyclo VR IK
 → 腕7関節×左右 → arm_l/r_controller

ContactGlove 3 Pro → PCのDivingStation → contact_glove ROSブリッジ
 → ContactGloveSharpa → 指20関節×左右 → hand_l/r_controller
```

VRコントローラの姿勢から腕の関節角度をIKで求める。Questがロボットの関節角度を
直接計測する方式ではない。VRは腕だけ、ContactGloveは指だけに指令を出す。
頭・昇降・台車・グリッパー指令はVuer側で未使用トピックへリマップする。
標準のグリッパーbringupやSH5用の指リターゲットは併用しない。

## 一括停止

このPCでリポジトリのルートから実行する。

```bash
./stop_teleop.sh
```

PCのグローブサービスと実機のVR指令ゲートを停止する。
終了前に新鮮な実測値へ保持指令を送り、腕・指のモーター電源は変更しない。
実測値が取れない場合は最終指令を保持する。小角度診断スクリプトも停止対象。
SSH接続済みの環境では、ROSの新規サービス探索を待たずに停止できる。

ハンドを脱力させる場合は追加オプションを使う（把持物は保持できなくなる）。

```bash
./stop_teleop.sh --disable-hands
```

接続先は`AI_WORKER_SSH_HOST`、共有ソケットは`AI_WORKER_SSH_SOCKET`で変更可能。
SSHパスワードは保存しない。

## 首掛けQuestでの6方向校正とURDF確認

このPCのリポジトリで実行する。実機の追従を最初に停止する。
Questはページを開いてEnter VR済みであること。校正中に画面を見る必要はない。

```bash
./calibrate_vr.sh
./preview_vr.sh
```

校正は左右それぞれ、移動3方向（前・左・上）と回転3方向
（前後軸まわりのロール・左右軸まわりのピッチ・上下軸まわりのヨー）を記録する。
方向は操作者の前・左・上をロボットの前・左・上に対応付ける。
移動は方向だけを記録し、距離を揃える必要はない。揺れと区別できる程度に動かす。
方向を読み取れないほど小さい移動（3cm未満）は再記録する。移動倍率は1倍で保存する。
回転は手首の中心を保って約45度行う。
各操作前に中立姿勢も記録する。各画面でEnter後5秒待ってから1.2秒記録し、ベル文字を出す。
途中終了はCtrl+C。移動不足はその場で距離を表示し、該当方向だけ再記録する。
移動軸の組合せが不成立なら、その手の6方向を再記録する。
待ち時間は`./calibrate_vr.sh --delay 8`のように指定できる。

保存先はPCの`~/.config/ai_worker/vr_motion_calibration.yaml`と、実機コンテナの同名パス。
同じPCディレクトリへ計測記録JSONも保存する。両手の計算が成立するまでは既存校正を上書きしない。
失敗・中断時を含むターミナルログと計測JSONLは、
`~/.local/state/ai_worker/calibration/`へ逐次保存する。
移動の方向から座標軸、回転からコントローラ基準から手首中心までの局所位置を推定する。
移動距離による倍率の推定はしない。
移動中の手首回転と小さな手ぶれを理由にやり直しにはしない。位置は中央値で集約する。
座標軸を求められないほど方向が重なる場合と、不正・欠落した入力は再記録する。
各方向のなす角度が約53〜127度なら近い直交座標へ補正する。
その補正量が10度を超える場合は警告を残す。方向が近すぎる場合や符号不整合は再記録する。
回転の大きさ自体は20〜100度に制限せず、5度以上の記録を位置補正の推定に使う。
手首位置推定の残差は30mmまで許容する。回転軸が重なる、残差が大きいなどの場合は
位置補正を0にし、警告付きで座標軸の校正を保存する。回転3方向の記録は残す。
回転中に腕全体を動かすと手首位置推定が不正確になるため、前腕を支えて行う。

URDF確認はROS domain 98の仮想ロボットで動く。実機domain 0からは
姿勢・グローブ入力・関節状態のみを読み取り、実機側のROS publisherは作らない。
モデルの腕は実機の現在姿勢をコピーせず、両腕を下げ、手のひらを体側に向けた姿勢から開始する。
腕関節は0を基に、前腕の回転`arm_l_joint5=-pi/2`、`arm_r_joint5=pi/2`を設定する。
手のひらを正面に向けたい場合は`VR_PREVIEW_PALMS=front ./preview_vr.sh`で起動する。
最初の有効化時には、操作者もその姿勢で手を保持する。
それ以外の関節もモデルの0姿勢で初期化するため、プレビューは実機の関節状態配信を必要としない。
腕の計算結果と指のプレビュー結果を仮想関節へ反映し、RVizで表示する。
終了はターミナルのCtrl+CまたはRVizを閉じる。終了しても実機の追従は再開しない。
プレビュー中は、入力の飛びや途切れで停止しても左右の入力が約1秒安定したら
現在の仮想位置へ基準を取り直して自動再開する。手の向きは最初に登録した基準を保持する。
これはmodel domainだけの動作で、
実機の明示的な再有効化条件は変えない。
再開しない場合はログ`~/.cache/ai_worker/vr_preview/`を確認し、プレビューコマンドを再実行する。
初回は実機のURDF・メッシュをPCへキャッシュする。モデル変更後は`--refresh-model`を指定する。
指も確認する場合はPC側の`contact_glove`ノードを起動しておく。
初期姿勢を使う通常のプレビューに、実機のSharpa bringupは不要。

首掛け用Vuerラッパーは頭部姿勢を差し引かず、VR空間の固定座標を使う。
`/vr/input/reference_frame`の`world_v1`を確認し、旧頭部基準の入力を混ぜない。
Vuer更新後はページの再読み込み・Enter VRが一度必要。
VRの再センタリングや原点変更後は7番で基準を取り直す。装着位置を変えた場合は再校正する。
固定座標ではBODY_MOVEの到着を待たず、コントローラの姿勢更新で配信する。
開始時の座標軸は、`use_headset_heading: true`なら現在のQuestの水平な向きから登録する。
VR電源再投入で空間が回っても、Questを体に対して同じ向きにして7番で開始すれば取り直せる。
頭部の右方向を水平面へ投影して向きを求めるため、首の上下の傾きは除去する。
`position_y_sign: -1.0`は左右の反転座標で、位置と回転の両方へ適用する。
回転は反転行列との共役で変換するため、有効な回転行列のまま左右の動作と整合する。
現在の設定は`1.0`。直前の反転設定から左右移動・roll・yawを反転し、前後上下移動・pitchは維持した。
保存した移動軸は、頭部基準を無効にした場合の参照になる。
校正・モデル確認のどちらも、実機への追従を自動で開始しない。
同じVRセッション・同じ装着位置であれば、`./calibrate_vr.sh --resume-log <poses-....jsonl>`で
成立する手の記録を再計算して復元し、未記録の手から続行できる。

## 用意するもの

- Quest 3、左右Touchコントローラをグローブの甲へ固定する。
  現在の設定はグリップ不要で、PCのサービスで開始・停止できる。
- QuestをAI WorkerのWi-Fiへ接続。Vuerは実機の`robotis-applications`コンテナで動かす。
- AI Workerの既存Sharpa bringupと、既存のPC側グローブ設定。
- ヘッドセットはEnter VR状態にする。首掛け用ラッパーは固定のVR空間を基準とし、
  首の動きを手の座標へ加えない。標準SG2の頭部基準入力とは混在させない。

確認対象の実機依存ソース:

- robotis_applications: `5f0d6c340792e4d497225792a8d7b2e4c16619d3`
- cyclo_control: `951f5d2f185b4df9e84983f9bca97fa07cefe381`
- Vuer: ROBOTISコンテナ同梱版（公式指定は`v0.1.6-robotis`）

## 1. Cycloへの小さな修正とビルド（実機、一度だけ）

標準CycloのVRノードは腕以外について主にliftだけを実測値へ更新する。
Sharpaの指は別ノードが動かすため、**指を含む非VR関節を毎周期実測値へ更新**する修正が必要。
修正は`patches/cyclo_vr_passive_feedback.patch`。別バージョンで文脈が合わない場合は
スクリプトが何も変更せず終了する。修正前のVRバイナリと組み合わせない。

実機の`ai_worker`コンテナ内で、ソース配置先を読み替えて実行する。

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /workspace/contactgrove_sharpa/src/ai_worker
bash ffw_teleop/scripts/patch_cyclo_sharpa.sh /root/ros2_ws/src/cyclo_control
# この実機はsymlink-install済みのため、対象実行ファイルの再ビルドで反映される
cmake --build /root/ros2_ws/build/cyclo_motion_controller_ros --target vr_controller_node -j1
cd /workspace/contactgrove_sharpa
colcon build --symlink-install --packages-select ffw_teleop
source install/setup.bash
```

新規ワークスペースではCycloの依存関係を公式手順で導入し、上のパッチ後に
`colcon build --symlink-install --packages-up-to cyclo_motion_controller_ros ffw_teleop`でビルドする。
SDK・モデルの導入は[Sharpa手順](../ffw_sharpa_hardware/README.md)を参照。

## 2. Sharpa本体

既存の実機bringupを使う。同じバスを使うbringupを二重に起動しない。
初期姿勢へ自動で動かす設定はここでは使わない。

```bash
ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py \
  hands:=both init_position:=false hand_speed_coeff:=0.5 \
  launch_cameras:=false launch_lidar:=false
```

既存bringupでハンドをinactiveにしてある場合は、前の[グローブ手順](CONTACTGLOVE_SHARPA.md)
の再有効化手順でハンドだけを再開できる。VRのプレビュー検証にはハンドのトルクは不要。

## 3. Vuerサーバー

`robotis-applications`コンテナには標準の`robotis_vuer`が導入済み。
このコンテナにも`ffw_teleop`をビルドする（この環境では`/workspace/sharpa_vr`へ導入済み）。
標準SG2ノードを利用し、URLのWebSocket指定補完と出力リマップを加えたラッパーを使う。

```bash
# robotis-applicationsコンテナ内
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/sharpa_vr/install/setup.bash
ros2 launch ffw_teleop vr_sharpa_publisher.launch.py
```

QuestがAI WorkerのWi-Fiに接続されていれば、Quest内のブラウザで:

```text
https://192.168.6.2:8012?ws=wss://192.168.6.2:8012
```

証明書警告を確認して続行し、操作位置で`Enter VR`を押す。
短い`https://192.168.6.2:8012`だけでも同じホストのWSS接続先を補完する。
Vuerサーバー再起動時はページを再読み込みしてEnter VRし直す。

USBでPCからブラウザを開く場合は、開発者モード・USBデバッグ許可後に:

```bash
bash ffw_teleop/scripts/connect_quest_usb.sh robotis@ffw-snpr48a1130.local
```

この補助スクリプトはPC→実機のSSHトンネルとADB reverseを作り、Quest Browserを開く。
USBデバッグ未許可やUSB機器として未認識の状態ではリモート操作できない。
Quest内の証明書確認・WebXR権限・Enter VRの操作が必要になる場合がある。

## 4. まず腕をプレビュー

`ai_worker`コンテナのSharpa overlayをsourceしたターミナルで:

```bash
ros2 launch ffw_teleop sharpa_vr.launch.py dry_run:=true
```

- 実際のSharpa付きxacroから計算用URDF/SRDFを生成する。
- 指・台車・頭・liftの最適化速度を0に固定し、実測の指姿勢で計算する。
- continuous関節は計算用モデルのみscalar revoluteへ変換し、Cycloのnq/nv不一致を防ぐ。
- 衝突計算は各STLを包むリンク座標系のboxへ置換する。可視モデルと実機ドライバは変更しない。
- 同じハンド内と機械的な取付部は衝突対象から除外する。左右ハンド間・ハンドと胴体/腕は残す。
- プレビューの出力先は`/vr/preview/{left,right}/joint_trajectory`。

PC側から次のサービスで有効化する（現在はグリップ不要）。

```bash
ros2 service call /sharpa_vr_bridge/enable std_srvs/srv/SetBool '{data: true}'
```

左Xと右Aを同時に押して有効化する操作も使える。
有効化時のVR姿勢を現在の実測手首姿勢へ対応付ける相対方式なので、
再開時に固定の絶対姿勢へ飛ぶことはない。そこからの移動・回転を等倍で反映する。
公式Cycloの開始時照合・3秒待機・slow startも残る。

```bash
ros2 topic echo /vr/preview/left/joint_trajectory
ros2 topic echo /vr/active
ros2 topic echo /vr/state/left/wrist_pose --once
```

各出力が`arm_l_joint1..7`または`arm_r_joint1..7`だけで、指関節を含まないことを確認する。
VRデータが0.3秒途絶する・姿勢が不連続になる・robot feedbackが途絶すると
腕追従を停止し、新鮮な実測値がある場合は腕へ保持指令を送る。自動再開はしない。
現在の急変判定は、連続する入力間で位置0.8m超または回転170度超。
停止ログに左右、位置・回転の変化量、入力時間間隔を残す。
再開は上記サービス、停止もサービスで操作できる:

```bash
ros2 service call /sharpa_vr_bridge/enable std_srvs/srv/SetBool '{data: false}'
```

## 5. 腕と指を同時に動かす

通常の7番（`./resume_teleop.sh`）は、先に実機の両腕をURDF初期姿勢へ移動する。
最低5秒の軌道で移動し、到着した実測姿勢に対してその時のVR姿勢を登録する。
モデルと実機は`vr_start_pose.py`の同じ関節値を使う。操作者も腕を下げた初期姿勢で保持する。
8番は初期姿勢への移動も中止し、その後の追従開始を取り消す。
低レベルの`~/enable`サービス単体は移動を行わず、現在の実測姿勢を基準にする。

プレビューを停止してから実機用の腕ブリッジを起動する。

```bash
ros2 launch ffw_teleop sharpa_vr.launch.py dry_run:=false
```

PC側は既存のグローブ方式で開始する:

```bash
systemctl --user start contactglove-sharpa
ros2 launch contact_glove contact_glove.launch.xml
```

ハンドのcontroller/hardwareがinactiveなら先に有効化する。
グローブのlaunch停止は指の追従停止、VRのenableサービスで`false`を送ると腕の追従停止。
同時に止める場合はVRを無効化し、PCのグローブサービスも止める。
ハンド自体の脱力は前の手順のhardware inactive操作。

グリップによるdeadman操作を戻す場合は、`sharpa_vr.yaml`の`require_squeeze: true`と
Vuer launchの`squeeze_threshold:=0.8`を組み合わせる。
Vuer起動中は`ros2 param set /vr_trajectory_publisher goal_pose_squeeze_threshold 0.8`でも
閾値を変更できる。グリップ不要の現在値は0.0。

## グローブの横振り

`include_abduction: true`で、屈曲15関節に次の5関節を追加する。

| SDKのTilt | Sharpa |
| --- | --- |
| thumb.tilt（index 3） | thumb_MCP_AA |
| index.tilt（7） | index_MCP_AA |
| middle.tilt（11） | middle_MCP_AA |
| ring.tilt（15） | ring_MCP_AA |
| little.tilt（19） | pinky_MCP_AA |

各側の`abduction_gains` / `abduction_offsets`は上の順に5要素。
`rad(Tilt) * gain + offset`を送る。負方向も保持する。
現在のPCプロファイルは5関節ともgain 1.0で、取得時の実機姿勢に合うoffsetを設定済み。
取り付けやcalibを変えたら基準を再取得する。方向が逆なら対応するgainとoffsetを調整する。
`clamp_targets: false`なら追加のソフト角度制限は使わず、実機URDF範囲をドライバが適用する。
`true`では横振りは±0.35 radに制限する。
`thumb_CMC_AA`と`pinky_CMC`は対応する独立した計測値がないため保持し、合計20関節を制御する。

## 甲に固定する際の合わせ方と制限

- `sharpa_vr.yaml`の`left/right_mount_xyz`は、コントローラ基準から手首までの局所位置[m]。
  `left/right_mount_rpy_deg`で局所回転を指定する。0は未測定の初期値。
- Vuer標準の「コントローラを握る」前提の12cm補正は、甲装着用設定では0にする。
  実測した位置ずれを`mount_xyz`へ設定し、二重に補正しない。
- 甲への固定で中立時の向きが傾く場合は、ロボット手首と対応させたい中立姿勢で
  両グローブを保持し、次のサービスを実行する。

  ```bash
  ros2 service call /sharpa_vr_bridge/calibrate_mount std_srvs/srv/Trigger '{}'
  ```

  左右のコントローラ姿勢を実測手首の向きに合わせる回転補正を求める。
  腕を新しい姿勢へ動かす操作ではなく、校正後も追従は停止したまま。
  `calibration_file`（既定は実機コンテナ内の`~/.config/ai_worker/vr_mount_calibration.yaml`）に保存し、
  次の起動時にも読む。装着位置を変えたら校正し直す。
  相対方式では有効化時の固定傾きは元々相殺されるため、この校正は中立時の座標軸も
  対応付けるためのもの。物理的な取付位置のずれは`mount_xyz`で別に合わせる。
- 再有効化時に相対基準を取得する。取り付けがずれたら停止して付け直し、再度有効化する。
- VR目標は0.8 m/s・3 rad/sを上限に連続更新し、瞬間的な参照ジャンプを抑える。
  これは腕の目標姿勢の設定で、指の等倍・追加速度制限なしの設定とは独立。
- 衝突用boxはメッシュの包絡形状なので、実形状より早く接近を制限することがある。
  同じハンド内の指同士の干渉は腕のQPでは扱わない。グローブ横振りの実機干渉は未検証。
- ソフトの停止はモーター電源断ではない。全姿勢・全障害物の衝突回避を保証するものではない。
- 実機bringup・Vuer・Cyclo・PCを同じROS_DOMAIN_ID/RMWに揃える。
  この環境は既存稼働スタックに合わせたDDS/domain 0。公式Docker既定のZenoh/domain 30を混在させない。

## 検証

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="ffw_teleop:$PYTHONPATH" python -m pytest ffw_teleop/test -q
```

座標変換、モデル生成、横振りマッピング、7関節のみの指令通過、deadman、入力途絶、
異常値、追跡復帰時の再有効化を検証する。2026-11-16時点で29テストが成功。

計算と実機を切り分けるスクリプト:

- `scripts/check_vr_ik.py --urdf <展開済みURDF> --domain 97`：別ROSドメインの
  `sharpa_vr.launch.py dry_run:=true`へ合成入力を送り、3cmの手首移動が関節目標を変えるか検証。
  物理ロボット用domain 0/30や実機出力設定では実行を拒否する。
- `scripts/check_arm_joint.py`：VR追従を停止した状態で、実機右手首を0.02 radだけ往復させる。
  JTC参照値と実測値を比較する。実機コンテナ内でのみ使用する。

確認結果：3cm入力で関節目標が約0.098 rad変化し、予測手首誤差は約0.4mm。
実機右手首の試験では実測変化0.020013 rad、復帰誤差約0.00001 radを確認した。
Quest入力による左右腕の実測角度変化と、左右ハンドの動作も確認した。
再起動後の最後の確認では左右腕と右指の動作を確認し、左指は追従有効まで確認。

実機で確認された制限:

- 追跡姿勢の急変で停止したため、判定を0.5m/120度から0.8m/170度に変更。
- 左VR入力が0.342秒途絶した際にも停止。通信監視は0.3秒のまま維持している。
- 物理非常停止後にコントローラがactiveでも腕が動かない状態があり、
  追従を停止した上で`init_position:=false`のbringup再起動により駆動を復旧した。
- 同梱Dynamixelドライバの`dxl_state.torque_state`は更新されない内部マップを参照しており、
  false表示だけでは実モータのトルクOFFと断定できない。上流ドライバの修正はこのブランチに含まない。
- followerとbaseのトルク操作サービス名が重複しているため、サービスの成功応答だけで
  腕の駆動復旧と判断しない。エラーランプ、通信状態、実測角度を併せて確認する。
- 6方向の記録から左右の補正を保存し、仮想モデルと実機の左右腕・指の追従を確認した。
  最後の実機右手首試験では指令0.02 radに対し実測0.020054 rad、復帰誤差0.000021 rad。
- その後の再開時には、ボタン情報を受信していても手首Poseの更新が長時間途切れ、
  実機の再有効化を拒否する状態が残った。首掛け時の装着検知・コントローラ追跡は未解決。
  BODY_MOVEの同期待ちは除いたが、Pose更新断の解消までは確認できていない。
  古いPoseを新しい時刻で再配信して停止監視を回避しない。
- 校正値の例は`config/vr_motion_calibration_example.yaml`。最新記録から左右共通の軸を求め、
  手首中心補正は左右とも0へフォールバックした。装着変更時は再校正する。
  通常の開始時の水平軸はQuestの向きから登録し直す。

## 中央付近の到達性（調査中）

中央だけを禁止する追加の作業領域制限は見つかっていない。
右肩`arm_r_joint2`のURDF範囲は`[-3.14, 0]` radで、IKはこの範囲と衝突判定を使う。
この範囲が機構上の限界か、運用上の保守的な設定かは未確認。まだ変更していない。

衝突を評価しない静的IKで、右手首の位置・向きを同時に指定すると、中央を越える位置で
解が出にくいケースを確認した。例としてbase_link座標`[0.2, 0.1, 1.05]` mでは位置誤差約32mm。
向きの条件を外すと位置誤差はほぼ0となった。
診断上だけ肩の範囲を広げた計算でも解が出たが、衝突・実機可動範囲は未検証なので採用していない。
初期姿勢付近の特異姿勢・局所解・手首の向き・衝突を切り分ける調査が残っている。
この計算では実機に指令を送っていない。
