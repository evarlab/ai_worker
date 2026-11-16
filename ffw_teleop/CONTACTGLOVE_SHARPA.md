# ContactGlove 3 Pro → Sharpa Wave（日本語手順）

`devel/sharpa`を起点にした`devel/contactgrove_sharpa`向け。
既存のSharpa ros2_controlドライバを使い、ContactGloveの指の屈曲を左右の
`hand_l_controller` / `hand_r_controller`へ送る。腕・台車の操作は含まない。

## 現在の運用：手元のlaunchだけで開始・停止

AI Workerの実機bringupと、PCの`contactglove-sharpa.service`を起動しておき、
手元では次のコマンドだけを実行する。

```bash
ros2 launch contact_glove contact_glove.launch.xml
```

左右の新鮮な追跡成功データと実機feedback、ハンド指令の受信先が揃うと自動追従。
このlaunchをCtrl+Cで終了すると、新しい指令の送信を停止して最終指令を保持する。
データ途絶の検出は最大約0.3秒、送信済み軌道は最大約0.04秒先まで残る。
ハンドの電源・トルクを切る操作ではない。launchの再起動で自動追従を再開する。
追跡失敗等による停止はデータが戻っただけでは再開せず、launchを再起動するか
`/contactglove_sharpa/enable`に`true`を送る。
キャリブレーション時もこのlaunchを停止し、DivingStationで調整する。
キャリブレーションで基準が変わった場合はoffsetも再調整する。

現在のPC設定は`~/.config/ai_worker/contactglove_sharpa.yaml`。
実機とグローブのID、現在の対応姿勢で求めたoffsetを含む。

- `input_mode: ros`：公式ROSブリッジの`/contact_glove/{left,right}/ergonomics`を購読。
- `controlled_fingers: all`、`include_abduction: true`：親指を含む屈曲15＋横振り5関節。
- `left_gains` / `right_gains`：全て1.0（等倍）。
- `clamp_targets: false`：追加のソフト角度制限を無効。
- `max_speed: 0.0`：ソフトの指令変化率制限を無効。
- `rate: 60.0`、`horizon: 0.04`：60 Hzで軌道を更新。

ハンドのURDF可動限界は実機ドライバで適用する。
ハンド本体の保護機能も有効なまま。実機bringupの`hand_speed_coeff`は
ドライバ対応上限の`0.5`で起動している。
`thumb_CMC_AA`と`pinky_CMC`は保持するため、22自由度の完全再現ではない。
横振りの対応表・5要素のgain/offsetは[VR＋Sharpa手順](VR_SHARPA.md#グローブの横振り)を参照。

PC側の常駐処理を手動操作する場合:

```bash
systemctl --user status contactglove-sharpa
systemctl --user stop contactglove-sharpa
systemctl --user start contactglove-sharpa
journalctl --user -u contactglove-sharpa -f
```

このサービスはPCへ導入済み。ログイン時の自動起動は設定していない。
別PCへの導入時は、ROS 2 Jazzyと`contact_glove_msgs`を導入し、補正済み設定を用意してから
`scripts/contactglove-sharpa.service`を`~/.config/systemd/user/`へ配置し、
リポジトリの場所に合わせて`ExecStart`を調整する。
`systemctl --user daemon-reload`と`systemctl --user start contactglove-sharpa`で起動できる。
このROS入力モードではPython SDKは不要。

サービスを使わず起動する場合（同時には起動しない）:

```bash
bash ffw_teleop/scripts/run_contactglove_sharpa.sh ~/.config/ai_worker/contactglove_sharpa.yaml
```

または`ffw_teleop`をビルドして、
`ros2 launch ffw_teleop contactglove_sharpa.launch.py params_file:=<補正済み設定の絶対パス>`。
付属の`contactglove_sharpa_ros.yaml`はプレビュー用の設定例。
`contactglove_sharpa_calibrated_example.yaml`は今回動作確認した等倍offsetのスナップショット。
グローブIDは除去し、プレビュー出力にしてある。
実機で再利用する場合は装着・基準姿勢を確認し、左右IDと`dry_run: false`を設定する。
以下は初回確認およびPython SDKから直接受信する場合の手順。

### ハンドを脱力させる

PCで`systemctl --user stop contactglove-sharpa`を実行してから、
実機コンテナ内のROS環境で次を実行する。

```bash
ros2 control switch_controllers --deactivate hand_l_controller hand_r_controller --strict
ros2 control set_hardware_component_state sharpa_left_hand inactive
ros2 control set_hardware_component_state sharpa_right_hand inactive
```

左右ハンドだけを無効化するため、全身のbringupは起動したままにできる。
再開時はハンドをそれぞれactiveにしてからcontrollerを有効化し、PCの追従サービスを開始する。

```bash
ros2 control set_hardware_component_state sharpa_left_hand active
ros2 control set_hardware_component_state sharpa_right_hand active
ros2 control switch_controllers --activate hand_l_controller hand_r_controller --strict
```

## 対応と前提

- ContactGlove **3 Pro**、DivingStation、公式Python SDK V2 `contactglovesdk==1.0.0`。
- ROS 2 Jazzy。Sharpaのネットワーク・SDK・URDFは
  [既存のセットアップ](../ffw_sharpa_hardware/README.md)を先に済ませる。
- DivingStationは同じPCでも別PCでもよい。別PCでは両者の時計をNTP等で同期する。
- SDK接続要求はDivingStation側で承認する。接続API成功だけでは受信開始を意味しない。

構成:

```text
ContactGlove3 Pro → DivingStation → OSC/UDP → contactglove_sharpa
 → JointTrajectory → hand_l/r_controller → 既存Sharpaドライバ → Sharpa Wave
```

## 1. インストールとビルド

既存手順と同じ`ai_worker`コンテナ内で実行する例。ホストでビルドする場合は
ワークスペースのパスを読み替える。CondaではなくROSに対応する`/usr/bin/python3`を使う。

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m venv --system-site-packages /root/contactglove-venv
source /root/contactglove-venv/bin/activate
python -m pip install 'contactglovesdk==1.0.0'
# ament_pythonの実行ファイルもSDKを入れたPythonで生成する
python -m colcon build --symlink-install --packages-select ffw_teleop
source install/setup.bash
```

`venv`未導入ならUbuntuの`python3-venv`が必要。
新しいターミナルではROS、venv、ワークスペースの順にsourceする。
DivingStationをホストPCで起動している場合、コンテナの127.0.0.1が同じPCを指すのは
host network使用時。bridge networkでは到達可能なホストIPとUDPの戻り経路を設定する。
SDK受信ポートは設定例ではUDP 25792、DivingStation受信ポートはUDP 25791。

## 2. まずmockでプレビュー

ターミナルA（既存Sharpa関連パッケージのビルド済み環境）:

```bash
ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py \
  use_mock_hardware:=true hands:=right init_position:=false \
  launch_cameras:=false launch_lidar:=false
```

DivingStationでグローブを接続し、メーカー手順に従い装着・キャリブレーションを行う。
ターミナルB:

```bash
cp /root/ros2_ws/src/ai_worker/ffw_teleop/config/contactglove_sharpa.yaml /tmp/contactglove_sharpa.yaml
# 別PCならhostをDivingStation PCのIPへ変更する
ros2 run ffw_teleop contactglove_sharpa --ros-args --params-file /tmp/contactglove_sharpa.yaml
```

DivingStationで接続を承認すると、ログに`right glove id=...`が出る。
`hands: left`または`both`で左側も確認できる。

```bash
ros2 service call /contactglove_sharpa/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /contactglove/right/preview
```

`dry_run: true`では実機用トピックに送信しない。プレビューにも新鮮な
`/joint_states`が必要なためmockを起動しておく。15関節の名前・角度・方向を確認する。
mockを指に追従させてRViz等で確認したい場合は、**実機bringupが停止した環境で**
次節のID指定と`dry_run: false`をmockに対して試す。

## 3. 実機の片手で確認

1. プレビューのノードとmockを停止する。Sharpa Pilot、ポリシーなど同じハンドへの
   指令プログラムも停止する。
2. `/tmp/contactglove_sharpa.yaml`の`right_glove_id`を受信ログのIDに設定。
   `hands: right`、`dry_run: false`とし、初回は`max_speed: 0.2`程度で確認する。
3. 実機bringupを起動する。`init_position:=false`にして起動時の全身初期姿勢移動を避ける。

```bash
ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py \
  hands:=right init_position:=false hand_speed_coeff:=0.1 \
  launch_cameras:=false launch_lidar:=false
```

別ターミナルでcontrollerがactive、関節値が更新されることを確認する。

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 run ffw_teleop contactglove_sharpa --ros-args --params-file /tmp/contactglove_sharpa.yaml
```

指周辺を空けて有効化し、一本ずつ小さい屈曲で方向と可動範囲を確認する。

```bash
ros2 service call /contactglove_sharpa/enable std_srvs/srv/SetBool '{data: true}'
# 停止（最終指令を保持、トルクは切らない）
ros2 service call /contactglove_sharpa/enable std_srvs/srv/SetBool '{data: false}'
```

左右とも確認したら`hands: both`と両側のglove IDを設定し、bringupも`hands:=both`へ変更する。
実機送信時は選択した全側のIDが必須。左右はSDKの`is_left`とIDの両方で照合する。

## マッピングと調整

公式SDKの`ErgonomicsData.values`は20要素・度単位。屈曲15要素をradに変換する。
設定のgains/offsetsは**左右それぞれ15要素**で、次の順序:

| 設定位置（0始まり） | Sharpa関節 | SDKインデックス |
| --- | --- | --- |
| 0–2 | thumb CMC_FE, MCP_FE, IP | 0, 1, 2 |
| 3–5 | index MCP_FE, PIP, DIP | 4, 5, 6 |
| 6–8 | middle MCP_FE, PIP, DIP | 8, 9, 10 |
| 9–11 | ring MCP_FE, PIP, DIP | 12, 13, 14 |
| 12–14 | pinky MCP_FE, PIP, DIP | 16, 17, 18 |

`target = radians(glove_angle) * gain + offset`。offsetはrad。
伸ばした状態の値が`a`度なら、offsetを`-radians(a)*gain`として合わせる。
反転が必要ならgainの符号とoffsetを合わせて調整する。
現状の出力範囲はthumb CMC 0–0.6、thumb MCP/IPと各DIP 0–1.0、
その他MCP/PIP 0–1.2 rad。実機で最適化した値ではない。

さらに`left_lower/upper`と`right_lower/upper`で、同じ15関節順に
各関節の出力範囲を指定できる（rad、変更後はノードを再起動）。
現在の初期設定は全関節0–0.35 rad（約20度）、設定例の指令変化率は0.2 rad/s。
上の固定上限より広い範囲は設定できない。
有効化時には実測姿勢から範囲内の目標へ徐々に移るため、
実測姿勢が範囲外なら一時的に範囲外の指令も出る。
この範囲は衝突判定で検証したものではない。特に親指と人差し指は
屈曲以外の保持姿勢によっても干渉するため、各指を個別に確認する。

Sharpa全22関節中、残りの7関節（CMC/MCPのAA、pinky CMC）は指令に含めず、
既存controllerのpartial-joint機能で保持する。最初にその姿勢を確認する。
`include_abduction: false`ではTiltを使わず、AA軸など7関節を保持する。
`true`ではSDKのMCP Tiltを5指のMCP_AAへ割り当て、専用gain/offsetで調整する。
完全な22自由度の姿勢再現・指先位置IK・触覚フィードバックは今回の範囲外。

`controlled_fingers: four_fingers`では親指の3関節も指令から除き、
人差し指・中指・薬指・小指の12関節だけを追従させる。
既定の`all`は親指を含み、横振り無効時は15、有効時は20関節。
`four_fingers`で横振りも有効なら、親指を除いた16関節を指令する。
生の角度が負であることだけでは異常とは判断できないため、表示と実測を確認し、
同じ姿勢のグローブ角度と実機角度からoffsetを合わせる。
最初の制限付き実機確認に使用した設定はPCの`/tmp/contactglove-sharpa-live.yaml`に保存し、
4指のみ・gain 0.5・実機基準姿勢の±0.15 rad以内・変化率0.05 rad/sとしている。
この一時設定はその時の姿勢に依存するため、別の装着やキャリブレーション後に流用しない。

## 通信途絶・再開・制限

- 起動時は無効。有効化は新鮮な追跡成功フレームと選択した全関節分のfeedbackが揃った場合のみ。
- 現在の実測姿勢から目標を`max_speed`で徐々に更新する。
  これは指令の変化率であり、実機の実速度保証ではない。
- SDKの`NotAttempted`は新しい指令値として使わず、最後の追跡成功フレームの
  有効期限も延長しない。`IkFailed`は直ちにその側の入力を無効にする。
- 追跡失敗、NaN、時計ずれ、古い/重複フレーム、入力またはfeedbackが`timeout`以上
  途絶すると指令更新を停止して無効化。最後の指令位置を保持する。
  既に送った軌道は最大`horizon`秒先まで残りうる。
- 復帰後も自動では動かず、enableサービスを再実行する。
- プロセス強制終了やネットワーク断でも既存controllerの最終軌道は残る。
  ハンド自体の無効化は既存bringup/ドライバの手順を使う。bringup停止時は脱力する。
- `Need fresh ...`ならSDK接続承認、時計、glove ID、`/joint_states`、hands設定を確認する。
- 同一ハンドのcontrollerへ別プログラムから同時送信しない。

## ソフトウェア検証

```bash
cd /root/ros2_ws/src/ai_worker
# ROSとSDK入りvenvをsource済みのターミナルで実行
PYTHONPATH="ffw_teleop:$PYTHONPATH" python -m pytest ffw_teleop/test -q
```

マッピング・無効データ・速度制限・ROSノードの有効化/途絶処理をテストする。
実グローブと左右実機で受信と屈曲追従を確認した。
全姿勢での干渉確認は未実施。特に親指の解剖学的な軸・符号・offsetは
実機ごとに確認する必要がある。

参照（SDK 1.0.0、確認したソースcommit `af36f2f7735756d982eb9d5dbf2b8d3d62d00b7c`）:

- [公式SDK](https://github.com/MeltInterfaceTechnologies/ContactGloveSDKv2Python)
- [SDKデータ定義](https://github.com/MeltInterfaceTechnologies/ContactGloveSDKv2Python/blob/af36f2f7735756d982eb9d5dbf2b8d3d62d00b7c/contactglovesdk/_models.py)
- [メーカーのSDK一覧](https://docs.melt-interface-technologies.com/en/docs/contact/sdks)
