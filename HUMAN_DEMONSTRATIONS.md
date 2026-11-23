# 人間のContactGlove＋Quest実演を収集し、AI Workerへ展開する

## 今回の機能

### Quest Proへの変更

Quest Pro＋Touch Proでも、位置・姿勢はWebXRの標準gripSpaceで受ける。プロファイル名をQuest 3専用に固定していない。ブラウザがimmersive-arを提供しない場合はimmersive-vrへ切り替える。Touch Proはヘッドセットとは独立した自己追跡を行うが、コントローラ自身のカメラをグローブ等で覆わないように取り付ける。追跡欠落や位置の推定補完は引き続き検査する。

**Quest ProはMetaの撮像用Passthrough Camera APIの対応機種に含まれていない。** パススルーが見えることと、校正可能な生RGBをアプリが取得できることは異なる。下記のQuest RGB収集経路をQuest Proで利用可能とは扱わない。今回のQuest Proチェックはコントローラ追跡のみ。今後の人間実演映像は、外付け頭部カメラ等の別経路を検討する。画面キャストを生の校正済みカメラ画像の代用にはしない。

- [Meta: Touch Proの自己追跡](https://www.meta.com/blog/meta-quest-pro-price-release-date-specs/)
- [Meta: Passthrough Camera API対応機種](https://developers.meta.com/vr/documentation/spatial-sdk/spatial-sdk-pca-overview/)

ロボットと接続しないQuest側チェック:

```bash
./check_quest.sh
```

ADBで機種・OS・Browserを確認し、USB reverseでPCのローカル診断ページを開く。Questで証明書を承認してEnter XR後、正面で両コントローラを動かし、その後片手ずつ頭の後ろで3秒程度小さく動かす。左右が新しい追跡姿勢を返すこと、位置がemulatedでないこと、頭の後ろでも2秒以上かつ3cm以上の位置変化があることを確認する。結果とBrowserの入力profileを `~/.local/state/ai_worker/quest-check/` に保存する。これは機体や公式Vuerとの接続確認そのものではなく、その前提となるQuest標準WebXR入力の確認。実機への指令は出さない。

`adb devices -l` がno permissionsの場合、このPC向けの設定例は次。通常ユーザーがplugdevに所属していることが前提。

```bash
sudo install -m 0644 ffw_teleop/udev/99-meta-quest.rules /etc/udev/rules.d/99-meta-quest.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=usb --attr-match=idVendor=2833
```

その後unauthorizedが出た場合は、Quest内でこのPCのUSBデバッグを許可する。

ロボットを接続せず、人間が実際にタスクを実行したデータをPCへ保存する。実機の7番、IK、Sharpaドライバ、実機向けrosbag再生は起動しない。フットペダルの実機テストと機体への反映は後回し。

Quest Browserから外向きの実RGBカメラ1台、両コントローラの6DoF、ヘッドセットの6DoFを受信し、ContactGloveの左右20角度とまとめる。生データは変換せず保存する。WebXR座標は右手系、+X右、+Y上、−Z前、単位m。WebXR行列は列優先、書き出しNPZは通常の4×4行列。

**RGBから深度は生成しない。ブラウザが渡さないカメラ内部パラメータ・カメラ外部パラメータ・センサ露光時刻はnullとして保存する。** 頭の姿勢をカメラ姿勢とみなさない。グローブ角度をSharpa関節角度とみなさない。

## 起動

初回のみ。このPCでは依存環境を作成済み。

```bash
./record_human_demo.sh setup
```

DivingStationで左右グローブを接続し、通常どおり校正する。別ターミナルで、グローブの入力ブリッジだけを起動する。

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch contact_glove contact_glove.launch.xml
```

QuestをPCにUSB接続し、USBデバッグを許可する。その後、収集サーバを起動する。

```bash
./record_human_demo.sh serve --usb
```

このコマンドは `adb reverse tcp:8444 tcp:8444` を設定する。Quest Browserで表示された `https://localhost:8444/?token=...` のURLを開く。初回はローカルの自己署名証明書の承認が必要。URLのtokenは起動ごとに変わる。USB抜き差し後はサーバを再起動してADB reverseを設定し直す。複数Android機器がつながっている場合は、対象Questだけ接続して実行する。

Questのページでカメラ利用を許可すると、外向きカメラを自動選択する。映像を確認して「Enter XR」。映像が実際の机と手になっていることを確認する。avatar/front/selfieは使用しない。両コントローラは手の甲に固定したままでよく、グリップボタンは不要。

初期設定は希望1280×960、上限15fps、JPEG品質0.85。実際の解像度・fpsは機器との交渉結果を保存する。画像アップロード中の次フレームは間引き、画像待ちの無制限キューを作らない。間引き累計を画像メタデータに記録する。生RGBではなくJPEGなので圧縮誤差がある。左右ステレオやRGB-Dとしては扱わない。

**Questの実カメラとXRを同時に取得できるかは、このヘッドセットでは未検証。** ブラウザ・OS・権限に依存する。外向きカメラが列挙されない／許可されない場合はエラー表示し、代わりにアバターを記録しない。Metaのブラウザ権限の報告も参照する。

- [Meta: Quest Browser Headset Camera Permission Issue](https://developers.meta.com/vr/support/investigations/1557993751977106/)
- [Meta: Passthrough Camera API、内部・外部パラメータと時刻](https://developers.meta.com/vr/documentation/unity/unity-pca-documentation/)

正確なカメラキャリブレーションや露光時刻が必要な次段階では、MetaのネイティブPCAアプリに取得側を置き換える。このリポジトリにネイティブAPKは含めていない。今回のPC収集APIは、画像ごとにmetadataを保持する構成にしている。

ROSブリッジの代わりにSDKへ直接接続する場合は `--glove-source sdk --glove-host 127.0.0.1`。これはROSブリッジ不要だが、DivingStationのSDK接続が必要。`--glove-source none` はHTTP等の開発確認用で、左右グローブがないため通常の収集開始チェックは通らない。

## 作業台基準を登録

Questの起動時座標をロボットのbase_linkとして保存しない。右コントローラを同じ向きに保ち、机の原点、原点から右、原点から前の3点へ順に置く。点間の距離を10cmぴったりにする必要はない。互いに十分離し、一直線にしない。各点でPCから実行する。

```bash
./record_human_demo.sh mark origin
./record_human_demo.sh mark right
./record_human_demo.sh mark forward
```

この3点から右手系の作業座標（X右、Y前、Z上）を生成する。**基準点はコントローラgrip原点。指先を点に当てる場合、コントローラ→指先の取付変換が別途必要。** 向きを変えるとgrip原点と指先の距離が回転するので、指先を同じ場所へ当てただけでは同じ基準点にならない。

Questのrecenter、XRセッション再開始、reference-space resetごとに新しいreference_idを記録し、古い作業基準は無効化する。新しいreferenceでは3点を取り直す。収集中に既存の作業基準を変更したい場合、一度stopして取り直す。3点登録なしでも生データは収集できるが、書き出した作業座標は無効マスクになる。

## タスクごとに収集

準備後、起動した端末で `follow-start` を入力して頭の初期基準を設定する。別ターミナルからは以下のようにも操作できる。記録開始時は新しい両手XR、左右グローブ、Quest画像が必要。

```bash
./record_human_demo.sh status
./record_human_demo.sh follow-start
./record_human_demo.sh start --task pick_and_place
# 人間がタスクを実行
./record_human_demo.sh stop --outcome success
```

失敗した実演は `--outcome failure`。複数エピソードを続けて収集できる。サーバのCtrl+Cは収集中のエピソードをinterruptedとして閉じる。異常終了でmanifestが未完了のエピソードは通常の書き出しから除外する。収集中に映像や追跡が途切れても、その区間を有効な値で埋めない。

既定保存先は `~/human_demos/SESSION/`。

- `manifest.json`: タスク、成功／失敗、座標の定義、作業基準、ストリームの件数とカメラ情報。
- `events.jsonl.gz`: XR、指角度、追跡切れ、reference reset、時計推定、画像への参照。
- `images/quest_rgb/*.jpg`: Questの外向き実映像。

元のデバイス時刻とPC受信時刻を両方残す。ブラウザ時計→PC単調時計は往復通信の中点で近似し、RTTも保存する。WebXR時刻は表示予測時刻、VideoFrameCallback時刻はブラウザの動画通知時刻であり、カメラ露光との厳密同期ではない。USB接続でもこの差は残る。

## 書き出し

```bash
./record_human_demo.sh export ~/human_demos/SESSION
```

`human_arrays.npz` と `export_quality.json` を作成する。NPZは左右順に、元のコントローラ姿勢、頭基準姿勢、作業台基準姿勢、前フレームからの局所SE(3)変化、指20角度、画像パスを持つ。対応するvalid maskと時刻差を必ず併用する。画像と指の対応はPC到着時刻による近似で、画像は未来のフレームを参照しない。追跡切れ・recenterをまたぐ姿勢差を教師値として生成しない。

このNPZはデータ検査・後処理の中間形式。LeRobot等へ投入する未来action chunkの生成、画像の歪み補正、学習はまだ含まない。元のJPEGも一緒に保持する。`delta_prev_controller`は過去→現在の変化で、そのまま未来のpolicy actionとして使わない。

## AI Workerへどう展開するか

### 座標の違い

`T_A_B`を「B座標からA座標へ変換する行列」とする。人間の手首の作業台基準姿勢は次で求める。

```text
T_task_wrist = inverse(T_world_task) × T_world_controller × T_controller_wrist
```

`T_controller_wrist`は手の甲への取り付けで決まる固定変換。左右それぞれ測る。今回の収集器はコントローラgripを保存するので、手首へ変換したふりはしない。AI Workerで同じ作業基準を測れば、`T_robotBase_task × T_task_wrist`をロボットの目標姿勢へ変換できる。最後にロボット用ツール座標の補正が必要。人間とロボットの可動域、腕の長さ、指の接触形状も異なるので、関節角度の直接コピーはしない。

### 視点の違い

カメラの位置・向き・内部パラメータが分かっても、遮蔽で見えていない部分は画像の単純変換では復元できない。Quest RGBしかない人間実演から、ロボットの頭RGB-Dや左右手元RGB-Dを同等な教師として捏造しない。

最初は**ロボットの頭RGBを共通の観測にし、人間データで視覚・手の動作表現を事前学習し、少量の実機実演を混ぜて適応させる**方針を提案する。ロボットの手元カメラと深度は、実機データにしかない追加観測として扱い、欠けている人間サンプルにはmaskを付ける。学習時に人物／ロボットの手の見た目やカメラ視点の違いを扱う必要がある。

これは実装済みのpolicyではなく、今後の設計案。EgoMimicの人間・ロボット共同学習、UMIの相対軌跡actionと遅延整合が参考になる。ただしUMIは人間とロボットで観測・把持インターフェースを揃えており、今回の5指ハンド＋頭視点へそのまま適用できるわけではない。

- [EgoMimic: 人間・ロボットデータの共同学習とdomain alignment](https://egomimic.github.io/)
- [UMI: relative-trajectory actionとlatency matching](https://umi-gripper.github.io/)

### 段階

1. 今回の収集器で、同じタスクの成功・失敗実演を集める。手元が映ること、指と姿勢の追跡欠落、同期誤差を検査する。
2. カメラ内部パラメータ、頭→カメラ、コントローラ→手首を校正する。精密な接触タスクでは、センサ時刻とカメラ姿勢を取得できるネイティブPCA経路へ移る。
3. 実機テスト再開後、同じ作業台基準・タスクで少量のAI Worker実演を既存rosbag機能で集める。頭・手元RGB-D、関節状態、電流を使い、人間データには無い情報は欠損として扱う。
4. 学習用PCでデータを統一する。policy出力は手先の相対SE(3)の未来chunkと、ロボットへretargetした指の目標。学習時と推論時の遅延・action周期を揃える。必要な実機データ量はタスクと精度を見て決める。
5. 学習済み重み・前処理・座標変換・指のretarget設定をAI Workerへ配置する。データを機体へコピーするだけでは自律動作にはならない。
6. 記録した入力を使ったオフライン評価、URDF/RViz上でIK・到達性・衝突を確認後、実機で閉ループ評価する。人間実演の全軌跡がロボットで実現できるとは限らない。

人間だけのデータで即座に安定した自律実行ができる保証はない。まず価値が失われない生データを収集し、座標変換で解ける問題と、学習・実機データが必要な問題を分けて進める。

## 検証範囲

模擬XR・指・JPEGを使い、HTTPS/WebSocket相当のAPIから収集、保存終了、NPZ書き出しまで自動検証する。右手系の基準、recenterによる無効化、追跡切れのマスク、カメラが無い場合の開始拒否も検証する。Quest実機での外向きカメラとXRの同時取得、USBの実効fps、ロボットへのpolicy展開は未検証。

## PCの開始・停止UI（Quest Pro・映像なし）

入力サーバは次で起動する。DivingStationとROSグローブ入力は別途必要。

```bash
./record_human_demo.sh serve --usb --allow-no-camera
```

Quest側でEnter XRした後、PCで次を実行する。

```bash
./record_human_demo.sh ui
```

「記録開始」「記録停止」、タスク名、success/failureを操作できる。記録状態、VR受信Hz、入力の鮮度、保存先を表示する。UIは既存サーバの制御APIだけを使い、QuestのXR接続を奪わない。UIを閉じても収集サーバは停止しない。開始の最終可否はサーバが左右の追跡と指角度を検査する。

## 前後を切ってモデル再生

元の圧縮ログを保持して、XRの先頭と末尾からそれぞれ指定秒数を除いた配列を書き出す。切り出し後の最初のdeltaは無効。指角度は切り出し前からの到着履歴も参照して因果的に対応させる。

```bash
./record_human_demo.sh export ~/human_demos/SESSION --trim-start 10 --trim-end 10
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/ffw_teleop${PYTHONPATH:+:$PYTHONPATH}"
~/.local/share/ai_worker/human-demo-venv/bin/python -m ffw_teleop.human_demo_sim convert \
  ~/human_demos/SESSION/human_arrays_trimmed.npz \
  --output ~/human_demos/SESSION/sim_trajectory.npz
./replay_human_sim.sh ~/human_demos/SESSION/sim_trajectory.npz --loop
```

キャッシュ済みの実機と同じSharpa付きURDFを使用する。初回の頭の水平方向からWebXR→ロボット（X前・Y左・Z上）を定義する。既定では左右共通の水平移動だけを使い、WebXR local-floorの高さと左右間隔を維持する。開始時の両手中央を前方0.35m・左右0mへ仮配置する（`--forward`で変更）。liftは計算と再生を−0.15mで統一する（`--lift`で変更）。回転は開始からの相対回転。ArUco基準・コントローラ取付位置は未校正で、届かない目標のIK誤差や机との干渉は残る。旧来の左右個別の開始位置合わせは`--alignment relative`。腕はURDF関節限界を守るオフラインIK、指は既存の手元校正設定を適用してURDF限界内に収める。設定とIK誤差はsim_trajectory.jsonに保存する。両手・指が有効なサンプルだけ約10Hzで変換し、欠落区間は最後のモデル姿勢を保持する。

再生はlocalhost限定のROS_DOMAIN_ID=97でjoint_statesとrobot_descriptionだけを使用する。実機ドライバは起動しない。これはRVizでの運動学モデル再生で、物理・接触・衝突の検証や学習済みpolicyの配備ではない。作業基準と手の取り付け変換が未校正なら、到達誤差を確認するための暫定対応づけとなる。終了は起動ターミナルでCtrl+C。

### 既存MuJoCo環境への読み込み

`~/ffw_sharpa_sim` のモデルを読み込む再生も可能。既存側のソースは変更しない。

```bash
./replay_human_mujoco.sh ~/human_demos/SESSION/sim_trajectory.npz --check
./replay_human_mujoco.sh ~/human_demos/SESSION/sim_trajectory.npz --loop
```

環境パスを変更する場合は `AI_WORKER_MUJOCO_ROOT=/path/to/ffw_sharpa_sim` を指定する。腕14＋指44関節を名前で照合し、MuJoCo側の関節限界も適用する。`--check` は全フレームをheadlessで評価し、追加クランプ数・モデル内接触・最大食い込みを `sim_trajectory.mujoco.json` に保存する。ビューアは `mj_forward` による運動学再生であり、物理演算で接触を解く再生ではない。干渉があればそのまま表示されるので、把持成功や物理的実行可能性を示すものではない。既定の机・キューブは記録した人間タスクの物体位置ではない。

## 起動したターミナルでの記録操作とArUco

`serve` は対話端末から起動すると、その端末の行入力で操作できる。

```bash
./record_human_demo.sh serve --usb --aruco --task pick_and_place
# 同じターミナルへ入力し、Enter
follow-start
start
stop success
start another_task
status
follow-stop
quit
```

`start [タスク名]` と `stop [success|failure|unspecified]` はHTTPの操作と同じ
入力鮮度チェック・保存処理を通る。`quit` はサーバを終了し、記録中なら
interruptedとして保存する。入力EOFは端末操作だけを終了し、サーバは継続する。
サービス起動や別端末からの操作は従来どおり。`--no-terminal` で対話入力を無効にできる。

ArUcoは `--aruco` で有効化する。依存を更新する場合は
`./record_human_demo.sh setup` を実行する。既定辞書は `DICT_4X4_50`、
`--aruco-dictionary DICT_5X5_100` などで変更できる。
外向きRGBはページ起動時に自動接続し、検出した枠・IDをそのRGB画像に重ねる。
通常はブラウザ画面で映像と検出結果を確認し、XR中はパネルを表示しない。
URLに `&xr_preview=1` を付けた場合のみWebGLの両眼確認パネルを表示する（DOM overlayは不要）。
これは実空間に固定する3D表示ではない。画像更新が1.5秒以上途絶えたパネルは表示しない。
カメラ画像が取得できない機種・ブラウザでは検出できない。
検出したIDと画像上の四隅は `aruco` イベントとして記録し、生JPEGには描画しない。
Camera2校正情報がない場合、マーカーの距離・実空間姿勢は推定しない。対応する校正情報があればカメラ→紙の姿勢を推定する。

### 追従と頭の初期基準

`follow-start` は直近300ms以内の頭の位置・向きを初期基準として保存する。
頭は `T_head_origin_head = inverse(T_world_head_origin) × T_world_head`、
手はその時点の頭を基準とする
`T_head_controller = inverse(T_world_head) × T_world_controller` で扱う。
手の開始姿勢をゼロにはしない。元のWebXR姿勢は引き続き保存する。
将来のArUco基準への変換は未実装。

記録には先に `follow-start` が必要。`stop success` 後も追従は続き、
次の記録でも同じ頭の初期基準を使う。記録していないときに
`follow-start` を再入力すると頭の初期基準を取り直せる。
`follow-stop` は基準を無効化し、記録中ならinterruptedで閉じる。
Quest切断・XR終了・reference resetでも追従と記録を終了する。
新しい基準を設定するため、追従停止中も入力の受信・状態監視は続く。

manifestの `head_origin` に基準姿勢を保存し、XRイベントに
`T_head_origin_head`、`T_head_left`、`T_head_right` を追加する。
NPZには `T_head_origin_head` と `head_origin_valid` を追加する。
従来の `T_head_controller` は各時点の頭に対する左右の手の姿勢。
頭が追跡できないフレームは相対座標を補完しない。

## 人間収集専用Terminator・フットペダル

```bash
./human_demo_terminal.sh
```

4分割で、1: 収集サーバ、2: 人間収集ペダル、3: 状態表示、4: 任意のMuJoCo場面表示。
1〜3は自動起動し、4はEnter待ち。Quest Pro向けに画像なしを明示し、
SDKから左右グローブを受信する。USB認証・Questの証明書承認とEnter XR、
DivingStationの接続・校正・SDK許可が必要。実機本体やSSHは起動しない。

- 左ペダル: 追従開始／停止。開始時に頭の初期基準を保存。
- 中央: 記録開始／successとして保存終了。開始には追従と新しい両手・指データが必要。
- 右: failureとして保存終了。追従は継続。
- Ctrl+C・ペダル切断: 追従停止、記録中ならinterruptedとして保存。

`./human_demo_pedal.sh --task pick_and_place` でタスク名を設定できる。
実機ペダルと同じロック・専用デバイス入力を使い、同時起動と長押しの再発火を防ぐ。
状態と操作結果はターミナルへ表示する。XR中はDOM上の操作ボタンや状態表示を隠す。
Quest Proは外向きRGB取得非対応のため、ArUco検出・表示はこのモードではできない。

`ffw_teleop/config/human_demo_workspace.json` を各エピソードのmanifestへ保存する。
印刷した50mmマーカー、辞書DICT_4X4_50、ID0〜8の紙内の配置を定義する。
ボード原点はID4の中心、Xは紙の右、Yは紙の上、Zは紙の表方向。
実機展開ではこのボードを共通基準として測定する。
実機基準とWebXR基準の変換はまだ測定されておらずnullで保存する。
頭の初期基準をArUco基準とみなさない。

MuJoCoの人間データ再生と `./replay_human_mujoco.sh --scene-only` は、
天面700mmの机を正面に置き、紙とID0〜8を配置した場面を使う。
机の中心はロボット基準で前方0.62m、紙の上が奥方向と仮定している。
配置は実測ではなくシミュレーション設定。別の机配置ならJSONを変更する。

Quest 3では専用Terminatorが `--aruco` を指定してRGB収集を有効化する。
Quest ProをADBで検出した場合だけ `--allow-no-camera` に切り替える。
Quest Browserのアプリ側カメラ権限とサイト側カメラ許可が必要。
ページの自動接続で外向き実カメラを確認できなければ収集開始を拒否する。
アバター・自撮りカメラへの自動フォールバックは行わない。
画像は記録中だけJPEGとして保存し、ArUco検出ID・画像上の四隅もイベントへ保存する。
カメラの内部・外部パラメータがない現段階では、3D姿勢を推定したものとして扱わない。

再生に `--show-targets` を付けると、記録を変換したコントローラ位置を左は水色・右は紫の球で表示する。球は指先ではなくコントローラの追跡原点。IK後のロボットハンドとはずれる場合がある。机の表面へ手を強制的に押し上げる補正や衝突回避は行わない。

## ArUco基準と手首の取り付け校正

Quest標準WebXRの `hand-tracking` を要求し、25関節と手首を `xr.data.hands`
に保存する。光学ハンド入力とコントローラ入力は別々に扱い、手の入力で
コントローラ姿勢を上書きしない。グローブ装着時の光学追跡や、同じ側の
コントローラとの同時取得ができるかは端末・遮蔽・入力モードに依存する。
`status.capture_calibration` と3番の端末に、手首の有無と同時サンプル数を表示する。

```bash
# 1番のサーバ端末へ行入力（別端末なら ./record_human_demo.sh を前置）
board-start
# 紙を固定し、9マーカーを見ながら頭を上下・左右へ各数回、ゆっくり動かす
board-finish
# グローブ・コントローラ装着状態で手首とコントローラを同時に追跡
mount-calibrate
```

USB接続したQuest 3のCamera2内部パラメータを読み取り、同一カメラの
1280×1280・resizeMode=noneの正方形映像だけで3D推定を行う。
内部パラメータはセンサの正方形領域から等方的に縮小する。
仮想パススルー出力は歪みゼロを仮定するため、再投影誤差と静止紙の位置の
ばらつきで検証する。別カメラ・異なる解像度・crop-and-scaleの映像を
同じ内部パラメータで扱わない。校正情報がない場合は従来の2D検出のみ。

ArUcoの3D推定は既知の紙上配置・50mmの黒い正方形を使う。
2個以上のマーカー、表向き、再投影誤差2px以下を要求する。
`board-finish` は頭の多軸回転と静止したボードの観測から
`T_head_camera` と `T_world_board` を同時推定する。
20組以上の頭・画像サンプル（ブラウザ時刻差50ms以内）を必要とし、
回転方向が不足する場合、頭→カメラが30cmを超える場合、
位置p95が20mm・角度p95が3度を超える場合などは保存を拒否する。
画像露光とXRは厳密同期していないため、この残差を絶対精度の保証としない。

取り付け変換は同じXRFrameの
`T_controller_wrist = inverse(T_world_controller) × T_world_optical_wrist` を使う。
直近10秒の30サンプル以上から回転平均・位置中央値を求め、
15mm・12度以内のサンプルが70%以上なければ拒否する。
光学手首は推定値であり、解剖学的な真値とは扱わない。
グローブ・取り付けを変更したら再校正する。

校正結果は `~/.config/ai_worker/human-demo/calibration.json` に保存し、
記録開始時のmanifestにもスナップショットを保存する。
別セッションへ自動適用せず、そのセッションで校正を実行する。
ボード基準はWebXR reference reset時に無効化する。

校正後のXRイベントには `T_board_left_controller` / `T_board_right_controller` を追加し、
取り付けも校正済みなら `T_board_left_wrist` / `T_board_right_wrist` を追加する。
NPZは `T_board_controller` / `T_board_wrist` と対応するvalid maskを持つ。
既存の未校正記録には真値を捏造せず、これらのmaskをfalseにする。
ロボット手首の軸・ツール原点への変換は別途必要で、これだけで実機配備済みにはならない。

## SDK骨格とコントローラモデルで手動調整する

```bash
./record_human_demo.sh mount-editor
```

PCの別ウィンドウで `/mount-editor` を開く。Questのページではなく、
PCのマウスで位置・向きを比較するための画面。
Touch Pro / Touch Plusの左右GLBモデルと、ContactGlove SDKのリターゲット済み
25関節を表示する。外形CADや光学ハンドトラッキングは不要。
SDK骨格は手モデル原点（手首）からのメートル座標をそのまま使い、
指の長さや形状を目視で拡縮しない。生の25点・19回転・生成時刻・プロファイルUUIDも
`skeleton_left/right` イベントとして保存する。追跡無効や古い骨格は表示しない。

左右を選び、実際の機種の外形モデルを選択する。位置XYZはコントローラのgrip座標系で
手モデル原点を移動する値（mm）。回転つまみはコントローラの固定軸で回す操作角度（度）。
オイラー角の姿勢表現にはせず、つまみの変化量をクォータニオンへ左から合成する。
大きく回した後の「回転つまみを0に」は、モデル姿勢を維持して操作量だけをリセットする。
「指の姿勢を固定」で比較中の骨格を固定できる。

「この手の補正を保存して適用」で、その手の `T_controller_wrist` を保存する。
`target_frame=contactglove_skeleton_root` と `source=manual_model_alignment` を付け、
光学手首関節を使った測定と区別する。手動値は再起動後も読み込むが、
コントローラの装着・グローブの取り付け・手モデルのプロファイルを変えたら再調整する。
記録中は補正の保存・変更を拒否する。

確認・調整した値は次のデータ収集からボード基準の手モデル原点への変換に使う。
手モデルの原点とロボットのツール原点の対応づけは、別の変換として扱う。
初期表示は未校正の見本であり、保存ボタンを押すまでは測定済みとして適用しない。
モデルとJavaScriptはローカル配信で、出典・ライセンスは `mount_assets` 内に同梱する。

同一機器・同一カメラ・同一解像度の測定済み `T_head_camera` がある場合、
次のXRセッションでは20フレーム以上の静止ボード観測から原点を再取得する。
保存済みの古い `T_world_board` は流用せず、ボードの位置・角度のばらつきを検査する。

## 保存した手動補正を過去の記録へ適用して再生する

記録と同じ `reference_id` のボード校正へ、調整画面で保存した左右の
`mounts` を組み合わせた校正JSONを、エピソード内へ別ファイルとして保存する。
現在のセッションの `board=null` や異なるreferenceの原点を、過去の記録に流用しない。
`export --calibration <校正JSON>` で `T_board_wrist` を書き出す。

校正済みの再生用変換は `ffw_teleop.human_demo_retarget` を使う。

```bash
PYTHONPATH="ffw_teleop:$HOME/ffw_sharpa_sim/src" \
  "$HOME/ffw_sharpa_sim/.venv/bin/python" -m ffw_teleop.human_demo_retarget \
  ~/human_demos/SESSION/human_arrays.npz \
  --calibration ~/human_demos/SESSION/calibration_manual_for_replay.json \
  --output ~/human_demos/SESSION/sim_trajectory_manual_mount.npz
./replay_human_mujoco.sh ~/human_demos/SESSION/sim_trajectory_manual_mount.npz --check
./replay_human_mujoco.sh ~/human_demos/SESSION/sim_trajectory_manual_mount.npz --loop --show-targets
```

この変換では、実測ボード座標をシミュレーションの紙へ対応付け、
左右の手動取り付け補正を位置・向きとも適用する。最初の手の姿勢への
再アンカー・左右間隔の拡縮は行わない。IKはキャッシュURDFではなく、
実際に再生するMuJoCoモデルのハンド本体を使って解く。

手の基準はSDK骨格原点に対応するロボットの手首取付面とし、
Sharpaの「+Z指方向・+X手のひら方向」とSDKの「−X指方向・+Y手の甲方向」を
固定の軸変換で対応付ける。手首取付面の位置はモデルの57mmオフセットを使う。
これは機械モデル上の対応づけで、人体の手首位置や把持中心の実測値ではない。
指角度変換の設定は再生用校正JSONの `glove_config` に保存する。
元の記録は変更せず、適用した補正・目標・IK誤差を別の軌道とJSONへ保存する。
接触を解く物理再生・衝突回避ではないため、机や自己干渉が残る場合がある。

## 実機の机高さ・頭部カメラのArUco基準

実機で天板からの高さを保つには、実機 `base_link` の座標で解く。
外部MuJoCoモデルに含まれるルートの+150mm配置、胴体Xの+25.4mm差、
手の取付位置差を実機の目標へ流用しない。`human_demo_robot.load_live_robot_scene`
は実機URDFのlift・頭・手の取付変換を適用し、プランナーはFK一致を検査する。
机の高さは700mm、紙上面は700.5mmとして設定した。liftは手動で決めて固定し、
手目標と協調させて自動昇降させない。

頭部ZEDの画像配信が起動している状態で、次を実行する。

```bash
./observe_robot_board.sh
```

このコマンドは検出だけを行い、モータを動かさない。配信が未起動の場合は
実機コンテナで `ros2 launch ffw_bringup camera_zed.launch.py camera_model:=zedm
publish_urdf:=false publish_tf:=false publish_map_tf:=false` を起動する。
既存の本体ノードを二重起動しない。

`robot_board` はrectified画像と対応するCameraInfo.P、画像時刻の100ms以内の
関節値、実機URDFの光学フレームを使い、`T_base_board` を求める。
既知マーカー3枚以上、20フレームの安定性、水平・表向き、700mm机との高さ整合を
確認して `/human_demo/board_pose` と `/tmp/human_demo_robot_board.json` を出力する。
未検出・古い画像・終了時はJSONを `valid:false` にする。
外部パラメータは公称URDFに依存し、実測カメラ取付校正の代わりではない。
JPEGプレビューは同名の `.jpg` に保存する。

`ffw_teleop/scripts/capture_robot_state.py` をROS環境内で実行すると、実機URDF、
関節値、ROS時刻、ボード観測をまとめたJSONスナップショットを取得できる。
PCと実機の壁時計が異なっていても、観測の鮮度は同じ実機ROS時計で判定する。

```bash
PYTHONPATH="ffw_teleop:$HOME/ffw_sharpa_sim/src" \
  "$HOME/ffw_sharpa_sim/.venv/bin/python" -m ffw_teleop.human_demo_robot_plan \
  ~/human_demos/SESSION/human_arrays.npz --snapshot robot_state.json \
  --output short_board_plan.json --seconds 0.6
```

このプランナーもモータ指令は送らない。`T_base_hand = T_base_board @ T_board_hand`
を左右共通に適用し、手同士の位置・向きと天板からの高さを保つ。
安定した最新観測、実機URDFとのFK一致、到達誤差、ゆっくりした開始姿勢への移動、
机・自己干渉を検査する。lift・頭・指は保持し、腕だけの短い試運転用JSONを作る。
机の寸法・衝突メッシュは近似であり、実際の周辺物体の安全を保証しない。

最初の17:23の記録は実測の収集時ボード座標がない。
その場合に `sim_trajectory_current_mount.npz` 等の目標を使うには、
`--assumed-source-board ffw_teleop/config/human_demo_workspace.json` を明示する。
この方法は仮の紙位置を使うため、正確な収集時ボード相対軌道とは区別する。
校正済み記録の `T_board_wrist` がある場合は仮配置を使わない。
実機への送信前には、実際の机・手・周辺物との間隔と開始経路を確認する。

## テレオペのための現在姿勢を保持して基準登録する

`./resume_teleop.sh --keep-current` は、通常の7番にある登録済み待機姿勢への
移動を省略し、現在の実測ロボット姿勢とVR姿勢を対応付けて追従を開始する。
通常の `./resume_teleop.sh` と左ペダルの開始は、`ffw_teleop/config/teleop_start_pose.json` に保存した待機姿勢へ移動する。
このオプション自体は机の回避機能ではない。

今回用意した待機姿勢は、lift約−0.201m、左右のハンド原点を前方350mm・
左右±280mm・高さ1050mmとし、手のひら下向き・指先前向きで指を軽く開く。
実機URDFを反映したモデルで、liftを100mm上げる経路と腕・指の移動経路を検証した。
姿勢と実測値は `~/.local/state/ai_worker/teleop-ready/` に保存する。
姿勢の準備だけでは追従は開始しない。7番と左ペダルは、待機姿勢への到着後にVR基準を登録して追従を開始する。
liftは自動移動せず、基準の約−0.201mから1cm以内か確認する。腕は最低5秒・最大0.1rad/sで移動する。
別の現在姿勢をそのまま基準にしたい場合だけ、明示的に `--keep-current` を使う。
