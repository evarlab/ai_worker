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

Questのページで「カメラを選ぶ」→背面／外向きカメラを選択→「選択したカメラを使う」→「Enter XR」。映像が実際の机と手になっていることを確認する。avatar/front/selfieは使用しない。両コントローラは手の甲に固定したままでよく、グリップボタンは不要。

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

準備後、別ターミナルで実行する。開始時は新しい両手XR、左右グローブ、Quest画像が必要。

```bash
./record_human_demo.sh status
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

キャッシュ済みの実機と同じSharpa付きURDFを使用する。初回の頭の水平方向からWebXR→ロボット（X前・Y左・Z上）を定義し、最初の両手をURDFの腕を下げた姿勢へ合わせる。移動量は等倍、回転は開始からの相対回転。腕はURDF関節限界を守るオフラインIK、指は既存の手元校正設定を適用してURDF限界内に収める。設定とIK誤差はsim_trajectory.jsonに保存する。両手・指が有効なサンプルだけ約10Hzで変換し、欠落区間は最後のモデル姿勢を保持する。

再生はlocalhost限定のROS_DOMAIN_ID=97でjoint_statesとrobot_descriptionだけを使用する。実機ドライバは起動しない。これはRVizでの運動学モデル再生で、物理・接触・衝突の検証や学習済みpolicyの配備ではない。作業基準と手の取り付け変換が未校正なら、到達誤差を確認するための暫定対応づけとなる。終了は起動ターミナルでCtrl+C。

### 既存MuJoCo環境への読み込み

`~/ffw_sharpa_sim` のモデルを読み込む再生も可能。既存側のソースは変更しない。

```bash
./replay_human_mujoco.sh ~/human_demos/SESSION/sim_trajectory.npz --check
./replay_human_mujoco.sh ~/human_demos/SESSION/sim_trajectory.npz --loop
```

環境パスを変更する場合は `AI_WORKER_MUJOCO_ROOT=/path/to/ffw_sharpa_sim` を指定する。腕14＋指44関節を名前で照合し、MuJoCo側の関節限界も適用する。`--check` は全フレームをheadlessで評価し、追加クランプ数・モデル内接触・最大食い込みを `sim_trajectory.mujoco.json` に保存する。ビューアは `mj_forward` による運動学再生であり、物理演算で接触を解く再生ではない。干渉があればそのまま表示されるので、把持成功や物理的実行可能性を示すものではない。既定の机・キューブは記録した人間タスクの物体位置ではない。
