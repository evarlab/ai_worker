# ROBOTIS AI Worker ROS 2 Packages

This repository contains the official ROS 2 packages for the ROBOTIS AI Worker platform. These packages provide the necessary interfaces and tools to control the robot, integrate with its sensors, and develop physical AI applications, including those utilizing frameworks like LeRobot. For detailed usage instructions, please refer to the documentation below.
  - [Documentation for AI Worker](https://ai.robotis.com/)

To learn more about the Physical AI Tools, visit:
  - [Physical AI Tools](https://github.com/ROBOTIS-GIT/physical_ai_tools)

To explore our open-source platforms in a simulation environment, visit:
  - [Simulation Models](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)

For usage instructions and demonstrations of the AI Worker, check out:
  - [Tutorial Videos](https://www.youtube.com/@ROBOTISOpenSourceTeam)

To access datasets and pre-trained models for our open-source platforms, see:
  - [AI Models & Datasets](https://huggingface.co/ROBOTIS)

To use the Docker image for running ROS packages and Physical AI tools with the AI Worker, visit:
  - [Docker Images](https://hub.docker.com/r/robotis/ros/tags)

## Sharpa Wave hands (branch `devel/sharpa`)

ContactGlove 3 Proで操作するブランチは`devel/contactgrove_sharpa`です。
導入・mock確認・実機操作は[日本語手順](ffw_teleop/CONTACTGLOVE_SHARPA.md)を参照してください。
Quest 3で腕と指を同時に操作する保存ブランチは`devel/vr_contactgrove_sharpa_20261116`です。
[普段の操作手順](ffw_teleop/OPERATION_SHARPA_VR.md)と
[VR＋Sharpaの導入・詳細](ffw_teleop/VR_SHARPA.md)を参照してください。
`./teleop_terminal.sh`でTerminatorを9分割し、各起動コマンドをEnter待ちで並べられます。9番がフットペダルです。
校正は`./calibrate_vr.sh`、URDF確認は`./preview_vr.sh`、実機の開始・再開は`./resume_teleop.sh`です。
開始・再開時は、実機の両腕をURDF初期姿勢へ移動してからVRの基準を登録します。
腕と指の追従を一括停止する場合は、PCで`./stop_teleop.sh`を実行します。
昇降の独立操作、RGB-D・関節・電流のrosbag記録、RVizと実機での再生は
[記録・再生手順](RECORDING_REPLAY.md)を参照してください。
3連フットペダルは`./foot_pedal.sh`で有効になり、左＝開始／停止、中央＝記録、右＝初期姿勢への低速移動です。
人間がContactGloveとQuestを装着して行うタスクの収集は
[人間実演データの収集と展開方針](HUMAN_DEMONSTRATIONS.md)を参照してください。`./record_human_demo.sh`で実機と独立して収集します。
Quest ProのUSB・ブラウザ・コントローラ追跡は`./check_quest.sh`で実機へ指令を出さず確認できます。

The SG2 follower can run with Sharpa Wave hands instead of the grippers, or with bare flanges, as one
ros2_control robot: `ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py`. Setup, launch
arguments, controllers and fault handling: [ffw_sharpa_hardware/README.md](ffw_sharpa_hardware/README.md).
