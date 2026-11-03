# Sharpa Wave hands on the AI Worker SG2 (ROS 2)

This branch (`devel/sharpa`) runs the FFW-SG2 with **Sharpa Wave hands instead of the grippers**
as one ros2_control robot.

- **One controller manager** drives the SG2 body (Dynamixel, grippers removed) and each hand
  (`ffw_sharpa_hardware/SharpaWaveSystem`, on the vendor's SharpaWave C++ SDK).
- **One `/joint_states`** carries the body and all finger joints at 100 Hz.
- **Trajectory controllers** command each part: the arms, each hand, the head and the lift.
- **Grippers:** the original gripper bringup (`ffw_sg2_follower_ai.launch.py`) is unchanged.

| | |
| --- | --- |
| Launch | `ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py` |
| Model | `ffw_description` SG2 xacro with `end_effector:=sharpa` (default: A v2 adapter, left hand 90°, right hand 270°) |
| Hand model | official `left_sharpa_wave` / `right_sharpa_wave` packages ([sharpa-urdf-usd-xml](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml), Apache-2.0) |
| Hand driver | `ffw_sharpa_hardware` (this package), SharpaWave SDK 5.0.11 |

Sharpa does not ship a ROS 2 driver for the current SDK. SDK 4.6.6 had a sample Python topic bridge
(`wave/<hand>/joint_states`, 10 Hz, outside ros2_control). This package replaces it with a
ros2_control system on the same SDK.

## Setup (once per robot)

1. **Hand network** (on the robot host, not in a container; needs sudo). Each hand is on its own
   USB 3 Ethernet adapter. The script gives the host 192.168.10.240/24 on them and bridges two
   adapters. It never touches the default route.

   ```bash
   bash ffw_sharpa_hardware/scripts/hand_network.sh           # dry run
   bash ffw_sharpa_hardware/scripts/hand_network.sh --apply
   ```

2. **SDK and hand descriptions.** The SDK is proprietary and not stored in this repository. The
   script downloads the pinned release and checks its SHA-256. It also fetches the two official
   description packages (sparse checkout). Both go into the git-ignored `third_party/`.

   ```bash
   bash ffw_sharpa_hardware/scripts/setup_sharpa.sh
   ```

3. **Build** inside the `ai_worker` container (`~/ai_worker` is mounted at
   `/root/ros2_ws/src/ai_worker`):

   ```bash
   cd /root/ros2_ws
   colcon build --symlink-install --packages-select \
     ffw_description ffw_bringup ffw_sharpa_hardware left_sharpa_wave right_sharpa_wave
   source install/setup.bash
   ```

   Without the SDK, `ffw_sharpa_hardware` builds no plugin (CMake warns). Mock hardware still works.

## Bring-up

Stop any other program that uses the follower bus or the hands first. That includes the gripper
bringup or ROBOTIS service, the evarl Python stack, and Sharpa Pilot.

```bash
ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py                 # both hands
ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py hands:=left     # one hand mounted
ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py hands:=none     # no gripper, no hand
ros2 launch ffw_bringup ffw_sg2_sharpa_follower_ai.launch.py use_mock_hardware:=true  # no robot
```

With `init_position:=true` (default), start-up moves the arms, head, lift and steering to zero and
opens the hands. Use `init_position:=false` to start without motion.

**Hand identities stay out of the repository.** Pass them as arguments or environment variables.
Without them, the factory IP of each side is used: right 192.168.10.20, left 192.168.10.10. A serial
number, when given, takes precedence over the IP.

| Argument | Environment default | Meaning |
| --- | --- | --- |
| `hands` | | `both` (default), `left`, `right`, `none` |
| `left_hand_ip` / `right_hand_ip` | `SHARPA_LEFT_IP` / `SHARPA_RIGHT_IP` | hand address |
| `left_hand_serial` / `right_hand_serial` | `SHARPA_LEFT_SERIAL` / `SHARPA_RIGHT_SERIAL` | hand serial number |
| `hand_speed_coeff` | `SHARPA_SPEED_COEFF` (0.3) | SDK joint speed coefficient, (0, 0.5] |
| `sharpa_adapter` | | `a_v2` (only; arms keep 7 joints) |
| `left_hand_mount_deg` / `right_hand_mount_deg` | | hand rotation about the wrist axis, 90 / 270 |
| `init_position`, `launch_cameras`, `launch_lidar`, `start_rviz` | | as in the SG2 launch |

To view the model only:

```bash
ros2 launch ffw_description ffw_sg2_rev1.launch.py end_effector:=sharpa      # or none / gripper
```

## Joints, controllers, topics

| Controller | Joints |
| --- | --- |
| `arm_l_controller`, `arm_r_controller` | `arm_{l,r}_joint1..7` (no gripper joint) |
| `hand_l_controller`, `hand_r_controller` | `{left,right}_{thumb_CMC_FE, thumb_CMC_AA, thumb_MCP_FE, thumb_MCP_AA, thumb_IP, index_MCP_FE, index_MCP_AA, index_PIP, index_DIP, middle_…, ring_…, pinky_CMC, pinky_MCP_FE, pinky_MCP_AA, pinky_PIP, pinky_DIP}` (22 each, SDK order) |
| `head_controller`, `lift_controller`, `swerve_drive_controller` | as on the SG2 |

All controllers are `joint_trajectory_controller`s with partial-joint goals allowed. Use either
interface:

- **Action:** `/<controller>/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`).
- **Topic:** `/<controller>/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`). It also works
  for streaming single points from a policy or teleop.

The head and lift follow the leader joysticks as on the SG2. The leader's arm topics carry gripper
joints, so the arm controllers keep their own topics. `cyclo_teleoperation` is not started; its
configuration is gripper-based.

```bash
# Curl the right index finger over 2 s
ros2 action send_goal /hand_r_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [right_index_MCP_FE, right_index_PIP, right_index_DIP],
    points: [{positions: [0.6, 0.6, 0.4], time_from_start: {sec: 2}}]}}"
```

- **`/joint_states`:** 67 joints (20 body, 3 wheel drives, 2 × 22 fingers). Finger position is in
  rad, velocity in rad/s (checked against differentiated position), and effort is the SDK torque.
- **`/diagnostics`:** one status per hand, `sharpa_wave: <side> hand`. It reports the device
  status code with the fault name and joints, whether the motors are enabled, the device feedback
  rate (500 Hz) and the speed coefficient.

## Faults and recovery

`read()` and `write()` never return an error, because that would also stop the
`joint_state_broadcaster` for the whole robot. Instead, the hand **holds**: no new targets are sent,
so the hand stays at its last target. This happens when:

- the hand reports a device fault (any status other than 0x0000 / 0x0030),
- its feedback stops for 0.5 s (positions then read NaN), or
- its motors are disabled from outside ROS.

The log and `/diagnostics` say which. Activation refuses a hand that reports a fault. After fixing
the cause (e.g. in Sharpa Pilot), restart only that hand:

```bash
bash ffw_sharpa_hardware/scripts/restart_hand.sh right
```

The script stops the hand controller, cycles the hardware component (disable, then enable holding
the measured pose) and starts the controller again from the measured pose. Stopping the launch
disables both hands: the fingers go limp and a held object drops.

## Notes

- **Mass:** the adapter is a massless box (88 × 68 × 16.6 mm). Each hand with wrist is about 1.3 kg,
  against about 0.5 kg for a gripper.
- **Speed:** the speed coefficient limits finger speed inside the hand. Trajectories faster than
  that lag behind.
- **Lock-contention log:** with asynchronous hardware components, ros2_control 4.x sometimes logs
  `Failed to set value for joint ... in command interface` when a controller's write meets the
  component's thread. The SG2 body does the same. The value is applied on the next cycle.
- **Not covered:** Gazebo, MoveIt and the B adapter (6-joint arms).
- **Measured on the robot:** 100 Hz control loop; 500 Hz hand feedback; hand `read` 35–40 µs on
  average (up to 7 ms during the once-a-second enable query, hence `is_async`), `write` 7 µs.
  In trajectory tests, arms and hands moved together and reached targets within 0.004 rad (fingers)
  and 0.001 rad (elbows).
