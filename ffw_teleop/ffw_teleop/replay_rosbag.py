"""Play a recording in an isolated RViz domain, or explicitly replay joints."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml
from .recording_model import replay_model


def display_topics(selected):
    return sorted({'/joint_states', '/tf', '/tf_static'} | {
        topic for group, topic in selected.items()
        if group.endswith(('/rgb', '/depth', '/rgb_info', '/depth_info'))})


def read_frames(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/joint_states']))
    frames = []
    previous = -1
    while reader.has_next():
        _, data, stamp = reader.read_next()
        msg = deserialize_message(data, JointState)
        if stamp <= previous:
            continue
        values = dict(zip(msg.name, msg.position))
        if any(not math.isfinite(v) for v in values.values()):
            raise ValueError('関節角度に非有限値があります')
        frames.append((stamp, values))
        previous = stamp
    if len(frames) < 2:
        raise ValueError('再生に必要なjoint_statesがありません')
    return frames


def hardware_replay(session, bag, rate, domain):
    """Only joint groups present in the recording; never replay base commands."""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    frames = read_frames(bag)
    robot = ET.parse(session/'model/robot.urdf').getroot()
    limits = {j.get('name'): (float(j.find('limit').get('lower')),
                              float(j.find('limit').get('upper')))
              for j in robot.findall('joint') if j.get('type') in ('revolute', 'prismatic')}
    # Discover controller joint lists from the recorded controller configuration
    # through the live controller manager rather than assuming arm/hand ordering.
    from controller_manager_msgs.srv import ListControllers
    rclpy.init(args=[], domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('recorded_joint_replay')
    stopping = [False]
    old = {s: signal.signal(s, lambda *_: stopping.__setitem__(0, True))
           for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    goals = []
    futures = []
    feedback = [{}, 0.]
    node.create_subscription(JointState, '/joint_states',
        lambda m: feedback.__setitem__(slice(None), [dict(zip(m.name, m.position)), time.monotonic()]),
        qos_profile_sensor_data)
    def wait(future, timeout=5.):
        end = time.monotonic()+timeout
        while not future.done() and not stopping[0] and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done() or stopping[0]:
            raise RuntimeError('再生の応答待ちを中断しました')
        return future.result()
    try:
        service = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not service.wait_for_service(timeout_sec=3.):
            raise RuntimeError('controller_managerに接続できません')
        controllers = wait(service.call_async(ListControllers.Request())).controller
        end = time.monotonic()+3.
        while time.monotonic()-feedback[1] > .5 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if time.monotonic()-feedback[1] > .5:
            raise RuntimeError('実機の関節角度を受信できません')
        groups = []
        allowed = {'arm_l_controller', 'arm_r_controller', 'hand_l_controller',
                   'hand_r_controller', 'head_controller', 'lift_controller'}
        for controller in controllers:
            if controller.name not in allowed:
                continue
            names = list(dict.fromkeys(i.rsplit('/', 1)[0] for i in controller.claimed_interfaces))
            if not names or not any(n in frames[0][1] for n in names):
                continue
            if controller.state != 'active' or any(n not in feedback[0] for n in names):
                raise RuntimeError(controller.name+'が利用できません')
            topic = '/'+controller.name+'/joint_trajectory'
            if node.count_publishers(topic):
                raise RuntimeError(topic+'に操作ノードが存在します。VR・グローブ・昇降操作を停止してください')
            for _, values in frames:
                for name in names:
                    if name not in values or name not in limits:
                        raise ValueError('関節データまたはURDF制限不足: '+name)
                    if not limits[name][0]-.0001 <= values[name] <= limits[name][1]+.0001:
                        raise ValueError('URDF範囲外: '+name)
            client = ActionClient(node, FollowJointTrajectory, '/'+controller.name+'/follow_joint_trajectory')
            if not client.wait_for_server(timeout_sec=3.):
                raise RuntimeError(controller.name+'の軌跡Actionがありません')
            groups.append((names, client))
        if not groups:
            raise RuntimeError('再生できる関節コントローラがありません')
        def duration(seconds):
            ns = int(seconds*1e9)
            return Duration(sec=ns//10**9, nanosec=ns%10**9)
        # Shared future stamp synchronizes controllers; 5 seconds to first pose.
        stamp = node.get_clock().now().nanoseconds+2_000_000_000
        futures = []
        for names, client in groups:
            trajectory = JointTrajectory(joint_names=names)
            trajectory.header.stamp.sec = stamp//10**9
            trajectory.header.stamp.nanosec = stamp%10**9
            trajectory.points.append(JointTrajectoryPoint(positions=[feedback[0][n] for n in names],
                time_from_start=duration(.1)))
            for t, values in frames:
                trajectory.points.append(JointTrajectoryPoint(positions=[values[n] for n in names],
                    time_from_start=duration(5.+(t-frames[0][0])/1e9/rate)))
            futures.append(client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory)))
        for future in futures:
            handle = wait(future)
            goals.append(handle)
            if not handle.accepted:
                raise RuntimeError('軌跡が拒否されました')
        results = [g.get_result_async() for g in goals]
        print('実機再生中。Ctrl+Cで全関節の再生をキャンセルします。', flush=True)
        while not all(f.done() for f in results) and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic()-feedback[1] > 1.:
                raise RuntimeError('実機フィードバックが途絶えました')
            if any(f.done() and f.result().result.error_code != 0 for f in results):
                raise RuntimeError('コントローラが軌跡を中断しました')
        if any(f.done() and f.result().result.error_code != 0 for f in results):
            raise RuntimeError('軌跡の実行に失敗しました')
    finally:
        # A server may accept another group's goal while a rejection/stop is
        # being handled. Include those accepted goals in cancellation as well.
        acceptance_deadline = time.monotonic()+2.
        while futures and not all(f.done() for f in futures) and time.monotonic() < acceptance_deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        for future in futures:
            if future.done() and future.result() is not None:
                handle = future.result()
                if handle not in goals:
                    goals.append(handle)
        cancellations = [g.cancel_goal_async() for g in goals if g.accepted]
        deadline = time.monotonic()+2.
        while cancellations and not all(f.done() for f in cancellations) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.try_shutdown()
        for s, handler in old.items():
            signal.signal(s, handler)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, help='session.yamlのある記録フォルダ')
    parser.add_argument('--real', action='store_true', help='明示的に実機の関節を動かす')
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--domain', type=int, default=None)
    options = parser.parse_args(args)
    if not math.isfinite(options.rate) or options.rate <= 0:
        parser.error('rateは正の有限値です')
    session = options.session.expanduser().resolve()
    manifest = yaml.safe_load((session/'session.yaml').read_text())
    if not manifest.get('complete'):
        parser.error('記録が正常に完了していません')
    bag = session/'rosbag'
    if options.real:
        hardware_replay(session, bag, options.rate, options.domain if options.domain is not None else 0)
        return 0
    domain = options.domain if options.domain is not None else 94
    if domain in (0, 30):
        parser.error('RViz再生は実機と別ドメインを指定してください（既定94）')
    model = replay_model(session/'model')
    robot = ET.parse(model).getroot()
    children = {j.find('child').get('link') for j in robot.findall('joint')}
    root = next(l.get('name') for l in robot.findall('link') if l.get('name') not in children)
    displays = [{'Class': 'rviz_default_plugins/RobotModel', 'Name': 'Robot', 'Enabled': True,
                 'Description Topic': {'Value': '/robot_description', 'Durability Policy': 'Transient Local'}}]
    for group, topic in manifest['selected'].items():
        if group.endswith(('/rgb', '/depth')):
            displays.append({'Class': 'rviz_default_plugins/Image', 'Name': group, 'Enabled': True,
                             'Topic': {'Value': topic, 'Reliability Policy': 'Best Effort'}})
    env = {**os.environ, 'ROS_DOMAIN_ID': str(domain)}
    processes = []
    try:
        with tempfile.TemporaryDirectory(prefix='bag-rviz-') as tmp:
            config = Path(tmp)/'replay.rviz'
            config.write_text(yaml.safe_dump({'Visualization Manager': {
                'Global Options': {'Fixed Frame': root}, 'Displays': displays}}))
            processes.append(subprocess.Popen(['ros2', 'run', 'robot_state_publisher',
                'robot_state_publisher', str(model), '--ros-args', '-p', 'use_sim_time:=true'], env=env))
            processes.append(subprocess.Popen(['rviz2', '-d', str(config), '--ros-args',
                '-p', 'use_sim_time:=true'], env=env))
            time.sleep(2.)
            processes.append(subprocess.Popen(['ros2', 'bag', 'play', str(bag), '--clock',
                '--rate', str(options.rate), '--topics', *display_topics(manifest['selected'])], env=env))
            processes[-1].wait()
            print('再生終了。RVizを閉じるかCtrl+Cで終了。', flush=True)
            processes[1].wait()
    except KeyboardInterrupt:
        pass
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    process.terminate()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
