"""Record RGB-D, joint trajectories and raw motor currents; never commands motors."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rosidl_runtime_py.utilities import get_message
import yaml

from .recording_model import bundle_model


def resolve_topics(config, available):
    selected, missing = {}, []
    groups = dict(config.get('required', {}))
    for camera, streams in config.get('cameras', {}).items():
        for stream, candidates in streams.items():
            groups[f'{camera}/{stream}'] = candidates
    for group, candidates in groups.items():
        chosen = next((topic for topic in candidates if topic in available), None)
        if chosen:
            selected[group] = chosen
        else:
            missing.append(group)
    topics = sorted(set(selected.values()) | set(config.get('extra_topics', [])))
    return selected, missing, topics


def current_interfaces(msg):
    currents = {}
    for name, values in zip(msg.joint_names, msg.interface_values):
        for interface, value in zip(values.interface_names, values.values):
            if interface == 'Present Current' and math.isfinite(value):
                currents[name] = float(value)
    return currents


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    default_config = Path(__file__).resolve().parents[1]/'config/rosbag_rgbd.yaml'
    if not default_config.exists():
        from ament_index_python.packages import get_package_share_directory
        default_config = Path(get_package_share_directory('ffw_teleop'))/'config/rosbag_rgbd.yaml'
    parser.add_argument('--config', type=Path, default=default_config)
    parser.add_argument('--output-root', type=Path, default=Path('/workspace/contactgrove_sharpa/recordings'))
    parser.add_argument('--domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--wait', type=float, default=20.)
    parser.add_argument('--allow-missing', action='store_true', help='不足トピックをmanifestに記載して記録する')
    parser.add_argument('--stop', action='store_true', help='実行中の記録を保存して終了する')
    parser.add_argument('--toggle', action='store_true', help='記録中なら保存終了、停止中なら開始')
    options = parser.parse_args(args)
    from .recording_control import RecorderLock, stop_recording, recording_active
    if options.stop or (options.toggle and recording_active(options.domain)):
        return stop_recording(options.domain)
    if not math.isfinite(options.wait) or options.wait < 0:
        parser.error('waitは0以上の有限値です')
    config = yaml.safe_load(options.config.read_text())
    rclpy.init(args=[], domain_id=options.domain, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('rosbag_rgbd_inventory')
    stopping = [False]
    previous = {sig: signal.signal(sig, lambda *_: stopping.__setitem__(0, True))
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    description, currents, current_sub = [None], [{}], None
    image_samples, image_probes = {}, {}
    node.create_subscription(String, '/robot_description', lambda msg: description.__setitem__(0, msg.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    recorder = None
    session = None
    manifest = None
    control = None
    code = 0
    try:
        control = RecorderLock(options.domain)
        deadline = time.monotonic()+options.wait
        selected, missing, topics = {}, [], []
        while not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.1)
            available = dict(node.get_topic_names_and_types())
            selected, missing, topics = resolve_topics(config, available)
            for group, topic in selected.items():
                if group.endswith(('/rgb', '/depth')) and topic not in image_samples and topic not in image_probes:
                    def image(msg, t=topic):
                        image_samples[t] = dict(encoding=msg.encoding, width=msg.width, height=msg.height,
                                                frame_id=msg.header.frame_id)
                    image_probes[topic] = node.create_subscription(Image, topic, image, qos_profile_sensor_data)
            for topic in list(image_probes):
                if topic in image_samples:
                    node.destroy_subscription(image_probes.pop(topic))
            current_topic = selected.get('motor_interfaces')
            if current_topic and current_sub is None:
                message_type = get_message(available[current_topic][0])
                current_sub = node.create_subscription(message_type, current_topic,
                    lambda msg: currents.__setitem__(0, current_interfaces(msg)), qos_profile_sensor_data)
            image_topics = [topic for group, topic in selected.items() if group.endswith(('/rgb', '/depth'))]
            complete = not missing and description[0] and currents[0] and all(t in image_samples for t in image_topics)
            if complete or time.monotonic() >= deadline:
                break
        if stopping[0]:
            return 130
        if not description[0]:
            missing.append('robot_description sample')
        if not currents[0]:
            missing.append('Present Current values')
        for group, topic in selected.items():
            if group.endswith(('/rgb', '/depth')):
                sample = image_samples.get(topic)
                if sample is None:
                    missing.append(group+' image sample')
                elif group.endswith('/depth') and sample['encoding'] not in ('16UC1', '32FC1'):
                    missing.append(group+' metric depth (encoding='+sample['encoding']+')')
        if missing and not options.allow_missing:
            raise RuntimeError('記録に必要な配信が不足しています: '+', '.join(missing)
                +'。カメラ・本体を起動するか、明示的に --allow-missing を指定してください')
        session = options.output_root.expanduser()/f'{datetime.now():%Y%m%d-%H%M%S}-{time.time_ns()}'
        session.mkdir(parents=True)
        if description[0]:
            from ament_index_python.packages import get_package_share_directory
            (session/'robot_description.original.urdf').write_text(description[0])
            bundle_model(description[0], session/'model', get_package_share_directory)
        manifest = dict(started_at=datetime.now().astimezone().isoformat(), complete=False,
            domain=options.domain, selected=selected, missing=missing, topics=topics,
            image_streams=image_samples,
            current_source=dict(topic=selected.get('motor_interfaces'), interface='Present Current',
                unit='driver-native values; not assumed to be amperes', names=sorted(currents[0])),
            note='JointState effort may be current/load for Dynamixel and torque for Sharpa; raw interfaces are preserved.')
        (session/'session.yaml').write_text(yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False))
        print('保存先: '+str(session), flush=True)
        print(json.dumps(selected, ensure_ascii=False, indent=2), flush=True)
        command = ['ros2', 'bag', 'record', '-s', 'mcap', '--storage-preset-profile', 'zstd_fast',
            '--max-cache-size', '256000000', '--max-bag-size', '2000000000', '--disable-keyboard-controls',
            '--include-unpublished-topics', '-o', str(session/'rosbag'), '--topics', *topics]
        recorder = subprocess.Popen(command, env={**os.environ, 'ROS_DOMAIN_ID': str(options.domain)},
                                    stdin=subprocess.DEVNULL, start_new_session=True)
        print('記録中。終了はCtrl+C。昇降操作は別のターミナルで起動してください。', flush=True)
        while not stopping[0] and recorder.poll() is None:
            rclpy.spin_once(node, timeout_sec=.1)
    except (RuntimeError, OSError, ValueError) as error:
        print(str(error), flush=True)
        code = 1
    finally:
        if recorder is not None:
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
                try:
                    recorder.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    recorder.terminate()
                    recorder.wait(timeout=5)
            code = code or (0 if recorder.returncode in (0, -signal.SIGINT) else 1)
            if manifest is not None:
                manifest['complete'] = code == 0 and (session/'rosbag/metadata.yaml').exists()
                manifest['finished_at'] = datetime.now().astimezone().isoformat()
                (session/'session.yaml').write_text(yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False))
                print('記録終了: '+str(session), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if control is not None:
            control.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
