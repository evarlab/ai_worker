"""Start only the physical-domain gate, independently of model previews."""
import fcntl
import os
from pathlib import Path
import subprocess
import time


def process_ids(fragment, domain=0, proc=Path('/proc')):
    found = []
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            env = dict(item.split(b'=', 1) for item in (entry/'environ').read_bytes().split(b'\0') if b'=' in item)
            if fragment in command and int(env.get(b'ROS_DOMAIN_ID', b'0')) == domain:
                found.append(int(entry.name))
        except (OSError, ValueError):
            continue
    return sorted(found)


def main():
    with open('/tmp/sharpa-vr-gate-start.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        # A stopped command gate is restartable; inactive motor controllers are
        # a separate hardware condition and must not be reported as resumed.
        import rclpy
        from controller_manager_msgs.srv import ListControllers
        rclpy.init(domain_id=0)
        node = rclpy.create_node('check_live_controllers')
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        try:
            if not client.wait_for_service(timeout_sec=8):
                raise SystemExit('2番の実機本体が起動していません。先に2番を起動してください。')
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done():
                raise SystemExit('実機本体の応答がありません。通信状態を確認してください。')
            states = {controller.name: controller.state for controller in future.result().controller}
            inactive = [name for name in ('arm_l_controller', 'arm_r_controller', 'joint_state_broadcaster')
                        if states.get(name) != 'active']
            if inactive:
                raise SystemExit('実機本体の制御がinactiveです。電源・非常停止・通信を確認し、2番を再起動してください: '
                                 + ', '.join(inactive))
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
        if process_ids('/lib/ffw_teleop/sharpa_vr_bridge'):
            print('実機用の腕ブリッジを再利用します。', flush=True)
            return
        if not process_ids('/vr_controller_node'):
            raise SystemExit('5番の腕IKが実機用ドメインで起動していません。先に5番を起動してください。')
        logs = Path('~/.local/state/ai_worker/terminal').expanduser()
        logs.mkdir(parents=True, exist_ok=True)
        path = logs/f'vr-gate-{time.time_ns()}.log'
        with path.open('ab') as output:
            subprocess.Popen(['ros2', 'run', 'ffw_teleop', 'sharpa_vr_bridge', '--ros-args',
                '--params-file', '/workspace/contactgrove_sharpa/install/ffw_teleop/share/ffw_teleop/config/sharpa_vr.yaml',
                '-p', 'dry_run:=false'], stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True, env={**os.environ, 'ROS_DOMAIN_ID': '0'})
        print(f'実機用の腕ブリッジを再起動しました。ログ: {path}', flush=True)


if __name__ == '__main__':
    main()
