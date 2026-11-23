"""One bounded recovery on an operator's explicit start; never bypass device faults."""
import math


def recovery_action(status, diagnostic_age, joint_age, controller_active=True):
    if not math.isfinite(diagnostic_age) or not 0 <= diagnostic_age <= 2.5:
        raise ValueError('ハンド診断が古いか、取得できません')
    if not math.isfinite(joint_age) or not 0 <= joint_age <= .5:
        raise ValueError('ハンドの実測値が古いか、取得できません')
    if not controller_active:
        raise ValueError('ハンドコントローラがactiveではありません')
    values = status['values']
    if values.get('ros2_control_active') != 'true':
        raise ValueError('ハンド本体がinactiveです。自動では有効化しません')
    try:
        code = int(values['device_status'], 0)
        rate = float(values['device_feedback_hz'])
    except (KeyError, ValueError, TypeError) as error:
        raise ValueError('機器のエラー状態・実測更新を確認できません') from error
    if code not in (0x0000, 0x0030):
        raise ValueError('機器エラーが現在も残っています: '+values['device_status'])
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('ハンドの実測値が更新されていません')
    message = status['message']
    if status['level'] == 0 and message == 'enabled' and values.get('motors_enabled') == 'true':
        return 'ready'
    recoverable = ('holding: ERROR_TORQUE_LIMIT_EXCEEDED',
                   'holding: ERROR_JOINT_LIMIT_EXCEEDED', 'holding: joint feedback lost')
    if status['level'] == 1 and message.startswith(recoverable):
        return 'recover'
    raise ValueError('自動復旧対象ではないハンド状態: '+message)


def recover_once(side, command):
    if side not in ('left', 'right'):
        raise ValueError('Invalid hand side')
    controller = 'hand_'+('l' if side == 'left' else 'r')+'_controller'
    component = 'sharpa_'+side+'_hand'
    # Stop stale controller setpoints first. Hardware activation refuses a
    # current device fault and initializes position mode from measured joints.
    for args in (['switch_controllers', '--deactivate', controller],
                 ['set_hardware_component_state', component, 'inactive'],
                 ['set_hardware_component_state', component, 'active'],
                 ['switch_controllers', '--activate', controller]):
        command(['ros2', 'control', *args])


def main():
    import argparse
    import os
    import signal
    import subprocess
    import time
    import json
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import JointState
    from controller_manager_msgs.srv import ListControllers
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tag', nargs='?', default='ai-worker-hand-recovery')
    parser.add_argument('--recover', action='store_true', help='明示的な開始操作でのみ、一度だけ復旧する')
    options = parser.parse_args()
    if options.tag != 'ai-worker-hand-recovery':parser.error('Invalid process tag')
    # The stop command identifies this process by its exact marker. It never
    # restarts a failed operation in the background.
    try:os.setsid()
    except PermissionError:pass
    stopped = [False]
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    deadline = time.monotonic()+75.
    def check_stop():
        if stopped[0]:raise RuntimeError('ハンド復旧を中止しました')
        if time.monotonic() > deadline:raise RuntimeError('ハンド復旧の制限時間を超えました。再試行しません')
    rclpy.init(domain_id=0, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('check_hands_before_teleop')
    diagnostics, feedback = {}, {}
    def on_diagnostics(msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        for status in msg.status:
            for side in ('left', 'right'):
                if status.name == f'sharpa_wave: {side} hand':
                    level = int.from_bytes(status.level,'little') if isinstance(status.level,bytes) else int(status.level)
                    diagnostics[side] = dict(level=level,message=status.message,
                        values={v.key:v.value for v in status.values}, received=now, stamp=stamp)
    def on_joints(msg):
        now = time.monotonic()
        for name, value in zip(msg.name,msg.position):
            if name.startswith(('left_', 'right_')) and 'wheel' not in name and math.isfinite(value):
                feedback[name] = now
    node.create_subscription(DiagnosticArray,'/diagnostics',on_diagnostics,qos_profile_sensor_data)
    node.create_subscription(JointState,'/joint_states',on_joints,qos_profile_sensor_data)
    service = node.create_client(ListControllers,'/controller_manager/list_controllers')
    def spin_until(predicate, seconds):
        end = time.monotonic()+seconds
        while not predicate() and time.monotonic() < end:
            check_stop();rclpy.spin_once(node,timeout_sec=.05)
        check_stop()
        if not predicate():raise RuntimeError('ハンドの診断・実測値・応答を確認できません')
    def command(args):
        check_stop();print(' '.join(args),flush=True)
        process = subprocess.Popen(args, env={**os.environ, 'ROS_DOMAIN_ID': '0'})
        end = min(deadline,time.monotonic()+12.)
        try:
            while process.poll() is None:
                check_stop()
                if time.monotonic()>end:raise RuntimeError('復旧コマンドの応答がありません。再試行しません')
                rclpy.spin_once(node,timeout_sec=.05)
            if process.returncode:raise RuntimeError('復旧コマンドに失敗しました。再試行しません')
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=2)
                except subprocess.TimeoutExpired:process.kill();process.wait()
    try:
        spin_until(lambda:service.service_is_ready(),8)
        future = service.call_async(ListControllers.Request())
        spin_until(future.done,5)
        controllers = {c.name:c for c in future.result().controller}
        names = {}
        for side,short in (('left','l'),('right','r')):
            controller = controllers.get('hand_'+short+'_controller')
            if controller is None or controller.state != 'active':
                raise RuntimeError(side+' hand controller is not active')
            names[side] = list(dict.fromkeys(i.rsplit('/',1)[0] for i in controller.claimed_interfaces))
            if len(names[side]) != 22:raise RuntimeError('Expected 22 hand joints: '+side)
        spin_until(lambda:len(diagnostics)==2 and all(all(n in feedback for n in names[s]) for s in names),8)
        actions = {}
        for side in names:
            d = diagnostics[side];now=time.monotonic()
            actions[side] = recovery_action(d,now-d['received'],max(now-feedback[n] for n in names[side]))
        print('ハンド確認: '+json.dumps(actions,ensure_ascii=False),flush=True)
        if not options.recover:return 0
        # Classify BOTH hands before changing either component.
        for side,action in actions.items():
            check_stop()
            if action == 'ready':continue
            print(side+' hand: 機器の現在エラーなし。過去の停止状態を一度だけ復旧します。',flush=True)
            recover_once(side,command)
            after = node.get_clock().now().nanoseconds
            spin_until(lambda:diagnostics.get(side,{}).get('stamp',0)>after,6)
            d=diagnostics[side];now=time.monotonic()
            if recovery_action(d,now-d['received'],max(now-feedback[n] for n in names[side])) != 'ready':
                raise RuntimeError(side+' handは復旧後も停止中です。再試行しません')
        print('左右ハンドの開始準備を確認しました。追従はまだ開始していません。',flush=True)
        return 0
    except (ValueError,RuntimeError,KeyError) as error:
        print('ハンドの開始を中止: '+str(error),flush=True);return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
