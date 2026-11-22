"""Three-pedal controls; USB events are exclusive and independent of window focus."""
import argparse
from datetime import datetime
import fcntl
import os
from pathlib import Path
import select
import signal
import subprocess
import time


DEFAULT_KEYS = {30: 'teleop', 48: 'record', 46: 'home'}  # observed left A, middle B, right C


class PedalEdges:
    def __init__(self, mapping=None):
        self.mapping = dict(DEFAULT_KEYS if mapping is None else mapping)
        self.held = set()
        self.last = {}

    def feed(self, device, code, value, now=None):
        key = (device, code)
        now = time.monotonic() if now is None else now
        if value == 0:
            self.held.discard(key)
            return None
        if value != 1 or key in self.held:
            return None
        self.held.add(key)
        action = self.mapping.get(code)
        if action is None or now-self.last.get(action, -float('inf')) < .4:
            return None
        self.last[action] = now
        return action


class PedalActions:
    def __init__(self, repo, log_dir, dry_run=False):
        self.repo, self.log_dir, self.dry_run = Path(repo), Path(log_dir), dry_run
        self.motion = None
        self.motion_kind = None
        self.record = None
        self.record_stop = None
        self.following = False
        self.pending_home = False
        self.generation_file = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home()/'.cache')))/'ai_worker/stop-generation'
        self.generation = self.read_generation()

    def read_generation(self):
        return self.generation_file.read_text() if self.generation_file.exists() else ''

    def following_service(self):
        try:
            return subprocess.run(['systemctl', '--user', 'is-active', '--quiet',
                'contactglove-sharpa.service'], timeout=1., stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL).returncode == 0
        except subprocess.TimeoutExpired:
            return False

    def launch(self, script, *args):
        logfile = self.log_dir/f'{datetime.now():%Y%m%d-%H%M%S-%f}-{Path(script).stem}.log'
        with logfile.open('ab') as output:
            process = subprocess.Popen(['bash', str(self.repo/script), *args],
                stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT,
                env={**os.environ, 'AI_WORKER_SESSION_LOG_ACTIVE': '1'}, start_new_session=True)
        print(f'{script} {" ".join(args)} → {logfile}', flush=True)
        return process

    @staticmethod
    def interrupt(process):
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def stop_motion(self, then_home=False):
        self.pending_home = then_home
        if self.motion_kind == 'stop' and self.motion is not None:
            return
        self.interrupt(self.motion)
        self.motion = self.launch('stop_teleop.sh')
        self.motion_kind = 'stop'
        self.following = False

    def press(self, action):
        if self.dry_run:
            print('DRY-RUN: '+action+'（指令なし）', flush=True)
            return
        self.poll()
        if action == 'teleop':
            if self.motion is not None or self.following or self.following_service():
                self.stop_motion()
            else:
                self.motion = self.launch('resume_teleop.sh')
                self.motion_kind = 'start'
        elif action == 'home':
            if self.motion_kind == 'home':
                self.stop_motion()
            elif self.motion is not None:
                self.stop_motion(then_home=True)
            else:
                self.following = False
                self.motion = self.launch('home_teleop.sh')
                self.motion_kind = 'home'
        elif action == 'record':
            if self.record_stop is not None:
                print('記録を保存中です。', flush=True)
            elif self.record is not None:
                self.record_stop = self.launch('record_rosbag.sh', '--stop')
            else:
                self.record = self.launch('record_rosbag.sh', '--toggle')

    def poll(self):
        generation = self.read_generation()
        if generation != self.generation:
            self.following = False
            self.generation = generation
        if self.motion is not None and self.motion.poll() is not None:
            code, kind = self.motion.returncode, self.motion_kind
            print(f'{kind} 終了コード={code}', flush=True)
            self.following = kind == 'start' and code == 0
            self.motion = self.motion_kind = None
            if self.pending_home:
                self.pending_home = False
                if code == 0:
                    self.motion = self.launch('home_teleop.sh')
                    self.motion_kind = 'home'
        for name in ('record', 'record_stop'):
            process = getattr(self, name)
            if process is not None and process.poll() is not None:
                print(f'{name} 終了コード={process.returncode}', flush=True)
                if process.stdin:
                    process.stdin.close()
                setattr(self, name, None)

    def shutdown(self):
        if self.dry_run:
            return
        pending = []
        if self.motion is not None or self.following:
            self.interrupt(self.motion)
            pending.append(self.launch('stop_teleop.sh'))
        if self.record is not None or self.record_stop is not None:
            pending.append(self.record_stop or self.launch('record_rosbag.sh', '--stop'))
        for process in pending:
            try:
                process.wait(timeout=30.)
            except subprocess.TimeoutExpired:
                print('終了応答がありません。各アクションのログを確認してください。', flush=True)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--dry-run', action='store_true', help='入力表示のみ、実機・記録の操作なし')
    options = parser.parse_args(args)
    import evdev
    state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state')))/'ai_worker'
    state.mkdir(parents=True, exist_ok=True)
    lock = (state/'foot-pedal.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('フットペダル操作は既に起動しています')
    log_dir = state/'foot-pedal'
    log_dir.mkdir(exist_ok=True)
    manager = PedalActions(options.repo, log_dir, options.dry_run)
    devices = []
    stopping = [False]
    old = {sig: signal.signal(sig, lambda *_: stopping.__setitem__(0, True))
           for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        for path in evdev.list_devices():
            device = evdev.InputDevice(path)
            if device.info.vendor == 0x3553 and device.info.product == 0xb001 and evdev.ecodes.EV_KEY in device.capabilities():
                device.grab()
                devices.append(device)
            else:
                device.close()
        if not devices:
            raise RuntimeError('PCsensor FootSwitchが見つかりません。USB接続を確認してください')
        edges = PedalEdges()
        # A key already held at launch must be released before its first action.
        for device in devices:
            edges.held.update((device.path, code) for code in device.active_keys())
        print('左: 7番 開始/停止 | 中央: rosbag 開始/保存終了 | 右: 両腕の初期姿勢へ低速移動（再度で中止）', flush=True)
        print('終了: Ctrl+C。入力は他のウィンドウへ送られません。', flush=True)
        while not stopping[0]:
            ready, _, _ = select.select(devices, [], [], .1)
            for device in ready:
                for event in device.read():
                    if event.type == evdev.ecodes.EV_SYN and event.code == evdev.ecodes.SYN_DROPPED:
                        raise RuntimeError('ペダル入力が欠落しました。再起動してください')
                    if event.type == evdev.ecodes.EV_KEY:
                        action = edges.feed(device.path, event.code, event.value)
                        if action:
                            manager.press(action)
            manager.poll()
    except (OSError, RuntimeError) as error:
        print(str(error), flush=True)
        return 1
    finally:
        manager.shutdown()
        for device in devices:
            device.ungrab()
            device.close()
        for sig, previous in old.items():
            signal.signal(sig, previous)
        lock.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
