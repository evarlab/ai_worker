"""Host-local recorder lock and graceful stop; no ROS or motor commands."""
import fcntl
import json
import os
from pathlib import Path
import signal
import tempfile
import time


def control_path(domain):
    return Path(tempfile.gettempdir())/f'ai_worker_rgbd_{os.getuid()}_{domain}.json'


def process_identity(pid):
    return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[19]


def recording_active(domain):
    try:
        data = json.loads(control_path(domain).read_text())
        return process_identity(data['pid']) == data['identity']
    except (FileNotFoundError, ProcessLookupError):
        return False


def stop_recording(domain, timeout=25.):
    path = control_path(domain)
    if not path.exists():
        print('記録は停止しています。', flush=True)
        return 0
    data = json.loads(path.read_text())
    pid = data['pid']
    try:
        if process_identity(pid) != data['identity']:
            raise ProcessLookupError()
        command = Path(f'/proc/{pid}/cmdline').read_bytes()
        if b'ffw_teleop.record_rosbag' not in command and b'/ffw_teleop/record_rosbag' not in command:
            raise RuntimeError('記録プロセスの識別に失敗しました')
        os.kill(pid, signal.SIGINT)
    except (FileNotFoundError, ProcessLookupError):
        print('記録プロセスは既に終了しています。', flush=True)
        return 0
    end = time.monotonic()+timeout
    while path.exists() and time.monotonic() < end:
        time.sleep(.1)
    if path.exists():
        raise RuntimeError('記録終了待ちがタイムアウトしました。ログを確認してください')
    print('記録を保存して終了しました。', flush=True)
    return 0


class RecorderLock:
    def __init__(self, domain):
        self.path = control_path(domain)
        self.file = self.path.with_suffix('.lock').open('a')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.file.close()
            raise RuntimeError('既に記録中です。終了は ./record_rosbag.sh --stop')
        self.path.write_text(json.dumps(dict(pid=os.getpid(), identity=process_identity(os.getpid()))))

    def close(self):
        self.path.unlink(missing_ok=True)
        self.file.close()
