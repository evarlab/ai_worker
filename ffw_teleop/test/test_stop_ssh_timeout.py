import os
from pathlib import Path
import subprocess


def test_disconnected_shared_ssh_cannot_block_stop_indefinitely(tmp_path):
    ssh = tmp_path/'ssh'
    ssh.write_text('#!/bin/sh\nexec /bin/sleep 30\n')
    ssh.chmod(0o755)
    systemctl = tmp_path/'systemctl'
    systemctl.write_text('#!/bin/sh\nexit 0\n')
    systemctl.chmod(0o755)
    pkill = tmp_path/'pkill'
    pkill.write_text('#!/bin/sh\nexit 0\n')
    pkill.chmod(0o755)
    script = Path(__file__).resolve().parents[1]/'scripts/stop_teleop.sh'
    result = subprocess.run(['bash', str(script)], capture_output=True, text=True,
        env={**os.environ, 'PATH': str(tmp_path)+':'+os.environ['PATH'],
             'XDG_CACHE_HOME': str(tmp_path/'cache')}, timeout=10)
    assert result.returncode == 1
    assert 'Stop could not be confirmed' in result.stderr
    assert 'robot=124' in result.stderr
