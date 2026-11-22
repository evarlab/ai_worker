"""ADB + local WebXR capability and out-of-headset-view tracking check."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import shlex
import signal
import ssl
import subprocess
import sys
import time
from urllib.request import Request, urlopen

from .human_demo import STATE


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8445)
    parser.add_argument('--serial')
    parser.add_argument('--duration', type=float, default=300.)
    options = parser.parse_args(args)
    import math
    if not math.isfinite(options.duration) or options.duration <= 0:
        parser.error('durationは正の値')
    adb = shutil.which('adb') or str(Path('~/.local/share/ai_worker/android/platform-tools/adb').expanduser())
    prefix = [adb]+(['-s', options.serial] if options.serial else [])
    def run(*command):
        result = subprocess.run([*prefix, *command], text=True, capture_output=True, timeout=8.)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        return result.stdout.strip()
    try:
        if run('get-state') != 'device':
            raise RuntimeError('ADB接続が未承認です')
        model = run('shell', 'getprop', 'ro.product.model')
        device = run('shell', 'getprop', 'ro.product.device')
        android = run('shell', 'getprop', 'ro.build.version.release')
        package = run('shell', 'dumpsys', 'package', 'com.oculus.browser')
        version = next((line.strip() for line in package.splitlines() if 'versionName=' in line), 'not found')
        print(f'ADB OK: model={model} device={device} Android={android} Browser {version}', flush=True)
        run('reverse', f'tcp:{options.port}', f'tcp:{options.port}')
    except (RuntimeError, subprocess.TimeoutExpired) as error:
        print('Quest側の確認を開始できません: '+str(error), flush=True)
        print('USB権限、開発者モード、USBデバッグ許可を確認してください。', flush=True)
        return 1
    if STATE.exists():
        state = json.loads(STATE.read_text())
        if Path(f'/proc/{state["pid"]}').exists():
            print('既存の人間実演サーバを終了してから実行してください。', flush=True)
            return 1
    directory = Path('~/.local/state/ai_worker/quest-check').expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    stem = directory/f'{datetime.now():%Y%m%d-%H%M%S}-{os.getpid()}'
    server_log = stem.with_suffix('.server.log').open('w')
    report = dict(model=model, device=device, android=android, browser=version,
        both_tracked=False, behind_head_tracked=dict(left=False, right=False),
        scope='local WebXR check only; no robot commands', samples=[])
    server = subprocess.Popen([sys.executable, '-u', '-m', 'ffw_teleop.human_demo', 'serve',
        '--port', str(options.port), '--glove-source', 'none', '--allow-no-camera'],
        stdout=server_log, stderr=subprocess.STDOUT)
    stop = [False]
    old = {sig: signal.signal(sig, lambda *_: stop.__setitem__(0, True)) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        end = time.monotonic()+8.
        while time.monotonic() < end:
            if STATE.exists() and json.loads(STATE.read_text()).get('pid') == server.pid:
                break
            if server.poll() is not None:
                raise RuntimeError('診断サーバの起動失敗: '+str(stem.with_suffix('.server.log')))
            time.sleep(.1)
        else:
            raise RuntimeError('診断サーバの起動待ちがタイムアウトしました')
        state = json.loads(STATE.read_text())
        url = f'https://localhost:{options.port}/?token={state["token"]}&tracking_only=1'
        run('shell', 'am', 'start', '-a', 'android.intent.action.VIEW', '-p', 'com.oculus.browser', '-d', shlex.quote(url))
        print('Questに診断ページを開きました。証明書を承認してEnter XRを押してください。', flush=True)
        print('正面で両手を動かし、その後片手ずつ頭より後ろで3秒ほど小さく動かします。コントローラ自身のカメラは覆わないでください。', flush=True)
        behind_samples = {'left': [], 'right': []}
        context = ssl.create_default_context(cafile=str(Path('~/.config/ai_worker/human-demo/cert.pem').expanduser()))
        end = time.monotonic()+options.duration
        while not stop[0] and time.monotonic() < end:
            request = Request(f'https://localhost:{options.port}/status', headers={'Authorization':'Bearer '+state['token']})
            with urlopen(request, context=context, timeout=3.) as response:
                status = json.load(response)
            tracking = status['tracking']
            both = all(tracking[side]['tracked'] for side in ('left', 'right'))
            report['both_tracked'] |= both
            for side in ('left', 'right'):
                if tracking[side]['tracked'] and tracking[side]['behind_head']:
                    samples = behind_samples[side]
                    samples.append((time.monotonic(), tracking[side]['head_relative_position_m']))
                    import numpy as np
                    span = np.linalg.norm(np.ptp(np.asarray([s[1] for s in samples]), axis=0))
                    if samples[-1][0]-samples[0][0] >= 2. and span >= .03:
                        report['behind_head_tracked'][side] = True
                else:
                    behind_samples[side] = []
            report['samples'].append(dict(wall_time=time.time(), tracking=tracking, xr_rate_hz=status['xr_rate_hz']))
            print(f'XR {status["xr_rate_hz"]:.1f}Hz | 左={tracking["left"]} | 右={tracking["right"]}', flush=True)
            stem.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            if both and all(report['behind_head_tracked'].values()):
                print('PASS: 両手追跡と、頭の後方での左右追跡を確認しました。', flush=True)
                return 0
            time.sleep(1.)
        print('確認終了。未確認の項目は未確認として保存しました。', flush=True)
        return 0 if report['both_tracked'] else 1
    except (RuntimeError, OSError) as error:
        print(str(error), flush=True)
        return 1
    finally:
        report['finished_at'] = datetime.now().astimezone().isoformat()
        stem.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        server.send_signal(signal.SIGTERM)
        try:
            server.wait(timeout=8.)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()
        server_log.close()
        for sig, handler in old.items():
            signal.signal(sig, handler)
        print('診断結果: '+str(stem.with_suffix('.json')), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
