"""Pinned local collector client, shared by pedals and the status monitor."""
import json
from pathlib import Path
import ssl
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def request(data=None):
    state = json.loads(Path('~/.local/state/ai_worker/human-demo-server.json').expanduser().read_text())
    context = ssl.create_default_context(cafile=str(Path('~/.config/ai_worker/human-demo/cert.pem').expanduser()))
    req = Request(f'https://localhost:{state["port"]}'+('/control' if data else '/status'),
        data=json.dumps(data).encode() if data else None,
        headers={'Authorization': 'Bearer '+state['token'], 'Content-Type': 'application/json'})
    try:
        with urlopen(req, context=context, timeout=3.) as response:
            return json.load(response)
    except HTTPError as error:
        raise ValueError(error.read().decode()) from error


def main():
    import time
    while True:
        try:
            s = request()
            ages = s['ages_s']
            text = ' | '.join(f'{key}: {ages[key]:.2f}s' if key in ages else key+': 未受信'
                              for key in ('xr', 'glove_left', 'glove_right', 'camera_frame'))
            calibration=s.get('capture_calibration') or {}
            text += ' | 手首同時サンプル='+str(calibration.get('mount_samples',{}))+' ボード校正='+str(calibration.get('board_calibrated',False))+' 校正画像='+str(calibration.get('board_samples',0))
            print(f"追従={s['following']} 記録={s['recording']} XR={s['xr_rate_hz']:.1f}Hz | {text}", flush=True)
        except (OSError, ValueError) as error:
            print('サーバ待機: '+str(error), flush=True)
        time.sleep(1.)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
