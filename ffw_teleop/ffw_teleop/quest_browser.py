"""Open only the local teleop URL; do not enter XR or enable robot control."""
import argparse
import asyncio
import json
import shlex
import subprocess
from urllib.request import Request, urlopen

URL = 'https://127.0.0.1:8012/?ws=wss://127.0.0.1:8012'


async def navigate(page):
    import aiohttp
    async with aiohttp.ClientSession() as client:
        async with client.ws_connect(page['webSocketDebuggerUrl']) as ws:
            await ws.send_json(dict(id=1, method='Page.navigate', params=dict(url=URL)))
            while True:
                message = await ws.receive_json(timeout=5.)
                if message.get('id') == 1:
                    if 'error' in message:
                        raise RuntimeError(message['error'])
                    return


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--adb', required=True)
    options = parser.parse_args(args)
    subprocess.run([options.adb, 'shell', 'am', 'start', '-a', 'android.intent.action.VIEW',
        '-p', 'com.oculus.browser', '-d', shlex.quote(URL)], check=True, timeout=8.)
    port = None
    try:
        # Quest Browser may show only New Tab on its first VIEW intent.
        port = subprocess.check_output([options.adb, 'forward', 'tcp:0',
            'localabstract:chrome_devtools_remote'], text=True, timeout=5.).strip()
        if not port.isdecimal():
            raise RuntimeError('Unexpected ADB forward port')
        root = f'http://127.0.0.1:{port}'
        with urlopen(root+'/json/list', timeout=3.) as response:
            pages = json.load(response)
        page = next((p for p in pages if p.get('type') == 'page' and p.get('url', '').startswith((
            'https://127.0.0.1:8012', 'https://localhost:8012', 'https://localhost:8445'))), None)
        if page is not None:
            asyncio.run(navigate(page))
        else:
            with urlopen(Request(root+'/json/new?'+URL, method='PUT'), timeout=3.) as response:
                json.load(response)
        print('Quest Browserでteleop URLを開きました。', flush=True)
    except Exception as error:
        print('ブラウザへURLを送信しました。ページ確認: '+str(error), flush=True)
    finally:
        if port is not None and port.isdecimal():
            subprocess.run([options.adb, 'forward', '--remove', 'tcp:'+port], timeout=3., check=False)
    print('Quest Browser: '+URL, flush=True)
    print('初回の証明書警告を承認し、Enter VRを押してください。実機追従の開始は7番です。', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
