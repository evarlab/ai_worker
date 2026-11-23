"""Open the collector page on the attached Quest; XR requires a headset click."""
import asyncio
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from urllib.request import Request, urlopen


def open_page(state, pose_only=False):
    adb = shutil.which('adb') or str(Path('~/.local/share/ai_worker/android/platform-tools/adb').expanduser())
    url = f'https://localhost:{state["port"]}/?token={state["token"]}'+('&pose_only=1' if pose_only else '')
    subprocess.run([adb, 'shell', 'am', 'start', '-a', 'android.intent.action.VIEW',
                    '-p', 'com.oculus.browser', '-d', shlex.quote(url)], check=True, timeout=10.)
    port = None
    try:
        port = subprocess.check_output([adb, 'forward', 'tcp:0',
            'localabstract:chrome_devtools_remote'], text=True, timeout=5.).strip()
        if not port.isdecimal():
            raise ValueError('Unexpected debugger port')
        root = f'http://127.0.0.1:{port}'
        with urlopen(root+'/json/list', timeout=3.) as response:
            pages = json.load(response)
        page = next((p for p in pages if p.get('type') == 'page' and
            p.get('url', '').startswith(f'https://localhost:{state["port"]}/')), None)
        if page:
            async def navigate():
                import aiohttp
                async with aiohttp.ClientSession() as client:
                    async with client.ws_connect(page['webSocketDebuggerUrl']) as ws:
                        await ws.send_json(dict(id=1, method='Page.navigate', params=dict(url=url)))
                        while True:
                            msg = await ws.receive_json(timeout=5.)
                            if msg.get('id') == 1:
                                if 'error' in msg:
                                    raise RuntimeError(msg['error'])
                                return
            asyncio.run(navigate())
        else:
            with urlopen(Request(root+'/json/new?'+url, method='PUT'), timeout=3.) as response:
                json.load(response)
    except Exception as error:
        print('QuestへURLを送信済み。ページ確認: '+str(error), flush=True)
    finally:
        if port and port.isdecimal():
            subprocess.run([adb, 'forward', '--remove', 'tcp:'+port], timeout=3., check=False)
    print('Questで証明書を承認し、Enter XRを押してください。', flush=True)
