"""Local human-only collector: WebXR poses, Quest RGB and ContactGlove ergonomics."""
import argparse
import asyncio
import fcntl
import json
import os
from pathlib import Path
import secrets
import signal
import shutil
import ssl
import subprocess
import threading
import time
from urllib.request import Request, urlopen

from aiohttp import web
from .human_demo_data import HumanEpisodes, rigid


STATE = Path('~/.local/state/ai_worker/human-demo-server.json').expanduser()


def validate_xr(data):
    if not isinstance(data.get('reference_id'), str) or len(data['reference_id']) > 128:
        raise ValueError('reference_id is required')
    import numpy as np
    for name in ('head', 'left', 'right'):
        if data.get(name) is not None:
            rigid(np.asarray(data[name]).reshape(4, 4, order='F'))
    return data


def application(episodes, token, require_camera=True):
    @web.middleware
    async def guard(request, handler):
        if request.path != '/' and request.headers.get('Authorization') != 'Bearer '+token and request.query.get('token') != token:
            raise web.HTTPUnauthorized()
        try:
            return await handler(request)
        except (ValueError, KeyError, TypeError) as error:
            return web.json_response(dict(error=str(error)), status=400)

    app = web.Application(middlewares=[guard], client_max_size=5*1024*1024)
    sockets = set()
    async def close_sockets(app):
        if sockets:
            await asyncio.gather(*(ws.close() for ws in list(sockets)))
    app.on_shutdown.append(close_sockets)

    async def index(request):
        return web.FileResponse(Path(__file__).resolve().parent/'human_demo_web.html')

    async def status(request):
        return web.json_response(episodes.status())

    async def control(request):
        data = await request.json()
        action = data['action']
        if action in ('start', 'toggle') and not (action == 'toggle' and episodes.stream is not None):
            status = episodes.status()
            missing = [kind for kind in ('xr', 'glove_left', 'glove_right') if status['ages_s'].get(kind, 1e9) > .5]
            xr = episodes.latest.get('xr', {}).get('data', {})
            if xr.get('left') is None or xr.get('right') is None:
                missing.append('tracked left/right controllers')
            for side in ('left', 'right'):
                if xr.get('controllers', {}).get(side, {}).get('emulated_position'):
                    missing.append(side+' controller position is emulated')
                valid = episodes.latest.get('glove_'+side, {}).get('receive_monotonic_ns', 0)
                invalid = episodes.latest.get('glove_invalid_'+side, {}).get('receive_monotonic_ns', 0)
                if invalid > valid:
                    missing.append(side+' glove tracking invalid')
            if require_camera and status['ages_s'].get('camera_frame', 1e9) > 1.:
                missing.append('Quest RGB frames')
            if require_camera and episodes.latest.get('camera_frame', {}).get('data', {}).get('metadata', {}).get('reference_id') != episodes.reference_id:
                missing.append('camera reference_id')
            if missing:
                raise ValueError('新しい配信が不足: '+', '.join(missing))
            result = episodes.start(data.get('task', 'unspecified'))
        elif action in ('stop', 'toggle'):
            result = episodes.stop(data.get('outcome', 'unspecified'))
        elif action == 'mark':
            result = episodes.mark(data['name'], data.get('side', 'right'))
        else:
            raise ValueError('Unknown action')
        return web.json_response(dict(result=result, status=episodes.status()))

    async def events(request):
        if sockets:
            raise web.HTTPConflict(text='別のQuestページが接続中です。既存ページを閉じてください')
        ws = web.WebSocketResponse(max_msg_size=128*1024, heartbeat=10., timeout=1.)
        await ws.prepare(request)
        sockets.add(ws)
        client_id = secrets.token_hex(8)
        episodes.record('client_connect', dict(client_id=client_id))
        try:
            async for message in ws:
                if message.type != web.WSMsgType.TEXT:
                    continue
                data = json.loads(message.data)
                kind, payload = data['event'], data['data']
                if kind == 'ping':
                    await ws.send_json(dict(event='pong', client_ms=payload['client_ms'], server_monotonic_ns=time.monotonic_ns()))
                    continue
                if kind not in ('xr', 'reference', 'tracking', 'clock_sync', 'camera_status', 'capabilities'):
                    raise ValueError('Unknown event')
                if kind == 'xr':
                    validate_xr(payload)
                if not isinstance(payload, dict):
                    raise ValueError('Expected event object')
                payload['client_id'] = client_id
                episodes.record(kind, payload)
        finally:
            sockets.discard(ws)
            episodes.record('client_disconnect', dict(client_id=client_id))
        return ws

    async def camera(request):
        metadata = json.loads(request.headers.get('X-Frame-Metadata', '{}'))
        if metadata.get('source') != 'quest_passthrough_browser' or not metadata.get('verified_environment'):
            raise ValueError('Questの外向き実カメラを選択してください')
        content = await request.read()
        if not content.startswith(b'\xff\xd8'):
            raise ValueError('Expected JPEG')
        # Frames arriving before start are readiness checks, not saved samples.
        episodes.record('camera_frame', dict(camera=request.match_info['name'], metadata=metadata, saved=episodes.stream is not None))
        if episodes.stream is not None:
            episodes.image(request.match_info['name'], content, metadata)
        return web.json_response(dict(recording=episodes.stream is not None))

    app.router.add_get('/', index)
    app.router.add_get('/status', status)
    app.router.add_post('/control', control)
    app.router.add_get('/events', events)
    app.router.add_post('/camera/{name}', camera)
    return app


def glove_event(episodes, frame):
    values = [float(v) for v in frame.values]
    import math
    side = 'left' if frame.is_left else 'right'
    payload = dict(id=str(frame.id), source_timestamp_ms=int(frame.timestamp_ms),
        tracking_state=int(frame.tracking_state), angles_degrees=values)
    if int(frame.tracking_state) == 2:
        # NotAttempted is an intentional DivingStation IK skip. Preserve the
        # report without refreshing valid data or declaring tracking loss.
        payload['angles_degrees'] = values if all(math.isfinite(v) for v in values) else None
        episodes.record('glove_skipped_'+side, payload)
        return
    previous = episodes.latest.get('glove_'+side, {}).get('data')
    if int(frame.tracking_state) == 0 and previous and previous['id'] == payload['id'] and previous['source_timestamp_ms'] >= payload['source_timestamp_ms']:
        episodes.record('glove_duplicate_'+side, dict(source_timestamp_ms=payload['source_timestamp_ms']))
        return
    # Retain invalid tracking reports without refreshing valid-input readiness.
    valid = len(values) == 20 and all(math.isfinite(v) for v in values) and int(frame.tracking_state) == 0
    if not all(math.isfinite(v) for v in values):
        payload['angles_degrees'] = None
    episodes.record('glove_'+side if valid else 'glove_invalid_'+side, payload)


async def serve(options):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    server_lock = STATE.with_suffix('.lock').open('a')
    try:
        fcntl.flock(server_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('人間実演の収集サーバは既に起動しています')
    episodes = HumanEpisodes(options.output)
    config = Path('~/.config/ai_worker/human-demo').expanduser()
    config.mkdir(parents=True, exist_ok=True)
    cert, key = config/'cert.pem', config/'key.pem'
    if not cert.exists() or not key.exists():
        subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
            '-keyout', str(key), '-out', str(cert), '-days', '365', '-subj', '/CN=localhost',
            '-addext', 'subjectAltName=DNS:localhost,IP:127.0.0.1'], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        key.chmod(0o600)
    if options.usb:
        adb = shutil.which('adb') or str(Path('~/.local/share/ai_worker/android/platform-tools/adb').expanduser())
        subprocess.run([adb, 'reverse', f'tcp:{options.port}', f'tcp:{options.port}'], check=True)
    token = secrets.token_urlsafe(24)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert, key)
    app = application(episodes, token, not options.allow_no_camera)
    runner = web.AppRunner(app, shutdown_timeout=2.)
    await runner.setup()
    site = web.TCPSite(runner, options.bind, options.port, ssl_context=context)
    await site.start()
    STATE.parent.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(dict(port=options.port, token=token, pid=os.getpid())))
    STATE.chmod(0o600)
    suffix = '&pose_only=1' if options.allow_no_camera else ''
    print(f'Quest: https://localhost:{options.port}/?token={token}{suffix}', flush=True)
    print('Questで証明書を承認してEnter XRを実行。start/stopは別ターミナル。実機へは出力しません。', flush=True)
    client = None
    ros_thread = None
    ros_node = None
    try:
        if options.glove_source == 'sdk':
            from contactglovesdk import ContactGloveClient, Host
            client = ContactGloveClient(local_port=0)
            client.register_callback_for_ergonomics_stream(lambda frame: glove_event(episodes, frame))
            try:
                await asyncio.to_thread(client.connect_to_host, Host('DivingStation', options.glove_host, options.glove_port))
            except Exception as error:
                episodes.record('glove_source_error', dict(message=str(error)))
                print('DivingStation接続: '+str(error), flush=True)
        elif options.glove_source == 'ros':
            import rclpy
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.signals import SignalHandlerOptions
            from contact_glove_msgs.msg import Ergonomics
            from .contactglove_mapping import ergonomics_frame
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            ros_node = rclpy.create_node('human_glove_read_only')
            for side in ('left', 'right'):
                ros_node.create_subscription(Ergonomics, f'/contact_glove/{side}/ergonomics',
                    lambda msg: glove_event(episodes, ergonomics_frame(msg)), qos_profile_sensor_data)
            ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
            ros_thread.start()
        stopped = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            loop.add_signal_handler(sig, stopped.set)
        await stopped.wait()
    finally:
        episodes.stop('interrupted')
        if client:
            client.close()
        if ros_node:
            rclpy.try_shutdown()
            ros_thread.join(timeout=2.)
            ros_node.destroy_node()
        await runner.cleanup()
        STATE.unlink(missing_ok=True)
        server_lock.close()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    server = sub.add_parser('serve')
    server.add_argument('--port', type=int, default=8444)
    server.add_argument('--bind', default='127.0.0.1')
    server.add_argument('--usb', action='store_true')
    server.add_argument('--output', type=Path, default=Path('~/human_demos').expanduser())
    server.add_argument('--glove-source', choices=('sdk', 'ros', 'none'), default='ros')
    server.add_argument('--glove-host', default='127.0.0.1')
    server.add_argument('--glove-port', type=int, default=25791)
    server.add_argument('--allow-no-camera', action='store_true', help='明示的に位置・指のみの収集を許可')
    start = sub.add_parser('start')
    start.add_argument('--task', required=True)
    stop = sub.add_parser('stop')
    stop.add_argument('--outcome', choices=('success', 'failure', 'unspecified'), default='unspecified')
    sub.add_parser('status')
    mark = sub.add_parser('mark')
    mark.add_argument('name', choices=('origin', 'right', 'forward'))
    mark.add_argument('--side', choices=('left', 'right'), default='right')
    export = sub.add_parser('export')
    export.add_argument('directory', type=Path)
    export.add_argument('--trim-start', type=float, default=0.)
    export.add_argument('--trim-end', type=float, default=0.)
    sub.add_parser('ui', help='PCで開始・停止する操作画面')
    options = parser.parse_args(args)
    if options.command == 'ui':
        from .human_demo_ui import main as ui_main
        return ui_main()
    if options.command == 'export':
        from .export_human_demo import export_episode
        print(export_episode(options.directory, trim_start=options.trim_start, trim_end=options.trim_end))
        return 0
    if options.command == 'serve':
        try:
            asyncio.run(serve(options))
        except KeyboardInterrupt:
            pass
        return 0
    state = json.loads(STATE.read_text())
    url = f'https://localhost:{state["port"]}'
    data = None
    if options.command != 'status':
        data = dict(vars(options), action=options.command)
        if options.command == 'mark':
            data['name'] = options.name
    request = Request(url+('/control' if data else '/status'),
        data=json.dumps(data).encode() if data else None,
        headers={'Authorization': 'Bearer '+state['token'], 'Content-Type': 'application/json'})
    # Pin this server's generated certificate; never disable TLS verification.
    context = ssl.create_default_context(cafile=str(Path('~/.config/ai_worker/human-demo/cert.pem').expanduser()))
    from urllib.error import HTTPError
    try:
        with urlopen(request, context=context, timeout=5.) as response:
            print(json.dumps(json.load(response), ensure_ascii=False, indent=2))
    except HTTPError as error:
        print(error.read().decode())
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
