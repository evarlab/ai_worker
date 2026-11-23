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
import sys
import shlex
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
    for side, hand in data.get('hands', {}).items():
        if side not in ('left', 'right') or not isinstance(hand, dict):
            raise ValueError('Invalid hand input')
        if hand.get('wrist') is not None:
            rigid(np.asarray(hand['wrist']).reshape(4,4,order='F'))
        joints=hand.get('joints', {})
        if not isinstance(joints,dict) or len(joints)>25:
            raise ValueError('Invalid hand joints')
        for joint in joints.values():
            if joint is not None:
                rigid(np.asarray(joint['matrix']).reshape(4,4,order='F'))
    return data


def application(episodes, token, require_camera=True, detector=None, camera_configs=None):
    from .human_demo_calibration import Calibration
    from .quest_camera_calibration import for_frame
    if episodes.capture_calibration is None:
        episodes.capture_calibration = Calibration()
    calibration = episodes.capture_calibration
    camera_configs = camera_configs or {}
    @web.middleware
    async def guard(request, handler):
        if request.path != '/' and not request.path.startswith('/mount-assets/') and request.headers.get('Authorization') != 'Bearer '+token and request.query.get('token') != token:
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

    def apply_control(data):
        action = data['action']
        if action in ('start', 'toggle') and not (action == 'toggle' and episodes.stream is not None):
            if not episodes.following:
                raise ValueError('先にfollow-startで頭の初期位置を設定してください')
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
        elif action == 'follow-toggle':
            result = episodes.follow_stop() if episodes.following else episodes.follow_start()
        elif action == 'follow-start':
            result = episodes.follow_start()
        elif action == 'follow-stop':
            result = episodes.follow_stop()
        elif action == 'mount-set':
            if episodes.stream is not None:
                raise ValueError('記録中は取り付け補正を変更できません。先にstopしてください')
            skeleton=episodes.latest.get('skeleton_'+data['side'],{}).get('data',{})
            result=calibration.set_manual_mount(data['side'],data['T_controller_wrist'],data['controller_model'],skeleton.get('profile_uuid'))
            episodes.record('mount_calibration',dict(side=data['side'],**result))
        elif action in ('mount-calibrate','board-start','board-finish'):
            if episodes.stream is not None:
                raise ValueError('校正前に記録をstopしてください')
            if action == 'mount-calibrate':
                result = calibration.calibrate_mount()
                episodes.record('mount_calibration', result)
            elif action == 'board-start':
                result = calibration.begin_board()
            else:
                result = calibration.finish_board()
                episodes.record('board_calibration', result)
        elif action == 'mark':
            result = episodes.mark(data['name'], data.get('side', 'right'))
        else:
            raise ValueError('Unknown action')
        current = episodes.status()
        print(f"操作 {action}: 追従={'ON' if current['following'] else 'OFF'} / 記録={'ON' if current['recording'] else 'OFF'} / {result}", flush=True)
        return dict(result=result, status=current)

    async def control(request):
        return web.json_response(apply_control(await request.json()))

    app['apply_control'] = apply_control

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
                if kind == 'tracking' and payload.get('state') == 'session_end':
                    episodes.follow_stop()
                episodes.record(kind, payload)
        finally:
            sockets.discard(ws)
            episodes.follow_stop()
            episodes.record('client_disconnect', dict(client_id=client_id))
        return ws

    async def camera(request):
        metadata = json.loads(request.headers.get('X-Frame-Metadata', '{}'))
        if metadata.get('source') != 'quest_passthrough_browser' or not metadata.get('verified_environment'):
            raise ValueError('Questの外向き実カメラを選択してください')
        frame_calibration=for_frame(camera_configs,metadata)
        metadata['intrinsics']=frame_calibration['K'] if frame_calibration else None
        if frame_calibration:
            metadata['intrinsics_source']=frame_calibration
        content = await request.read()
        if not content.startswith(b'\xff\xd8'):
            raise ValueError('Expected JPEG')
        # Frames arriving before start are readiness checks, not saved samples.
        episodes.record('camera_frame', dict(camera=request.match_info['name'], metadata=metadata, saved=episodes.stream is not None))
        if episodes.stream is not None:
            episodes.image(request.match_info['name'], content, metadata)
        detections = await asyncio.to_thread(detector.detect, content, frame_calibration, episodes.workspace) if detector else None
        if detections is not None:
            anchored=calibration.observe_board(detections,metadata)
            if anchored:
                episodes.record('board_calibration',anchored)
            episodes.record('aruco', dict(**detections, metadata=metadata))
        return web.json_response(dict(recording=episodes.stream is not None, aruco=detections))

    async def capture_config(request):
        return web.json_response(dict(camera_size=[1280,1280] if camera_configs else [1280,960],camera_labels=list(camera_configs)))

    async def mount_editor(request):
        return web.FileResponse(Path(__file__).resolve().parent/'mount_editor.html')

    async def mount_state(request):
        with episodes.lock:
            state=episodes.status()
            skeletons={}
            for side in ('left','right'):
                valid=episodes.latest.get('skeleton_'+side,{})
                invalid=episodes.latest.get('skeleton_invalid_'+side,{})
                if valid.get('receive_monotonic_ns',0)>invalid.get('receive_monotonic_ns',0):
                    skeletons[side]=valid.get('data')
            return web.json_response(dict(status=state,skeletons=skeletons,
                xr=episodes.latest.get('xr',{}).get('data'),mounts=calibration.mounts))

    app.router.add_get('/mount-editor', mount_editor)
    app.router.add_get('/mount-state', mount_state)
    app.router.add_static('/mount-assets/',Path(__file__).resolve().parent/'mount_assets')
    app.router.add_get('/capture-config', capture_config)
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


def skeleton_event(episodes, frame):
    """Retargeted SDK hand coordinates in metres, independent of Quest optics."""
    import numpy as np
    side='left' if frame.is_left else 'right'
    positions=[[float(p.x),float(p.y),float(p.z)] for p in frame.positions]
    rotations=[[float(q.w),float(q.x),float(q.y),float(q.z)] for q in frame.rotations]
    values=np.asarray(positions)
    valid=(int(frame.tracking_state)==0 and values.shape==(25,3) and
           np.isfinite(values).all() and np.asarray(rotations).shape==(19,4) and np.isfinite(rotations).all())
    if valid:
        previous=episodes.latest.get('skeleton_'+side,{}).get('data')
        if previous and previous['id']==str(frame.id) and previous['source_timestamp_ms']>=int(frame.timestamp_ms):
            return
    payload=dict(id=str(frame.id),profile_uuid=frame.profile_uuid,source_timestamp_ms=int(frame.timestamp_ms),
        tracking_state=int(frame.tracking_state),positions_m=positions if np.isfinite(values).all() else None,
        rotations_wxyz=rotations if np.isfinite(rotations).all() else None,
        frame='ContactGlove retargeted skeleton root (wrist); +X tips to wrist +Y palm to back; metres')
    kind='skeleton_skipped_'+side if int(frame.tracking_state)==2 else 'skeleton_'+side if valid else 'skeleton_invalid_'+side
    episodes.record(kind,payload)


async def terminal_controls(app, stopped, task):
    """Read lines through the event loop; EOF only detaches terminal controls."""
    loop = asyncio.get_running_loop()
    reader = asyncio.StreamReader()
    protocol = asyncio.StreamReaderProtocol(reader)
    transport, _ = await loop.connect_read_pipe(lambda: protocol, sys.stdin)
    print('操作: mount-calibrate / board-start / board-finish / follow-start / follow-stop / start [タスク名] / stop [success|failure|unspecified] / status / quit', flush=True)
    try:
        while not stopped.is_set():
            line = await reader.readline()
            if not line:
                break
            try:
                words = shlex.split(line.decode())
                if not words:
                    continue
                command, *args = words
                if command == 'quit':
                    stopped.set()
                    break
                if command == 'status':
                    result = app['episodes'].status()
                elif command in ('follow-start', 'follow-stop', 'mount-calibrate', 'board-start', 'board-finish'):
                    if args:
                        raise ValueError(command+' は引数不要です')
                    result = app['apply_control'](dict(action=command))
                elif command == 'start':
                    result = app['apply_control'](dict(action='start', task=' '.join(args) or task))
                elif command == 'stop':
                    outcome = args[0] if args else 'unspecified'
                    if len(args) > 1 or outcome not in ('success', 'failure', 'unspecified'):
                        raise ValueError('stop [success|failure|unspecified]')
                    result = app['apply_control'](dict(action='stop', outcome=outcome))
                else:
                    raise ValueError('操作: follow-start / follow-stop / start / stop / status / quit')
                print(json.dumps(result, ensure_ascii=False), flush=True)
            except (ValueError, KeyError) as error:
                print('操作エラー: '+str(error), flush=True)
    finally:
        transport.close()


async def serve(options):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    server_lock = STATE.with_suffix('.lock').open('a')
    try:
        fcntl.flock(server_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('人間実演の収集サーバは既に起動しています')
    workspace = json.loads(options.workspace.read_text()) if options.workspace else None
    episodes = HumanEpisodes(options.output, workspace=workspace)
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
    camera_configs = {}
    if options.usb and options.aruco:
        from .quest_camera_calibration import read_camera2
        try:
            camera_configs = await asyncio.to_thread(read_camera2, adb)
            print('Camera2校正情報: '+', '.join(camera_configs), flush=True)
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            print('Camera2校正情報なし（2D検出のみ）: '+str(error), flush=True)
    from .human_demo_calibration import Calibration
    episodes.capture_calibration = Calibration(config/'calibration.json')
    token = secrets.token_urlsafe(24)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert, key)
    detector = None
    if options.aruco:
        from .human_demo_aruco import ArucoDetector
        detector = ArucoDetector(options.aruco_dictionary)
    app = application(episodes, token, not options.allow_no_camera, detector, camera_configs)
    app['episodes'] = episodes
    runner = web.AppRunner(app, shutdown_timeout=2.)
    await runner.setup()
    site = web.TCPSite(runner, options.bind, options.port, ssl_context=context)
    await site.start()
    STATE.parent.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(dict(port=options.port, token=token, pid=os.getpid())))
    STATE.chmod(0o600)
    suffix = '&pose_only=1' if options.allow_no_camera else ''
    print(f'Quest: https://localhost:{options.port}/?token={token}{suffix}', flush=True)
    print('Questで証明書を承認してEnter XRを実行。start/stopはこのターミナルで入力。実機へは出力しません。', flush=True)
    if options.open_quest:
        from .human_demo_quest import open_page
        await asyncio.to_thread(open_page, dict(port=options.port, token=token), options.allow_no_camera)
    terminal_task = None
    client = None
    ros_thread = None
    ros_node = None
    try:
        if options.glove_source == 'sdk':
            from contactglovesdk import ContactGloveClient, Host
            client = ContactGloveClient(local_port=0)
            client.register_callback_for_ergonomics_stream(lambda frame: glove_event(episodes, frame))
            client.register_callback_for_skeleton_stream(lambda frame: skeleton_event(episodes, frame))
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
        if sys.stdin.isatty() and not options.no_terminal:
            terminal_task = asyncio.create_task(terminal_controls(app, stopped, options.task))
        await stopped.wait()
    finally:
        if terminal_task:
            terminal_task.cancel()
            await asyncio.gather(terminal_task, return_exceptions=True)
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
    server.add_argument('--open-quest', action='store_true', help='Quest Browserに収集ページを開く')
    server.add_argument('--workspace', type=Path, help='作業台・マーカー配置の設定JSON（実測校正とは別）')
    server.add_argument('--aruco', action='store_true', help='RGB上のArUco検出とXR内映像への表示')
    server.add_argument('--aruco-dictionary', default='DICT_4X4_50')
    server.add_argument('--task', default='human_demo')
    server.add_argument('--no-terminal', action='store_true')
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
    sub.add_parser('watch', help='ターミナルに受信・追従・記録の状態を連続表示')
    sub.add_parser('status')
    sub.add_parser('mount-calibrate', help='直近10秒の左右手首・コントローラ同時追跡から取り付け校正')
    sub.add_parser('board-start', help='固定紙ボードと頭の校正サンプルを集める')
    sub.add_parser('board-finish', help='頭→カメラとWebXR→ボードの校正を推定・検証・保存')
    sub.add_parser('follow-start', help='現在の頭の姿勢を初期基準にして追従開始')
    sub.add_parser('follow-stop', help='追従停止。記録中ならinterruptedで終了')
    mark = sub.add_parser('mark')
    mark.add_argument('name', choices=('origin', 'right', 'forward'))
    mark.add_argument('--side', choices=('left', 'right'), default='right')
    export = sub.add_parser('export')
    export.add_argument('directory', type=Path)
    export.add_argument('--calibration',type=Path,help='同一reference_idの校正スナップショット')
    export.add_argument('--trim-start', type=float, default=0.)
    export.add_argument('--trim-end', type=float, default=0.)
    sub.add_parser('mount-editor', help='PCで手の骨格とコントローラの取り付け位置・角度を調整')
    sub.add_parser('ui', help='PCで開始・停止する操作画面')
    options = parser.parse_args(args)
    if options.command == 'mount-editor':
        state=json.loads(STATE.read_text())
        url=f'https://localhost:{state["port"]}/mount-editor?token={state["token"]}'
        subprocess.Popen(['firefox','--new-window',url],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        print('PCの取り付け調整画面を開きました。')
        return 0
    if options.command == 'watch':
        from .human_demo_client import main as watch
        try:
            watch()
        except KeyboardInterrupt:
            pass
        return 0
    if options.command == 'ui':
        from .human_demo_ui import main as ui_main
        return ui_main()
    if options.command == 'export':
        from .export_human_demo import export_episode
        print(export_episode(options.directory, trim_start=options.trim_start, trim_end=options.trim_end, calibration_file=options.calibration))
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
