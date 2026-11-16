"""Timestamped, compressed telemetry for each VR/robot process session."""
import atexit
import gzip
import json
import os
from pathlib import Path
import threading
import time


class SessionRecorder:
    def __init__(self, name, directory=None):
        directory = Path(directory or '~/.local/state/ai_worker/teleop').expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f'{name}-{time.strftime("%Y%m%d-%H%M%S")}-{os.getpid()}-{time.time_ns()}.jsonl.gz'
        self.stream = gzip.open(self.path, 'wt', encoding='utf-8', compresslevel=1)
        self.lock = threading.Lock()
        self.last = {}
        self.flushed_at = time.monotonic()
        atexit.register(self.close)
        self.record('session_start', {'pid': os.getpid(), 'name': name})

    @staticmethod
    def encode(value):
        if hasattr(value, 'tolist'):
            return value.tolist()
        if hasattr(value, 'get_fields_and_field_types'):
            return {key: getattr(value, key) for key in value.get_fields_and_field_types()}
        raise TypeError(f'Unsupported log value: {type(value).__name__}')

    def record(self, event, data, every=0.):
        now = time.monotonic()
        with self.lock:
            if self.stream is None or now-self.last.get(event, -float('inf')) < every:
                return
            self.last[event] = now
            self.stream.write(json.dumps(dict(wall_time=time.time(), monotonic=now,
                event=event, data=data), default=self.encode, ensure_ascii=False)+'\n')
            if now-self.flushed_at >= 1. or event in ('stop', 'anchor'):
                self.stream.flush()
                self.flushed_at = now

    def close(self):
        with self.lock:
            if self.stream is not None:
                self.stream.close()
                self.stream = None
