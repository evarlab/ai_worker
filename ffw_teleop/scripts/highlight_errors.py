#!/usr/bin/env python3
"""Highlight only [ERROR], streaming partial lines (including SSH prompts)."""
import os
import signal
import sys


def main():
    # Let the command in the PTY handle Ctrl+C; drain its final output.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    token = b'[ERROR]'
    pending = b''
    while True:
        chunk = os.read(sys.stdin.fileno(), 65536)
        if not chunk:
            sys.stdout.buffer.write(pending)
            sys.stdout.buffer.flush()
            return
        pending += chunk
        # Keep only a potential split token, never a full line or prompt.
        keep = next((n for n in range(len(token)-1, 0, -1)
                     if pending.endswith(token[:n])), 0)
        ready = pending[:-keep] if keep else pending
        pending = pending[-keep:] if keep else b''
        sys.stdout.buffer.write(ready.replace(token, b'\x1b[31m[ERROR]\x1b[39m'))
        sys.stdout.buffer.flush()


if __name__ == '__main__':
    main()
