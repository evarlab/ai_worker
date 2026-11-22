#!/usr/bin/env python3
"""Relay hand broadcasts to the Pilot PC; never send commands to hands."""
import argparse
import ipaddress
import select
import signal
import socket
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--peer', required=True, type=ipaddress.IPv4Address)
    options = parser.parse_args()
    peer = str(options.peer)
    sockets = []
    stopping = [False]
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopping.__setitem__(0, True))
    output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    counts = {}
    try:
        for port in (54321, 50000, 50005, 50011, 50010, 50015, 50001):
            source = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            source.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            source.bind(('0.0.0.0', port))
            sockets.append(source)
            counts[port] = 0
        print('Pilot receive-only UDP relay ready: '+peer, flush=True)
        deadline = time.monotonic()+10.
        while not stopping[0]:
            ready, _, _ = select.select(sockets, [], [], .2)
            for source in ready:
                payload, address = source.recvfrom(65535)
                if address[0] not in ('192.168.10.10', '192.168.10.20'):
                    continue
                port = source.getsockname()[1]
                output.sendto(payload, (peer, port))
                counts[port] += 1
            if time.monotonic() >= deadline:
                print('relayed packets: '+str(counts), flush=True)
                deadline = time.monotonic()+10.
    finally:
        for source in sockets:
            source.close()
        output.close()


if __name__ == '__main__':
    main()
