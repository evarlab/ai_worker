"""Keep the ROS context alive long enough to publish measured holds on Ctrl+C."""
import signal
import threading

import rclpy
from rclpy.duration import Duration


def flush_holds(publishers):
    if rclpy.ok():
        for publisher in publishers:
            publisher.wait_for_all_acked(Duration(seconds=0.1))


def spin_until_stopped(node):
    stop = threading.Event()
    previous = {}
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.getsignal(sig)
            signal.signal(sig, lambda *_: stop.set())
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
