#!/usr/bin/env python3
"""
USB utility functions for FTDI device communication.
"""
import threading

def recv_with_timeout(usb, length, timeout=2.0):
    """
    Receive data with timeout using a thread (in case library blocks).

    Args:
        usb: USB device object with recv() method
        length: Number of bytes to receive
        timeout: Timeout in seconds (default 2.0)

    Returns:
        bytes: Received data, or b'' if timeout
    """
    result = [b'']

    def do_recv():
        result[0] = usb.recv(length)

    thread = threading.Thread(target=do_recv)
    thread.start()
    thread.join(timeout=timeout)

    if thread.is_alive():
        # Thread still running - recv is blocked
        return result[0] if result[0] else b''
    return result[0]
