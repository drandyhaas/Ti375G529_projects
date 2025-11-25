#!/usr/bin/env python3
"""
Check for and drain stale data from USB buffer.
Uses internal timeout to avoid hanging if no data available.

Usage: python test_stale_data.py [timeout_seconds]
       Default timeout is 3 seconds.
"""
import sys
import threading
import time

sys.path.insert(0, '.')

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode

# Parse timeout argument
TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0

print(f"Opening USB device (timeout={TIMEOUT}s)...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

print("Checking for stale data...")

CHUNK_SIZE = 16384
total_stale = 0
chunks = 0
stop_flag = threading.Event()
recv_done = threading.Event()


def drain_thread():
    """Thread that drains data until empty or stopped."""
    global total_stale, chunks
    try:
        while not stop_flag.is_set():
            data = usb.recv(CHUNK_SIZE)
            if len(data) == 0:
                break
            total_stale += len(data)
            chunks += 1
            if chunks <= 3 or chunks % 100 == 0:
                print(f"  Chunk {chunks}: {len(data)} bytes (total: {total_stale:,} bytes)")
    except Exception as e:
        print(f"  Recv error: {e}")
    finally:
        recv_done.set()


# Start drain thread
t = threading.Thread(target=drain_thread, daemon=True)
t.start()

# Wait for completion or timeout
start_time = time.time()
while not recv_done.is_set():
    elapsed = time.time() - start_time
    if elapsed > TIMEOUT:
        print(f"  Timeout after {TIMEOUT}s (recv blocked - no data available)")
        stop_flag.set()
        break
    time.sleep(0.1)

# Give thread a moment to finish
t.join(timeout=0.5)

# Close USB
try:
    usb.close()
except:
    pass

# Report results
if total_stale > 0:
    print(f"\nWARNING: Drained {total_stale:,} bytes ({total_stale/1024/1024:.1f} MB) of stale data in {chunks} chunks!")
    sys.exit(1)
else:
    print("\nNo stale data found (recv blocked = buffer empty)")
    sys.exit(0)
