#!/usr/bin/env python3
"""
Check for and drain stale data from USB buffer.
Run this before/after other tests to detect leftover data.

Usage: python test_stale_data.py
       timeout 5 python test_stale_data.py  (with 5 second timeout)
"""
import sys
sys.path.insert(0, '.')

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

print("Checking for stale data...")

# Try to receive data - will return early if nothing available
CHUNK_SIZE = 16384
total_stale = 0
chunks = 0

while True:
    data = usb.recv(CHUNK_SIZE)
    if len(data) == 0:
        break
    total_stale += len(data)
    chunks += 1
    print(f"  Chunk {chunks}: {len(data)} bytes")

usb.close()

if total_stale > 0:
    print(f"\nWARNING: Drained {total_stale} bytes of stale data in {chunks} chunks!")
    sys.exit(1)
else:
    print("\nNo stale data found (good)")
    sys.exit(0)
