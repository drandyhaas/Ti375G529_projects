#!/usr/bin/env python3
# Test firmware version command
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode

CMD_PREFIX = 0xFE
CMD_GET_VERSION = 0x04

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

print("\n=== Testing GET_VERSION Command (0xFE 0x04) ===")
print("Sending GET_VERSION command...")
# Just prefix + command, no padding needed
txdata = bytes([CMD_PREFIX, CMD_GET_VERSION])
print(f"TX: {txdata.hex()}")
usb.send(txdata)

import time
time.sleep(0.1)

print("Receiving 4-byte version...")
rxdata = usb.recv(4)
print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")

if len(rxdata) == 4:
    version = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    print(f"\n✓ Firmware Version: 0x{version:08X}")

    if version == 0x20250120:
        print("✓ CORRECT! This is the updated firmware from 2025-01-20")
    else:
        print(f"✗ UNEXPECTED version (expected 0x20250120)")
        print("  This suggests an old firmware is loaded or build didn't include changes")
else:
    print(f"✗ FAILED - Expected 4 bytes, got {len(rxdata)}")
    print("  The command may not be recognized (old firmware?)")

usb.close()
print("\nDone!")
