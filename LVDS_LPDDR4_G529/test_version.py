#!/usr/bin/env python3
# Test firmware version command
# Updated: Now uses 8-byte command format with 0x23 command code
#          (previously used 0xFE 0x04 prefix format)
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode

# New command codes (consolidated into command_processor)
CMD_GET_VERSION = 0x23  # Was 0xFE 0x04

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

print("\n=== Testing GET_VERSION Command (0x23) ===")
print("Sending GET_VERSION command (8 bytes)...")
# Now uses 8-byte command format like other scope commands
txdata = bytes([CMD_GET_VERSION, 0, 0, 0, 0, 0, 0, 0])
print(f"TX: {txdata.hex()}")
usb.send(txdata)

import time
time.sleep(0.1)

print("Receiving 4-byte version...")
rxdata = usb.recv(4)
print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")

if len(rxdata) == 4:
    version = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    print(f"Firmware Version: 0x{version:08X}")
else:
    print(f"FAILED - Expected 4 bytes, got {len(rxdata)}")

usb.close()
print("Done!")
