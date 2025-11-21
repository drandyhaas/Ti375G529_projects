#!/usr/bin/env python3
# Simple register read test
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
import time

CMD_REG_READ = 0x03

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

print("\n=== Testing REG_READ Command (0x03) ===")
addr = 0x0044  # REG_17_TESTER_PAT

print(f"Sending REG_READ command for address 0x{addr:04X}...")
txdata = bytes([CMD_REG_READ,
                addr & 0xFF, (addr >> 8) & 0xFF,
                (addr >> 16) & 0xFF, (addr >> 24) & 0xFF])
print(f"TX: {txdata.hex()}")
usb.send(txdata)

time.sleep(0.1)

print("Receiving 4-byte response...")
rxdata = usb.recv(4)
print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")

if len(rxdata) == 4:
    value = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    print(f"\nRegister Read SUCCESS! Value: 0x{value:08X}")
else:
    print(f"\nRegister Read FAILED - Expected 4 bytes, got {len(rxdata)}")

usb.close()
print("\nDone!")
