#!/usr/bin/env python3
# Test TX_MASS first, then REG_READ
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
import time

CMD_TX_MASS = 0x01
CMD_REG_READ = 0x03

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

print("\n=== Step 1: TX_MASS (16 bytes) ===")
length = 16
txdata = bytes([CMD_TX_MASS,
                length & 0xff, (length >> 8) & 0xff,
                (length >> 16) & 0xff, (length >> 24) & 0xff])
print(f"TX: {txdata.hex()}")
usb.send(txdata)
rxdata = usb.recv(length)
print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")
print("TX_MASS OK\n")

print("=== Step 2: REG_READ from 0x0044 ===")
addr = 0x0044
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
    print(f"\nSUCCESS! Register value: 0x{value:08X}")
else:
    print(f"\nFAILED - Expected 4 bytes, got {len(rxdata)}")

usb.close()
print("\nDone!")
