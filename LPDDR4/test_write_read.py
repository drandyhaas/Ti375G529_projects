#!/usr/bin/env python3
# Test register write then read
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
import time

CMD_REG_WRITE = 0x02
CMD_REG_READ = 0x03

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

addr = 0x0044  # REG_17_TESTER_PAT
test_value = 0xDEADBEEF

print(f"\n=== Writing 0x{test_value:08X} to register 0x{addr:04X} ===")
txdata = bytes([CMD_REG_WRITE,
                addr & 0xFF, (addr >> 8) & 0xFF,
                (addr >> 16) & 0xFF, (addr >> 24) & 0xFF,
                test_value & 0xFF, (test_value >> 8) & 0xFF,
                (test_value >> 16) & 0xFF, (test_value >> 24) & 0xFF])
print(f"TX: {txdata.hex()}")
usb.send(txdata)
time.sleep(0.1)

print(f"\n=== Reading from register 0x{addr:04X} ===")
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
    print(f"\nRead value: 0x{value:08X}")
    if value == test_value:
        print("SUCCESS! Read value matches written value!")
    else:
        print(f"MISMATCH! Expected 0x{test_value:08X}, got 0x{value:08X}")
else:
    print(f"\nFAILED - Expected 4 bytes, got {len(rxdata)}")

usb.close()
print("\nDone!")
