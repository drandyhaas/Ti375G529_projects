#!/usr/bin/env python3
# Test firmware status command
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode

CMD_GET_STATUS = 0x05

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

print("\n=== Testing GET_STATUS Command (0x05) ===")
print("Sending GET_STATUS command...")
txdata = bytes([CMD_GET_STATUS])
print(f"TX: {txdata.hex()}")
usb.send(txdata)

import time
time.sleep(0.1)

print("Receiving 4-byte status...")
rxdata = usb.recv(4)
print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")

if len(rxdata) == 4:
    status = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    print(f"\nStatus: 0x{status:08X}")
    print(f"  bit[0] ddr_pll_lock : {status & 0x01}")
    print(f"  bit[1] axi_arready  : {(status >> 1) & 0x01}")
    print(f"  bit[2] axi_rvalid   : {(status >> 2) & 0x01}")

    if status & 0x01:
        print("\nDDR PLL is LOCKED - good!")
    else:
        print("\nDDR PLL is NOT LOCKED - this is why AXI doesn't work!")
else:
    print(f"\nFAILED - Expected 4 bytes, got {len(rxdata)}")

usb.close()
print("\nDone!")
