# Simple USB test to verify firmware is responding
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode

# Command codes
CMD_TX_MASS   = 0x01
CMD_REG_WRITE = 0x02
CMD_REG_READ  = 0x03

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(('FT60X', 'Haasoscope USB3'),
                                                           ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

# Test 1: TX_MASS command (should work with any firmware version)
print("\n=== Test 1: TX_MASS Command ===")
print("Sending TX_MASS command for 16 bytes...")
expect_len = 16
txdata = bytes([CMD_TX_MASS,
                expect_len & 0xff, (expect_len >> 8) & 0xff,
                (expect_len >> 16) & 0xff, (expect_len >> 24) & 0xff])
print(f"TX: {txdata.hex()}")
usb.send(txdata)

rxdata = usb.recv(expect_len)
print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")
if len(rxdata) == expect_len:
    print("✓ TX_MASS test PASSED")
else:
    print(f"✗ TX_MASS test FAILED (expected {expect_len} bytes, got {len(rxdata)})")

# Test 2: REG_READ command (requires new firmware)
print("\n=== Test 2: REG_READ Command ===")
print("Sending REG_READ command for register 0x0044...")
txdata = bytes([CMD_REG_READ, 0x44, 0x00, 0x00, 0x00])
print(f"TX: {txdata.hex()}")
usb.send(txdata)

import time
time.sleep(0.1)  # Give firmware time to respond

rxdata = usb.recv(4)
print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")
if len(rxdata) == 4:
    value = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    print(f"✓ REG_READ test PASSED - Value: 0x{value:08X}")
else:
    print(f"✗ REG_READ test FAILED (expected 4 bytes, got {len(rxdata)})")
    print("  This indicates the firmware may not have the updated usb_command_handler")

usb.close()
print("\nDone!")
