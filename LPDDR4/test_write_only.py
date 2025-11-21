# Test register write and verify with LED or other observable output
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
import time

CMD_REG_WRITE = 0x02

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(('FT60X', 'Haasoscope USB3'),
                                                           ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

print("\n=== Testing REG_WRITE Command ===")

# Write different patterns to REG_17_TESTER_PAT (0x44)
for i in range(5):
    value = 0x11111111 * (i + 1)
    print(f"Writing 0x{value:08X} to register 0x0044...")

    txdata = bytes([
        CMD_REG_WRITE,
        0x44, 0x00, 0x00, 0x00,  # address
        value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF  # data
    ])

    print(f"  TX: {txdata.hex()}")
    usb.send(txdata)
    time.sleep(0.2)

print("\nWrites completed. If writes are working, the register should contain 0x55555555 now.")
print("(No way to verify without working reads though!)")

usb.close()
