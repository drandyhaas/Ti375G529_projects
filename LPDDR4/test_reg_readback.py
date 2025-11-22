#!/usr/bin/env python3
"""
Test register write/readback to verify register interface is working
"""

from usb_ddr_control import USBDDRControl
import time

usb = USBDDRControl()

try:
    print("Testing register write/readback...")
    print("=" * 70)

    # Test writing and reading back various registers
    test_values = [
        (0x08, 0x00000000, "REG_2_CONTROL"),
        (0x08, 0x00000001, "REG_2_CONTROL"),
        (0x08, 0x00000002, "REG_2_CONTROL"),
        (0x08, 0x00000003, "REG_2_CONTROL"),
        (0x18, 0x00000000, "REG_6_LFSR"),
        (0x18, 0x00000001, "REG_6_LFSR"),
        (0x24, 0x00100000, "REG_9_SIZE (1MB)"),
        (0x24, 0x00400000, "REG_9_SIZE (4MB)"),
    ]

    for addr, value, name in test_values:
        usb.reg_write(addr, value)
        time.sleep(0.001)
        readback = usb.reg_read(addr)
        match = "PASS" if readback == value else "FAIL"
        print(f"{match}: Write 0x{value:08x} to {name} (0x{addr:02x}), read back 0x{readback:08x}")

    print("\n" + "=" * 70)
    print("Register interface test complete")

except Exception as e:
    print(f"\nException: {e}")
    import traceback
    traceback.print_exc()

finally:
    usb.close()
