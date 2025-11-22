#!/usr/bin/env python3
"""Test if TX breaks after a single write"""

import sys
from usb_ddr_control import USBDDRControl

def main():
    print("Opening USB device...")
    usb_ctrl = USBDDRControl()

    try:
        # Test 1: Get version (should work)
        print("\n=== Test 1: Get version BEFORE write ===")
        version = usb_ctrl.get_version()
        print(f"OK - Version: 0x{version:08X}")

        # Test 2: Do a single write
        print("\n=== Test 2: Single register write ===")
        usb_ctrl.reg_write(0x00, 0xDEADBEEF)
        print("OK - Write completed")

        # Test 3: Try to get version again (this is where it hangs)
        print("\n=== Test 3: Get version AFTER write ===")
        print("Attempting to read version (timeout=5s)...")
        version = usb_ctrl.get_version()
        print(f"OK - Version: 0x{version:08X}")

        print("\n*** ALL TESTS PASSED - TX works after write! ***")

    except Exception as e:
        print(f"\n*** TEST FAILED: {e} ***")
        return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())
