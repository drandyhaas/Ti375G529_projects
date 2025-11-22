#!/usr/bin/env python3
"""Test how many writes it takes before TX breaks"""

import sys
from usb_ddr_control import USBDDRControl

def main():
    print("Opening USB device...")
    usb_ctrl = USBDDRControl()

    try:
        # Test version before any writes
        print("\n=== Get version BEFORE writes ===")
        version = usb_ctrl.get_version()
        print(f"OK - Version: 0x{version:08X}")

        # Do many writes, testing version after every 10 writes
        num_writes = 100
        check_interval = 1  # Check after EVERY write to find exact failure point

        for i in range(num_writes):
            # Do a write to DDR CTL register (addr >= 0x80)
            # CTL base is 0x80, so writing to CTL register 0
            usb_ctrl.lpddr4_ctrl_write('CTL', 0, 0xDEAD0000 | i)

            # Add small delay between writes
            import time
            time.sleep(0.01)  # 10ms delay

            # Check version after every N writes
            if (i + 1) % check_interval == 0:
                print(f"\n=== After {i+1} writes ===")
                try:
                    version = usb_ctrl.get_version()
                    print(f"OK - Version still works: 0x{version:08X}")
                except Exception as e:
                    print(f"*** TX BROKEN after {i+1} writes! ***")
                    print(f"Error: {e}")
                    return 1

        print(f"\n*** ALL {num_writes} WRITES COMPLETED - TX still works! ***")
        return 0

    except Exception as e:
        print(f"\n*** TEST FAILED: {e} ***")
        return 1

if __name__ == "__main__":
    sys.exit(main())
