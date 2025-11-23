#!/usr/bin/env python3
"""
Test DDR after hardware auto-initialization

This script assumes the FPGA has been modified to auto-initialize
the DDR controller on power-up using the cfg_* signals tied in hardware.

The DDR should initialize automatically when the FPGA configures, so
this script just waits a moment then checks if it's ready.
"""

from usb_ddr_control import USBDDRControl
import time
import sys

def main():
    print("=" * 60)
    print("Testing DDR After Hardware Auto-Init")
    print("=" * 60)

    usb = USBDDRControl()

    try:
        print("Waiting for DDR auto-init to complete (1 second)...")
        time.sleep(1.0)  # Give DDR time to initialize

        # Check if initialization completed
        cfg_reg = usb.reg_read(0x28)  # REG_10_CONFIG
        cfg_done = (cfg_reg >> 3) & 0x1

        print(f"\nREG_10_CONFIG = 0x{cfg_reg:08X}")
        print(f"  cfg_rst   (bit 0) = {(cfg_reg >> 0) & 0x1}")
        print(f"  cfg_sel   (bit 1) = {(cfg_reg >> 1) & 0x1}")
        print(f"  cfg_start (bit 2) = {(cfg_reg >> 2) & 0x1}")
        print(f"  cfg_done  (bit 3) = {cfg_done}")

        if cfg_done:
            print("\n+ DDR initialization complete!")
            print("\nRunning memory test...")
            result = usb.memtest_run(size_mb=4, verbose=True)

            if result:
                print("\n++ Memory test PASSED! DDR is working!")
                return 0
            else:
                print("\nX Memory test FAILED")
                return 1
        else:
            print("\n! DDR not ready yet, waiting longer...")
            print("Waiting additional 2 seconds...")
            time.sleep(2.0)

            # Check again
            cfg_reg = usb.reg_read(0x28)
            cfg_done = (cfg_reg >> 3) & 0x1
            print(f"cfg_done = {cfg_done}")

            if cfg_done:
                print("\n+ DDR initialization complete (took longer)!")
                print("\nRunning memory test...")
                result = usb.memtest_run(size_mb=4, verbose=True)

                if result:
                    print("\n++ Memory test PASSED! DDR is working!")
                    return 0
                else:
                    print("\nX Memory test FAILED")
                    return 1
            else:
                print("\nX DDR initialization failed or taking too long")
                return 1

    except Exception as e:
        print(f"\nX Exception occurred: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        usb.close()

if __name__ == '__main__':
    sys.exit(main())
