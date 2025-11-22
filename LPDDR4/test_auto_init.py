#!/usr/bin/env python3
"""
Test DDR auto-initialization using cfg_sel=0 (built-in configuration)
"""

from usb_ddr_control import USBDDRControl
import sys

def main():
    print("=" * 60)
    print("DDR Auto-Initialization Test")
    print("=" * 60)

    usb = USBDDRControl()

    try:
        # Test auto-init
        if usb.ddr_auto_init(timeout=10.0):
            print("\n✓ DDR initialization SUCCEEDED!")
            print("\nNow testing memory access...")

            # Try a simple memory test
            result = usb.memtest_run(size_mb=4, verbose=True)

            if result:
                print("\n✓✓ Memory test PASSED! DDR is working!")
                return 0
            else:
                print("\n✗ Memory test FAILED")
                return 1
        else:
            print("\n✗ DDR initialization FAILED")
            print("\nDebugging info:")

            # Read config register
            config_reg = usb.reg_read(0x28)
            print(f"  REG_10_CONFIG = 0x{config_reg:08X}")
            print(f"    cfg_rst   (bit 0) = {(config_reg >> 0) & 0x1}")
            print(f"    cfg_sel   (bit 1) = {(config_reg >> 1) & 0x1}")
            print(f"    cfg_start (bit 2) = {(config_reg >> 2) & 0x1}")
            print(f"    cfg_done  (bit 3) = {(config_reg >> 3) & 0x1}")

            return 1

    except Exception as e:
        print(f"\n✗ Exception occurred: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        usb.close()

if __name__ == '__main__':
    sys.exit(main())
