#!/usr/bin/env python3
"""
Debug DDR controller status and AXI interface
"""

from usb_ddr_control import USBDDRControl
import time

def main():
    print("=" * 60)
    print("DDR Controller Status Debug")
    print("=" * 60)

    usb = USBDDRControl()

    try:
        # Check config status
        print("\n--- Configuration Status ---")
        cfg_reg = usb.reg_read(0x28)  # REG_10_CONFIG
        print(f"REG_10_CONFIG = 0x{cfg_reg:08X}")
        print(f"  cfg_rst   = {(cfg_reg >> 0) & 0x1}")
        print(f"  cfg_sel   = {(cfg_reg >> 1) & 0x1}")
        print(f"  cfg_start = {(cfg_reg >> 2) & 0x1}")
        print(f"  cfg_done  = {(cfg_reg >> 3) & 0x1}")

        # Check reset status
        print("\n--- Reset Status ---")
        reset_reg = usb.reg_read(0x0C)  # REG_3_RESET
        print(f"REG_3_RESET = 0x{reset_reg:08X}")
        print(f"  phy_rstn     = {(reset_reg >> 0) & 0x1}")
        print(f"  ctrl_rstn    = {(reset_reg >> 1) & 0x1}")
        print(f"  reg_axi_rstn = {(reset_reg >> 2) & 0x1}")
        print(f"  axi0_rstn    = {(reset_reg >> 3) & 0x1}")
        print(f"  axi1_rstn    = {(reset_reg >> 4) & 0x1}")

        # Check memtest control
        print("\n--- Memory Test Control ---")
        ctrl_reg = usb.reg_read(0x08)  # REG_2_CONTROL
        print(f"REG_2_CONTROL = 0x{ctrl_reg:08X}")
        print(f"  memtest_start = {(ctrl_reg >> 0) & 0x1}")
        print(f"  memtest_rstn  = {(ctrl_reg >> 1) & 0x1}")

        # Check memtest status
        status_reg = usb.reg_read(0x04)  # REG_1_STATUS
        print(f"REG_1_STATUS = 0x{status_reg:08X}")
        print(f"  memtest_done = {(status_reg >> 0) & 0x1}")
        print(f"  memtest_fail = {(status_reg >> 1) & 0x1}")

        # Try to read DQ fail register
        print("\n--- DQ Fail Status ---")
        dq_fail = usb.reg_read(0x00)  # REG_0_DQ_FAIL
        print(f"REG_0_DQ_FAIL = 0x{dq_fail:08X}")

        # Check if resets are properly released
        print("\n--- Analysis ---")
        if reset_reg == 0x1F:
            print("✓ All resets properly released (0x1F)")
        else:
            print(f"⚠ Reset issue detected - expected 0x1F, got 0x{reset_reg:08X}")
            print("  Releasing all resets...")
            usb.reg_write(0x0C, 0x1F)
            time.sleep(0.1)
            reset_reg = usb.reg_read(0x0C)
            print(f"  After reset: REG_3_RESET = 0x{reset_reg:08X}")

        # Try a simple write/read test to a single address
        print("\n--- Simple AXI Write/Read Test ---")
        print("Attempting to write/read pattern to DDR via memtest...")

        # Configure a very small test
        usb.memtest_data(0xDEADBEEFCAFEBABE)
        usb.memtest_size(1)  # Just 1MB

        print("Starting test...")
        usb.memtest_restart(lfsr_en=False)

        # Poll for a bit
        for i in range(50):
            status = usb.reg_read(0x04)
            done = status & 0x1
            fail = (status >> 1) & 0x1

            if done:
                print(f"✓ Test completed after {i*0.1:.1f}s")
                if fail:
                    print("  Result: FAILED")
                    dq_fail = usb.reg_read(0x00)
                    print(f"  DQ_FAIL = 0x{dq_fail:08X}")
                else:
                    print("  Result: PASSED")
                break

            time.sleep(0.1)
        else:
            print(f"✗ Test did not complete after 5s")
            print("  Checking if test is stuck...")

            # Check tester status
            tester_stat = usb.reg_read(0x3C)  # REG_15_TESTER_STAT
            print(f"  REG_15_TESTER_STAT = 0x{tester_stat:08X}")

        # Stop test
        usb.memtest_stop()

    except Exception as e:
        print(f"\n✗ Exception: {e}")
        import traceback
        traceback.print_exc()

    finally:
        usb.close()

if __name__ == '__main__':
    main()
