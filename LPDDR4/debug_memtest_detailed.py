#!/usr/bin/env python3
"""
Detailed memtest debugging - check what's preventing memtest from running
"""

from usb_ddr_control import USBDDRControl
import time

def main():
    print("=" * 60)
    print("Detailed Memory Test Debug")
    print("=" * 60)

    usb = USBDDRControl()

    try:
        # First, release all resets manually
        print("\n--- Releasing All Resets ---")
        usb.reg_write(0x0C, 0x1F)  # REG_3_RESET - release all
        time.sleep(0.1)

        reset_reg = usb.reg_read(0x0C)
        print(f"REG_3_RESET = 0x{reset_reg:08X} (expected 0x1F)")

        if reset_reg != 0x1F:
            print("WARNING: Resets not properly released!")
            print("This means phy_rstn/ctrl_rstn are being driven by hardware")
            print("and REG_3 writes have no effect (which is correct for auto-init)")

        # Check cfg_done
        print("\n--- DDR Init Status ---")
        cfg_reg = usb.reg_read(0x28)
        cfg_done = (cfg_reg >> 3) & 0x1
        print(f"cfg_done = {cfg_done}")

        if not cfg_done:
            print("ERROR: DDR not initialized!")
            return

        # Stop any running test
        print("\n--- Stopping Any Running Test ---")
        usb.memtest_stop()
        time.sleep(0.1)

        # Configure a tiny test - just 4KB
        print("\n--- Configuring Minimal Test (4KB) ---")
        usb.reg_write(0x10, 0xDEADBEEF)  # REG_4_DATA_L
        usb.reg_write(0x14, 0xCAFEBABE)  # REG_5_DATA_H
        usb.reg_write(0x18, 0)           # REG_6_LFSR = 0 (use fixed pattern)
        usb.reg_write(0x24, 4096)        # REG_9_SIZE = 4KB

        # Start test
        print("Starting test...")
        usb.reg_write(0x08, 0x00)  # Clear control
        time.sleep(0.01)
        usb.reg_write(0x08, 0x03)  # Set start=1, rstn=1

        # Monitor status closely
        print("\nMonitoring test progress (20 iterations, 100ms each):")
        for i in range(20):
            status = usb.reg_read(0x04)  # REG_1_STATUS
            ctrl = usb.reg_read(0x08)    # REG_2_CONTROL

            done = status & 0x1
            fail = (status >> 1) & 0x1
            start = ctrl & 0x1
            rstn = (ctrl >> 1) & 0x1

            print(f"  [{i:2d}] status=0x{status:08X} ctrl=0x{ctrl:08X} "
                  f"done={done} fail={fail} start={start} rstn={rstn}")

            if done:
                print(f"\nTest completed!")
                if fail:
                    dq_fail = usb.reg_read(0x00)
                    print(f"  FAILED - DQ_FAIL=0x{dq_fail:08X}")
                else:
                    print(f"  PASSED!")
                break

            time.sleep(0.1)
        else:
            print("\nTest did not complete after 2 seconds")

            # Check AXI signals
            print("\n--- Checking AXI Interface ---")
            print("Reading tester status registers...")

            try:
                tester_stat = usb.reg_read(0x3C)  # REG_15_TESTER_STAT
                print(f"REG_15_TESTER_STAT = 0x{tester_stat:08X}")
                print(f"  tester_loop_done = {(tester_stat >> 0) & 0x1}")
                print(f"  tester_error     = {(tester_stat >> 1) & 0x1}")
            except:
                print("  (Could not read tester status)")

            # Try reading/writing a simple register to verify USB works
            print("\n--- USB Communication Test ---")
            test_val = 0x12345678
            usb.reg_write(0x44, test_val)  # REG_17_TESTER_PAT
            read_back = usb.reg_read(0x44)
            if read_back == test_val:
                print(f"USB OK - wrote 0x{test_val:08X}, read 0x{read_back:08X}")
            else:
                print(f"USB ISSUE - wrote 0x{test_val:08X}, read 0x{read_back:08X}")

        # Stop test
        usb.memtest_stop()

        print("\n--- Summary ---")
        print("If test is stuck at done=0:")
        print("  1. Check if AXI0 clock is running")
        print("  2. Check if axi0_ARESETn is released (should be 1)")
        print("  3. Check if DDR controller CTRL_MEM_RST_VALID is high")
        print("  4. Verify memory_checker_lfsr is not stuck in reset")

    except Exception as e:
        print(f"\nException: {e}")
        import traceback
        traceback.print_exc()

    finally:
        usb.close()

if __name__ == '__main__':
    main()
