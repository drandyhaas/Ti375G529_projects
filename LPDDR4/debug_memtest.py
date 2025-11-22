#!/usr/bin/env python3
"""
Debug script to check memory test control signals
"""

from usb_ddr_control import USBDDRControl
import time

usb = USBDDRControl()

try:
    print("=" * 70)
    print("Memory Test Debug")
    print("=" * 70)

    # Check initial state
    print("\n1. Initial register state:")
    cfg = usb.reg_read(0x28)
    print(f"   REG_10_CONFIG   = 0x{cfg:08x} (cfg_done={(cfg>>3)&1})")

    ctrl = usb.reg_read(0x08)
    print(f"   REG_2_CONTROL   = 0x{ctrl:08x} (start={ctrl&1}, rstn={(ctrl>>1)&1})")

    status = usb.reg_read(0x04)
    print(f"   REG_1_STATUS    = 0x{status:08x} (done={status&1}, fail={(status>>1)&1})")

    size = usb.reg_read(0x24)
    print(f"   REG_9_SIZE      = 0x{size:08x} ({size} bytes)")

    lfsr = usb.reg_read(0x18)
    print(f"   REG_6_LFSR      = 0x{lfsr:08x}")

    # Stop any running test
    print("\n2. Stopping any running test...")
    usb.reg_write(0x08, 0x00)  # Clear both start and rstn
    time.sleep(0.01)

    ctrl = usb.reg_read(0x08)
    print(f"   REG_2_CONTROL after stop = 0x{ctrl:08x}")

    # Release reset
    print("\n3. Releasing reset (rstn=1, start=0)...")
    usb.reg_write(0x08, 0x02)  # rstn=1, start=0
    time.sleep(0.01)

    ctrl = usb.reg_read(0x08)
    print(f"   REG_2_CONTROL after reset = 0x{ctrl:08x}")

    status = usb.reg_read(0x04)
    print(f"   REG_1_STATUS = 0x{status:08x} (done={status&1}, fail={(status>>1)&1})")

    # Configure test
    print("\n4. Configuring test (1MB, LFSR)...")
    test_size = 1 * 1024 * 1024
    usb.reg_write(0x24, test_size)  # 1MB
    usb.reg_write(0x18, 1)           # LFSR enable
    time.sleep(0.01)

    size = usb.reg_read(0x24)
    lfsr = usb.reg_read(0x18)
    print(f"   REG_9_SIZE = 0x{size:08x} ({size} bytes)")
    print(f"   REG_6_LFSR = 0x{lfsr:08x}")

    # Start test
    print("\n5. Starting test (rstn=1, start=1)...")
    usb.reg_write(0x08, 0x03)  # rstn=1, start=1
    time.sleep(0.01)

    ctrl = usb.reg_read(0x08)
    print(f"   REG_2_CONTROL = 0x{ctrl:08x}")

    # Monitor for a bit
    print("\n6. Monitoring test progress...")
    for i in range(10):
        status = usb.reg_read(0x04)
        done = status & 1
        fail = (status >> 1) & 1
        print(f"   [{i}] status=0x{status:08x} done={done} fail={fail}")

        if done:
            print(f"\n   Test completed! (fail={fail})")
            if fail:
                dq_fail = usb.reg_read(0x00)
                print(f"   DQ_FAIL = 0x{dq_fail:08x}")
            break

        time.sleep(0.1)
    else:
        print("\n   Test still running after 1 second")

    print("\n" + "=" * 70)

except Exception as e:
    print(f"\nException: {e}")
    import traceback
    traceback.print_exc()

finally:
    usb.close()
