#!/usr/bin/env python3
"""
Test different memory sizes to find the limit
"""

from usb_ddr_control import USBDDRControl
import time

def test_size(usb, size_kb, timeout=30.0):
    """Test a specific size in KB"""
    print(f"\n--- Testing {size_kb}KB ---")

    # Stop any running test
    usb.memtest_stop()
    time.sleep(0.05)

    # Configure test
    usb.reg_write(0x10, 0xDEADBEEF)  # REG_4_DATA_L
    usb.reg_write(0x14, 0xCAFEBABE)  # REG_5_DATA_H
    usb.reg_write(0x18, 0)           # REG_6_LFSR = 0 (fixed pattern)
    usb.reg_write(0x24, size_kb * 1024)  # REG_9_SIZE in bytes

    # Start test
    usb.reg_write(0x08, 0x00)
    time.sleep(0.01)
    usb.reg_write(0x08, 0x03)

    # Poll for completion
    start_time = time.time()
    while True:
        status = usb.reg_read(0x04)
        done = status & 0x1
        fail = (status >> 1) & 0x1

        if done:
            elapsed = time.time() - start_time
            if fail:
                dq_fail = usb.reg_read(0x00)
                print(f"  FAILED in {elapsed:.2f}s - DQ_FAIL=0x{dq_fail:08X}")
                return False
            else:
                print(f"  PASSED in {elapsed:.2f}s")
                return True

        if time.time() - start_time > timeout:
            print(f"  TIMEOUT after {timeout}s")
            return False

        time.sleep(0.01)

def main():
    print("=" * 60)
    print("Memory Size Test")
    print("=" * 60)

    usb = USBDDRControl()

    try:
        # Release resets
        usb.reg_write(0x0C, 0x1F)
        time.sleep(0.1)

        # Check DDR ready
        cfg_reg = usb.reg_read(0x28)
        if not ((cfg_reg >> 3) & 0x1):
            print("ERROR: DDR not initialized (cfg_done=0)")
            return

        print("DDR initialized and ready")

        # Test progressively larger sizes
        sizes = [4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096]  # KB

        for size_kb in sizes:
            if not test_size(usb, size_kb, timeout=60.0):
                print(f"\nStopped at {size_kb}KB")
                break

            # Brief pause between tests
            time.sleep(0.1)
        else:
            print("\nAll sizes passed!")

    except Exception as e:
        print(f"\nException: {e}")
        import traceback
        traceback.print_exc()

    finally:
        usb.memtest_stop()
        usb.close()

if __name__ == '__main__':
    main()
