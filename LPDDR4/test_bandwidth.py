#!/usr/bin/env python3
"""
DDR bandwidth test - measure read/write throughput

Target: ~80 Gb/s = 10 GB/s
To test for 1 second, we need to transfer ~10 GB of data
"""

from usb_ddr_control import USBDDRControl
import time
import sys

def test_bandwidth(usb, size_mb, pattern_type='lfsr'):
    """
    Run bandwidth test

    Args:
        usb: USBDDRControl instance
        size_mb: Amount of memory to test in MB
        pattern_type: 'lfsr' or 'fixed'

    Returns:
        (passed, elapsed_time, bandwidth_gbps)
    """
    print(f"\n--- Testing {size_mb}MB ({pattern_type} pattern) ---")

    # Stop any running test
    usb.memtest_stop()
    time.sleep(0.05)

    # Configure test
    if pattern_type == 'fixed':
        usb.reg_write(0x10, 0xDEADBEEF)  # REG_4_DATA_L
        usb.reg_write(0x14, 0xCAFEBABE)  # REG_5_DATA_H
        usb.reg_write(0x18, 0)           # REG_6_LFSR = 0 (fixed pattern)
    else:
        usb.reg_write(0x18, 1)           # REG_6_LFSR = 1 (LFSR pattern)

    # Set size
    size_bytes = size_mb * 1024 * 1024
    usb.reg_write(0x24, size_bytes)  # REG_9_SIZE

    # Start test
    usb.reg_write(0x08, 0x00)
    time.sleep(0.01)

    start_time = time.time()
    usb.reg_write(0x08, 0x03)  # Start test

    # Poll for completion
    while True:
        status = usb.reg_read(0x04)
        done = status & 0x1
        fail = (status >> 1) & 0x1

        if done:
            elapsed = time.time() - start_time

            if fail:
                dq_fail = usb.reg_read(0x00)
                print(f"  FAILED - DQ_FAIL=0x{dq_fail:08X}")
                return False, elapsed, 0.0
            else:
                # Calculate bandwidth
                # Memory test does: write entire buffer, then read entire buffer
                # So total data transferred = 2 * size_bytes
                total_bytes = 2 * size_bytes
                bandwidth_bps = total_bytes * 8 / elapsed  # bits per second
                bandwidth_gbps = bandwidth_bps / 1e9       # Gbps

                print(f"  PASSED in {elapsed:.3f}s")
                print(f"  Data transferred: {total_bytes / 1e6:.1f} MB")
                print(f"  Bandwidth: {bandwidth_gbps:.2f} Gb/s ({total_bytes / elapsed / 1e6:.1f} MB/s)")

                return True, elapsed, bandwidth_gbps

        # Safety timeout
        if time.time() - start_time > 120.0:
            print(f"  TIMEOUT after 120s")
            return False, 120.0, 0.0

        time.sleep(0.001)  # Poll every 1ms

def main():
    print("=" * 60)
    print("DDR Bandwidth Test")
    print("=" * 60)
    print("Expected bandwidth: ~80 Gb/s (10 GB/s)")
    print("Test performs: WRITE all data, then READ all data")
    print("=" * 60)

    usb = USBDDRControl()

    try:
        # Check DDR ready
        cfg_reg = usb.reg_read(0x28)
        if not ((cfg_reg >> 3) & 0x1):
            print("ERROR: DDR not initialized (cfg_done=0)")
            return 1

        print("DDR initialized and ready\n")

        # Test progressively larger sizes to measure bandwidth
        test_sizes = [
            16,      # 16 MB
            64,      # 64 MB
            256,     # 256 MB
            512,     # 512 MB
            1024,    # 1 GB
            2048,    # 2 GB (if this works and is fast enough)
        ]

        results = []

        for size_mb in test_sizes:
            passed, elapsed, bandwidth = test_bandwidth(usb, size_mb, pattern_type='lfsr')

            if not passed:
                print(f"\nStopped at {size_mb}MB due to failure/timeout")
                break

            results.append((size_mb, elapsed, bandwidth))

            # If test took more than 2 seconds, we have enough data
            if elapsed > 2.0:
                print(f"\nTest duration sufficient ({elapsed:.1f}s), stopping")
                break

        # Summary
        print("\n" + "=" * 60)
        print("SUMMARY")
        print("=" * 60)
        print(f"{'Size':>10} | {'Time':>8} | {'Bandwidth':>12} | {'MB/s':>10}")
        print("-" * 60)

        for size_mb, elapsed, bandwidth in results:
            mb_per_sec = (size_mb * 2) / elapsed  # 2x because write+read
            print(f"{size_mb:>8}MB | {elapsed:>6.3f}s | {bandwidth:>9.2f} Gb/s | {mb_per_sec:>8.1f}")

        if results:
            # Average bandwidth
            avg_bandwidth = sum(r[2] for r in results) / len(results)
            print("-" * 60)
            print(f"Average bandwidth: {avg_bandwidth:.2f} Gb/s")

            if avg_bandwidth < 1.0:
                print("\nWARNING: Bandwidth much lower than expected (80 Gb/s)")
                print("This might be limited by:")
                print("  1. AXI clock frequency")
                print("  2. Memory checker logic (not DDR itself)")
                print("  3. Single AXI port usage (should use both AXI0 and AXI1)")
            elif avg_bandwidth < 10.0:
                print(f"\nBandwidth is {avg_bandwidth:.1f} Gb/s")
                print("Note: This is the memory_checker_lfsr throughput,")
                print("not the full DDR controller bandwidth.")
            else:
                print(f"\nExcellent! Achieving {avg_bandwidth:.1f} Gb/s")

        return 0

    except Exception as e:
        print(f"\nException: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        usb.memtest_stop()
        usb.close()

if __name__ == '__main__':
    sys.exit(main())
