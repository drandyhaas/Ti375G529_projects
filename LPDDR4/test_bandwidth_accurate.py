#!/usr/bin/env python3
"""
Accurate DDR bandwidth test - removes Python/USB overhead

Strategy:
1. Run small test (1MB) to measure overhead
2. Run large test (512MB) multiple times
3. Subtract overhead to get true DDR performance
"""

from usb_ddr_control import USBDDRControl
import time
import sys

def run_test(usb, size_mb, pattern_type='lfsr'):
    """
    Run a single memory test and measure time

    Returns:
        (passed, elapsed_time)
    """
    # Stop any running test
    usb.memtest_stop()
    time.sleep(0.01)

    # Configure test
    if pattern_type == 'fixed':
        usb.reg_write(0x10, 0xDEADBEEF)  # REG_4_DATA_L
        usb.reg_write(0x14, 0xCAFEBABE)  # REG_5_DATA_H
        usb.reg_write(0x18, 0)           # REG_6_LFSR = 0
    else:
        usb.reg_write(0x18, 1)           # REG_6_LFSR = 1

    # Set size
    size_bytes = size_mb * 1024 * 1024
    usb.reg_write(0x24, size_bytes)  # REG_9_SIZE

    # Start test
    usb.reg_write(0x08, 0x00)
    time.sleep(0.005)

    start_time = time.time()
    usb.reg_write(0x08, 0x03)  # Start test

    # Poll for completion with minimal overhead
    poll_count = 0
    while True:
        status = usb.reg_read(0x04)
        done = status & 0x1
        fail = (status >> 1) & 0x1
        poll_count += 1

        if done:
            elapsed = time.time() - start_time

            if fail:
                dq_fail = usb.reg_read(0x00)
                return False, elapsed, poll_count
            else:
                return True, elapsed, poll_count

        # Safety timeout
        if time.time() - start_time > 120.0:
            return False, 120.0, poll_count

        time.sleep(0.0001)  # 100us poll interval

def main():
    print("=" * 70)
    print("DDR Bandwidth Test - Overhead Compensated")
    print("=" * 70)

    usb = USBDDRControl()

    try:
        # Check DDR ready
        cfg_reg = usb.reg_read(0x28)
        if not ((cfg_reg >> 3) & 0x1):
            print("ERROR: DDR not initialized (cfg_done=0)")
            return 1

        print("DDR initialized and ready\n")

        # Step 1: Measure overhead with small test
        print("Step 1: Measuring Python/USB overhead with 1MB test...")
        print("-" * 70)

        overhead_times = []
        for i in range(5):
            passed, elapsed, polls = run_test(usb, 1, pattern_type='lfsr')
            if not passed:
                print(f"  Test {i+1} FAILED - Data verification error")
                return 1
            overhead_times.append(elapsed)
            print(f"  Run {i+1}: {elapsed:.6f}s ({polls} polls) - PASS (data verified)")

        avg_overhead = sum(overhead_times) / len(overhead_times)
        print(f"\nAverage 1MB time: {avg_overhead:.6f}s")
        print(f"This includes: DDR transfer time + Python/USB overhead")

        # Step 2: Run large tests
        print("\n" + "=" * 70)
        print("Step 2: Running 512MB tests...")
        print("-" * 70)

        test_times = []
        for i in range(5):
            passed, elapsed, polls = run_test(usb, 512, pattern_type='lfsr')
            if not passed:
                print(f"  Test {i+1} FAILED - Data verification error")
                return 1

            test_times.append(elapsed)

            # Calculate raw bandwidth (without compensation)
            size_bytes = 512 * 1024 * 1024
            total_bytes = 2 * size_bytes  # write + read
            bandwidth_raw_gbps = (total_bytes * 8) / elapsed / 1e9

            print(f"  Run {i+1}: {elapsed:.6f}s ({polls} polls) - {bandwidth_raw_gbps:.2f} Gb/s - PASS (data verified)")

        # Step 3: Analyze results
        print("\n" + "=" * 70)
        print("ANALYSIS")
        print("=" * 70)

        avg_512mb_time = sum(test_times) / len(test_times)
        min_512mb_time = min(test_times)
        max_512mb_time = max(test_times)

        print(f"\n512MB test times:")
        print(f"  Average: {avg_512mb_time:.6f}s")
        print(f"  Min:     {min_512mb_time:.6f}s (best case)")
        print(f"  Max:     {max_512mb_time:.6f}s (worst case)")

        # Calculate bandwidth with overhead compensation
        # Assume overhead scales linearly with size (conservative estimate)
        # Overhead for 512MB = overhead_1mb * 512
        estimated_overhead_512mb = avg_overhead * 512

        # However, polling overhead doesn't scale linearly - it depends on poll count
        # Better estimate: assume constant overhead per test
        # Use the minimum time as it has least variance from other factors

        print(f"\nEstimated overhead for 512MB: {estimated_overhead_512mb:.6f}s")
        print(f"(This is likely an overestimate)")

        # Calculate compensated time (use best case time to minimize overhead impact)
        compensated_time = min_512mb_time - avg_overhead

        if compensated_time <= 0:
            print("\nWARNING: Overhead compensation resulted in negative time!")
            print("Using raw measurements instead.")
            compensated_time = min_512mb_time

        print(f"\nCompensated time (512MB - overhead): {compensated_time:.6f}s")

        # Calculate bandwidths
        size_bytes = 512 * 1024 * 1024
        total_bytes = 2 * size_bytes  # write + read
        total_mb = total_bytes / 1e6

        # Raw bandwidth
        bandwidth_raw_gbps = (total_bytes * 8) / avg_512mb_time / 1e9
        bandwidth_raw_mbps = total_bytes / avg_512mb_time / 1e6

        # Compensated bandwidth
        bandwidth_comp_gbps = (total_bytes * 8) / compensated_time / 1e9
        bandwidth_comp_mbps = total_bytes / compensated_time / 1e6

        print("\n" + "=" * 70)
        print("BANDWIDTH RESULTS")
        print("=" * 70)
        print(f"\nData transferred per test: {total_mb:.1f} MB (512MB write + 512MB read)")
        print(f"\nRaw measurements (includes overhead):")
        print(f"  Bandwidth: {bandwidth_raw_gbps:.2f} Gb/s ({bandwidth_raw_mbps:.1f} MB/s)")
        print(f"  Test time: {avg_512mb_time:.6f}s")

        print(f"\nCompensated measurements (overhead removed):")
        print(f"  Bandwidth: {bandwidth_comp_gbps:.2f} Gb/s ({bandwidth_comp_mbps:.1f} MB/s)")
        print(f"  Test time: {compensated_time:.6f}s")

        print("\n" + "=" * 70)
        print("DATA INTEGRITY")
        print("=" * 70)
        print("All tests PASSED with 100% data verification")
        print("- Write: LFSR pseudo-random pattern generated")
        print("- Read: Every byte compared against expected LFSR value")
        print("- Result: NO data corruption detected")

        print("\n")
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
