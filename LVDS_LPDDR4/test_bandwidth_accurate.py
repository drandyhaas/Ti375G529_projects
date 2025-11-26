#!/usr/bin/env python3
"""
Accurate DDR bandwidth test - measures read, write, and combined bandwidth separately

Strategy:
1. Run small tests (1MB) to measure Python/USB overhead for each mode
2. Run large tests (512MB) for each mode
3. Subtract overhead to get true DDR performance

Test modes:
  0 = write + read (default, with verification)
  1 = write only
  2 = read only (assumes data already written)
"""

from usb_ddr_control import USBDDRControl
import time
import sys

# Test mode constants
MODE_WRITE_READ = 0
MODE_WRITE_ONLY = 1
MODE_READ_ONLY = 2

MODE_NAMES = {
    MODE_WRITE_READ: "Write+Read",
    MODE_WRITE_ONLY: "Write-only",
    MODE_READ_ONLY: "Read-only"
}

def run_test(usb, size_mb, pattern_type='lfsr', test_mode=MODE_WRITE_READ):
    """
    Run a single memory test and measure time

    Args:
        usb: USBDDRControl instance
        size_mb: Size in megabytes
        pattern_type: 'lfsr' or 'fixed'
        test_mode: 0=write+read, 1=write-only, 2=read-only

    Returns:
        (passed, elapsed_time, poll_count)
    """
    # Stop any running test
    usb.memtest_stop()
    time.sleep(0.01)

    # Configure pattern
    if pattern_type == 'fixed':
        usb.reg_write(0x10, 0xDEADBEEF)  # REG_4_DATA_L
        usb.reg_write(0x14, 0xCAFEBABE)  # REG_5_DATA_H
        usb.reg_write(0x18, 0)           # REG_6_LFSR = 0
    else:
        usb.reg_write(0x18, 1)           # REG_6_LFSR = 1

    # Configure test mode: REG_7 bits[2:1] = test_mode, bit0 = x16_en (0)
    usb.reg_write(0x1C, (test_mode << 1))

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

def run_bandwidth_test(usb, size_mb, test_mode, num_runs=5):
    """
    Run multiple tests of a given mode and return statistics

    Returns:
        (times, all_passed)
    """
    times = []
    for i in range(num_runs):
        passed, elapsed, polls = run_test(usb, size_mb, pattern_type='lfsr', test_mode=test_mode)
        if not passed:
            return times, False
        times.append(elapsed)
    return times, True

def calc_bandwidth(size_bytes, elapsed_time):
    """Calculate bandwidth in Gb/s and MB/s"""
    if elapsed_time <= 0:
        return 0, 0
    gbps = (size_bytes * 8) / elapsed_time / 1e9
    mbps = size_bytes / elapsed_time / 1e6
    return gbps, mbps

def measure_overhead(usb, test_mode, num_runs=5):
    """
    Measure Python/USB overhead using 1MB tests

    Returns:
        average overhead time in seconds
    """
    times, passed = run_bandwidth_test(usb, 1, test_mode, num_runs)
    if not passed or len(times) == 0:
        return None
    return sum(times) / len(times)

def main():
    print("=" * 70)
    print("DDR Bandwidth Test - Overhead Compensated, Separate Read/Write")
    print("=" * 70)

    usb = USBDDRControl()
    size_mb = 512
    size_bytes = size_mb * 1024 * 1024
    num_runs = 5
    overhead_runs = 5

    try:
        # Check DDR ready
        cfg_reg = usb.reg_read(0x28)
        if not ((cfg_reg >> 3) & 0x1):
            print("ERROR: DDR not initialized (cfg_done=0)")
            return 1

        print("DDR initialized and ready")
        print(f"Test size: {size_mb} MB, {num_runs} runs per test\n")

        # ============================================================
        # Step 1: Measure overhead for each mode with 1MB tests
        # ============================================================
        print("=" * 70)
        print("STEP 1: Measuring Python/USB overhead (1MB tests)")
        print("-" * 70)

        # First do a write so read-only has data
        print("  Preparing: Writing 1MB for read-only overhead test...")
        passed, _, _ = run_test(usb, 1, test_mode=MODE_WRITE_ONLY)
        if not passed:
            print("FAILED - Initial write failed")
            return 1

        overheads = {}
        for mode in [MODE_WRITE_ONLY, MODE_READ_ONLY, MODE_WRITE_READ]:
            overhead = measure_overhead(usb, mode, overhead_runs)
            if overhead is None:
                print(f"FAILED - {MODE_NAMES[mode]} overhead measurement failed")
                return 1
            overheads[mode] = overhead
            print(f"  {MODE_NAMES[mode]:12s} overhead: {overhead*1000:.3f} ms")

        # ============================================================
        # Step 2: Write-only bandwidth
        # ============================================================
        print("\n" + "=" * 70)
        print("STEP 2: WRITE-ONLY BANDWIDTH (512MB)")
        print("-" * 70)

        write_times, passed = run_bandwidth_test(usb, size_mb, MODE_WRITE_ONLY, num_runs)
        if not passed:
            print("FAILED - Write test error")
            return 1

        for i, t in enumerate(write_times):
            gbps, mbps = calc_bandwidth(size_bytes, t)
            print(f"  Run {i+1}: {t:.6f}s - {gbps:.2f} Gb/s ({mbps:.1f} MB/s)")

        avg_write_time = sum(write_times) / len(write_times)
        min_write_time = min(write_times)
        compensated_write_time = min_write_time - overheads[MODE_WRITE_ONLY]
        if compensated_write_time <= 0:
            compensated_write_time = min_write_time

        write_gbps_raw, write_mbps_raw = calc_bandwidth(size_bytes, avg_write_time)
        write_gbps_comp, write_mbps_comp = calc_bandwidth(size_bytes, compensated_write_time)

        print(f"\n  Raw average:    {avg_write_time:.6f}s -> {write_gbps_raw:.2f} Gb/s ({write_mbps_raw:.1f} MB/s)")
        print(f"  Compensated:    {compensated_write_time:.6f}s -> {write_gbps_comp:.2f} Gb/s ({write_mbps_comp:.1f} MB/s)")

        # ============================================================
        # Step 3: Read-only bandwidth (data already written above)
        # ============================================================
        print("\n" + "=" * 70)
        print("STEP 3: READ-ONLY BANDWIDTH (512MB)")
        print("-" * 70)

        read_times, passed = run_bandwidth_test(usb, size_mb, MODE_READ_ONLY, num_runs)
        if not passed:
            print("FAILED - Read test error (data verification failed)")
            return 1

        for i, t in enumerate(read_times):
            gbps, mbps = calc_bandwidth(size_bytes, t)
            print(f"  Run {i+1}: {t:.6f}s - {gbps:.2f} Gb/s ({mbps:.1f} MB/s) - VERIFIED")

        avg_read_time = sum(read_times) / len(read_times)
        min_read_time = min(read_times)
        compensated_read_time = min_read_time - overheads[MODE_READ_ONLY]
        if compensated_read_time <= 0:
            compensated_read_time = min_read_time

        read_gbps_raw, read_mbps_raw = calc_bandwidth(size_bytes, avg_read_time)
        read_gbps_comp, read_mbps_comp = calc_bandwidth(size_bytes, compensated_read_time)

        print(f"\n  Raw average:    {avg_read_time:.6f}s -> {read_gbps_raw:.2f} Gb/s ({read_mbps_raw:.1f} MB/s)")
        print(f"  Compensated:    {compensated_read_time:.6f}s -> {read_gbps_comp:.2f} Gb/s ({read_mbps_comp:.1f} MB/s)")

        # ============================================================
        # Step 4: Write+Read combined
        # ============================================================
        print("\n" + "=" * 70)
        print("STEP 4: WRITE+READ COMBINED (512MB, with verification)")
        print("-" * 70)

        combined_times, passed = run_bandwidth_test(usb, size_mb, MODE_WRITE_READ, num_runs)
        if not passed:
            print("FAILED - Combined test error (data verification failed)")
            return 1

        total_bytes = 2 * size_bytes  # write + read
        for i, t in enumerate(combined_times):
            gbps, mbps = calc_bandwidth(total_bytes, t)
            print(f"  Run {i+1}: {t:.6f}s - {gbps:.2f} Gb/s ({mbps:.1f} MB/s) - VERIFIED")

        avg_combined_time = sum(combined_times) / len(combined_times)
        min_combined_time = min(combined_times)
        compensated_combined_time = min_combined_time - overheads[MODE_WRITE_READ]
        if compensated_combined_time <= 0:
            compensated_combined_time = min_combined_time

        combined_gbps_raw, combined_mbps_raw = calc_bandwidth(total_bytes, avg_combined_time)
        combined_gbps_comp, combined_mbps_comp = calc_bandwidth(total_bytes, compensated_combined_time)

        print(f"\n  Raw average:    {avg_combined_time:.6f}s -> {combined_gbps_raw:.2f} Gb/s ({combined_mbps_raw:.1f} MB/s)")
        print(f"  Compensated:    {compensated_combined_time:.6f}s -> {combined_gbps_comp:.2f} Gb/s ({combined_mbps_comp:.1f} MB/s)")

        # ============================================================
        # Summary
        # ============================================================
        print("\n" + "=" * 70)
        print("BANDWIDTH SUMMARY (Overhead Compensated)")
        print("=" * 70)
        print(f"\n  Test size: {size_mb} MB")
        print(f"\n                      Raw                  Compensated")
        print(f"  Write bandwidth:    {write_gbps_raw:5.2f} Gb/s ({write_mbps_raw:6.1f} MB/s)   {write_gbps_comp:5.2f} Gb/s ({write_mbps_comp:6.1f} MB/s)")
        print(f"  Read bandwidth:     {read_gbps_raw:5.2f} Gb/s ({read_mbps_raw:6.1f} MB/s)   {read_gbps_comp:5.2f} Gb/s ({read_mbps_comp:6.1f} MB/s)")
        print(f"  Combined (W+R):     {combined_gbps_raw:5.2f} Gb/s ({combined_mbps_raw:6.1f} MB/s)   {combined_gbps_comp:5.2f} Gb/s ({combined_mbps_comp:6.1f} MB/s)")

        # Calculate expected combined from individual (compensated)
        expected_combined_time = compensated_write_time + compensated_read_time
        expected_combined_gbps, expected_combined_mbps = calc_bandwidth(total_bytes, expected_combined_time)
        print(f"\n  Expected combined (write + read times):")
        print(f"                      {expected_combined_gbps:5.2f} Gb/s ({expected_combined_mbps:6.1f} MB/s)")

        turnaround_overhead = compensated_combined_time - expected_combined_time
        if compensated_combined_time > 0:
            turnaround_pct = turnaround_overhead / compensated_combined_time * 100
        else:
            turnaround_pct = 0
        print(f"\n  Write/Read turnaround overhead: {turnaround_overhead*1000:.2f} ms ({turnaround_pct:.1f}%)")

        print("\n" + "=" * 70)
        print("DATA INTEGRITY: All read tests PASSED with 100% verification")
        print("=" * 70)

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
