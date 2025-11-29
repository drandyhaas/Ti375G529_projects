#!/usr/bin/env python3
"""
Accurate DDR bandwidth test - measures read, write, and combined bandwidth separately

Uses hardware cycle counters for accurate timing (no Python/USB overhead).
Also measures Python timing for comparison.

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

# AXI clock frequency in MHz (memory checker runs on axi0_ACLK)
# Adjust this if your PLL configuration is different
AXI_CLK_MHZ = 200.0

def read_cycle_counts(usb):
    """Read hardware cycle counters for write and read phases"""
    write_cycles_l = usb.reg_read(0x48)  # REG_18
    write_cycles_h = usb.reg_read(0x4C)  # REG_19
    read_cycles_l = usb.reg_read(0x50)   # REG_20
    read_cycles_h = usb.reg_read(0x54)   # REG_21

    write_cycles = (write_cycles_h << 32) | write_cycles_l
    read_cycles = (read_cycles_h << 32) | read_cycles_l
    return write_cycles, read_cycles

def cycles_to_seconds(cycles):
    """Convert cycle count to seconds based on AXI clock frequency"""
    return cycles / (AXI_CLK_MHZ * 1e6)

def run_test(usb, size_mb, pattern_type='lfsr', test_mode=MODE_WRITE_READ):
    """
    Run a single memory test and measure time

    Args:
        usb: USBDDRControl instance
        size_mb: Size in megabytes
        pattern_type: 'lfsr' or 'fixed'
        test_mode: 0=write+read, 1=write-only, 2=read-only

    Returns:
        (passed, elapsed_time, write_cycles, read_cycles)
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
    while True:
        status = usb.reg_read(0x04)
        done = status & 0x1
        fail = (status >> 1) & 0x1

        if done:
            elapsed = time.time() - start_time
            write_cycles, read_cycles = read_cycle_counts(usb)

            if fail:
                return False, elapsed, write_cycles, read_cycles
            else:
                return True, elapsed, write_cycles, read_cycles

        # Safety timeout
        if time.time() - start_time > 120.0:
            return False, 120.0, 0, 0

        time.sleep(0.0001)  # 100us poll interval

def run_bandwidth_test(usb, size_mb, test_mode, num_runs=5):
    """
    Run multiple tests of a given mode and return statistics

    Returns:
        (results, all_passed) where results is list of (elapsed, write_cycles, read_cycles)
    """
    results = []
    for i in range(num_runs):
        passed, elapsed, write_cycles, read_cycles = run_test(usb, size_mb, pattern_type='lfsr', test_mode=test_mode)
        if not passed:
            return results, False
        results.append((elapsed, write_cycles, read_cycles))
    return results, True

def calc_bandwidth(size_bytes, elapsed_time):
    """Calculate bandwidth in Gb/s and MB/s"""
    if elapsed_time <= 0:
        return 0, 0
    gbps = (size_bytes * 8) / elapsed_time / 1e9
    mbps = size_bytes / elapsed_time / 1e6
    return gbps, mbps

def main():
    print("=" * 70)
    print("DDR Bandwidth Test - Hardware Cycle Counter Timing")
    print("=" * 70)

    usb = USBDDRControl()
    size_mb = 1023
    size_bytes = size_mb * 1024 * 1024
    num_runs = 3

    try:
        # Check DDR ready
        cfg_reg = usb.reg_read(0x28)
        if not ((cfg_reg >> 3) & 0x1):
            print("ERROR: DDR not initialized (cfg_done=0)")
            return 1

        print("DDR initialized and ready")
        print(f"Test size: {size_mb} MB, {num_runs} runs per test")
        print(f"AXI clock: {AXI_CLK_MHZ} MHz (adjust AXI_CLK_MHZ if needed)\n")

        # ============================================================
        # Test 1: Write-only bandwidth
        # ============================================================
        print("=" * 70)
        print(f"TEST 1: WRITE-ONLY BANDWIDTH ({size_mb}MB)")
        print("-" * 70)

        write_results, passed = run_bandwidth_test(usb, size_mb, MODE_WRITE_ONLY, num_runs)
        if not passed:
            print("FAILED - Write test error")
            return 1

        write_cycles_list = []
        for i, (elapsed, write_cyc, read_cyc) in enumerate(write_results):
            hw_time = cycles_to_seconds(write_cyc)
            hw_gbps, hw_mbps = calc_bandwidth(size_bytes, hw_time)
            sw_gbps, sw_mbps = calc_bandwidth(size_bytes, elapsed)
            write_cycles_list.append(write_cyc)
            print(f"  Run {i+1}: HW: {write_cyc:,} cycles = {hw_time*1000:.3f}ms -> {hw_gbps:.2f} Gb/s ({hw_mbps:.1f} MB/s)")
            print(f"          SW: {elapsed*1000:.3f}ms -> {sw_gbps:.2f} Gb/s ({sw_mbps:.1f} MB/s)")

        avg_write_cycles = sum(write_cycles_list) / len(write_cycles_list)
        avg_write_hw_time = cycles_to_seconds(avg_write_cycles)
        write_gbps, write_mbps = calc_bandwidth(size_bytes, avg_write_hw_time)
        print(f"\n  WRITE: {avg_write_cycles:,.0f} cycles = {avg_write_hw_time*1000:.3f}ms")
        print(f"         {write_gbps:.2f} Gb/s ({write_mbps:.1f} MB/s)")

        # ============================================================
        # Test 2: Read-only bandwidth (data already written above)
        # ============================================================
        print("\n" + "=" * 70)
        print(f"TEST 2: READ-ONLY BANDWIDTH ({size_mb}MB)")
        print("-" * 70)

        read_results, passed = run_bandwidth_test(usb, size_mb, MODE_READ_ONLY, num_runs)
        if not passed:
            print("FAILED - Read test error (data verification failed)")
            return 1

        read_cycles_list = []
        for i, (elapsed, write_cyc, read_cyc) in enumerate(read_results):
            hw_time = cycles_to_seconds(read_cyc)
            hw_gbps, hw_mbps = calc_bandwidth(size_bytes, hw_time)
            sw_gbps, sw_mbps = calc_bandwidth(size_bytes, elapsed)
            read_cycles_list.append(read_cyc)
            print(f"  Run {i+1}: HW: {read_cyc:,} cycles = {hw_time*1000:.3f}ms -> {hw_gbps:.2f} Gb/s ({hw_mbps:.1f} MB/s) - VERIFIED")
            print(f"          SW: {elapsed*1000:.3f}ms -> {sw_gbps:.2f} Gb/s ({sw_mbps:.1f} MB/s)")

        avg_read_cycles = sum(read_cycles_list) / len(read_cycles_list)
        avg_read_hw_time = cycles_to_seconds(avg_read_cycles)
        read_gbps, read_mbps = calc_bandwidth(size_bytes, avg_read_hw_time)
        print(f"\n  READ: {avg_read_cycles:,.0f} cycles = {avg_read_hw_time*1000:.3f}ms")
        print(f"        {read_gbps:.2f} Gb/s ({read_mbps:.1f} MB/s)")

        # ============================================================
        # Test 3: Write+Read combined
        # ============================================================
        print("\n" + "=" * 70)
        print(f"TEST 3: WRITE+READ COMBINED ({size_mb}MB, with verification)")
        print("-" * 70)

        combined_results, passed = run_bandwidth_test(usb, size_mb, MODE_WRITE_READ, num_runs)
        if not passed:
            print("FAILED - Combined test error (data verification failed)")
            return 1

        total_bytes = 2 * size_bytes
        combined_write_cycles = []
        combined_read_cycles = []
        for i, (elapsed, write_cyc, read_cyc) in enumerate(combined_results):
            total_cycles = write_cyc + read_cyc
            hw_time = cycles_to_seconds(total_cycles)
            hw_gbps, hw_mbps = calc_bandwidth(total_bytes, hw_time)
            sw_gbps, sw_mbps = calc_bandwidth(total_bytes, elapsed)
            combined_write_cycles.append(write_cyc)
            combined_read_cycles.append(read_cyc)
            print(f"  Run {i+1}: HW: W={write_cyc:,} + R={read_cyc:,} = {total_cycles:,} cycles")
            print(f"          {hw_time*1000:.3f}ms -> {hw_gbps:.2f} Gb/s ({hw_mbps:.1f} MB/s) - VERIFIED")
            print(f"          SW: {elapsed*1000:.3f}ms -> {sw_gbps:.2f} Gb/s ({sw_mbps:.1f} MB/s)")

        avg_combined_write = sum(combined_write_cycles) / len(combined_write_cycles)
        avg_combined_read = sum(combined_read_cycles) / len(combined_read_cycles)
        avg_combined_total = avg_combined_write + avg_combined_read
        avg_combined_hw_time = cycles_to_seconds(avg_combined_total)
        combined_gbps, combined_mbps = calc_bandwidth(total_bytes, avg_combined_hw_time)
        print(f"\n  COMBINED: W={avg_combined_write:,.0f} + R={avg_combined_read:,.0f} = {avg_combined_total:,.0f} cycles")
        print(f"            {avg_combined_hw_time*1000:.3f}ms -> {combined_gbps:.2f} Gb/s ({combined_mbps:.1f} MB/s)")

        # ============================================================
        # Summary
        # ============================================================
        print("\n" + "=" * 70)
        print("BANDWIDTH SUMMARY (Hardware Timed)")
        print("=" * 70)
        print(f"\n  Test size: {size_mb} MB")
        print(f"  AXI clock: {AXI_CLK_MHZ} MHz")
        print(f"\n  Write bandwidth:    {write_gbps:6.2f} Gb/s  ({write_mbps:7.1f} MB/s)")
        print(f"  Read bandwidth:     {read_gbps:6.2f} Gb/s  ({read_mbps:7.1f} MB/s)")
        print(f"  Combined (W+R):     {combined_gbps:6.2f} Gb/s  ({combined_mbps:7.1f} MB/s)")

        # Calculate expected combined from individual
        expected_cycles = avg_write_cycles + avg_read_cycles
        expected_time = cycles_to_seconds(expected_cycles)
        expected_gbps, expected_mbps = calc_bandwidth(total_bytes, expected_time)
        print(f"\n  Expected combined (sum of individual):")
        print(f"                      {expected_gbps:6.2f} Gb/s  ({expected_mbps:7.1f} MB/s)")

        turnaround_cycles = avg_combined_total - expected_cycles
        turnaround_time = cycles_to_seconds(turnaround_cycles)
        if avg_combined_total > 0:
            turnaround_pct = turnaround_cycles / avg_combined_total * 100
        else:
            turnaround_pct = 0
        print(f"\n  Write/Read turnaround: {turnaround_cycles:,.0f} cycles = {turnaround_time*1000:.3f}ms ({turnaround_pct:.1f}%)")

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
