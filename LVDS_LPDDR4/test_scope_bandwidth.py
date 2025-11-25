#!/usr/bin/env python3
"""
Test bandwidth of command_processor RAM readout (command 0).
Measures data transfer rate from FPGA to PC via USB at various transfer sizes.

NOTE: This test uses scope commands (0x00, 0x01) which are unchanged
in the consolidated command_processor. These commands do NOT use
the old 0xFE prefix format.
"""
import sys
sys.path.insert(0, '.')

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
import time

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))


def arm_trigger(length=4096):
    """Arm trigger to ensure there's data in the buffer."""
    arm_cmd = bytes([
        1,  # command 1 = arm trigger
        0,  # trigger type
        0,  # channel type
        0,  # unused
        length & 0xFF,
        (length >> 8) & 0xFF,
        0, 0
    ])
    usb.send(arm_cmd)
    time.sleep(0.05)
    return usb.recv(4)


def read_ram_data(length):
    """
    Send command 0 to read data from RAM buffer.
    Returns (bytes_received, elapsed_time)
    """
    cmd = bytes([
        0,  # command 0
        0, 0, 0,  # unused
        length & 0xFF,
        (length >> 8) & 0xFF,
        (length >> 16) & 0xFF,
        (length >> 24) & 0xFF
    ])

    usb.send(cmd)

    start_time = time.time()
    rxdata = usb.recv(length)
    elapsed = time.time() - start_time

    return len(rxdata), elapsed


def run_bandwidth_test(transfer_size, num_iterations=5):
    """
    Run bandwidth test for a specific transfer size.
    Returns list of rate_mbs for successful transfers.
    """
    results = []

    for i in range(num_iterations):
        # Re-arm trigger before each transfer to ensure fresh data
        arm_trigger(min(transfer_size, 65536))

        bytes_received, elapsed = read_ram_data(transfer_size)

        if bytes_received > 0 and elapsed > 0:
            rate_mbs = bytes_received / (elapsed * 1_000_000)

            status = "OK" if bytes_received == transfer_size else f"INCOMPLETE {bytes_received}/{transfer_size}"
            print(f"    [{i+1:2d}/{num_iterations}] {bytes_received:,} bytes in {elapsed:.3f}s = {rate_mbs:.1f} MB/s [{status}]")

            if bytes_received == transfer_size:
                results.append(rate_mbs)
        else:
            print(f"    [{i+1:2d}/{num_iterations}] ERROR - no data received")

    return results


# Arm trigger initially
print("\n=== Arming trigger ===")
resp = arm_trigger()
if resp:
    print(f"  Trigger armed, response: {resp.hex()}")
else:
    print("  Warning: No response from trigger arm")

# Define test sizes - from small to large
# NOTE: Scope RAM buffer is ~112KB, but we test larger sizes to see behavior
TEST_SIZES = [
    (200, "200 B", 10),           # Very small
    (1024, "1 KB", 10),           # Small
    (4 * 1024, "4 KB", 10),       # Medium-small
    (16 * 1024, "16 KB", 10),     # Medium
    (64 * 1024, "64 KB", 5),      # Medium-large
    (100 * 1024, "100 KB", 5),    # Near max RAM size
    (256 * 1024, "256 KB", 3),    # Large (may wrap RAM)
    (512 * 1024, "512 KB", 3),    # Larger
    (1024 * 1024, "1 MB", 3),     # Very large
    (2 * 1024 * 1024, "2 MB", 3), # Extra large
    (4 * 1024 * 1024, "4 MB", 2), # Maximum test size
]

all_results = {}

print("\n" + "=" * 70)
print("BANDWIDTH TEST - Multiple Transfer Sizes")
print("=" * 70)

for size, size_name, iterations in TEST_SIZES:
    print(f"\n--- {size_name} transfers ({iterations} iterations) ---")
    results = run_bandwidth_test(size, num_iterations=iterations)
    all_results[size_name] = results

    if results:
        avg_rate = sum(results) / len(results)
        print(f"    => Avg: {avg_rate:.1f} MB/s, Success: {len(results)}/{iterations}")
    else:
        print(f"    => No successful transfers")

usb.close()

# Summary
print("\n" + "=" * 70)
print("SUMMARY")
print("=" * 70)
print(f"{'Size':<12} {'Success':<12} {'Avg MB/s':<12} {'Min MB/s':<12} {'Max MB/s':<12}")
print("-" * 70)

for size_name, results in all_results.items():
    if results:
        success = f"{len(results)}/{len(results)}"
        avg = f"{sum(results)/len(results):.1f}"
        min_r = f"{min(results):.1f}"
        max_r = f"{max(results):.1f}"
    else:
        success = "0/?"
        avg = min_r = max_r = "-"

    print(f"{size_name:<12} {success:<12} {avg:<12} {min_r:<12} {max_r:<12}")

print("-" * 70)

# Overall assessment
successful_tests = sum(1 for r in all_results.values() if r)
total_tests = len(all_results)

if successful_tests == total_tests:
    print(f"\nAll {total_tests} transfer sizes working!")
else:
    print(f"\n{successful_tests}/{total_tests} transfer sizes working")

# Rate assessment based on largest successful transfer
large_results = all_results.get("4 MB") or all_results.get("2 MB") or all_results.get("1 MB") or all_results.get("512 KB") or all_results.get("256 KB") or all_results.get("100 KB")
if large_results:
    avg_large = sum(large_results) / len(large_results)
    if avg_large > 200:
        print(f"Performance: Excellent USB3 ({avg_large:.0f} MB/s)")
    elif avg_large > 100:
        print(f"Performance: Good USB3 ({avg_large:.0f} MB/s)")
    elif avg_large > 30:
        print(f"Performance: Moderate ({avg_large:.0f} MB/s)")
    else:
        print(f"Performance: Low ({avg_large:.0f} MB/s) - check connection")

print("=" * 70)
