#!/usr/bin/env python3
"""
Test bandwidth of command_processor RAM readout (command 0).
Measures data transfer rate from FPGA to PC via USB.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
from usb_utils import recv_with_timeout
import time

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))


def read_ram_data(length, timeout=10.0):
    """
    Send command 0 to read data from RAM buffer.
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
    rxdata = recv_with_timeout(usb, length, timeout=timeout)
    elapsed = time.time() - start_time

    return len(rxdata), elapsed


# First, arm the trigger to ensure there's data in the buffer
print("\n=== Arming trigger to fill buffer ===")
arm_cmd = bytes([1, 0, 0, 0, 0x00, 0x10, 0, 0])  # length=4096
usb.send(arm_cmd)
time.sleep(0.2)
resp = recv_with_timeout(usb, 4)
if resp:
    print(f"Trigger armed, response: {resp.hex()}")
else:
    print("Warning: No response from trigger arm")

# Run bandwidth test with 16KB transfers (known to work) - 10 iterations
print("\n" + "="*60)
print("Bandwidth Test - 16 KB transfers (10 iterations)")
print("="*60)

LENGTH = 16 * 1024  # 16 KB
NUM_ITERATIONS = 10

rates = []
for i in range(NUM_ITERATIONS):
    bytes_received, elapsed = read_ram_data(LENGTH, timeout=5.0)

    if bytes_received > 0 and elapsed > 0:
        rate_mbs = bytes_received / (elapsed * 1_000_000)
        status = "PASS" if bytes_received == LENGTH else f"INCOMPLETE ({bytes_received}/{LENGTH})"
        print(f"  [{i+1:2d}/{NUM_ITERATIONS}] {bytes_received:,} bytes in {elapsed:.3f}s = {rate_mbs:.2f} MB/s [{status}]")
        if bytes_received == LENGTH:
            rates.append(rate_mbs)
    else:
        print(f"  [{i+1:2d}/{NUM_ITERATIONS}] TIMEOUT or ERROR")

# Flush any stale data before closing
print("\n=== Flushing stale data ===")
total_flushed = 0
for _ in range(100):
    data = recv_with_timeout(usb, 65536, timeout=0.1)
    if not data:
        break
    total_flushed += len(data)
print(f"  Flushed {total_flushed:,} bytes")

usb.close()

# Summary
print("\n" + "="*60)
print("SUMMARY:")
print("="*60)

if rates:
    avg_rate = sum(rates) / len(rates)
    min_rate = min(rates)
    max_rate = max(rates)
    print(f"  Successful transfers: {len(rates)}/{NUM_ITERATIONS}")
    print(f"  Average rate: {avg_rate:.2f} MB/s")
    print(f"  Min rate: {min_rate:.2f} MB/s")
    print(f"  Max rate: {max_rate:.2f} MB/s")

    # USB3 theoretical max is ~400 MB/s, FT601 typically achieves 200-300 MB/s
    if avg_rate > 100:
        print(f"  Assessment: Good USB3 performance")
    elif avg_rate > 30:
        print(f"  Assessment: Moderate performance (USB2 or bottleneck)")
    else:
        print(f"  Assessment: Low performance (check connection)")
else:
    print("  No successful transfers!")

print("="*60)
