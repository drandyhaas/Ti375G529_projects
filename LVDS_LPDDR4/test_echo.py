#!/usr/bin/env python3
"""
Test echo command to verify multi-byte reception in usb_command_handler.
Protocol: 0xFE 0x06 [LEN_LO] [LEN_HI] [DATA...] -> returns [DATA...]
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
from usb_utils import recv_with_timeout
import time

CMD_PREFIX = 0xFE
CMD_ECHO = 0x06  # New echo command

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

def test_echo(data_bytes, description=""):
    """Send echo command and verify response matches."""
    length = len(data_bytes)
    print(f"\n=== Echo Test: {description} ({length} bytes) ===")

    # Build command: prefix + cmd + length (2 bytes LE) + data
    txdata = bytes([CMD_PREFIX, CMD_ECHO, length & 0xFF, (length >> 8) & 0xFF]) + bytes(data_bytes)
    print(f"TX: {txdata.hex()} ({len(txdata)} total bytes)")
    usb.send(txdata)

    time.sleep(0.1)

    # Receive echoed data with timeout wrapper
    rxdata = recv_with_timeout(usb, length)
    print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")

    if len(rxdata) != length:
        print(f"FAILED - Expected {length} bytes, got {len(rxdata)}")
        return False

    if rxdata == bytes(data_bytes):
        print("PASS - Echo matches!")
        return True
    else:
        print("FAILED - Data mismatch!")
        print(f"  Expected: {bytes(data_bytes).hex()}")
        print(f"  Got:      {rxdata.hex()}")
        # Show byte-by-byte comparison
        for i in range(min(len(data_bytes), len(rxdata))):
            if data_bytes[i] != rxdata[i]:
                print(f"  Byte {i}: expected 0x{data_bytes[i]:02X}, got 0x{rxdata[i]:02X}")
        return False

# Run tests
results = []

# Test 1: Single byte
results.append(test_echo([0x42], "single byte"))

# Test 2: Two bytes
results.append(test_echo([0x12, 0x34], "two bytes"))

# Test 3: Four bytes
results.append(test_echo([0x11, 0x22, 0x33, 0x44], "four bytes"))

# Test 4: Eight bytes (common packet size)
results.append(test_echo([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08], "eight bytes"))

# Test 5: Ascending pattern
results.append(test_echo(list(range(16)), "16 bytes ascending"))

# Test 6: All same value
results.append(test_echo([0xAA] * 8, "8 bytes all 0xAA"))

# Test 7: Alternating pattern
results.append(test_echo([0x55, 0xAA] * 4, "alternating 0x55/0xAA"))

usb.close()

# Summary
print("\n" + "="*50)
print("SUMMARY:")
passed = sum(results)
total = len(results)
print(f"  {passed}/{total} tests passed")
if passed == total:
    print("  All tests PASSED!")
else:
    print("  Some tests FAILED!")
print("="*50)
