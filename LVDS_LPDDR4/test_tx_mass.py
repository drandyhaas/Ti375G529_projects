#!/usr/bin/env python3
"""
Test TX_MASS command to verify multi-byte reception works.
This command was already in the firmware before our changes.
Protocol: 0xFE 0x01 [LEN0] [LEN1] [LEN2] [LEN3] -> returns LEN bytes
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
import time

CMD_PREFIX = 0xFE
CMD_TX_MASS = 0x01

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

def test_tx_mass(length, description=""):
    """Send TX_MASS command and verify we get the right number of bytes back."""
    print(f"\n=== TX_MASS Test: {description} ({length} bytes) ===")

    # Build command: prefix + cmd + length (4 bytes LE)
    txdata = bytes([CMD_PREFIX, CMD_TX_MASS,
                    length & 0xFF,
                    (length >> 8) & 0xFF,
                    (length >> 16) & 0xFF,
                    (length >> 24) & 0xFF])
    print(f"TX: {txdata.hex()} ({len(txdata)} total bytes)")
    usb.send(txdata)

    time.sleep(0.2)

    # Receive data
    rxdata = usb.recv(length)
    print(f"RX: {rxdata[:32].hex()}{'...' if len(rxdata) > 32 else ''} ({len(rxdata)} bytes)")

    if len(rxdata) == length:
        print(f"PASS - Got {length} bytes back!")
        return True
    else:
        print(f"FAILED - Expected {length} bytes, got {len(rxdata)}")
        return False

# Run tests
results = []

# Test 1: 4 bytes (minimum meaningful size)
results.append(test_tx_mass(4, "4 bytes"))

# Test 2: 8 bytes
results.append(test_tx_mass(8, "8 bytes"))

# Test 3: 16 bytes
results.append(test_tx_mass(16, "16 bytes"))

# Test 4: 64 bytes
results.append(test_tx_mass(64, "64 bytes"))

usb.close()

# Summary
print("\n" + "="*50)
print("SUMMARY:")
passed = sum(results)
total = len(results)
print(f"  {passed}/{total} tests passed")
if passed == total:
    print("  TX_MASS works - multi-byte RX is functional!")
else:
    print("  TX_MASS has issues - problem is in RX path")
print("="*50)
