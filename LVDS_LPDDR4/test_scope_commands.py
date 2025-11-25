#!/usr/bin/env python3
"""
Test command_processor functions via usb_command_handler.
Commands are forwarded when first byte is NOT 0xFE.
Protocol: Send 8 bytes -> command_processor processes and responds.
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

def send_scope_command(cmd_bytes, description="", expected_response_len=4):
    """
    Send 8-byte command to command_processor and receive response.

    Args:
        cmd_bytes: List/tuple of up to 8 bytes (will be padded with 0s)
        description: Test description
        expected_response_len: Expected response length in bytes

    Returns:
        bytes: Response data, or None on failure
    """
    # Pad to 8 bytes
    cmd = list(cmd_bytes) + [0] * (8 - len(cmd_bytes))
    cmd = cmd[:8]  # Ensure exactly 8 bytes

    print(f"\n=== {description} ===")
    print(f"TX: {bytes(cmd).hex()} (8 bytes)")

    usb.send(bytes(cmd))
    time.sleep(0.1)

    rxdata = recv_with_timeout(usb, expected_response_len)

    if len(rxdata) > 0:
        print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")
        return rxdata
    else:
        print("RX: No response (timeout)")
        return None


def test_get_version():
    """Command 2, subcommand 0: Get firmware version."""
    # rx_data[0]=2, rx_data[1]=0 -> returns version
    rxdata = send_scope_command([2, 0], "Get command_processor version", 4)

    if rxdata and len(rxdata) == 4:
        version = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
        print(f"  Version: {version}")
        return True
    return False


def test_get_boardin():
    """Command 2, subcommand 1: Get board input status."""
    # rx_data[0]=2, rx_data[1]=1 -> returns boardin_sync
    rxdata = send_scope_command([2, 1], "Get board input status", 4)

    if rxdata and len(rxdata) == 4:
        boardin = rxdata[0]
        print(f"  Board input byte: 0x{boardin:02X}")
        return True
    return False


def test_get_event_counter():
    """Command 2, subcommand 3: Get event counter."""
    # rx_data[0]=2, rx_data[1]=3 -> returns eventcounter_sync
    rxdata = send_scope_command([2, 3], "Get event counter", 4)

    if rxdata and len(rxdata) == 4:
        count = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
        print(f"  Event counter: {count}")
        return True
    return False


def test_get_lock_info():
    """Command 2, subcommand 5: Get lock/clock info."""
    # rx_data[0]=2, rx_data[1]=5 -> returns lock info
    rxdata = send_scope_command([2, 5], "Get lock/clock info", 4)

    if rxdata and len(rxdata) == 4:
        value = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
        clkswitch = (value >> 0) & 1
        lockinfo = (value >> 8) & 0xF
        lvdsin_spare = (value >> 16) & 1
        print(f"  Raw: 0x{value:08X}")
        print(f"  clkswitch: {clkswitch}, lockinfo: 0x{lockinfo:X}, lvdsin_spare: {lvdsin_spare}")
        return True
    return False


def test_trigger_arm():
    """Command 1: Arm trigger and get acquisition state."""
    # rx_data[0]=1, rx_data[1]=triggertype, rx_data[2]=channeltype
    # rx_data[4:5]=lengthtotake
    # Returns: acqstate in low byte, sample_triggered in upper bits
    rxdata = send_scope_command([1, 0, 0, 0, 0x10, 0x00, 0, 0],
                                 "Arm trigger (type=0, len=16)", 4)

    if rxdata and len(rxdata) == 4:
        acqstate = rxdata[0]
        sample_triggered = ((rxdata[3] << 16) | (rxdata[2] << 8) | rxdata[1]) >> 4
        print(f"  Acq state: {acqstate}")
        print(f"  Sample triggered: {sample_triggered}")
        return True
    return False


def test_read_ram_data():
    """Command 0: Read data from RAM buffer."""
    # First arm trigger to have some data
    send_scope_command([1, 0, 0, 0, 0x10, 0x00, 0, 0], "Arm trigger first", 4)
    time.sleep(0.1)

    # Command 0: read RAM data
    # rx_data[4:7] = length (32-bit LE)
    # Let's read 16 bytes (4 words)
    length = 16
    rxdata = send_scope_command(
        [0, 0, 0, 0, length & 0xFF, (length >> 8) & 0xFF, (length >> 16) & 0xFF, (length >> 24) & 0xFF],
        f"Read {length} bytes from RAM",
        length
    )

    if rxdata and len(rxdata) > 0:
        print(f"  Got {len(rxdata)} bytes of RAM data")
        # Show first few bytes
        print(f"  Data: {rxdata[:min(32, len(rxdata))].hex()}{'...' if len(rxdata) > 32 else ''}")
        return len(rxdata) == length
    return False


# Run tests
print("\n" + "="*60)
print("Testing command_processor via scope command forwarding")
print("="*60)

results = []

results.append(("Get version", test_get_version()))
results.append(("Get board input", test_get_boardin()))
results.append(("Get event counter", test_get_event_counter()))
results.append(("Get lock info", test_get_lock_info()))
results.append(("Arm trigger", test_trigger_arm()))
results.append(("Read RAM data", test_read_ram_data()))

usb.close()

# Summary
print("\n" + "="*60)
print("SUMMARY:")
passed = sum(1 for _, ok in results if ok)
total = len(results)
for name, ok in results:
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}] {name}")
print(f"\n  {passed}/{total} tests passed")
print("="*60)
