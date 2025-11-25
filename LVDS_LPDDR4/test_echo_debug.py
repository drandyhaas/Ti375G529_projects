#!/usr/bin/env python3
"""
Debug test for ECHO command - checks state machine status.
Updated for new consolidated command_processor (no 0xFE prefix).
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
import time

# New command codes (consolidated into command_processor)
CMD_GET_VERSION = 0x23  # Was 0xFE 0x04
CMD_GET_STATUS = 0x24   # Was 0xFE 0x05
CMD_ECHO = 0x25         # Was 0xFE 0x06

# State name lookup (for new command_processor)
STATE_NAMES = {
    0: "INIT",
    1: "RX",
    2: "PROCESS",
    3: "TX_DATA_CONST",
    4: "TX_DATA1", 5: "TX_DATA2", 6: "TX_DATA3", 7: "TX_DATA4",
    8: "PLLCLOCK",
    9: "BOOTUP",
    10: "TX_MASS",
    11: "AXI_WRITE", 12: "AXI_WRESP",
    13: "AXI_READ", 14: "AXI_RRESP", 15: "TX_RDATA",
    16: "ECHO_RX", 17: "ECHO_TX", 18: "RX_VARLEN"
}

def get_state_name(state_num):
    return STATE_NAMES.get(state_num, f"UNKNOWN({state_num})")

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

# Check version first
print("\n=== VERSION CHECK ===")
usb.send(bytes([CMD_GET_VERSION, 0, 0, 0, 0, 0, 0, 0]))
time.sleep(0.1)
rxdata = usb.recv(4)
if len(rxdata) == 4:
    version = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    print(f"Firmware Version: 0x{version:08X}")
    if version != 0x20251125:
        print("WARNING: Expected version 0x20251125 - please rebuild and reprogram!")
else:
    print(f"ERROR: Version read failed, got {len(rxdata)} bytes")

# Check initial status
print("\n=== INITIAL STATUS ===")
usb.send(bytes([CMD_GET_STATUS, 0, 0, 0, 0, 0, 0, 0]))
time.sleep(0.1)
rxdata = usb.recv(4)
if len(rxdata) == 4:
    status = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    state = status & 0x1F
    rx_counter = (status >> 8) & 0x0F
    cmd = (status >> 24) & 0xFF
    print(f"Raw status: 0x{status:08X}")
    print(f"  State: {state} ({get_state_name(state)})")
    print(f"  rx_counter: {rx_counter}")
    print(f"  command echo: 0x{cmd:02X}")
else:
    print(f"ERROR: Status read failed")

# Try the echo command with timeout
print("\n=== SENDING ECHO COMMAND ===")
data = bytes([0x42])  # Single byte echo
length = len(data)
# New 8-byte format: [CMD][LEN_LO][LEN_HI][DATA0][DATA1][DATA2][DATA3][DATA4]
cmd_data = [CMD_ECHO, length & 0xFF, (length >> 8) & 0xFF]
# Add first 5 bytes of data (or pad with 0s)
for i in range(5):
    if i < len(data):
        cmd_data.append(data[i])
    else:
        cmd_data.append(0)
txdata = bytes(cmd_data)
print(f"TX: {txdata.hex()} ({len(txdata)} bytes)")
usb.send(txdata)

# Try to receive the echo response
print("Attempting to receive echo response...")
time.sleep(0.1)
try:
    rxdata = usb.recv(length)
    if len(rxdata) > 0:
        print(f"RX: {rxdata.hex()} ({len(rxdata)} bytes)")
        if rxdata == data:
            print("SUCCESS: Echo received correctly!")
        else:
            print(f"MISMATCH: Expected {data.hex()}, got {rxdata.hex()}")
    else:
        print("No data received")
except Exception as e:
    print(f"Receive error: {e}")

# Wait for watchdog timeout (10+ seconds) then check status
print("\nWaiting 12 seconds for watchdog timeout...")
time.sleep(12.0)

print("\n=== POST-TIMEOUT STATUS (after watchdog reset) ===")
usb.send(bytes([CMD_GET_STATUS, 0, 0, 0, 0, 0, 0, 0]))
time.sleep(0.1)
rxdata = usb.recv(4)
if len(rxdata) == 4:
    status = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    state = status & 0x1F
    rx_counter = (status >> 8) & 0x0F
    cmd = (status >> 24) & 0xFF
    print(f"Raw status: 0x{status:08X}")
    print(f"  State: {state} ({get_state_name(state)})")
    print(f"  rx_counter: {rx_counter}")
    print(f"  command echo: 0x{cmd:02X}")

    # Interpretation
    if state == 0 or state == 1:
        print(f"\n=> State machine in {get_state_name(state)} (good!)")
    elif state == 16:
        print("\n=> STUCK in ECHO_RX - waiting for more data")
    elif state == 17:
        print("\n=> STUCK in ECHO_TX - transmission issue")
    else:
        print(f"\n=> State: {get_state_name(state)}")
else:
    print("ERROR: Could not read post-echo status")

usb.close()
print("\nDone!")
