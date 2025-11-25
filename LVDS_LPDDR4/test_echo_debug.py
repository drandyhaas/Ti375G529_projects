#!/usr/bin/env python3
"""
Debug test for ECHO command - checks state machine status.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
import time

CMD_PREFIX = 0xFE
CMD_GET_VERSION = 0x04
CMD_GET_STATUS = 0x05
CMD_ECHO = 0x06

# State name lookup
STATE_NAMES = {
    0: "RX_CMD",
    1: "RX_LEN0", 2: "RX_LEN1", 3: "RX_LEN2", 4: "RX_LEN3",
    5: "TX_DATA",
    6: "RX_ADDR0", 7: "RX_ADDR1", 8: "RX_ADDR2", 9: "RX_ADDR3",
    10: "RX_DATA0", 11: "RX_DATA1", 12: "RX_DATA2", 13: "RX_DATA3",
    14: "AXI_WRITE", 15: "AXI_WRESP",
    16: "AXI_READ", 17: "AXI_RRESP", 18: "TX_RDATA",
    19: "LOAD_VERSION",
    20: "SCOPE_TX", 21: "SCOPE_RX",
    22: "RX_USB_CMD",
    23: "ECHO_LEN0", 24: "ECHO_LEN1", 25: "ECHO_RX", 26: "ECHO_TX",
    31: "ERROR"
}

def get_state_name(state_num):
    return STATE_NAMES.get(state_num, f"UNKNOWN({state_num})")

print("Opening USB device...")
usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

# Check version first
print("\n=== VERSION CHECK ===")
usb.send(bytes([CMD_PREFIX, CMD_GET_VERSION]))
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
usb.send(bytes([CMD_PREFIX, CMD_GET_STATUS]))
time.sleep(0.1)
rxdata = usb.recv(4)
if len(rxdata) == 4:
    status = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    state = status & 0x1F
    echo_len = (status >> 8) & 0xFF
    echo_rx_cnt = (status >> 16) & 0xFF
    cmd = (status >> 24) & 0xFF
    print(f"Raw status: 0x{status:08X}")
    print(f"  State: {state} ({get_state_name(state)})")
    print(f"  echo_length[7:0]: {echo_len}")
    print(f"  echo_rx_count[7:0]: {echo_rx_cnt}")
    print(f"  command: 0x{cmd:02X}")
else:
    print(f"ERROR: Status read failed")

# Try the echo command with timeout
print("\n=== SENDING ECHO COMMAND ===")
data = bytes([0x42])  # Single byte echo
length = len(data)
txdata = bytes([CMD_PREFIX, CMD_ECHO, length & 0xFF, (length >> 8) & 0xFF]) + data
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

# Wait for watchdog timeout (2+ seconds) then check status
print("\nWaiting 3 seconds for watchdog timeout...")
time.sleep(3.0)

print("\n=== POST-TIMEOUT STATUS (after watchdog reset) ===")
usb.send(bytes([CMD_PREFIX, CMD_GET_STATUS]))
time.sleep(0.1)
rxdata = usb.recv(4)
if len(rxdata) == 4:
    status = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
    state = status & 0x1F
    echo_len = (status >> 8) & 0xFF
    echo_rx_cnt = (status >> 16) & 0xFF
    cmd = (status >> 24) & 0xFF
    print(f"Raw status: 0x{status:08X}")
    print(f"  State: {state} ({get_state_name(state)})")
    print(f"  echo_length[7:0]: {echo_len}")
    print(f"  echo_rx_count[7:0]: {echo_rx_cnt}")
    print(f"  command: 0x{cmd:02X}")

    # Interpretation
    if state == 0:
        print("\n=> State machine returned to RX_CMD (good!)")
        print("   But we didn't get the echo response...")
    elif state in [23, 24, 25]:
        print(f"\n=> STUCK in echo receive phase ({get_state_name(state)})")
        print(f"   Expected to receive {echo_len} bytes, got {echo_rx_cnt} so far")
    elif state == 26:
        print("\n=> STUCK in ECHO_TX - transmission issue")
    else:
        print(f"\n=> Unexpected state: {get_state_name(state)}")
else:
    print("ERROR: Could not read post-echo status")

usb.close()
print("\nDone!")
