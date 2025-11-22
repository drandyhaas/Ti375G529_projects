#!/usr/bin/env python3
"""
Debug script to understand why CTL registers work with JTAG but not USB
Run this with JTAG firmware loaded
"""

import sys
import os
import time
import logging
from pathlib import Path

# Add Efinity paths
sys.path.append(str(Path(os.environ['EFINITY_HOME'], 'debugger', 'bin')))
sys.path.append(str(Path(os.environ['EFINITY_HOME'], 'pgm', 'bin')))

# Setup logging
LOGGER = logging.getLogger('ddr_cali_tools')
logging.basicConfig()
stream_handler = logging.StreamHandler()
formatter = logging.Formatter('%(message)s')
stream_handler.setFormatter(formatter)
LOGGER.addHandler(stream_handler)
LOGGER.setLevel(logging.INFO)
LOGGER.propagate = False

print("="*70)
print("CTL Register Access Debug Script")
print("="*70)

# Initialize JTAG
print("\n[1] Initializing JTAG connection...")
from efx_dbg.jtag import DebugSession
from jtag_drv import jtag_drv

session = DebugSession()
session.connect()
jtag = jtag_drv(session)
print("JTAG connected successfully")

# Test 1: Read CTL registers via JTAG
print("\n" + "="*70)
print("[TEST 1] Reading CTL registers via JTAG")
print("="*70)

print("\nBefore config_restart() and config_ctrl_sel():")
try:
    ctl0_before = jtag.jtag2axi_read(0x0000)
    print(f"  CTL[0] @ 0x0000 = 0x{ctl0_before:08X}")
except Exception as e:
    print(f"  CTL[0] read failed: {e}")

try:
    pi0_before = jtag.jtag2axi_read(0x2000)
    print(f"  PI[0]  @ 0x2000 = 0x{pi0_before:08X}")
except Exception as e:
    print(f"  PI[0] read failed: {e}")

# Now do the config sequence
print("\nCalling config_restart()...")
jtag.config_restart()
time.sleep(0.1)

print("Calling config_ctrl_sel(1)...")
jtag.config_ctrl_sel(1)
time.sleep(0.1)

print("\nAfter config_restart() and config_ctrl_sel():")
try:
    ctl0_after = jtag.jtag2axi_read(0x0000)
    print(f"  CTL[0] @ 0x0000 = 0x{ctl0_after:08X}")

    # Read more CTL registers
    ctl1 = jtag.jtag2axi_read(0x0004)
    print(f"  CTL[1] @ 0x0004 = 0x{ctl1:08X}")

    ctl2 = jtag.jtag2axi_read(0x0008)
    print(f"  CTL[2] @ 0x0008 = 0x{ctl2:08X}")

    # Try reading CTL[122] (the problematic one)
    ctl122 = jtag.jtag2axi_read(0x01E8)
    print(f"  CTL[122] @ 0x01E8 = 0x{ctl122:08X}")

except Exception as e:
    print(f"  CTL register read failed: {e}")

try:
    pi0_after = jtag.jtag2axi_read(0x2000)
    print(f"  PI[0]  @ 0x2000 = 0x{pi0_after:08X}")

    pi1 = jtag.jtag2axi_read(0x2004)
    print(f"  PI[1]  @ 0x2004 = 0x{pi1:08X}")
except Exception as e:
    print(f"  PI register read failed: {e}")

# Test 2: Check what config registers look like
print("\n" + "="*70)
print("[TEST 2] Reading control/status registers")
print("="*70)

print("\nControl registers (< 0x80):")
for addr in [0x00, 0x04, 0x08, 0x0C, 0x10, 0x14]:
    try:
        val = jtag.jtag2axi_read(addr)
        print(f"  Addr 0x{addr:04X} = 0x{val:08X}")
    except Exception as e:
        print(f"  Addr 0x{addr:04X} failed: {e}")

# Test 3: Timing test - how fast are JTAG reads?
print("\n" + "="*70)
print("[TEST 3] JTAG read timing test")
print("="*70)

print("\nTiming 10 consecutive CTL[0] reads:")
times = []
for i in range(10):
    start = time.time()
    val = jtag.jtag2axi_read(0x0000)
    elapsed = (time.time() - start) * 1000
    times.append(elapsed)
    print(f"  Read {i+1}: {elapsed:.1f}ms, value = 0x{val:08X}")

avg_time = sum(times) / len(times)
print(f"\nAverage read time: {avg_time:.1f}ms")

print("\nTiming 10 consecutive PI[0] reads:")
times_pi = []
for i in range(10):
    start = time.time()
    val = jtag.jtag2axi_read(0x2000)
    elapsed = (time.time() - start) * 1000
    times_pi.append(elapsed)
    print(f"  Read {i+1}: {elapsed:.1f}ms, value = 0x{val:08X}")

avg_time_pi = sum(times_pi) / len(times_pi)
print(f"\nAverage PI read time: {avg_time_pi:.1f}ms")

# Test 4: Write test to CTL register
print("\n" + "="*70)
print("[TEST 4] CTL register write test")
print("="*70)

# Read a safe register first
print("\nReading CTL[1] before write...")
ctl1_before = jtag.jtag2axi_read(0x0004)
print(f"  CTL[1] before = 0x{ctl1_before:08X}")

print("\nWriting 0x12345678 to CTL[1]...")
try:
    jtag.jtag2axi_write(0x0004, 0x12345678)
    time.sleep(0.01)
    ctl1_after = jtag.jtag2axi_read(0x0004)
    print(f"  CTL[1] after  = 0x{ctl1_after:08X}")
    if ctl1_after != ctl1_before:
        print(f"  ✓ Write succeeded (value changed)")
    else:
        print(f"  ? Write may have failed or register is read-only")
except Exception as e:
    print(f"  ✗ Write failed: {e}")

# Test 5: Check initialization.py functions
print("\n" + "="*70)
print("[TEST 5] Using initialization.py functions")
print("="*70)

from initialization import initialization

init = initialization(jtag)

print("\nCalling init.read_ctl_id()...")
ctl_id = init.read_ctl_id()
print(f"  CTL Controller ID = 0x{ctl_id:04X}")
if ctl_id == 0x2040:
    print(f"  ✓ CTL ID correct")
else:
    print(f"  ✗ CTL ID incorrect (expected 0x2040)")

print("\nCalling init.read_pi_id()...")
pi_id = init.read_pi_id()
print(f"  PI Controller ID = 0x{pi_id:04X}")
if pi_id == 0x2040:
    print(f"  ✓ PI ID correct")
else:
    print(f"  ✗ PI ID incorrect (expected 0x2040)")

# Summary
print("\n" + "="*70)
print("TEST SUMMARY")
print("="*70)
print(f"""
CTL[0] accessible: {'YES' if 'ctl0_after' in locals() and ctl0_after != 0 else 'NO'}
PI[0] accessible:  {'YES' if 'pi0_after' in locals() and pi0_after != 0 else 'NO'}
CTL ID correct:    {'YES' if ctl_id == 0x2040 else 'NO'}
PI ID correct:     {'YES' if pi_id == 0x2040 else 'NO'}

Average JTAG CTL read time: {avg_time:.1f}ms
Average JTAG PI read time:  {avg_time_pi:.1f}ms
""")

print("="*70)
print("Debug script completed")
print("="*70)

session.disconnect()
