# LVDS LPDDR4 Scope Project

High-speed data acquisition system combining LVDS inputs with LPDDR4 memory on the Efinix Ti375C529 FPGA.

## Overview

This project implements a digital oscilloscope with LVDS data inputs, LPDDR4 memory buffering, and USB3 control interface. The system captures high-speed signals through LVDS receivers, processes them through downsampling and triggering logic, and stores waveforms in LPDDR4 memory for readback over USB3.

## Key Features

- **LVDS Input**: 4-channel 10-bit LVDS data acquisition at high sample rates
- **LPDDR4 Memory**: Large waveform buffer storage with dual-port RAM interface
- **USB3 Control**: FT60X USB3 bridge for command/control and data readback
- **Triggering System**: Configurable threshold-based triggering with multiple modes
- **Downsampling**: Adjustable sample rate reduction with configurable merging
- **Phase Detection**: Precise timing measurement between LVDS trigger signals
- **Dual Command Interface**:
  - Direct command_processor access for scope control (8-byte commands)
  - USB handler commands with 0xFE prefix for utility functions

## Architecture

### Hardware Components

#### Top-Level Integration (`rtl/top.v`)
- FPGA pin assignments and I/O
- Clock domain management (LVDS clocks, USB clock, system clocks)
- Module instantiation and interconnection
- LVDS input ports (lvds_rx_inst1_RX_DATA[9:0])
- USB3 interface (32-bit data bus with flow control)
- Peripheral I/O (SPI, flash, debug, LEDs, triggers)

#### Core System (`rtl/tools_core.v`)
- USB3 FT60X interface wrapper
- FPGA peripheral control (SPI, flash, LEDs)
- AXI-Lite bridge for register access
- Scope interface routing to command_processor

#### Scope Control Modules

**`rtl_scope/command_processor.v`** - Main scope command processor
- 8-byte command protocol (commands 0x00-0x17)
- RAM readout with streaming TX_DATA states
- SPI control for peripherals
- PLL phase adjustment
- Trigger configuration
- Flash memory programming
- Command 0: Read RAM buffer (streams large data)
- Command 1: Configure acquisition parameters
- Command 2: Status and version queries
- Command 8: Trigger threshold settings
- Command 9: Downsampling configuration

**`rtl_scope/triggerer.v`** - Trigger detection and acquisition control
- Configurable threshold triggering
- Pre-trigger sample buffering
- Time-over-threshold (ToT) measurement
- Trigger holdoff and delay
- RAM write address management
- Multiple trigger modes (rising, falling, level)

**`rtl_scope/downsampler.v`** - Sample rate reduction
- Configurable downsampling factor (1-32)
- Multiple merging modes (average, min, max, first, last)
- 4-channel simultaneous processing
- 140-bit input (4×10-bit channels + overhead)
- 560-bit output buffer (40 samples)

**`rtl_scope/phase_detector.v`** - Timing measurement
- Fast clock edge detection on LVDS triggers
- Phase difference calculation
- Supports both main and secondary trigger inputs
- 16-bit phase measurement output

**`rtl_scope/sample_ram.v`** - Dual-port RAM buffer
- 1024×560-bit dual-port RAM
- Independent write/read clock domains
- Write side: LVDS clock domain for sample capture
- Read side: System clock for USB readout
- 560 bits = 40 samples × 14 bits (data + metadata)

#### USB Command Interface

**`rtl/usb_command_handler.v`** - USB command protocol handler
- **Default behavior**: Forward 8-byte commands directly to command_processor
- **USB handler commands**: Require 0xFE prefix byte
  - `0xFE 0x01 [4B length]` - TX_MASS: Send test data pattern
  - `0xFE 0x02 [4B addr][4B data]` - REG_WRITE: Write AXI-Lite register
  - `0xFE 0x03 [4B addr]` - REG_READ: Read AXI-Lite register (returns 4 bytes)
  - `0xFE 0x04` - GET_VERSION: Get firmware version (returns 4 bytes)
  - `0xFE 0x05` - GET_STATUS: Get hardware status (returns 4 bytes)
- AXI-Stream protocol for command input and data output
- Streaming data support for large transfers (command_processor command 0)
- Watchdog timeout protection (2 seconds)

**`rtl/usb2reg_bridge.v`** - Address decoder for AXI-Lite
- Routes addresses to appropriate target modules

**`rtl/axi_lite_slave.v`** - Control register bank
- Memory test control
- System status and configuration
- DDR controller interface

### Signal Flow

```
LVDS Input → Downsampler → Triggerer → Sample RAM → Command Processor → USB3 Output
              ↑                ↓
         Phase Detector    Trigger Logic
```

**Acquisition Path:**
1. LVDS receivers capture 10-bit differential signals
2. Downsampler reduces sample rate and merges samples
3. Triggerer monitors for trigger conditions
4. Samples written to dual-port RAM at trigger point
5. Command processor reads RAM and streams to USB3

**Control Path:**
1. USB3 receives 8-byte commands from PC
2. Commands forwarded to command_processor (or handled by usb_command_handler if prefixed with 0xFE)
3. Command processor configures triggering, downsampling, acquisition
4. Status and waveform data streamed back to PC

## Command Protocol

### Scope Commands (Direct to command_processor)

Send 8 bytes directly without prefix:

```
[cmd_byte] [param1] [param2] [param3] [param4] [param5] [param6] [param7]
```

**Command 0 - Read RAM Buffer:**
```python
cmd = bytes([
    0x00,           # Command
    0x00, 0x00, 0x00,  # Reserved
    length & 0xFF, (length >> 8) & 0xFF, (length >> 16) & 0xFF, (length >> 24) & 0xFF
])
# Response: Streaming 32-bit data with AXI-Stream protocol (tvalid/tready/tdata/tlast)
```

**Command 1 - Configure Acquisition:**
```python
cmd = bytes([
    0x01,           # Command
    triggertype,    # Trigger type
    channeltype,    # Channel configuration
    0x00,           # Reserved
    length & 0xFF, (length >> 8) & 0xFF,  # Length to capture
    0x00, 0x00
])
# Response: 4 bytes (acqstate and sample_triggered info)
```

### USB Handler Commands (Require 0xFE Prefix)

**Register Write:**
```python
cmd = bytes([
    0xFE, 0x02,     # Prefix + REG_WRITE
    addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF, (addr >> 24) & 0xFF,
    data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF
])
# No response
```

**Register Read:**
```python
cmd = bytes([
    0xFE, 0x03,     # Prefix + REG_READ
    addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF, (addr >> 24) & 0xFF
])
# Response: 4 bytes (register value, little-endian)
```

**Get Version:**
```python
cmd = bytes([0xFE, 0x04])
# Response: 4 bytes (version 0x20251123)
```

**Get Status:**
```python
cmd = bytes([0xFE, 0x05])
# Response: 4 bytes (bit[0]=ddr_pll_lock, bit[1]=axi_arready, bit[2]=axi_rvalid)
```

## Python Control Library

### `usb_ddr_control.py`

Provides high-level Python interface for USB commands:

```python
from usb_ddr_control import USBDDRControl

usb = USBDDRControl()

# Register access (automatically adds 0xFE prefix)
usb.reg_write(0x28, 0x00)          # Write to register
value = usb.reg_read(0x28)          # Read from register
status = usb.get_status()           # Get hardware status

# Memory testing
usb.memtest_run(size_mb=4, lfsr_en=True, verbose=True)

usb.close()
```

### Test Scripts

**`test_version.py`** - Test firmware version command
```bash
python test_version.py
# Expected: 0x20251123 (2025-11-23)
```

**`usb_rx_mass.py`** - USB bandwidth test
```bash
python usb_rx_mass.py
# Tests TX_MASS command with 10MB transfers
```

**`test_hardware_autoinit.py`** - DDR initialization test
```bash
python test_hardware_autoinit.py
# Verifies DDR auto-init and runs 4MB memory test
```

**`test_bandwidth_accurate.py`** - DDR bandwidth measurement
```bash
python test_bandwidth_accurate.py
# Measures DDR performance with overhead compensation
```

## Register Map

### Control Registers (via AXI-Lite)

```
0x00 - REG_0_DQ_FAIL      : Failed DQ bits [RO]
0x04 - REG_1_STATUS       : bit[0]=memtest_done, bit[1]=memtest_fail [RO]
0x08 - REG_2_CONTROL      : bit[0]=memtest_start, bit[1]=memtest_rstn [WO]
0x0C - REG_3_RESET        : DDR reset controls [WO]
0x10 - REG_4_DATA_L       : Test pattern lower 32 bits [WO]
0x14 - REG_5_DATA_H       : Test pattern upper 32 bits [WO]
0x18 - REG_6_LFSR         : LFSR enable [WO]
0x1C - REG_7_X16          : x16 mode enable [WO]
0x20 - REG_8_ARLEN        : AXI read burst length [WO]
0x24 - REG_9_SIZE         : Test size in bytes [WO]
0x28 - REG_10_CONFIG      : bit[3]=cfg_done [RO], config control [WO]
```

## Building and Running

### Prerequisites
- Efinix Efinity IDE
- Python 3.x
- pylibftdi (for USB communication)
- FT60X USB3 drivers

### Build Steps
1. Open project in Efinity IDE
2. Compile to generate bitstream
3. Program FPGA

### Quick Test
```bash
# Test USB communication
python test_version.py

# Test scope command interface
# (Send 8-byte commands directly to command_processor)

# Test USB handler commands
python test_hardware_autoinit.py
```

## Project Structure

```
LVDS_LPDDR4/
├── rtl/
│   ├── top.v                      # Top-level FPGA module
│   ├── tools_core.v               # Core system integration
│   ├── usb_command_handler.v      # USB command protocol (0xFE prefix)
│   ├── usb2reg_bridge.v           # Address decoder
│   ├── axi_lite_slave.v           # Control registers
│   ├── memory_checker_lfsr.v      # DDR memory tester
│   └── clock_beat.v               # LED heartbeat
├── rtl_scope/
│   ├── command_processor.v        # Scope command processor (8-byte protocol)
│   ├── triggerer.v                # Trigger detection
│   ├── downsampler.v              # Sample rate reduction
│   ├── phase_detector.v           # Timing measurement
│   └── sample_ram.v               # Dual-port waveform buffer
├── USB_FTX232H_FT60X.py          # Low-level USB interface
├── usb_ddr_control.py            # High-level USB control library
├── test_version.py                # Version query test
├── usb_rx_mass.py                 # USB bandwidth test
├── test_hardware_autoinit.py      # DDR initialization test
├── test_bandwidth_accurate.py     # DDR bandwidth measurement
└── README.md                      # This file
```

## Technical Details

### Clock Domains
- **lvds_clk_slow_clkin**: LVDS slow clock for data sampling
- **lvds_clk_fast_clkin**: LVDS fast clock for phase detection
- **clk_100**: 100 MHz system clock from USB/DDR
- **clk_50**: 50 MHz clock for command processor

### LVDS Interface
- 4 channels × 10 bits = 40-bit parallel input
- Differential signaling for noise immunity
- Requires deserialization logic (TODO in current implementation)

### Memory Buffer
- Dual-port RAM: 1024 addresses × 560 bits
- Write side: LVDS clock domain (async to system)
- Read side: System clock domain (sync to USB)
- Circular buffer with trigger-based addressing

### Trigger System
- Threshold-based edge detection
- Pre-trigger and post-trigger capture
- Time-over-threshold measurement
- Holdoff and delay controls
- Per-channel trigger selection

### Downsampling
- Reduces sample rate by factor of 1-32
- Merging modes: average, min, max, first, last
- Preserves trigger alignment
- 40-sample output buffer for efficient RAM writes

## Troubleshooting

### No USB Communication
- Check FT60X drivers installed
- Verify USB device enumeration
- Try different USB3 port/cable

### Commands Not Working
- Scope commands: Send 8 bytes directly (no prefix)
- USB handler commands: Add 0xFE prefix before command byte
- Check command byte values match protocol

### No Trigger Events
- Verify trigger threshold settings (command 8)
- Check LVDS input signal levels
- Enable trigger live mode (command 1)
- Review trigger type configuration

### RAM Readout Issues
- Ensure acquisition complete before readout
- Check RAM address triggered point
- Verify length parameter in command 0
- Monitor for AXI-Stream tlast signal

## References

- Efinix Ti375C529 FPGA Documentation
- FT60X USB3 Device Documentation
- AXI4 Protocol Specification
- LVDS Signaling Standard

## License

Part of the Ti375G529 development tools.
