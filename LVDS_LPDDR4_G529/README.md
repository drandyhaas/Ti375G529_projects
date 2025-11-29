# LVDS LPDDR4 Scope Project

High-speed data acquisition system combining LVDS inputs with LPDDR4 memory on the Efinix Ti375C529 FPGA.

## Overview

This project implements a digital oscilloscope with LVDS data inputs, LPDDR4 memory buffering, and USB3 control interface. The system captures high-speed signals through LVDS receivers, processes them through downsampling and triggering logic, and stores waveforms in LPDDR4 memory for readback over USB3.

## Key Features

- **LVDS Input**: 4-channel 10-bit LVDS data acquisition at high sample rates
- **LPDDR4 Memory**: Large waveform buffer storage with dual-port RAM interface
- **USB3 Control**: FT60X USB3 bridge for command/control and data readback (~280 MB/s)
- **Triggering System**: Configurable threshold-based triggering with multiple modes
- **Downsampling**: Adjustable sample rate reduction with configurable merging
- **Phase Detection**: Precise timing measurement between LVDS trigger signals
- **Unified Command Interface**: All commands handled by command_processor (8-byte format)

## Architecture

### Clock Domains

- **clk_command (200 MHz)**: Command processor, USB interface, sample RAM read port
- **clk50 (50 MHz)**: Slow peripherals (fan PWM, flash clock, PLL reset)
- **axi0_ACLK (200 MHz)**: DDR AXI interface, memory test timing
- **regACLK (100 MHz)**: DDR register interface
- **lvds_clk**: LVDS data sampling

The command_processor runs at 200 MHz for maximum USB throughput. An AXI-Lite CDC (Clock Domain Crossing) module bridges between clk_command and regACLK domains for DDR register access.

### Hardware Components

#### Top-Level Integration (`rtl/top.v`)
- FPGA pin assignments and I/O
- Clock domain management
- Module instantiation and interconnection

#### Core System (`rtl/tools_core.v`)
- USB3 FT60X interface (ftdi_245fifo)
- AXI-Lite CDC for DDR register access
- Scope interface routing to command_processor

#### Scope Control Modules

**`rtl_scope/command_processor.v`** - Unified command processor handling:
- Scope commands (0x00-0x17): Acquisition, trigger, readout
- USB utility commands (0x20-0x25): Bandwidth test, register access, echo

**`rtl_scope/triggerer.v`** - Trigger detection and acquisition control

**`rtl_scope/downsampler.v`** - Sample rate reduction with merging modes

**`rtl_scope/phase_detector.v`** - Timing measurement between trigger signals

**`rtl_scope/sample_ram.v`** - Dual-port RAM buffer (1024 × 560 bits)

**`rtl/axi_lite_cdc.v`** - Clock domain crossing for AXI-Lite (clk_command → regACLK)

### Signal Flow

```
LVDS Input → Downsampler → Triggerer → Sample RAM → Command Processor → USB3 Output
              ↑                ↓
         Phase Detector    Trigger Logic
```

## Command Protocol

All commands use 8-byte format sent directly to command_processor.

### Scope Commands (0x00-0x17)

**Command 0 - Read RAM Buffer:**
```python
cmd = bytes([0x00, 0, 0, 0, len_b0, len_b1, len_b2, len_b3])
# Response: Streaming scope data (200 bytes per RAM row)
```

**Command 1 - Arm Trigger:**
```python
cmd = bytes([0x01, triggertype, channeltype, 0, len_lo, len_hi, 0, 0])
# Response: 4 bytes (acqstate, sample_triggered info)
```

**Command 2 - Status/Version Queries:**
```python
cmd = bytes([0x02, subcommand, param, ...])
# Response: 4 bytes (varies by subcommand)
```

### USB Utility Commands (0x20-0x25)

**0x20 - TX_MASS (Bandwidth Test):**
```python
cmd = bytes([0x20, len_b0, len_b1, len_b2, len_b3, 0, 0, 0])
# Response: Counting pattern of specified length
```

**0x21 - REG_WRITE:**
```python
cmd = bytes([0x21, addr_lo, addr_hi, data_b0, data_b1, data_b2, data_b3, 0])
# Response: 4 bytes (0x00000000 = success)
```

**0x22 - REG_READ:**
```python
cmd = bytes([0x22, addr_lo, addr_hi, 0, 0, 0, 0, 0])
# Response: 4 bytes (register value)
```

**0x23 - GET_VERSION:**
```python
cmd = bytes([0x23, 0, 0, 0, 0, 0, 0, 0])
# Response: 4 bytes (0x20251125)
```

**0x24 - GET_STATUS:**
```python
cmd = bytes([0x24, 0, 0, 0, 0, 0, 0, 0])
# Response: 4 bytes (state machine debug info)
```

**0x25 - ECHO:**
```python
cmd = bytes([0x25, len_lo, len_hi, data0, data1, data2, data3, data4])
# + additional data bytes if len > 5
# Response: Echoed data
```

## Test Results

### DDR Bandwidth (test_bandwidth_accurate.py)

LPDDR4 memory bandwidth measured with hardware cycle counters (1023 MB test, AXI @ 200 MHz):

| Operation | Bandwidth | Notes |
|-----------|-----------|-------|
| Write | 77.25 Gb/s (9,657 MB/s) | Hardware-timed |
| Read | 85.57 Gb/s (10,696 MB/s) | Hardware-timed, verified |
| Combined (W+R) | 81.20 Gb/s (10,150 MB/s) | Sequential write then read |

Memory test modes:
- **Mode 0 (Write+Read)**: Full write followed by verified read
- **Mode 1 (Write-only)**: Write without verification
- **Mode 2 (Read-only)**: Verify previously written data

Hardware cycle counters eliminate Python/USB overhead for accurate DDR timing.

### USB/Scope Bandwidth (test_scope_bandwidth.py)

Scope RAM readout performance at 200 MHz clk_command:

| Transfer Size | Avg Rate | Peak Rate |
|--------------|----------|-----------|
| 200 B | 0.5 MB/s | 0.6 MB/s |
| 1 KB | 2.2 MB/s | 3.1 MB/s |
| 4 KB | 8.5 MB/s | 11.4 MB/s |
| 16 KB | 16.2 MB/s | 21.1 MB/s |
| 64 KB | 49.8 MB/s | 63.5 MB/s |
| 100 KB | 82.9 MB/s | 101.6 MB/s |
| 256 KB | 112.5 MB/s | 125.2 MB/s |
| 512 KB | 163.9 MB/s | 191.6 MB/s |
| 1 MB | 190.6 MB/s | 206.3 MB/s |
| 2 MB | 240.5 MB/s | 245.7 MB/s |
| 5 MB | 259.0 MB/s | 264.0 MB/s |
| 15 MB | 280.2 MB/s | 285.9 MB/s |

**Peak: ~286 MB/s, Large transfers (>2MB): ~280 MB/s**

### Echo Test (test_echo.py)

All 7 tests pass - verifies command/response integrity at 200 MHz.

### DDR Memory

DDR registers accessible via AXI-Lite CDC at full speed. Memory test functionality available through register interface with configurable test modes and hardware timing.

## Test Scripts

| Script | Description |
|--------|-------------|
| `test_bandwidth_accurate.py` | DDR bandwidth with hardware cycle counters |
| `test_echo.py` | Verify echo command (data integrity) |
| `test_scope_bandwidth.py` | Measure scope readout bandwidth |
| `test_scope_commands.py` | Test scope command interface |
| `test_stale_data.py` | Check for leftover USB data |
| `test_version.py` | Query firmware version |
| `usb_rx_mass.py` | USB bandwidth test (TX_MASS) |

## Python Control Library

### USB_FTX232H_FT60X.py

Low-level USB interface for FT60X:

```python
from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode

usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
    ('FT60X', 'Haasoscope USB3'),
    ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')))

usb.send(command_bytes)
response = usb.recv(num_bytes)
usb.close()
```

### usb_ddr_control.py

High-level DDR control interface:

```python
from usb_ddr_control import USBDDRControl

ctrl = USBDDRControl()
ctrl.reg_write(0x28, 0x00)
value = ctrl.reg_read(0x28)
ctrl.memtest_run(size_mb=4, lfsr_en=True)
ctrl.close()
```

## Project Structure

```
LVDS_LPDDR4/
├── rtl/
│   ├── top.v                  # Top-level FPGA module
│   ├── tools_core.v           # Core system (USB, CDC, routing)
│   ├── axi_lite_cdc.v         # Clock domain crossing for AXI-Lite
│   ├── axi_lite_slave.v       # DDR control registers
│   ├── usb2reg_bridge.v       # AXI-Lite address decoder
│   ├── memory_checker_lfsr.v  # DDR memory tester
│   └── clock_beat.v           # LED heartbeat
├── rtl_scope/
│   ├── command_processor.v    # Unified command processor
│   ├── triggerer.v            # Trigger detection
│   ├── downsampler.v          # Sample rate reduction
│   ├── phase_detector.v       # Timing measurement
│   ├── sample_ram.v           # Dual-port waveform buffer
│   ├── SPI_Master.v           # SPI peripheral control
│   ├── neo_driver.v           # RGB LED driver
│   └── pwm_generator.v        # Fan PWM control
├── ftdi245fifo/               # FT60X USB interface IP
├── bsp/TI375C529/             # Board support package
├── USB_FTX232H_FT60X.py       # Low-level USB interface
├── usb_ddr_control.py         # High-level DDR control
├── test_*.py                  # Test scripts
└── README.md
```

## Building

### Prerequisites
- Efinix Efinity IDE 2025.1+
- Python 3.x with ftd3xx package
- FT60X USB3 drivers

### Build Steps
1. Open `bsp/TI375C529/tools_core.xml` in Efinity IDE
2. Synthesize and place/route
3. Program FPGA with generated bitstream

### Timing Constraints

Critical clock constraints in `tools_core.pt.sdc` and `tools_core_custom.sdc`:

| Clock | Frequency | Slack | Notes |
|-------|-----------|-------|-------|
| axi0_ACLK | 200 MHz | +0.276 ns | DDR AXI interface |
| clk_command | 200 MHz | +0.644 ns | USB/command processor |
| regACLK | 100 MHz | +5.777 ns | DDR register interface |
| lvds_clk_fast | 750 MHz | +0.535 ns | LVDS high-speed sampling |
| lvds_clk_slow | 150 MHz | +0.972 ns | LVDS data processing |
| ftdi_clk | 100 MHz | +5.610 ns | USB FT60X interface |

**Clock Domain Crossings (CDC):**
- `lvds_clk_slow ↔ clk_command`: False-pathed (sample_ram uses dual-port RAM)
- `lvds_clk_slow → lvds_clk_fast`: False-pathed (phase detector input)
- `lvds_clk_fast → clk_command`: False-pathed (2-FF synchronizer on phase_diff)
- `clk_command ↔ regACLK`: Handshake-based CDC in axi_lite_cdc.v

## Register Map (DDR Controller)

| Address | Name | Description |
|---------|------|-------------|
| 0x00 | DQ_FAIL | Failed DQ bits [RO] |
| 0x04 | STATUS | memtest_done, memtest_fail [RO] |
| 0x08 | CONTROL | memtest_start, memtest_rstn [WO] |
| 0x0C | RESET | DDR reset controls [WO] |
| 0x10 | DATA_L | Test pattern lower 32 bits [WO] |
| 0x14 | DATA_H | Test pattern upper 32 bits [WO] |
| 0x18 | LFSR | LFSR enable [WO] |
| 0x1C | MODE | bits[0]=x16_en, bits[2:1]=test_mode (0=W+R, 1=W, 2=R) [WO] |
| 0x20 | ARLEN | AXI read burst length [WO] |
| 0x24 | SIZE | Test size in bytes (max 1023 MB) [WO] |
| 0x28 | CONFIG | cfg_done [RO], config control [WO] |
| 0x48 | WRITE_CYC_L | Write phase cycle count [31:0] [RO] |
| 0x4C | WRITE_CYC_H | Write phase cycle count [63:32] [RO] |
| 0x50 | READ_CYC_L | Read phase cycle count [31:0] [RO] |
| 0x54 | READ_CYC_H | Read phase cycle count [63:32] [RO] |

## Troubleshooting

### No USB Communication
- Verify FT60X drivers installed
- Check USB device enumeration (`lsusb` or Device Manager)
- Try different USB3 port/cable

### Low Bandwidth
- Ensure clk_command is 200 MHz
- Check timing constraints are met
- Use larger transfer sizes (>100KB for best throughput)

### Stale Data After Tests
- Run `python test_stale_data.py` to drain buffer
- Check timing constraints for clk_command

### DDR Access Issues
- Verify AXI-Lite CDC is instantiated
- Check regACLK is running (100 MHz)
- Confirm cfg_done bit is set

## License

Part of the Ti375G529 development tools.
