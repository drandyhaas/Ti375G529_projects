# LPDDR4 DDR Memory Controller Project

This project implements a DDR memory test system for the Efinix Ti375C529 FPGA development board with LPDDR4x memory.

## Overview

The system provides hardware auto-initialization of the DDR controller and high-performance memory testing with full data verification over USB3.

## Key Features

- **Hardware Auto-Initialization**: DDR controller automatically initializes on FPGA power-up
- **USB3 Control Interface**: Python-based control and testing over FT60X USB3 bridge
- **Verified Memory Testing**: LFSR-based pattern generation with 100% data verification
- **High Performance**: 74.36 Gb/s (93% efficiency) using single AXI port
- **Data Integrity**: Every byte written is verified on readback

## Performance Results

- **Bandwidth**: 74.36 Gb/s (9294.6 MB/s)
- **Efficiency**: 92.9% of theoretical maximum (80 Gb/s)
- **Data Verification**: 100% - all tests verify every byte written
- **Test Size**: 512MB write + 512MB read = 1073.7 MB per test
- **Test Duration**: ~0.116s per 512MB test

## Architecture

### Hardware Components

The design uses **7 RTL files**:

#### Core Modules
1. **`rtl/top.v`** - Top-level FPGA module
2. **`rtl/tools_core.v`** - Main system integration
   - DDR controller interface
   - Hardware auto-initialization
   - USB and memory test integration

3. **`rtl/memory_checker_lfsr.v`** - DDR memory tester
   - 128-bit maximal-length LFSR pattern generator
   - Write-then-read with full verification
   - 256-transfer bursts × 64 bytes = 16KB per burst
   - 8 outstanding AXI IDs for parallel transactions
   - Bit-level failure detection

#### USB and Control
4. **`rtl/usb_command_handler.v`** - USB command protocol
   - REG_WRITE/REG_READ commands
   - Version query (0x20251123)

5. **`rtl/usb2reg_bridge.v`** - Address decoder
   - Routes <0x80 to control registers
   - Routes ≥0x80 to DDR controller registers

6. **`rtl/axi_lite_slave.v`** - Control register bank
   - 18 control registers
   - Test control and status
   - Configuration interface

7. **`rtl/clock_beat.v`** - LED heartbeat generator

### DDR Initialization

Hardware-driven auto-initialization:
```verilog
assign cfg_sel   = 1'b0;          // Use built-in configuration
assign cfg_reset = 1'b0;          // No reset assertion
assign cfg_start = ddr_pll_lock;  // Start when PLL locks

// All resets driven by PLL lock
assign phy_rstn      = ddr_pll_lock;
assign ctrl_rstn     = ddr_pll_lock;
assign regARESETn    = ddr_pll_lock;
assign axi0_ARESETn  = ddr_pll_lock;
```

### Memory Test Operation

**Write Phase:**
1. Generate LFSR pseudo-random pattern (128-bit maximal-length)
2. Write to DDR in 256-transfer bursts (16KB each)
3. Rotate through 8 AXI IDs for parallel transactions
4. Continue until test size reached

**Read Phase:**
1. Regenerate same LFSR pattern from same seed
2. Read data from DDR
3. Compare every byte against expected LFSR value
4. Set fail flag if ANY mismatch detected

**Data Verification:**
```verilog
// Compare read data with expected pattern
if ((rdata & mask) !== rdata_store) begin
    rdata_obs <= rdata;           // Store actual data
    rdata_exp <= rdata ^ rdata_store;  // XOR shows failed bits
    fail <= 1'b1;                 // Flag test failure
end
```

### Control Register Map

```
0x00 - REG_0_DQ_FAIL   : Failed DQ bits [RO]
0x04 - REG_1_STATUS    : done (bit 0), fail (bit 1) [RO]
0x08 - REG_2_CONTROL   : start (bit 0), rstn (bit 1) [WO]
0x10 - REG_4_DATA_L    : Fixed pattern lower 32 bits [WO]
0x14 - REG_5_DATA_H    : Fixed pattern upper 32 bits [WO]
0x18 - REG_6_LFSR      : LFSR enable (1=LFSR, 0=fixed) [WO]
0x24 - REG_9_SIZE      : Test size in bytes [WO]
0x28 - REG_10_CONFIG   : cfg_done (bit 3) [RO]
```

## Test Scripts

### `test_hardware_autoinit.py`
Quick DDR functionality test:
- Waits 1 second for DDR initialization
- Checks cfg_done status
- Runs 4MB memory test with verification
- Reports PASS/FAIL with data integrity check

**Usage**:
```bash
python test_hardware_autoinit.py
```

### `test_bandwidth_accurate.py`
Accurate bandwidth measurement with data verification:

**Strategy:**
1. **Overhead measurement**: 5× 1MB tests to establish baseline
2. **Bandwidth measurement**: 5× 512MB tests (write + read)
3. **Overhead compensation**: Subtract overhead to get true DDR performance

**Verification:**
- Every test performs full write/read comparison
- LFSR pattern ensures pseudo-random data coverage
- Prints "PASS (data verified)" for each successful test
- Final "DATA INTEGRITY" section confirms no corruption

**Usage**:
```bash
python test_bandwidth_accurate.py
```

**Example Output:**
```
Step 1: Measuring Python/USB overhead with 1MB test...
  Run 1: 0.009234s (92 polls) - PASS (data verified)
  ...

Step 2: Running 512MB tests...
  Run 1: 0.115678s (1156 polls) - 74.36 Gb/s - PASS (data verified)
  ...

======================================================================
DATA INTEGRITY
======================================================================
All tests PASSED with 100% data verification
- Write: LFSR pseudo-random pattern generated
- Read: Every byte compared against expected LFSR value
- Result: NO data corruption detected
```

## Building and Programming

### Prerequisites
- Efinix Efinity IDE
- FT60X USB3 drivers
- Python 3.x with `pylibftdi`

### Build Steps
1. Open project in Efinity IDE
2. Ensure these 7 RTL files are included in `tools_core.xml`:
   - `rtl/top.v`
   - `rtl/tools_core.v`
   - `rtl/memory_checker_lfsr.v`
   - `rtl/axi_lite_slave.v`
   - `rtl/usb_command_handler.v`
   - `rtl/usb2reg_bridge.v`
   - `rtl/clock_beat.v`
3. Click "Compile" to generate bitstream
4. Click "Program" to load onto Ti375C529 board

### Quick Test
```bash
# Test DDR initialization and basic functionality
python test_hardware_autoinit.py

# Measure accurate bandwidth with verification
python test_bandwidth_accurate.py
```

## Technical Details

### LFSR Pattern Generation

128-bit maximal-length LFSR for excellent test coverage:

```verilog
// Feedback polynomial (XNOR taps at bits 98, 100, 125, 127)
assign w_fb_0P = r_lfsr_1P[98] ^~ r_lfsr_1P[100] ^~
                 r_lfsr_1P[125] ^~ r_lfsr_1P[127];

// Shift and feedback
r_lfsr_1P <= {r_lfsr_1P[126:0], w_fb_0P};

// Generate 512-bit data from 128-bit LFSR
wdata <= {4{r_lfsr_1P}} & mask;
```

**Why LFSR?**
- Deterministic: Same seed produces same sequence
- Pseudo-random: Excellent pattern coverage
- Repeatable: Write and read use same sequence for verification
- Efficient: Simple hardware implementation

### Performance Optimizations

1. **256-transfer bursts** (ALEN=255)
   - 256 × 64 bytes = 16KB per burst
   - Reduces per-burst AXI overhead

2. **8 outstanding AXI IDs**
   - Cycles through IDs 0-7
   - Allows multiple transactions in-flight
   - Keeps AXI bus fully utilized

3. **Sequential addressing**
   - Simple address increment
   - ADDR_OFFSET = 16384 bytes (256 × 64)

### DDR Configuration
- **Memory Type**: LPDDR4x
- **Controller**: Efinix built-in LPDDR4 controller
- **Data Width**: 512 bits (64 bytes) per AXI transfer
- **AXI Ports**: Single port (AXI0) for memory testing
- **Theoretical Max**: 80 Gb/s per AXI port
- **Achieved**: 74.36 Gb/s (92.9% efficiency)

## Project Structure

```
LPDDR4/
├── rtl/                          # RTL source files
│   ├── top.v                     # Top-level module
│   ├── tools_core.v              # Main system integration
│   ├── memory_checker_lfsr.v    # Memory test module
│   ├── axi_lite_slave.v          # Control registers
│   ├── usb_command_handler.v    # USB command protocol
│   ├── usb2reg_bridge.v         # Address decoder
│   └── clock_beat.v             # LED heartbeat
├── bsp/TI375C529/
│   ├── outflow/
│   │   └── tools_core.bit       # Generated bitstream
│   └── tools_core.xml           # Project configuration
├── usb_ddr_control.py           # Python USB control library
├── test_hardware_autoinit.py    # Quick DDR test
├── test_bandwidth_accurate.py   # Bandwidth + verification test
└── README.md                    # This file
```

## Troubleshooting

### DDR Not Initializing
- Check `cfg_done` bit (REG_10_CONFIG bit 3)
- Wait at least 1 second after FPGA configuration
- Verify `ddr_pll_lock` signal in simulation

### Memory Test Failures
- Check `REG_0_DQ_FAIL` for failed data bits
- Review which DQ pins are failing
- Verify signal integrity on DDR traces
- Check power supply voltages

### Low Bandwidth
- Verify ALEN=255 in memory_checker_lfsr.v
- Check that 8 AXI IDs are cycling (axi_id_counter)
- Monitor for AXI ready signal stalls
- Review AXI clock frequency

## References

- Efinix Ti375C529 FPGA documentation
- Efinix Ti DDR Controller User Guide v2.8
- LPDDR4 JEDEC specification
- AXI4 protocol specification (ARM IHI 0022E)
- FT60X USB3 device documentation

## License

This project is part of the Ti375G529 development tools.
