# LPDDR4 DDR Memory Controller Project

This project implements a DDR memory test system for the Efinix Ti375C529 FPGA development board with LPDDR4x memory.

## Overview

The system provides hardware auto-initialization of the DDR controller and high-performance memory testing capabilities over USB3.

## Key Features

- **Hardware Auto-Initialization**: DDR controller automatically initializes on FPGA power-up/configuration
- **USB3 Control Interface**: Python-based control and testing over FT60X USB3 bridge
- **High-Performance Memory Testing**: LFSR-based pattern generation and verification
- **Optimized Bandwidth**: 74.36 Gb/s (93% efficiency) using single AXI port

## Performance Results

### Bandwidth Test Results
- **Current Performance**: 74.36 Gb/s (9294.6 MB/s)
- **Efficiency**: 92.9% of theoretical maximum (80 Gb/s)
- **Test Size**: 512MB write + 512MB read = 1073.7 MB per test
- **Test Duration**: ~0.116s per 512MB test

### Optimization History
| Configuration | Bandwidth | Efficiency | Improvement |
|--------------|-----------|------------|-------------|
| Original (64-burst, single ID) | 59.32 Gb/s | 74.1% | Baseline |
| Optimized (256-burst, 8 IDs) | 74.36 Gb/s | 92.9% | +25% |

### Key Optimizations Applied
1. **256-transfer bursts** (ALEN=255): Reduces per-burst overhead
2. **8 outstanding AXI IDs**: Allows multiple transactions in-flight simultaneously

## Architecture

### Hardware Components

#### DDR Controller (`rtl/tools_core.v`)
- LPDDR4x controller with AXI interfaces
- Hardware-driven initialization using DDR PLL lock signal
- Auto-configuration on FPGA startup

**Initialization Signals**:
```verilog
assign cfg_sel   = 1'b0;          // Use built-in configuration
assign cfg_reset = 1'b0;          // No reset assertion
assign cfg_start = ddr_pll_lock;  // Start when PLL locks

// All resets driven by PLL lock
assign phy_rstn      = ddr_pll_lock;
assign ctrl_rstn     = ddr_pll_lock;
assign regARESETn    = ddr_pll_lock;
assign axi0_ARESETn  = ddr_pll_lock;
assign axi1_ARESETn  = ddr_pll_lock;
```

#### Memory Checker (`rtl/memory_checker_lfsr.v`)
High-performance memory test module with:
- **LFSR pattern generation**: 128-bit maximal-length LFSR
- **Configurable burst length**: 256 transfers per burst (ALEN=255)
- **Multiple outstanding IDs**: 8 AXI IDs for parallel transactions
- **64-byte transfers**: 512-bit data width
- **Write-then-read verification**: Full memory write followed by readback

**Key Parameters**:
```verilog
parameter ALEN = 255;              // 256 transfers per burst
parameter ASIZE = 6;               // 64-byte transfers (2^6)
parameter START_ADDR = 33'h000000000;
parameter ADDR_OFFSET = (ALEN + 1)*64;  // 16384 bytes per burst
```

**AXI ID Management**:
```verilog
reg [2:0] axi_id_counter;          // Cycles 0-7
assign awid = {3'b000, axi_id_counter};  // Write ID
assign arid = {3'b000, axi_id_counter};  // Read ID
```

### Software Components

#### USB DDR Control (`usb_ddr_control.py`)
Python interface for DDR testing:
- FT60X USB3 device communication
- Register read/write functions
- Memory test execution
- Status monitoring

**Register Map** (AXI Lite Slave):
```
0x00 - REG_0_DQ_FAIL     : Failed DQ bits
0x04 - REG_1_STATUS      : Test status (done, fail)
0x08 - REG_2_CONTROL     : Test control (start/stop)
0x10 - REG_4_DATA_L      : Fixed pattern lower 32 bits
0x14 - REG_5_DATA_H      : Fixed pattern upper 32 bits
0x18 - REG_6_LFSR        : LFSR enable (1=LFSR, 0=fixed)
0x24 - REG_9_SIZE        : Test size in bytes
0x28 - REG_10_CONFIG     : DDR config status (cfg_done bit 3)
```

## Bandwidth Testing

### How the Bandwidth Test Works

The bandwidth test (`test_bandwidth_accurate.py`) measures true DDR performance by compensating for Python/USB overhead:

#### Test Strategy

1. **Overhead Measurement** (Step 1):
   - Run 5x small tests (1MB each)
   - Measure average time including Python/USB overhead
   - This establishes baseline overhead

2. **Large Test Execution** (Step 2):
   - Run 5x large tests (512MB each)
   - Each test performs:
     - Write 512MB to DDR
     - Read 512MB from DDR
     - Total data transferred: 1073.7 MB per test
   - Measure time for each test

3. **Overhead Compensation** (Step 3):
   - Use minimum test time (best case)
   - Subtract measured overhead
   - Calculate compensated bandwidth

#### Calculation Method

```python
# Total data per test
size_bytes = 512 * 1024 * 1024
total_bytes = 2 * size_bytes  # write + read

# Raw bandwidth (includes overhead)
bandwidth_raw_gbps = (total_bytes * 8) / avg_time / 1e9

# Compensated bandwidth (overhead removed)
compensated_time = min_time - overhead
bandwidth_comp_gbps = (total_bytes * 8) / compensated_time / 1e9

# Efficiency
efficiency = (bandwidth_comp_gbps / 80.0) * 100
```

#### Why This Method Works

- **Large test size (512MB)**: DDR transfer time >> overhead time
- **Multiple runs**: Statistical averaging reduces variance
- **Minimal polling**: 100us poll interval minimizes CPU overhead
- **Best-case timing**: Using minimum time reduces impact of OS scheduling jitter

#### Test Execution Flow

```
START
  ↓
Configure test (LFSR mode, 512MB size)
  ↓
Start test (set control register)
  ↓
Poll status register every 100us
  ↓
Done bit set? → Calculate elapsed time
  ↓
Check fail bit
  ↓
Report bandwidth
```

### Memory Test Operations

The memory checker performs these operations:

1. **Write Phase**:
   - Generate LFSR pattern
   - Issue AXI write bursts (256 transfers × 64 bytes = 16KB per burst)
   - Rotate through 8 AXI IDs
   - Increment address by ADDR_OFFSET (16384 bytes)
   - Continue until test_size reached

2. **Read Phase**:
   - Regenerate same LFSR pattern
   - Issue AXI read bursts
   - Compare read data with expected pattern
   - Flag any mismatches in DQ_FAIL register

### LFSR Pattern Generation

The test uses a maximal-length 128-bit LFSR:

```verilog
// Feedback polynomial for 128-bit maximal-length LFSR
assign w_fb_0P = r_lfsr_1P[98] ^~ r_lfsr_1P[100] ^~ r_lfsr_1P[125] ^~ r_lfsr_1P[127];

// Shift and feedback
r_lfsr_1P <= {r_lfsr_1P[126:0], w_fb_0P};

// Generate 512-bit data from 128-bit LFSR
wdata <= {4{r_lfsr_1P}} & mask;
```

This provides a pseudo-random pattern with excellent coverage for detecting memory errors.

## Test Scripts

### `test_hardware_autoinit.py`
Tests DDR after hardware auto-initialization:
- Waits 1 second for DDR init
- Checks cfg_done status
- Runs 4MB memory test
- **Usage**: `python test_hardware_autoinit.py`

### `test_bandwidth_accurate.py`
Measures DDR bandwidth with overhead compensation:
- 5× 1MB tests for overhead measurement
- 5× 512MB tests for bandwidth
- Reports raw and compensated bandwidth
- **Usage**: `python test_bandwidth_accurate.py`

### `test_bandwidth.py`
Simple progressive bandwidth test:
- Tests increasing sizes: 16MB, 64MB, 256MB, 512MB, 1GB, 2GB
- Reports bandwidth for each size
- Stops after 2 second duration or failure
- **Usage**: `python test_bandwidth.py`

## Building and Programming

### Prerequisites
- Efinix Efinity IDE
- FT60X USB3 drivers
- Python 3.x with `pylibftdi`

### Build Steps
1. Open project in Efinity IDE
2. Click "Compile" to generate bitstream
3. Click "Program" to load onto Ti375C529 board

### Quick Test
```bash
# Test DDR initialization and basic functionality
python test_hardware_autoinit.py

# Measure accurate bandwidth
python test_bandwidth_accurate.py
```

## Limitations and Future Improvements

### Current Limitations
- **Single AXI port**: Only AXI0 is used for testing
- **Sequential addressing**: No bank interleaving optimization
- **Fixed test pattern**: LFSR or fixed pattern only

### Potential Improvements
1. **Dual AXI port testing**: Use both AXI0 and AXI1 simultaneously
   - Could potentially double throughput to ~150 Gb/s

2. **Bank-interleaved addressing**: Optimize for LPDDR4 bank structure
   - Place bank selection at address bits [11:9]
   - Keep all accesses in Row 0 for maximum page hits

3. **Variable pattern testing**: Support multiple test patterns
   - Walking 1s/0s
   - Checkerboard
   - User-defined patterns

## Technical Notes

### DDR Configuration
- **Memory Type**: LPDDR4x
- **Controller**: Efinix built-in LPDDR4 controller
- **Data Width**: 512 bits (64 bytes) per AXI transfer
- **AXI Clock**: Configured via Efinity project
- **Theoretical Max Bandwidth**: 80 Gb/s per AXI port

### Why Hardware Auto-Init?
The hardware auto-initialization approach has several advantages:
- **Instant ready**: DDR available immediately after FPGA configuration
- **No software dependency**: Works without USB/Python connection
- **Reliable**: Initialization tied to PLL lock, no timing issues
- **Simple**: No complex software initialization sequence needed

### Register Access Timing
- Register read/write: ~100us over USB3
- Minimal impact on large tests (512MB = 116ms)
- Overhead <1% for tests >10MB

## Project Structure

```
LPDDR4/
├── rtl/
│   ├── tools_core.v              # Top-level with DDR controller
│   └── memory_checker_lfsr.v     # Memory test module
├── bsp/
│   └── TI375C529/
│       ├── outflow/
│       │   └── tools_core.bit    # Generated bitstream
│       └── tools_core.xml        # DDR controller config
├── usb_ddr_control.py            # Python USB control library
├── test_hardware_autoinit.py     # Quick DDR test
├── test_bandwidth_accurate.py    # Accurate bandwidth measurement
├── test_bandwidth.py             # Progressive bandwidth test
└── README.md                     # This file
```

## Troubleshooting

### DDR Not Initializing
- Check that `ddr_pll_lock` signal is high
- Verify `cfg_done` bit (REG_10_CONFIG bit 3) is set
- Wait at least 1 second after FPGA configuration

### Memory Test Failures
- Check `REG_0_DQ_FAIL` for failed data bits
- Verify LFSR seed matches between write and read
- Ensure test size doesn't exceed DDR capacity

### Low Bandwidth
- Verify ALEN=255 (256 transfers per burst)
- Check that 8 AXI IDs are cycling properly
- Monitor for AXI ready signal stalls

## References

- Efinix Ti375C529 FPGA documentation
- LPDDR4 JEDEC specification
- AXI4 protocol specification
- FT60X USB3 device documentation

## License

This project is part of the Ti375G529 development tools.
