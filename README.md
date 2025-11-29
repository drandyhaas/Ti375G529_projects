# Ti375G529_projects

Test projects for the Efinix Ti375C529 development kit with LPDDR4x memory and FT60X USB3 interface.

## Setup

Make sure to use Zadig to choose the libusb-win32 driver for the Ti dev board interfaces 0 and 1, then it will show up in the programmer.

## Projects

| Directory | Description |
|-----------|-------------|
| [Blink](Blink/) | Simple LED blink example - basic FPGA sanity test |
| [FT601_loopback](FT601_loopback/) | FT60X USB3 loopback test - verify USB communication |
| [LPDDR4](LPDDR4/) | DDR memory test with hardware auto-init and 74+ Gb/s bandwidth verification |
| [LVDS_LPDDR4](LVDS_LPDDR4/) | Digital scope with LVDS inputs, DDR buffering, USB3 readback (Ti375C529) |
| [LVDS_LPDDR4_G529](LVDS_LPDDR4_G529/) | Digital scope for Ti375G529 - 52-channel LVDS, 7 PLLs, extended I/O |
| [ti_lpddr4_debug_tools](ti_lpddr4_debug_tools/) | Python tools for DDR calibration and debugging via JTAG |

## Project Details

### [Blink](Blink/)
Minimal LED blink project for verifying FPGA programming and basic I/O.

### [FT601_loopback](FT601_loopback/)
USB3 FT60X interface test. Requires FT60X drivers (v1.3.0.10 recommended) and configuration to 1-channel 245 FIFO mode.

### [LPDDR4](LPDDR4/)
LPDDR4x memory controller test system:
- Hardware auto-initialization on power-up
- LFSR-based memory test with 100% data verification
- 74.36 Gb/s bandwidth (93% efficiency)
- USB3 control interface

### [LVDS_LPDDR4](LVDS_LPDDR4/)
High-speed data acquisition system for Ti375C529:
- 4-channel 10-bit LVDS input at 750 MHz
- LPDDR4 waveform buffer (81+ Gb/s)
- Configurable triggering and downsampling
- USB3 readout (~280 MB/s)
- Command processor with 8-byte protocol

### [LVDS_LPDDR4_G529](LVDS_LPDDR4_G529/)
Extended digital scope for Ti375G529 FPGA:
- 52-channel LVDS input (4 groups x 13 channels)
- 7 PLLs for clock generation
- Dynamic PLL clock selection
- Extended GPIO (debug, board I/O, SPI)
- See [FPGA_Pinout.md](LVDS_LPDDR4_G529/FPGA_Pinout.md) for pin assignments

### [ti_lpddr4_debug_tools](ti_lpddr4_debug_tools/)
Python-based DDR debugging and calibration tools:
- JTAG interface for register access
- DDR initialization and training
- Timing scan and calibration
- Gate leveling and read leveling
