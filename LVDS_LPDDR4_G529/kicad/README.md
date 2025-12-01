# KiCad Schematic Symbols for Haasoscope Pro Max

This directory contains KiCad schematic symbols and generators for the Haasoscope Pro Max FPGA board.

## Schematic and Board

- [Schematic PDF](haasoscope_pro_max/schematic.pdf)
- [Board Layout](haasoscope_pro_max/board.png)

## Generated Symbols

### Ti90G529_Functions.kicad_sym

Multi-unit FPGA symbol for the Efinix Ti90G529, organized by peripheral interface for easy schematic wiring:

| Unit | Name | Pin Side | Connects To |
|------|------|----------|-------------|
| 1 | FT601 Interface | LEFT | FT601Q USB3 FIFO |
| 2 | ADC 1/2 LVDS Data | RIGHT | ADC12DL500ACF (channels 1-2) |
| 3 | ADC 3/4 LVDS Data | RIGHT | ADC12DL500ACF (channels 3-4) |
| 4 | ADC LVDS Clocks | RIGHT | ADC clock outputs |
| 5 | DDR4 Interface | LEFT (data), RIGHT (control) | MT53D512M16D1DS LPDDR4 |
| 6 | Misc I/O | LEFT (inputs), RIGHT (outputs/SPI) | LEDs, buttons, SPI, etc. |
| 7 | CONFIG/JTAG | LEFT | Programming interface |
| 8 | VCC Power | LEFT and RIGHT | Power supplies |
| 9 | GND | LEFT and RIGHT | Ground |
| 10 | Unassigned GPIO | LEFT | Unused GPIO pins |

### Aligned_Peripheral_Symbols.kicad_sym

Peripheral chip symbols with pins arranged to align with the Ti90G529_Functions symbol:

- **FT601Q_Aligned** - USB3 FIFO, FIFO interface on RIGHT to connect to FPGA on LEFT
- **ADC12DL500ACF_Aligned** - Dual 12-bit ADC, LVDS outputs on LEFT to connect to FPGA on RIGHT
- **MT53D512M16D1DS_Aligned** - LPDDR4 memory, data on RIGHT, control/address on LEFT with gaps for dual-channel wiring

## Symbol Generators

### generate_ti90g529_symbol_with_functions.py

Generates `Ti90G529_Functions.kicad_sym` by parsing the Efinity peri.xml to automatically categorize GPIO pins by their assigned function.

### generate_aligned_peripheral_symbols.py

Generates `Aligned_Peripheral_Symbols.kicad_sym` with pin ordering matched to the FPGA symbol for horizontal wire alignment.

### generate_ti90g529_symbol.py

Base symbol generator with FPGA pin definitions imported by the other generators.

## Original Symbol Sources

- `ADC12DL/` - Original ADC12DL500ACF symbol
- `MT53D512M32D2DS-053 WT_D/` - Original MT53 DDR symbol
- `G529.pretty/` - Footprint library
