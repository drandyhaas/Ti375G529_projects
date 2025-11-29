# Ti375G529 FPGA Pinout Documentation

This document describes the pin assignments for the LVDS_LPDDR4_G529 project on the Efinix Ti375G529 FPGA.

## GPIO Bank R - FTDI USB3 Interface

### FTDI Data Bus (32-bit, bidirectional, 1.8V LVCMOS)

| Signal         | Pin        | Direction |
|----------------|------------|-----------|
| ftdi_data[0]   | GPIOR_P_00 | inout     |
| ftdi_data[1]   | GPIOR_P_01 | inout     |
| ftdi_data[2]   | GPIOR_P_02 | inout     |
| ftdi_data[3]   | GPIOR_P_03 | inout     |
| ftdi_data[4]   | GPIOR_P_04 | inout     |
| ftdi_data[5]   | GPIOR_P_05 | inout     |
| ftdi_data[6]   | GPIOR_P_06 | inout     |
| ftdi_data[7]   | GPIOR_P_07 | inout     |
| ftdi_data[8]   | GPIOR_P_08 | inout     |
| ftdi_data[9]   | GPIOR_P_16 | inout     |
| ftdi_data[10]  | GPIOR_P_17 | inout     |
| ftdi_data[11]  | GPIOR_P_18 | inout     |
| ftdi_data[12]  | GPIOR_P_19 | inout     |
| ftdi_data[13]  | GPIOR_P_20 | inout     |
| ftdi_data[14]  | GPIOR_P_21 | inout     |
| ftdi_data[15]  | GPIOR_P_22 | inout     |
| ftdi_data[16]  | GPIOR_P_23 | inout     |
| ftdi_data[17]  | GPIOR_P_24 | inout     |
| ftdi_data[18]  | GPIOR_P_25 | inout     |
| ftdi_data[19]  | GPIOR_P_26 | inout     |
| ftdi_data[20]  | GPIOR_P_27 | inout     |
| ftdi_data[21]  | GPIOR_P_28 | inout     |
| ftdi_data[22]  | GPIOR_P_29 | inout     |
| ftdi_data[23]  | GPIOR_P_30 | inout     |
| ftdi_data[24]  | GPIOR_P_31 | inout     |
| ftdi_data[25]  | GPIOR_P_36 | inout     |
| ftdi_data[26]  | GPIOR_P_37 | inout     |
| ftdi_data[27]  | GPIOR_P_38 | inout     |
| ftdi_data[28]  | GPIOR_P_39 | inout     |
| ftdi_data[29]  | GPIOR_P_40 | inout     |
| ftdi_data[30]  | GPIOR_P_41 | inout     |
| ftdi_data[31]  | GPIOR_P_42 | inout     |

### FTDI Byte Enable (4-bit, bidirectional, 1.8V LVCMOS)

| Signal      | Pin        | Direction |
|-------------|------------|-----------|
| ftdi_be[0]  | GPIOR_P_43 | inout     |
| ftdi_be[1]  | GPIOR_P_44 | inout     |
| ftdi_be[2]  | GPIOR_P_45 | inout     |
| ftdi_be[3]  | GPIOR_56   | inout     |

### FTDI Control Signals (1.8V LVCMOS)

| Signal       | Pin      | Direction | Description              |
|--------------|----------|-----------|--------------------------|
| ftdi_clk     | GPIOR_57 | input     | FTDI clock input         |
| ftdi_oe_n    | GPIOR_58 | output    | Output enable (active low)|
| ftdi_rd_n    | GPIOR_59 | output    | Read strobe (active low) |
| ftdi_wr_n    | GPIOR_60 | output    | Write strobe (active low)|
| ftdi_rxf_n   | GPIOR_61 | input     | RX FIFO not empty        |
| ftdi_txe_n   | GPIOR_63 | input     | TX FIFO not full         |
| ftdi_resetn  | GPIOR_65 | output    | Reset (active low)       |
| ftdi_siwu    | GPIOR_66 | output    | Send immediate/wake up   |
| ftdi_wakeupn | GPIOR_68 | output    | Wake up (active low)     |
| ftdi_gpio0   | GPIOR_69 | output    | GPIO 0                   |
| ftdi_gpio1   | GPIOR_70 | output    | GPIO 1                   |

## GPIO Bank R - LEDs (1.8V LVCMOS)

| Signal  | Pin        | Direction | Description    |
|---------|------------|-----------|----------------|
| LED[0]  | GPIOR_71   | output    | User LED 0     |
| LED[1]  | GPIOR_72   | output    | User LED 1     |
| LED[2]  | GPIOR_N_00 | output    | User LED 2     |
| LED[3]  | GPIOR_N_01 | output    | User LED 3     |

## GPIO Bank R - SPI Interface (1.8V LVCMOS)

| Signal    | Pin        | Direction | Description              |
|-----------|------------|-----------|--------------------------|
| spi_clk   | GPIOR_P_09 | output    | SPI clock                |
| spi_mosi  | GPIOR_P_10 | output    | SPI master out, slave in |
| spi_miso  | GPIOR_P_11 | input     | SPI master in, slave out |
| spics[0]  | GPIOR_P_12 | output    | SPI chip select 0        |
| spics[1]  | GPIOR_P_13 | output    | SPI chip select 1        |
| spics[2]  | GPIOR_P_14 | output    | SPI chip select 2        |
| spics[3]  | GPIOR_P_15 | output    | SPI chip select 3        |
| spics[4]  | GPIOR_N_09 | output    | SPI chip select 4        |
| spics[5]  | GPIOR_N_10 | output    | SPI chip select 5        |
| spics[6]  | GPIOR_N_11 | output    | SPI chip select 6        |
| spics[7]  | GPIOR_N_12 | output    | SPI chip select 7        |

## GPIO Bank R - NeoPixel and Fan Control (1.8V LVCMOS)

| Signal   | Pin        | Direction | Description                    |
|----------|------------|-----------|--------------------------------|
| neo_led  | GPIOR_N_02 | output    | WS2812B NeoPixel data output   |
| fan_out  | GPIOR_N_03 | output    | PWM output for fan control     |

## GPIO Bank R - Debug Output (1.8V LVCMOS)

| Signal       | Pin        | Direction | Description    |
|--------------|------------|-----------|----------------|
| debugout[0]  | GPIOR_N_04 | output    | Debug bit 0    |
| debugout[1]  | GPIOR_N_05 | output    | Debug bit 1    |
| debugout[2]  | GPIOR_N_06 | output    | Debug bit 2    |
| debugout[3]  | GPIOR_N_07 | output    | Debug bit 3    |
| debugout[4]  | GPIOR_N_08 | output    | Debug bit 4    |
| debugout[5]  | GPIOR_N_13 | output    | Debug bit 5    |
| debugout[6]  | GPIOR_N_14 | output    | Debug bit 6    |
| debugout[7]  | GPIOR_N_15 | output    | Debug bit 7    |
| debugout[8]  | GPIOR_N_16 | output    | Debug bit 8    |
| debugout[9]  | GPIOR_N_17 | output    | Debug bit 9    |
| debugout[10] | GPIOR_N_18 | output    | Debug bit 10   |
| debugout[11] | GPIOR_N_19 | output    | Debug bit 11   |

## GPIO Bank R - Board Interface (1.8V LVCMOS)

### Board Outputs

| Signal       | Pin        | Direction | Description        |
|--------------|------------|-----------|--------------------|
| boardout[0]  | GPIOR_N_20 | output    | Board output bit 0 |
| boardout[1]  | GPIOR_N_21 | output    | Board output bit 1 |
| boardout[2]  | GPIOR_N_22 | output    | Board output bit 2 |
| boardout[3]  | GPIOR_N_23 | output    | Board output bit 3 |
| boardout[4]  | GPIOR_N_24 | output    | Board output bit 4 |
| boardout[5]  | GPIOR_N_25 | output    | Board output bit 5 |
| boardout[6]  | GPIOR_N_26 | output    | Board output bit 6 |
| boardout[7]  | GPIOR_N_27 | output    | Board output bit 7 |

### Board Inputs

| Signal      | Pin        | Direction | Description       |
|-------------|------------|-----------|-------------------|
| boardin[0]  | GPIOR_N_28 | input     | Board input bit 0 |
| boardin[1]  | GPIOR_N_29 | input     | Board input bit 1 |
| boardin[2]  | GPIOR_N_30 | input     | Board input bit 2 |
| boardin[3]  | GPIOR_N_31 | input     | Board input bit 3 |
| boardin[4]  | GPIOR_N_32 | input     | Board input bit 4 |
| boardin[5]  | GPIOR_N_33 | input     | Board input bit 5 |
| boardin[6]  | GPIOR_N_34 | input     | Board input bit 6 |
| boardin[7]  | GPIOR_N_35 | input     | Board input bit 7 |

### Overrange and Lock Status Inputs

| Signal       | Pin        | Direction | Description             |
|--------------|------------|-----------|-------------------------|
| overrange[0] | GPIOR_N_36 | input     | ADC overrange bit 0     |
| overrange[1] | GPIOR_N_37 | input     | ADC overrange bit 1     |
| overrange[2] | GPIOR_N_38 | input     | ADC overrange bit 2     |
| overrange[3] | GPIOR_N_39 | input     | ADC overrange bit 3     |
| lockinfo[0]  | GPIOR_N_40 | input     | PLL lock status bit 0   |
| lockinfo[1]  | GPIOR_N_41 | input     | PLL lock status bit 1   |
| lockinfo[2]  | GPIOR_N_42 | input     | PLL lock status bit 2   |
| lockinfo[3]  | GPIOR_N_43 | input     | PLL lock status bit 3   |

## GPIO Bank L - DDR PLL Reference Clock

| Signal    | Pin      | Direction | IO Standard  | Description           |
|-----------|----------|-----------|--------------|------------------------|
| ddr_pllin | GPIOL_32 | input     | 3.3V LVCMOS  | DDR PLL reference clock|

## LVDS Interface - GPIO Bank T (Top)

All LVDS signals use differential pairs with 10-bit deserialization (half-rate mode).

### LVDS Clock Inputs (Bank T)

| Signal              | Pin         | Description                    |
|---------------------|-------------|--------------------------------|
| lvds_rx_top_clkin1  | GPIOT_PN_11 | LVDS clock for Group 1        |
| lvds_rx_top_clkin2  | GPIOT_PN_00 | LVDS clock for Group 2        |

### LVDS Group 1 Data (Bank T) - 13 channels

| Signal      | Pin         |
|-------------|-------------|
| lvds_rx1_1  | GPIOT_PN_27 |
| lvds_rx1_2  | GPIOT_PN_01 |
| lvds_rx1_3  | GPIOT_PN_02 |
| lvds_rx1_4  | GPIOT_PN_03 |
| lvds_rx1_5  | GPIOT_PN_04 |
| lvds_rx1_6  | GPIOT_PN_05 |
| lvds_rx1_7  | GPIOT_PN_06 |
| lvds_rx1_8  | GPIOT_PN_07 |
| lvds_rx1_9  | GPIOT_PN_08 |
| lvds_rx1_10 | GPIOT_PN_09 |
| lvds_rx1_11 | GPIOT_PN_10 |
| lvds_rx1_12 | GPIOT_PN_12 |
| lvds_rx1_13 | GPIOT_PN_13 |

### LVDS Group 2 Data (Bank T) - 13 channels

| Signal      | Pin         |
|-------------|-------------|
| lvds_rx2_1  | GPIOT_PN_14 |
| lvds_rx2_2  | GPIOT_PN_15 |
| lvds_rx2_3  | GPIOT_PN_16 |
| lvds_rx2_4  | GPIOT_PN_17 |
| lvds_rx2_5  | GPIOT_PN_18 |
| lvds_rx2_6  | GPIOT_PN_19 |
| lvds_rx2_7  | GPIOT_PN_20 |
| lvds_rx2_8  | GPIOT_PN_21 |
| lvds_rx2_9  | GPIOT_PN_22 |
| lvds_rx2_10 | GPIOT_PN_23 |
| lvds_rx2_11 | GPIOT_PN_24 |
| lvds_rx2_12 | GPIOT_PN_25 |
| lvds_rx2_13 | GPIOT_PN_26 |

## LVDS Interface - GPIO Bank B (Bottom)

### LVDS Clock Inputs (Bank B)

| Signal                 | Pin         | Description                    |
|------------------------|-------------|--------------------------------|
| lvds_rx_bottom_clkin3  | GPIOB_PN_00 | LVDS clock for Group 3        |
| lvds_rx_bottom_clkin4  | GPIOB_PN_11 | LVDS clock for Group 4        |

### LVDS Group 3 Data (Bank B) - 13 channels

| Signal      | Pin         |
|-------------|-------------|
| lvds_rx3_1  | GPIOB_PN_27 |
| lvds_rx3_2  | GPIOB_PN_01 |
| lvds_rx3_3  | GPIOB_PN_02 |
| lvds_rx3_4  | GPIOB_PN_03 |
| lvds_rx3_5  | GPIOB_PN_04 |
| lvds_rx3_6  | GPIOB_PN_05 |
| lvds_rx3_7  | GPIOB_PN_06 |
| lvds_rx3_8  | GPIOB_PN_07 |
| lvds_rx3_9  | GPIOB_PN_08 |
| lvds_rx3_10 | GPIOB_PN_09 |
| lvds_rx3_11 | GPIOB_PN_10 |
| lvds_rx3_12 | GPIOB_PN_12 |
| lvds_rx3_13 | GPIOB_PN_13 |

### LVDS Group 4 Data (Bank B) - 13 channels

| Signal      | Pin         |
|-------------|-------------|
| lvds_rx4_1  | GPIOB_PN_14 |
| lvds_rx4_2  | GPIOB_PN_15 |
| lvds_rx4_3  | GPIOB_PN_16 |
| lvds_rx4_4  | GPIOB_PN_17 |
| lvds_rx4_5  | GPIOB_PN_18 |
| lvds_rx4_6  | GPIOB_PN_19 |
| lvds_rx4_7  | GPIOB_PN_20 |
| lvds_rx4_8  | GPIOB_PN_21 |
| lvds_rx4_9  | GPIOB_PN_22 |
| lvds_rx4_10 | GPIOB_PN_23 |
| lvds_rx4_11 | GPIOB_PN_24 |
| lvds_rx4_12 | GPIOB_PN_25 |
| lvds_rx4_13 | GPIOB_PN_26 |

## DDR4 Memory Interface

The LPDDR4x memory interface uses the hard DDR block `DDR_0`:

| Parameter      | Value    |
|----------------|----------|
| DDR Block      | DDR_0    |
| Memory Type    | LPDDR4x  |
| Memory Density | 8Gb      |
| Data Width     | 32-bit   |
| Physical Rank  | 1        |

DDR pins are automatically assigned by the Efinix DDR hard block and are not user-configurable.

## PLL Configuration

### Main PLL (PLL_TL2)

| Output    | Divider | Description           |
|-----------|---------|------------------------|
| ddr_clk   | 3       | DDR clock output       |

Control signals:
- `ddr_pll_rstn` - PLL reset (active low)
- `ddr_pll_lock` - PLL lock status output

## Additional Top-Level Signals

These signals are defined in `top.v` but require pin assignments in `tools_core.peri.xml`:

### SPI Interface (External Flash/Peripherals)

| Signal   | Direction | Description                   |
|----------|-----------|-------------------------------|
| spi_clk  | output    | SPI clock                     |
| spi_mosi | output    | SPI master out, slave in      |
| spi_miso | input     | SPI master in, slave out      |
| spics[7:0] | output  | SPI chip select (active low)  |

### NeoPixel LED

| Signal  | Direction | Description                   |
|---------|-----------|-------------------------------|
| neo_led | output    | WS2812B NeoPixel data output  |

### Fan Control

| Signal  | Direction | Description                   |
|---------|-----------|-------------------------------|
| fan_out | output    | PWM output for fan control    |

### PLL Phase Shift Control

| Signal             | Width | Direction | Description                |
|--------------------|-------|-----------|----------------------------|
| pllreset           | 1     | output    | PLL reset                  |
| phasecounterselect | 3     | output    | Phase counter select       |
| phaseupdown        | 1     | output    | Phase shift direction      |
| phasestep          | 4     | output    | Phase step control         |
| scanclk            | 1     | output    | Scan clock                 |
| clkswitch          | 1     | output    | Clock switch control       |
| clkout_ena         | 1     | output    | Clock output enable        |
| clk_over_4         | 1     | output    | Clock divided by 4 (12.5MHz)|

### External Flash Interface

| Signal           | Width | Direction | Description                |
|------------------|-------|-----------|----------------------------|
| flash_addr       | 24    | output    | Flash address              |
| flash_bulk_erase | 1     | output    | Bulk erase control         |
| flash_datain     | 8     | output    | Data to flash              |
| flash_rden       | 1     | output    | Read enable                |
| flash_read       | 1     | output    | Read strobe                |
| flash_write      | 1     | output    | Write strobe               |
| flash_reset      | 1     | output    | Flash reset                |
| flash_busy       | 1     | input     | Flash busy status          |
| flash_data_valid | 1     | input     | Data valid flag            |
| flash_dataout    | 8     | input     | Data from flash            |

### Debug and Board Interface

| Signal    | Width | Direction | Description                |
|-----------|-------|-----------|----------------------------|
| debugout  | 12    | output    | Debug signals              |
| overrange | 4     | input     | ADC overrange indicators   |
| boardin   | 8     | input     | Board-level inputs         |
| boardout  | 8     | output    | Board-level outputs        |
| lockinfo  | 4     | input     | PLL lock status info       |

### Trigger Interface (LVDS)

| Signal        | Direction | Description                |
|---------------|-----------|----------------------------|
| lvdsin_trig   | input     | LVDS trigger input         |
| lvdsout_trig  | output    | LVDS trigger output        |
| lvdsin_trig_b | input     | LVDS trigger input B       |
| lvdsout_trig_b| output    | LVDS trigger output B      |
| lvdsin_spare  | input     | LVDS spare input           |
| lvdsout_spare | output    | LVDS spare output          |
| exttrigin     | input     | External trigger input     |
| auxout        | output    | Auxiliary output           |

### AXI4 Interface (DDR Controller)

The AXI4 interface connects to the LPDDR4 controller with:
- 512-bit data width
- 33-bit address (8GB addressable)
- 6-bit ID width
- Full AXI4 protocol support (burst, QoS, cache hints)

## Summary

| Interface           | Total Pins | IO Bank(s)      |
|---------------------|------------|-----------------|
| FTDI USB3           | 45         | GPIO Bank R     |
| LEDs                | 4          | GPIO Bank R     |
| SPI                 | 11         | GPIO Bank R     |
| NeoPixel + Fan      | 2          | GPIO Bank R     |
| Debug Output        | 12         | GPIO Bank R     |
| Board I/O           | 16         | GPIO Bank R     |
| Overrange + Lock    | 8          | GPIO Bank R     |
| LVDS Data           | 52 pairs   | GPIO Banks T, B |
| LVDS Clocks         | 4 pairs    | GPIO Banks T, B |
| DDR4                | Hard block | DDR_0           |
| PLL                 | Hard block | PLL_TL2         |
