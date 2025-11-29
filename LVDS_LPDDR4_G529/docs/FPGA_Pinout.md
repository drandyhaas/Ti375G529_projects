# Ti375G529 FPGA Pinout Documentation

This document describes the pin assignments for the LVDS_LPDDR4_G529 project on the Efinix Ti375G529 FPGA.

## GPIO Bank R - FTDI USB3 Interface

### FTDI Data Bus (32-bit, bidirectional, 1.8V LVCMOS)

| Signal         | Pin        | Ball | Direction |
|----------------|------------|------|-----------|
| ftdi_data[0]   | GPIOR_P_00 | C5   | inout     |
| ftdi_data[1]   | GPIOR_P_01 | A5   | inout     |
| ftdi_data[2]   | GPIOR_P_02 | E6   | inout     |
| ftdi_data[3]   | GPIOR_P_03 | B7   | inout     |
| ftdi_data[4]   | GPIOR_P_04 | C7   | inout     |
| ftdi_data[5]   | GPIOR_P_05 | E8   | inout     |
| ftdi_data[6]   | GPIOR_P_06 | A8   | inout     |
| ftdi_data[7]   | GPIOR_P_07 | C9   | inout     |
| ftdi_data[8]   | GPIOR_P_08 | E9   | inout     |
| ftdi_data[9]   | GPIOR_P_16 | E10  | inout     |
| ftdi_data[10]  | GPIOR_P_17 | C10  | inout     |
| ftdi_data[11]  | GPIOR_P_18 | H11  | inout     |
| ftdi_data[12]  | GPIOR_P_19 | A10  | inout     |
| ftdi_data[13]  | GPIOR_P_20 | C11  | inout     |
| ftdi_data[14]  | GPIOR_P_21 | G11  | inout     |
| ftdi_data[15]  | GPIOR_P_22 | B12  | inout     |
| ftdi_data[16]  | GPIOR_P_23 | E11  | inout     |
| ftdi_data[17]  | GPIOR_P_24 | G12  | inout     |
| ftdi_data[18]  | GPIOR_P_25 | D12  | inout     |
| ftdi_data[19]  | GPIOR_P_26 | A13  | inout     |
| ftdi_data[20]  | GPIOR_P_27 | H13  | inout     |
| ftdi_data[21]  | GPIOR_P_28 | B13  | inout     |
| ftdi_data[22]  | GPIOR_P_29 | F13  | inout     |
| ftdi_data[23]  | GPIOR_P_30 | G14  | inout     |
| ftdi_data[24]  | GPIOR_P_31 | E14  | inout     |
| ftdi_data[25]  | GPIOR_P_36 | B15  | inout     |
| ftdi_data[26]  | GPIOR_P_37 | C15  | inout     |
| ftdi_data[27]  | GPIOR_P_38 | A16  | inout     |
| ftdi_data[28]  | GPIOR_P_39 | A17  | inout     |
| ftdi_data[29]  | GPIOR_P_40 | C16  | inout     |
| ftdi_data[30]  | GPIOR_P_41 | B18  | inout     |
| ftdi_data[31]  | GPIOR_P_42 | A21  | inout     |

### FTDI Byte Enable (4-bit, bidirectional, 1.8V LVCMOS)

| Signal      | Pin        | Ball | Direction |
|-------------|------------|------|-----------|
| ftdi_be[0]  | GPIOR_P_43 | D17  | inout     |
| ftdi_be[1]  | GPIOR_P_44 | C19  | inout     |
| ftdi_be[2]  | GPIOR_P_45 | C20  | inout     |
| ftdi_be[3]  | GPIOR_56   | E17  | inout     |

### FTDI Control Signals (1.8V LVCMOS)

| Signal       | Pin      | Ball | Direction | Description              |
|--------------|----------|------|-----------|--------------------------|
| ftdi_clk     | GPIOR_57 | F15  | input     | FTDI clock input         |
| ftdi_oe_n    | GPIOR_58 | H15  | output    | Output enable (active low)|
| ftdi_rd_n    | GPIOR_59 | E16  | output    | Read strobe (active low) |
| ftdi_wr_n    | GPIOR_60 | G16  | output    | Write strobe (active low)|
| ftdi_rxf_n   | GPIOR_61 | G15  | input     | RX FIFO not empty        |
| ftdi_txe_n   | GPIOR_63 | E15  | input     | TX FIFO not full         |
| ftdi_resetn  | GPIOR_65 | G9   | output    | Reset (active low)       |
| ftdi_siwu    | GPIOR_66 | G8   | output    | Send immediate/wake up   |
| ftdi_wakeupn | GPIOR_68 | F7   | output    | Wake up (active low)     |
| ftdi_gpio0   | GPIOR_69 | H8   | output    | GPIO 0                   |
| ftdi_gpio1   | GPIOR_70 | G7   | output    | GPIO 1                   |

## GPIO Bank L - LEDs, SPI, NeoPixel and Fan Control (3.3V LVCMOS)

### LEDs

| Signal  | Pin       | Ball | Direction | Description                      |
|---------|-----------|------|-----------|----------------------------------|
| LED[0]  | GPIOL_07  | U1   | output    | User LED 0                       |
| LED[1]  | GPIOL_08  | W1   | output    | User LED 1                       |
| LED[2]  | GPIOL_09  | V3   | output    | User LED 2 (regACLK heartbeat)   |
| LED[3]  | GPIOL_10  | W3   | output    | User LED 3                       |
| led2    | GPIOL_20  | W4   | output    | Triggerer controlled LED         |

### SPI Interface

| Signal    | Pin       | Ball | Direction | Description              |
|-----------|-----------|------|-----------|--------------------------|
| spi_clk   | GPIOL_26  | V18  | output    | SPI clock                |
| spi_mosi  | GPIOL_27  | T19  | output    | SPI master out, slave in |
| spi_miso  | GPIOL_28  | T20  | input     | SPI master in, slave out |
| spics[0]  | GPIOL_29  | V19  | output    | SPI chip select 0        |
| spics[1]  | GPIOL_30  | T18  | output    | SPI chip select 1        |
| spics[2]  | GPIOL_31  | U20  | output    | SPI chip select 2        |
| spics[3]  | GPIOL_33  | V23  | output    | SPI chip select 3        |
| spics[4]  | GPIOL_34  | V21  | output    | SPI chip select 4        |
| spics[5]  | GPIOL_35  | V22  | output    | SPI chip select 5        |
| spics[6]  | GPIOL_37  | U21  | output    | SPI chip select 6        |
| spics[7]  | GPIOL_38  | V20  | output    | SPI chip select 7        |

### NeoPixel and Fan Control

| Signal   | Pin       | Ball | Direction | Description                    |
|----------|-----------|------|-----------|--------------------------------|
| neo_led  | GPIOL_22  | V4   | output    | WS2812B NeoPixel data output   |
| fan_out  | GPIOL_23  | W2   | output    | PWM output for fan control     |

### DDR PLL Reference Clock

| Signal    | Pin       | Ball | Direction | Description                |
|-----------|-----------|------|-----------|----------------------------|
| ddr_pllin | GPIOL_32  | U18  | input     | DDR PLL reference clock    |

## GPIO Bank R - Debug Output (1.8V LVCMOS)

| Signal       | Pin        | Ball | Direction | Description    |
|--------------|------------|------|-----------|----------------|
| debugout[0]  | GPIOR_N_04 | C8   | output    | Debug bit 0    |
| debugout[1]  | GPIOR_N_05 | D8   | output    | Debug bit 1    |
| debugout[2]  | GPIOR_N_06 | A9   | output    | Debug bit 2    |
| debugout[3]  | GPIOR_N_07 | B9   | output    | Debug bit 3    |
| debugout[4]  | GPIOR_N_08 | D9   | output    | Debug bit 4    |
| debugout[5]  | GPIOR_N_00 | C6   | output    | Debug bit 5    |
| debugout[6]  | GPIOR_N_01 | A6   | output    | Debug bit 6    |
| debugout[7]  | GPIOR_N_02 | E7   | output    | Debug bit 7    |
| debugout[8]  | GPIOR_N_16 | F10  | output    | Debug bit 8    |
| debugout[9]  | GPIOR_N_17 | B10  | output    | Debug bit 9    |
| debugout[10] | GPIOR_N_18 | H10  | output    | Debug bit 10   |
| debugout[11] | GPIOR_N_19 | A11  | output    | Debug bit 11   |

## GPIO Bank R - Board Interface (1.8V LVCMOS)

### Board Outputs

| Signal       | Pin        | Ball | Direction | Description        |
|--------------|------------|------|-----------|--------------------|
| boardout[0]  | GPIOR_N_20 | D11  | output    | Board output bit 0 |
| boardout[1]  | GPIOR_N_21 | F11  | output    | Board output bit 1 |
| boardout[2]  | GPIOR_N_22 | A12  | output    | Board output bit 2 |
| boardout[3]  | GPIOR_N_23 | E12  | output    | Board output bit 3 |
| boardout[4]  | GPIOR_N_24 | H12  | output    | Board output bit 4 |
| boardout[5]  | GPIOR_N_25 | C12  | output    | Board output bit 5 |
| boardout[6]  | GPIOR_N_26 | A14  | output    | Board output bit 6 |
| boardout[7]  | GPIOR_N_27 | G13  | output    | Board output bit 7 |

### Board Inputs

| Signal      | Pin        | Ball | Direction | Description       |
|-------------|------------|------|-----------|-------------------|
| boardin[0]  | GPIOR_N_28 | C13  | input     | Board input bit 0 |
| boardin[1]  | GPIOR_N_29 | E13  | input     | Board input bit 1 |
| boardin[2]  | GPIOR_N_30 | H14  | input     | Board input bit 2 |
| boardin[3]  | GPIOR_N_31 | D14  | input     | Board input bit 3 |
| boardin[4]  | GPIOR_N_03 | A7   | input     | Board input bit 4 |
| boardin[5]  | GPIOR_N_44 | B19  | input     | Board input bit 5 |
| boardin[6]  | GPIOR_N_45 | B20  | input     | Board input bit 6 |
| boardin[7]  | GPIOR_71   | H9   | input     | Board input bit 7 |

### Overrange and Lock Status Inputs

| Signal       | Pin        | Ball | Direction | Description             |
|--------------|------------|------|-----------|-------------------------|
| overrange[0] | GPIOR_N_36 | A15  | input     | ADC overrange bit 0     |
| overrange[1] | GPIOR_N_37 | D15  | input     | ADC overrange bit 1     |
| overrange[2] | GPIOR_N_38 | B16  | input     | ADC overrange bit 2     |
| overrange[3] | GPIOR_N_39 | A18  | input     | ADC overrange bit 3     |
| lockinfo[0]  | GPIOR_N_40 | C17  | input     | PLL lock status bit 0   |
| lockinfo[1]  | GPIOR_N_41 | C18  | input     | PLL lock status bit 1   |
| lockinfo[2]  | GPIOR_N_42 | A20  | input     | PLL lock status bit 2   |
| lockinfo[3]  | GPIOR_N_43 | D18  | input     | PLL lock status bit 3   |

## LVDS Interface - GPIO Bank T (Top)

All LVDS signals use differential pairs with 10-bit deserialization (half-rate mode).

### LVDS Clock Inputs (Bank T)

| Signal              | Pin         | Ball (P/N)  | Description                    |
|---------------------|-------------|-------------|--------------------------------|
| lvds_rx_top_clkin1  | GPIOT_PN_11 | J22/J23     | LVDS clock for Group 1        |
| lvds_rx_top_clkin2  | GPIOT_PN_00 | R22/R21     | LVDS clock for Group 2        |

### LVDS Group 1 Data (Bank T) - 13 channels

| Signal      | Pin         | Ball (P/N)  |
|-------------|-------------|-------------|
| lvds_rx1_1  | GPIOT_PN_27 | E21/E20     |
| lvds_rx1_2  | GPIOT_PN_01 | R23/P23     |
| lvds_rx1_3  | GPIOT_PN_02 | R18/R19     |
| lvds_rx1_4  | GPIOT_PN_03 | P21/P20     |
| lvds_rx1_5  | GPIOT_PN_04 | N22/N23     |
| lvds_rx1_6  | GPIOT_PN_05 | P18/P19     |
| lvds_rx1_7  | GPIOT_PN_06 | L23/L22     |
| lvds_rx1_8  | GPIOT_PN_07 | N20/N21     |
| lvds_rx1_9  | GPIOT_PN_08 | N16/N17     |
| lvds_rx1_10 | GPIOT_PN_09 | N19/N18     |
| lvds_rx1_11 | GPIOT_PN_10 | M21/M22     |
| lvds_rx1_12 | GPIOT_PN_12 | G22/H22     |
| lvds_rx1_13 | GPIOT_PN_13 | M19/M18     |

### LVDS Group 2 Data (Bank T) - 13 channels

| Signal      | Pin         | Ball (P/N)  |
|-------------|-------------|-------------|
| lvds_rx2_1  | GPIOT_PN_14 | L21/K21     |
| lvds_rx2_2  | GPIOT_PN_15 | F23/G23     |
| lvds_rx2_3  | GPIOT_PN_16 | L16/M16     |
| lvds_rx2_4  | GPIOT_PN_17 | E23/E22     |
| lvds_rx2_5  | GPIOT_PN_18 | J20/J21     |
| lvds_rx2_6  | GPIOT_PN_19 | L18/L17     |
| lvds_rx2_7  | GPIOT_PN_20 | G21/F21     |
| lvds_rx2_8  | GPIOT_PN_21 | K20/K19     |
| lvds_rx2_9  | GPIOT_PN_22 | L20/L19     |
| lvds_rx2_10 | GPIOT_PN_23 | D23/C23     |
| lvds_rx2_11 | GPIOT_PN_24 | B23/B22     |
| lvds_rx2_12 | GPIOT_PN_25 | K18/K17     |
| lvds_rx2_13 | GPIOT_PN_26 | H20/G20     |

## LVDS Interface - GPIO Bank B (Bottom)

### LVDS Clock Inputs (Bank B)

| Signal                 | Pin         | Ball (P/N)  | Description                    |
|------------------------|-------------|-------------|--------------------------------|
| lvds_rx_bottom_clkin3  | GPIOB_PN_00 | R4/R5       | LVDS clock for Group 3        |
| lvds_rx_bottom_clkin4  | GPIOB_PN_11 | G1/G2       | LVDS clock for Group 4        |

### LVDS Group 3 Data (Bank B) - 13 channels

| Signal      | Pin         | Ball (P/N)  |
|-------------|-------------|-------------|
| lvds_rx3_1  | GPIOB_PN_27 | C3/C2       |
| lvds_rx3_2  | GPIOB_PN_01 | R2/R3       |
| lvds_rx3_3  | GPIOB_PN_02 | P8/P7       |
| lvds_rx3_4  | GPIOB_PN_03 | P1/R1       |
| lvds_rx3_5  | GPIOB_PN_04 | P5/P6       |
| lvds_rx3_6  | GPIOB_PN_05 | N5/N6       |
| lvds_rx3_7  | GPIOB_PN_06 | N3/P3       |
| lvds_rx3_8  | GPIOB_PN_07 | N1/N2       |
| lvds_rx3_9  | GPIOB_PN_08 | N4/M4       |
| lvds_rx3_10 | GPIOB_PN_09 | L2/M2       |
| lvds_rx3_11 | GPIOB_PN_10 | K1/L1       |
| lvds_rx3_12 | GPIOB_PN_12 | E1/F1       |
| lvds_rx3_13 | GPIOB_PN_13 | M6/M7       |

### LVDS Group 4 Data (Bank B) - 13 channels

| Signal      | Pin         | Ball (P/N)  |
|-------------|-------------|-------------|
| lvds_rx4_1  | GPIOB_PN_14 | L3/K3       |
| lvds_rx4_2  | GPIOB_PN_15 | J3/J2       |
| lvds_rx4_3  | GPIOB_PN_16 | L4/L5       |
| lvds_rx4_4  | GPIOB_PN_17 | H2/H3       |
| lvds_rx4_5  | GPIOB_PN_18 | F3/G3       |
| lvds_rx4_6  | GPIOB_PN_19 | L7/L8       |
| lvds_rx4_7  | GPIOB_PN_20 | K4/J4       |
| lvds_rx4_8  | GPIOB_PN_21 | H5/J5       |
| lvds_rx4_9  | GPIOB_PN_22 | K6/L6       |
| lvds_rx4_10 | GPIOB_PN_23 | E2/E3       |
| lvds_rx4_11 | GPIOB_PN_24 | B1/C1       |
| lvds_rx4_12 | GPIOB_PN_25 | E4/F4       |
| lvds_rx4_13 | GPIOB_PN_26 | D2/D3       |

### LVDS Clock and Trigger Interface

#### LVDS Inputs

| Signal         | Pin          | Ball (P/N) | Direction | Description                              |
|----------------|--------------|------------|-----------|------------------------------------------|
| lvdsin_clk     | GPIOT_PN_23  | D23/C23    | input     | LVDS clock input (PLL reference)         |
| lvdsin_trig    | GPIOB_PN_29  | B2/A2      | input     | LVDS trigger input                       |
| lvdsin_trig_b  | GPIOB_PN_30  | D5/E5      | input     | LVDS trigger input B                     |
| lvdsin_spare   | GPIOB_PN_31  | G6/H6      | input     | LVDS spare input                         |

#### LVDS Outputs (Bank 2C - Top)

| Signal         | Pin          | Ball (P/N) | Direction | Description                              |
|----------------|--------------|------------|-----------|------------------------------------------|
| lvdsout_clk    | GPIOT_PN_28  | J18/J17    | output    | LVDS clock output (driven by lvds_clk_slow) |
| lvdsout_trig   | GPIOT_PN_29  | F19/E19    | output    | LVDS trigger output                      |
| lvdsout_trig_b | GPIOT_PN_30  | D21/D20    | output    | LVDS trigger output B                    |
| lvdsout_spare  | GPIOT_PN_31  | H19/H18    | output    | LVDS spare output                        |

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

## Other Trigger Signals (Unassigned)

| Signal        | Direction | Description                |
|---------------|-----------|----------------------------|
| exttrigin     | input     | External trigger input     |
| auxout        | output    | Auxiliary output           |

## Summary

| Interface           | Total Pins | IO Bank(s)      | IO Standard  |
|---------------------|------------|-----------------|--------------|
| FTDI USB3           | 45         | GPIO Bank R     | 1.8V LVCMOS  |
| LEDs                | 5          | GPIO Bank L     | 3.3V LVCMOS  |
| SPI                 | 11         | GPIO Bank L     | 3.3V LVCMOS  |
| NeoPixel + Fan      | 2          | GPIO Bank L     | 3.3V LVCMOS  |
| Debug Output        | 12         | GPIO Bank R     | 1.8V LVCMOS  |
| Board I/O           | 16         | GPIO Bank R     | 1.8V LVCMOS  |
| Overrange + Lock    | 8          | GPIO Bank R     | 1.8V LVCMOS  |
| LVDS Data           | 52 pairs   | GPIO Banks T, B | LVDS         |
| LVDS Clocks         | 4 pairs    | GPIO Banks T, B | LVDS         |
| LVDS Clock/Trigger  | 8 pairs    | Bank 2C, 4A     | LVDS         |
| DDR4                | Hard block | DDR_0           | -            |
| PLL                 | Hard block | PLL_TL2         | -            |
