#!/usr/bin/env python3
"""
Generate aligned KiCad schematic symbols for peripherals that connect to Ti90G529 FPGA.

This script creates modified versions of:
1. ADC12DL500ACF - LVDS outputs reordered to match FPGA ADC units
2. MT53D512M16D1DS - DDR pins reordered to match FPGA DDR unit
3. FT601Q_Aligned - FT601 pins reordered to match FPGA FT601 Interface unit

The pin ordering in each symbol matches the corresponding Ti90G529_Functions unit,
so wires will line up horizontally when symbols are placed side-by-side.
"""

from pathlib import Path


def generate_symbol_header():
    return '''(kicad_symbol_lib
  (version 20231120)
  (generator "kicad_symbol_editor")
  (generator_version "8.0")
'''


def generate_ft601q_aligned():
    """Generate FT601Q symbol with data pins aligned to FPGA FT601 Interface unit.

    FPGA FT601 Interface has pins on LEFT in this order:
    ftdi_data[0-31], ftdi_be[0-3], ftdi_clk, ftdi_txe, ftdi_rxf, ftdi_oe, ftdi_wr, ftdi_rd

    FT601Q_Aligned puts corresponding pins on RIGHT in same order.
    """
    lines = []
    lines.append('  (symbol "FT601Q_Aligned"')
    lines.append('    (pin_names (offset 1.016))')
    lines.append('    (exclude_from_sim no)')
    lines.append('    (in_bom yes)')
    lines.append('    (on_board yes)')

    # Properties
    lines.append('    (property "Reference" "U"')
    lines.append('      (at 0 1.27 0)')
    lines.append('      (effects (font (size 1.27 1.27)))')
    lines.append('    )')
    lines.append('    (property "Value" "FT601Q_Aligned"')
    lines.append('      (at 0 -1.27 0)')
    lines.append('      (effects (font (size 1.27 1.27)))')
    lines.append('    )')
    lines.append('    (property "Footprint" "Package_DFN_QFN:QFN-76-1EP_9x9mm_P0.4mm_EP5.81x6.31mm"')
    lines.append('      (at 0 -3.81 0)')
    lines.append('      (effects (font (size 1.27 1.27)) hide)')
    lines.append('    )')
    lines.append('    (property "Description" "FT601Q USB3 FIFO - pin ordering aligned with Ti90G529 FPGA"')
    lines.append('      (at 0 -6.35 0)')
    lines.append('      (effects (font (size 1.27 1.27)) hide)')
    lines.append('    )')

    # FT601Q pin numbers for each signal
    # DATA_0-31 on FT601Q
    data_pins = [
        ('DATA_0', '40'), ('DATA_1', '41'), ('DATA_2', '42'), ('DATA_3', '43'),
        ('DATA_4', '44'), ('DATA_5', '45'), ('DATA_6', '46'), ('DATA_7', '47'),
        ('DATA_8', '50'), ('DATA_9', '51'), ('DATA_10', '52'), ('DATA_11', '53'),
        ('DATA_12', '54'), ('DATA_13', '55'), ('DATA_14', '56'), ('DATA_15', '57'),
        ('DATA_16', '60'), ('DATA_17', '61'), ('DATA_18', '62'), ('DATA_19', '63'),
        ('DATA_20', '64'), ('DATA_21', '65'), ('DATA_22', '66'), ('DATA_23', '67'),
        ('DATA_24', '69'), ('DATA_25', '70'), ('DATA_26', '71'), ('DATA_27', '72'),
        ('DATA_28', '73'), ('DATA_29', '74'), ('DATA_30', '75'), ('DATA_31', '76'),
    ]
    be_pins = [('BE_0', '4'), ('BE_1', '5'), ('BE_2', '6'), ('BE_3', '7')]
    # Control pins ordered to match Ti90G529 FT601 unit:
    # clk, oe, rd, wr, rxf, txe, reset, siwu, wakeup, gpio0, gpio1
    control_pins = [
        ('CLK', '58', 'output'),
        ('~{OE}', '13', 'input'),
        ('~{RD}', '12', 'input'),
        ('~{WR}', '11', 'input'),
        ('~{RXF}', '9', 'output'),
        ('~{TXE}', '8', 'output'),
        ('~{RESET}', '15', 'input'),
        ('~{SIWU}', '10', 'input'),
        ('~{WAKEUP}', '16', 'bidirectional'),
        ('GPIO0', '17', 'bidirectional'),
        ('GPIO1', '18', 'bidirectional'),
    ]

    # Calculate total pins for alignment
    total_pins = len(data_pins) + len(be_pins) + len(control_pins)
    pin_spacing = 2.54
    box_height = total_pins * pin_spacing + 7.62
    box_width = 25.4

    y_start = box_height / 2

    # Unit 1 - FIFO Interface (pins that connect to FPGA)
    lines.append('    (symbol "FT601Q_Aligned_1_1"')

    # Rectangle
    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    # Unit label
    lines.append('      (text "FT601Q FIFO Interface"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Pins on RIGHT side to connect to FPGA on left
    y_pos = y_start - 5.08

    # Data pins
    for name, pin_num in data_pins:
        lines.append(f'      (pin bidirectional line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    # BE pins
    for name, pin_num in be_pins:
        lines.append(f'      (pin bidirectional line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    # Control pins
    for name, pin_num, pin_type in control_pins:
        lines.append(f'      (pin {pin_type} line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 2 - USB (pins not directly connected to FPGA)
    lines.append('    (symbol "FT601Q_Aligned_2_1"')

    other_pins = [
        ('DM', '25', 'bidirectional'),
        ('DP', '23', 'bidirectional'),
        ('RIDN', '34', 'input'),
        ('RIDP', '35', 'input'),
        ('TODN', '31', 'output'),
        ('TODP', '32', 'output'),
        ('XI', '21', 'input'),
        ('XO', '22', 'output'),
        ('RREF', '27', 'input'),
    ]

    num_other = len(other_pins)
    height2 = num_other * pin_spacing + 7.62
    y_start2 = height2 / 2

    lines.append(f'      (rectangle (start 0 {y_start2:.2f}) (end 20.32 {-y_start2:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "USB"')
    lines.append(f'        (at 10.16 {y_start2 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start2 - 5.08
    for name, pin_num, pin_type in other_pins:
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 3 - Power
    lines.append('    (symbol "FT601Q_Aligned_3_1"')

    power_pins = [
        ('VBUS', '37', 'power_in'),
        ('VCC33', '20', 'power_in'),
        ('VCC33', '24', 'passive'),
        ('VCC33', '38', 'passive'),
        ('VDDA', '28', 'power_in'),
        ('VCCIO', '14', 'power_in'),
        ('VCCIO', '49', 'passive'),
        ('VCCIO', '59', 'passive'),
        ('VCCIO', '68', 'passive'),
        ('VD10', '3', 'power_in'),
        ('VD10', '30', 'passive'),
        ('VD10', '33', 'passive'),
        ('VD10', '48', 'passive'),
        ('DV10', '39', 'power_out'),
        ('AVDD', '2', 'power_in'),
        ('GND', '1', 'power_in'),
        ('GND', '19', 'passive'),
        ('GND', '26', 'passive'),
        ('GND', '29', 'passive'),
        ('GND', '36', 'passive'),
        ('GND', '77', 'passive'),
        ('GND', '78', 'passive'),  # Exposed pad
    ]

    num_pwr = len(power_pins)
    height3 = num_pwr * pin_spacing + 7.62
    y_start3 = height3 / 2

    lines.append(f'      (rectangle (start 0 {y_start3:.2f}) (end 20.32 {-y_start3:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Power"')
    lines.append(f'        (at 10.16 {y_start3 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start3 - 5.08
    for name, pin_num, pin_type in power_pins:
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    lines.append('  )')
    return '\n'.join(lines)


def generate_adc12dl_aligned():
    """Generate ADC12DL500ACF symbol with LVDS outputs aligned to FPGA ADC units.

    FPGA ADC LVDS unit has pins on RIGHT in order:
    lvds_rx1_1, lvds_rx1_1_N, lvds_rx1_2, lvds_rx1_2_N, ... (channels 1 & 2)

    ADC12DL has channels A, B, C, D with data bits 0-11 and clock/strobe.
    ADC output naming: DA0+/-, DA1+/-, ... for channel A, etc.

    This symbol puts LVDS outputs on LEFT side to connect to FPGA on right.
    """
    lines = []
    lines.append('  (symbol "ADC12DL500ACF_Aligned"')
    lines.append('    (pin_names (offset 1.016))')
    lines.append('    (exclude_from_sim no)')
    lines.append('    (in_bom yes)')
    lines.append('    (on_board yes)')

    # Properties
    lines.append('    (property "Reference" "IC"')
    lines.append('      (at 0 1.27 0)')
    lines.append('      (effects (font (size 1.27 1.27)))')
    lines.append('    )')
    lines.append('    (property "Value" "ADC12DL500ACF_Aligned"')
    lines.append('      (at 0 -1.27 0)')
    lines.append('      (effects (font (size 1.27 1.27)))')
    lines.append('    )')
    lines.append('    (property "Footprint" "BGA256C100P16X16_1700X1700X331"')
    lines.append('      (at 0 -3.81 0)')
    lines.append('      (effects (font (size 1.27 1.27)) hide)')
    lines.append('    )')
    lines.append('    (property "Description" "ADC12DL500ACF - LVDS outputs aligned with Ti90G529 FPGA"')
    lines.append('      (at 0 -6.35 0)')
    lines.append('      (effects (font (size 1.27 1.27)) hide)')
    lines.append('    )')

    # ADC12DL LVDS output pins (from the symbol file)
    # Format: (name, pin_number)
    # Channel A data: DA0-DA11 (each + and -)
    # Channel B data: DB0-DB11 (each + and -)
    # Channel C data: DC0-DC11 (each + and -)
    # Channel D data: DD0-DD11 (each + and -)
    # Plus clock and strobe for each channel

    # Unit 1: Channel A & C data outputs (connects to ADC 1/2 LVDS Data)
    # Clock pins moved to separate Unit 3
    lines.append('    (symbol "ADC12DL500ACF_Aligned_1_1"')

    ch_a_data = [
        ('DA0+', 'A9'), ('DA0-', 'A10'),
        ('DA1+', 'B9'), ('DA1-', 'B10'),
        ('DA2+', 'C9'), ('DA2-', 'C10'),
        ('DA3+', 'D9'), ('DA3-', 'D10'),
        ('DA4+', 'E9'), ('DA4-', 'E10'),
        ('DA5+', 'F9'), ('DA5-', 'F10'),
        ('DA6+', 'A11'), ('DA6-', 'A12'),
        ('DA7+', 'B11'), ('DA7-', 'B12'),
        ('DA8+', 'C11'), ('DA8-', 'C12'),
        ('DA9+', 'D11'), ('DA9-', 'D12'),
        ('DA10+', 'E11'), ('DA10-', 'E12'),
        ('DA11+', 'F11'), ('DA11-', 'F12'),
        # DACLK moved to Unit 3
        ('DASTR+', 'H9'), ('DASTR-', 'H10'),
    ]

    ch_c_data = [
        ('DC0+', 'A13'), ('DC0-', 'A14'),
        ('DC1+', 'B13'), ('DC1-', 'B14'),
        ('DC2+', 'C13'), ('DC2-', 'C14'),
        ('DC3+', 'D13'), ('DC3-', 'D14'),
        ('DC4+', 'E13'), ('DC4-', 'E14'),
        ('DC5+', 'F13'), ('DC5-', 'F14'),
        ('DC6+', 'G13'), ('DC6-', 'G14'),
        ('DC7+', 'B15'), ('DC7-', 'B16'),
        ('DC8+', 'C15'), ('DC8-', 'C16'),
        ('DC9+', 'D15'), ('DC9-', 'D16'),
        ('DC10+', 'E15'), ('DC10-', 'E16'),
        ('DC11+', 'F15'), ('DC11-', 'F16'),
        # DCCLK moved to Unit 3
        ('DCSTR+', 'H15'), ('DCSTR-', 'H16'),
    ]

    all_ac = ch_a_data + ch_c_data
    num_pins = len(all_ac)
    pin_spacing = 2.54
    box_height = num_pins * pin_spacing + 7.62
    box_width = 25.4

    y_start = box_height / 2

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Ch A/C LVDS Out"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # LVDS outputs on LEFT to connect to FPGA on right
    y_pos = y_start - 5.08
    for name, pin_num in all_ac:
        lines.append(f'      (pin output line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 2: Channel B & D data outputs (connects to ADC 3/4 LVDS Data)
    # Clock pins moved to separate Unit 3
    lines.append('    (symbol "ADC12DL500ACF_Aligned_2_1"')

    ch_b_data = [
        ('DB0+', 'T9'), ('DB0-', 'T10'),
        ('DB1+', 'R9'), ('DB1-', 'R10'),
        ('DB2+', 'P9'), ('DB2-', 'P10'),
        ('DB3+', 'N9'), ('DB3-', 'N10'),
        ('DB4+', 'M9'), ('DB4-', 'M10'),
        ('DB5+', 'L9'), ('DB5-', 'L10'),
        ('DB6+', 'T11'), ('DB6-', 'T12'),
        ('DB7+', 'R11'), ('DB7-', 'R12'),
        ('DB8+', 'P11'), ('DB8-', 'P12'),
        ('DB9+', 'N11'), ('DB9-', 'N12'),
        ('DB10+', 'M11'), ('DB10-', 'M12'),
        ('DB11+', 'L11'), ('DB11-', 'L12'),
        # DBCLK moved to Unit 3
        ('DBSTR+', 'J9'), ('DBSTR-', 'J10'),
    ]

    ch_d_data = [
        ('DD0+', 'T13'), ('DD0-', 'T14'),
        ('DD1+', 'R13'), ('DD1-', 'R14'),
        ('DD2+', 'P13'), ('DD2-', 'P14'),
        ('DD3+', 'N13'), ('DD3-', 'N14'),
        ('DD4+', 'M13'), ('DD4-', 'M14'),
        ('DD5+', 'L13'), ('DD5-', 'L14'),
        ('DD6+', 'K13'), ('DD6-', 'K14'),
        ('DD7+', 'R15'), ('DD7-', 'R16'),
        ('DD8+', 'P15'), ('DD8-', 'P16'),
        ('DD9+', 'N15'), ('DD9-', 'N16'),
        ('DD10+', 'M15'), ('DD10-', 'M16'),
        ('DD11+', 'L15'), ('DD11-', 'L16'),
        # DDCLK moved to Unit 3
        ('DDSTR+', 'J15'), ('DDSTR-', 'J16'),
    ]

    all_bd = ch_b_data + ch_d_data
    num_pins2 = len(all_bd)
    box_height2 = num_pins2 * pin_spacing + 7.62
    y_start2 = box_height2 / 2

    lines.append(f'      (rectangle (start 0 {y_start2:.2f}) (end {box_width:.2f} {-y_start2:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Ch B/D LVDS Out"')
    lines.append(f'        (at {box_width/2:.2f} {y_start2 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start2 - 5.08
    for name, pin_num in all_bd:
        lines.append(f'      (pin output line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 3: LVDS Clock Outputs (connects to ADC LVDS Clocks unit on FPGA)
    # These align with lvds_rx_top_clkin1/2 and lvds_rx_bottom_clkin3/4
    lines.append('    (symbol "ADC12DL500ACF_Aligned_3_1"')

    lvds_clk_pins = [
        ('DBCLK+', 'K9'), ('DBCLK-', 'K10'),
        ('DDCLK+', 'K15'), ('DDCLK-', 'K16'),
        ('DACLK+', 'G9'), ('DACLK-', 'G10'),
        ('DCCLK+', 'G15'), ('DCCLK-', 'G16'),
    ]

    num_clk = len(lvds_clk_pins)
    height_clk = num_clk * pin_spacing + 7.62
    y_start_clk = height_clk / 2

    lines.append(f'      (rectangle (start 0 {y_start_clk:.2f}) (end {box_width:.2f} {-y_start_clk:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "LVDS Clock Out"')
    lines.append(f'        (at {box_width/2:.2f} {y_start_clk - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start_clk - 5.08
    for name, pin_num in lvds_clk_pins:
        lines.append(f'      (pin output line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 4: Analog Inputs
    lines.append('    (symbol "ADC12DL500ACF_Aligned_4_1"')

    analog_pins = [
        ('INA+', 'A4', 'input'),
        ('INA-', 'A5', 'input'),
        ('INB+', 'T4', 'input'),
        ('INB-', 'T5', 'input'),
        ('CLK+', 'H1', 'input'),
        ('CLK-', 'J1', 'input'),
    ]

    num_analog = len(analog_pins)
    height3 = num_analog * pin_spacing + 7.62
    y_start3 = height3 / 2

    lines.append(f'      (rectangle (start 0 {y_start3:.2f}) (end 20.32 {-y_start3:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Analog"')
    lines.append(f'        (at 10.16 {y_start3 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start3 - 5.08
    for name, pin_num, pin_type in analog_pins:
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 5: Control and SPI
    lines.append('    (symbol "ADC12DL500ACF_Aligned_5_1"')

    ctrl_pins = [
        ('CALSTAT', 'B1', 'output'),
        ('CALTRIG', 'B2', 'input'),
        ('SYNCSE', 'E2', 'input'),
        ('TMSTP+', 'D1', 'input'),
        ('TMSTP-', 'E1', 'input'),
        ('SYSREF+', 'M1', 'input'),
        ('SYSREF-', 'N1', 'input'),
        ('PD', 'R1', 'input'),
        ('SDI', 'P8', 'input'),
        ('SDO', 'N8', 'output'),
        ('SCLK', 'T8', 'input'),
        ('SCS', 'R8', 'input'),
        ('BG', 'D2', 'passive'),
        ('TDIODE+', 'M2', 'passive'),
        ('TDIODE-', 'N2', 'passive'),
    ]

    # Overrange outputs
    or_pins = [
        ('ORA0', 'B8', 'output'),
        ('ORA1', 'A8', 'output'),
        ('ORB0', 'D8', 'output'),
        ('ORB1', 'C8', 'output'),
    ]

    all_ctrl = ctrl_pins + or_pins
    num_ctrl = len(all_ctrl)
    height4 = num_ctrl * pin_spacing + 7.62
    y_start4 = height4 / 2

    lines.append(f'      (rectangle (start 0 {y_start4:.2f}) (end 20.32 {-y_start4:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Control/SPI"')
    lines.append(f'        (at 10.16 {y_start4 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start4 - 5.08
    for name, pin_num, pin_type in all_ctrl:
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 6: Power and Ground - ALL pins from original symbol
    lines.append('    (symbol "ADC12DL500ACF_Aligned_6_1"')

    # All power pins from original ADC12DL500ACF symbol
    power_pins = [
        # VA11 - Analog 1.1V (14 pins)
        ('VA11_1', 'E4', 'power_in'),
        ('VA11_2', 'E5', 'power_in'),
        ('VA11_3', 'E6', 'power_in'),
        ('VA11_4', 'F3', 'power_in'),
        ('VA11_5', 'F4', 'power_in'),
        ('VA11_6', 'F5', 'power_in'),
        ('VA11_7', 'F6', 'power_in'),
        ('VA11_8', 'L3', 'power_in'),
        ('VA11_9', 'L4', 'power_in'),
        ('VA11_10', 'L5', 'power_in'),
        ('VA11_11', 'L6', 'power_in'),
        ('VA11_12', 'M4', 'power_in'),
        ('VA11_13', 'M5', 'power_in'),
        ('VA11_14', 'M6', 'power_in'),
        # VA19 - Analog 1.9V (16 pins)
        ('VA19_1', 'G3', 'power_in'),
        ('VA19_2', 'G4', 'power_in'),
        ('VA19_3', 'G5', 'power_in'),
        ('VA19_4', 'G6', 'power_in'),
        ('VA19_5', 'H3', 'power_in'),
        ('VA19_6', 'H4', 'power_in'),
        ('VA19_7', 'H5', 'power_in'),
        ('VA19_8', 'H6', 'power_in'),
        ('VA19_9', 'J3', 'power_in'),
        ('VA19_10', 'J4', 'power_in'),
        ('VA19_11', 'J5', 'power_in'),
        ('VA19_12', 'J6', 'power_in'),
        ('VA19_13', 'K3', 'power_in'),
        ('VA19_14', 'K4', 'power_in'),
        ('VA19_15', 'K5', 'power_in'),
        ('VA19_16', 'K6', 'power_in'),
        # VD11 - Digital 1.1V (6 pins)
        ('VD11_1', 'E8', 'power_in'),
        ('VD11_2', 'F8', 'power_in'),
        ('VD11_3', 'G8', 'power_in'),
        ('VD11_4', 'K8', 'power_in'),
        ('VD11_5', 'L8', 'power_in'),
        ('VD11_6', 'M8', 'power_in'),
        # VLVDS - LVDS power (10 pins)
        ('VLVDS_1', 'G11', 'power_in'),
        ('VLVDS_2', 'G12', 'power_in'),
        ('VLVDS_3', 'H11', 'power_in'),
        ('VLVDS_4', 'H12', 'power_in'),
        ('VLVDS_5', 'H13', 'power_in'),
        ('VLVDS_6', 'J11', 'power_in'),
        ('VLVDS_7', 'J12', 'power_in'),
        ('VLVDS_8', 'J13', 'power_in'),
        ('VLVDS_9', 'K11', 'power_in'),
        ('VLVDS_10', 'K12', 'power_in'),
        # AGND - Analog ground (55 pins)
        ('AGND_1', 'A1', 'power_in'),
        ('AGND_2', 'A2', 'power_in'),
        ('AGND_3', 'A3', 'power_in'),
        ('AGND_4', 'A6', 'power_in'),
        ('AGND_5', 'A7', 'power_in'),
        ('AGND_6', 'B3', 'power_in'),
        ('AGND_7', 'B4', 'power_in'),
        ('AGND_8', 'B5', 'power_in'),
        ('AGND_9', 'B6', 'power_in'),
        ('AGND_10', 'B7', 'power_in'),
        ('AGND_11', 'C1', 'power_in'),
        ('AGND_12', 'C2', 'power_in'),
        ('AGND_13', 'C3', 'power_in'),
        ('AGND_14', 'C4', 'power_in'),
        ('AGND_15', 'C5', 'power_in'),
        ('AGND_16', 'C6', 'power_in'),
        ('AGND_17', 'C7', 'power_in'),
        ('AGND_18', 'D3', 'power_in'),
        ('AGND_19', 'D4', 'power_in'),
        ('AGND_20', 'D5', 'power_in'),
        ('AGND_21', 'D6', 'power_in'),
        ('AGND_22', 'E3', 'power_in'),
        ('AGND_23', 'F1', 'power_in'),
        ('AGND_24', 'F2', 'power_in'),
        ('AGND_25', 'G1', 'power_in'),
        ('AGND_26', 'G2', 'power_in'),
        ('AGND_27', 'H2', 'power_in'),
        ('AGND_28', 'J2', 'power_in'),
        ('AGND_29', 'K1', 'power_in'),
        ('AGND_30', 'K2', 'power_in'),
        ('AGND_31', 'L1', 'power_in'),
        ('AGND_32', 'L2', 'power_in'),
        ('AGND_33', 'M3', 'power_in'),
        ('AGND_34', 'N3', 'power_in'),
        ('AGND_35', 'N4', 'power_in'),
        ('AGND_36', 'N5', 'power_in'),
        ('AGND_37', 'N6', 'power_in'),
        ('AGND_38', 'P1', 'power_in'),
        ('AGND_39', 'P2', 'power_in'),
        ('AGND_40', 'P3', 'power_in'),
        ('AGND_41', 'P4', 'power_in'),
        ('AGND_42', 'P5', 'power_in'),
        ('AGND_43', 'P6', 'power_in'),
        ('AGND_44', 'P7', 'power_in'),
        ('AGND_45', 'R2', 'power_in'),
        ('AGND_46', 'R3', 'power_in'),
        ('AGND_47', 'R4', 'power_in'),
        ('AGND_48', 'R5', 'power_in'),
        ('AGND_49', 'R6', 'power_in'),
        ('AGND_50', 'R7', 'power_in'),
        ('AGND_51', 'T1', 'power_in'),
        ('AGND_52', 'T2', 'power_in'),
        ('AGND_53', 'T3', 'power_in'),
        ('AGND_54', 'T6', 'power_in'),
        ('AGND_55', 'T7', 'power_in'),
        # DGND - Digital ground (18 pins)
        ('DGND_1', 'A15', 'power_in'),
        ('DGND_2', 'A16', 'power_in'),
        ('DGND_3', 'D7', 'power_in'),
        ('DGND_4', 'E7', 'power_in'),
        ('DGND_5', 'F7', 'power_in'),
        ('DGND_6', 'G7', 'power_in'),
        ('DGND_7', 'H7', 'power_in'),
        ('DGND_8', 'H8', 'power_in'),
        ('DGND_9', 'H14', 'power_in'),
        ('DGND_10', 'J7', 'power_in'),
        ('DGND_11', 'J8', 'power_in'),
        ('DGND_12', 'J14', 'power_in'),
        ('DGND_13', 'K7', 'power_in'),
        ('DGND_14', 'L7', 'power_in'),
        ('DGND_15', 'M7', 'power_in'),
        ('DGND_16', 'N7', 'power_in'),
        ('DGND_17', 'T15', 'power_in'),
        ('DGND_18', 'T16', 'power_in'),
    ]

    # Separate power and ground pins
    # LEFT: VCC + DGND (46 + 18 = 64 pins)
    # RIGHT: AGND (55 pins)
    left_pins = [p for p in power_pins if not 'GND' in p[0] or 'DGND' in p[0]]  # VA11, VA19, VD11, VLVDS, DGND
    right_pins = [p for p in power_pins if 'AGND' in p[0]]  # AGND only

    # Use the larger count to determine height
    num_rows = max(len(left_pins), len(right_pins))
    height5 = num_rows * pin_spacing + 7.62
    y_start5 = height5 / 2
    box_width = 25.4  # Wider box for two columns

    lines.append(f'      (rectangle (start 0 {y_start5:.2f}) (end {box_width:.2f} {-y_start5:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Power"')
    lines.append(f'        (at {box_width/2:.2f} {y_start5 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # VCC + DGND pins on LEFT
    y_pos = y_start5 - 5.08
    for name, pin_num, pin_type in left_pins:
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    # AGND pins on RIGHT
    y_pos = y_start5 - 5.08
    for name, pin_num, pin_type in right_pins:
        lines.append(f'      (pin {pin_type} line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    lines.append('  )')
    return '\n'.join(lines)


def generate_ddr_aligned():
    """Generate MT53D512M16D1DS symbol with pins aligned to FPGA DDR unit.

    FPGA DDR4 Interface unit has:
    - Data pins (DQ, DQS, DM) on LEFT side
    - Control/Address pins (CS, CKE, CK, CA, RST) on RIGHT side

    DDR_Aligned symbol has:
    - Unit 1: Data pins on RIGHT (connects to FPGA data on LEFT)
    - Unit 2: Control/Address pins on LEFT (connects to FPGA control on RIGHT)
    """
    lines = []
    lines.append('  (symbol "MT53D512M16D1DS_Aligned"')
    lines.append('    (pin_names (offset 1.016))')
    lines.append('    (exclude_from_sim no)')
    lines.append('    (in_bom yes)')
    lines.append('    (on_board yes)')

    # Properties
    lines.append('    (property "Reference" "U"')
    lines.append('      (at 0 1.27 0)')
    lines.append('      (effects (font (size 1.27 1.27)))')
    lines.append('    )')
    lines.append('    (property "Value" "MT53D512M16D1DS_Aligned"')
    lines.append('      (at 0 -1.27 0)')
    lines.append('      (effects (font (size 1.27 1.27)))')
    lines.append('    )')
    lines.append('    (property "Footprint" "easyeda2kicad:WFBGA-200_L14.5-W10.0-BL_MT53D512M32D2DS"')
    lines.append('      (at 0 -3.81 0)')
    lines.append('      (effects (font (size 1.27 1.27)) hide)')
    lines.append('    )')
    lines.append('    (property "Description" "MT53D512M16D1DS LPDDR4 - pins aligned with Ti90G529 FPGA"')
    lines.append('      (at 0 -6.35 0)')
    lines.append('      (effects (font (size 1.27 1.27)) hide)')
    lines.append('    )')

    pin_spacing = 2.54
    box_width = 25.4

    # Unit 1 - Data pins (DQ, DQS, DMI) - pins on RIGHT to connect to FPGA LEFT
    lines.append('    (symbol "MT53D512M16D1DS_Aligned_1_1"')

    # Data pins ordered to match FPGA DDR data pin order
    # FPGA has: DQ[0-7], DQS[0], DQS_N[0], DM[0], DQ[8-15], DQS[1], DQS_N[1], DM[1], ...
    data_pins = []

    # Byte lane 0 (DQ0-7)
    dq0_pins_a = [
        ('DQ0_A', 'B2'), ('DQ1_A', 'C2'), ('DQ2_A', 'E2'), ('DQ3_A', 'F2'),
        ('DQ4_A', 'F4'), ('DQ5_A', 'E4'), ('DQ6_A', 'C4'), ('DQ7_A', 'B4'),
    ]
    dqs0_pins_a = [('DQS0_t_A', 'D3'), ('DQS0_c_A', 'E3'), ('DMI0_A', 'C3')]

    # Byte lane 1 (DQ8-15)
    dq1_pins_a = [
        ('DQ8_A', 'B11'), ('DQ9_A', 'C11'), ('DQ10_A', 'E11'), ('DQ11_A', 'F11'),
        ('DQ12_A', 'F9'), ('DQ13_A', 'E9'), ('DQ14_A', 'C9'), ('DQ15_A', 'B9'),
    ]
    dqs1_pins_a = [('DQS1_t_A', 'D10'), ('DQS1_c_A', 'E10'), ('DMI1_A', 'C10')]

    # Channel B byte lanes
    dq0_pins_b = [
        ('DQ0_B', 'AA2'), ('DQ1_B', 'Y2'), ('DQ2_B', 'V2'), ('DQ3_B', 'U2'),
        ('DQ4_B', 'U4'), ('DQ5_B', 'V4'), ('DQ6_B', 'Y4'), ('DQ7_B', 'AA4'),
    ]
    dqs0_pins_b = [('DQS0_t_B', 'W3'), ('DQS0_c_B', 'V3'), ('DMI0_B', 'Y3')]

    dq1_pins_b = [
        ('DQ8_B', 'AA11'), ('DQ9_B', 'Y11'), ('DQ10_B', 'V11'), ('DQ11_B', 'U11'),
        ('DQ12_B', 'U9'), ('DQ13_B', 'V9'), ('DQ14_B', 'Y9'), ('DQ15_B', 'AA9'),
    ]
    dqs1_pins_b = [('DQS1_t_B', 'W10'), ('DQS1_c_B', 'V10'), ('DMI1_B', 'Y10')]

    # Order to match FPGA: byte0, byte1, byte2, byte3
    for pin in dq0_pins_a:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dqs0_pins_a:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dq1_pins_a:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dqs1_pins_a:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dq0_pins_b:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dqs0_pins_b:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dq1_pins_b:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dqs1_pins_b:
        data_pins.append((pin[0], pin[1], 'bidirectional'))

    num_data = len(data_pins)
    box_height = num_data * pin_spacing + 7.62
    y_start = box_height / 2

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Data (DQ/DQS/DMI)"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Data pins on RIGHT to connect to FPGA data pins on LEFT
    y_pos = y_start - 5.08
    for name, pin_num, pin_type in data_pins:
        lines.append(f'      (pin {pin_type} line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 2 - Control and Address pins - pins on LEFT to connect to FPGA RIGHT
    lines.append('    (symbol "MT53D512M16D1DS_Aligned_2_1"')

    # Control pins ordered to align with FPGA DDR control pins on right side
    # FPGA has (top to bottom): CS_N[0-3], CKE[0-1], CK, CK_N, RST_N, A[0-5], CAL
    #
    # When one FPGA pin connects to two DDR pins (_A and _B), both DDR pins
    # are placed on consecutive rows so a wire can branch to both.
    # When FPGA pins are not connected (CS_N[2,3], CKE[1]), we leave a gap.
    #
    # Format: (name, pin_number, pin_type, y_offset)
    # y_offset is in units of pin_spacing from the top
    ctrl_pins = [
        # DDR_CS_N[0] -> CS0_A
        ('CS0_A', 'H4', 'input', 0),
        # DDR_CS_N[1] -> CS0_B
        ('CS0_B', 'R4', 'input', 1),
        # DDR_CS_N[2] -> not connected (gap at position 2)
        # DDR_CS_N[3] -> not connected (gap at position 3)
        # DDR_CKE[0] -> CKE0_A and CKE0_B (consecutive rows for branching)
        ('CKE0_A', 'J4', 'input', 4),
        ('CKE0_B', 'P4', 'input', 5),
        # DDR_CKE[1] -> not connected (gap at position 6)
        # DDR_CK -> CK_t_A and CK_t_B (consecutive rows)
        ('CK_t_A', 'J8', 'input', 7),
        ('CK_t_B', 'P8', 'input', 8),
        # DDR_CK_N -> CK_c_A and CK_c_B (consecutive rows)
        ('CK_c_A', 'J9', 'input', 9),
        ('CK_c_B', 'P9', 'input', 10),
        # DDR_RST_N -> RESET_n
        ('RESET_n', 'T11', 'input', 11),
        # DDR_A[0] -> CA0_A and CA0_B
        ('CA0_A', 'H2', 'input', 12),
        ('CA0_B', 'R2', 'input', 13),
        # DDR_A[1] -> CA1_A and CA1_B
        ('CA1_A', 'J2', 'input', 14),
        ('CA1_B', 'P2', 'input', 15),
        # DDR_A[2] -> CA2_A and CA2_B
        ('CA2_A', 'H9', 'input', 16),
        ('CA2_B', 'R9', 'input', 17),
        # DDR_A[3] -> CA3_A and CA3_B
        ('CA3_A', 'H10', 'input', 18),
        ('CA3_B', 'R10', 'input', 19),
        # DDR_A[4] -> CA4_A and CA4_B
        ('CA4_A', 'H11', 'input', 20),
        ('CA4_B', 'R11', 'input', 21),
        # DDR_A[5] -> CA5_A and CA5_B
        ('CA5_A', 'J11', 'input', 22),
        ('CA5_B', 'P11', 'input', 23),
        # DDR_CAL -> ZQ0 (and ODT_CA pins nearby)
        ('ZQ0', 'A5', 'passive', 24),
        ('ODT_CA_A', 'G2', 'input', 25),
        ('ODT_CA_B', 'T2', 'input', 26),
    ]

    # Total rows including gaps
    num_rows = 27
    box_height2 = num_rows * pin_spacing + 7.62
    y_start2 = box_height2 / 2

    lines.append(f'      (rectangle (start 0 {y_start2:.2f}) (end {box_width:.2f} {-y_start2:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Control/Address"')
    lines.append(f'        (at {box_width/2:.2f} {y_start2 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Control pins on LEFT to connect to FPGA control pins on RIGHT
    # Use explicit y_offset for each pin to allow gaps
    y_top = y_start2 - 5.08
    for name, pin_num, pin_type, y_offset in ctrl_pins:
        y_pos = y_top - (y_offset * pin_spacing)
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')

    lines.append('    )')

    # Unit 3 - Power (VDD1, VDD2, VDDQ)
    lines.append('    (symbol "MT53D512M16D1DS_Aligned_3_1"')

    vdd1_pins = [
        ('F1', 'VDD1'), ('F12', 'VDD1'), ('G4', 'VDD1'), ('G9', 'VDD1'),
        ('T4', 'VDD1'), ('T9', 'VDD1'), ('U1', 'VDD1'), ('U12', 'VDD1'),
    ]
    vdd2_pins = [
        ('A4', 'VDD2'), ('A9', 'VDD2'), ('AB4', 'VDD2'), ('AB9', 'VDD2'),
        ('F5', 'VDD2'), ('F8', 'VDD2'), ('H1', 'VDD2'), ('H5', 'VDD2'),
        ('H8', 'VDD2'), ('H12', 'VDD2'), ('K1', 'VDD2'), ('K3', 'VDD2'),
        ('K10', 'VDD2'), ('K12', 'VDD2'), ('N1', 'VDD2'), ('N3', 'VDD2'),
        ('N10', 'VDD2'), ('N12', 'VDD2'), ('R1', 'VDD2'), ('R5', 'VDD2'),
        ('R8', 'VDD2'), ('R12', 'VDD2'), ('U5', 'VDD2'), ('U8', 'VDD2'),
    ]
    vddq_pins = [
        ('AA3', 'VDDQ'), ('AA5', 'VDDQ'), ('AA8', 'VDDQ'), ('AA10', 'VDDQ'),
        ('B3', 'VDDQ'), ('B5', 'VDDQ'), ('B8', 'VDDQ'), ('B10', 'VDDQ'),
        ('D1', 'VDDQ'), ('D5', 'VDDQ'), ('D8', 'VDDQ'), ('D12', 'VDDQ'),
        ('F3', 'VDDQ'), ('F10', 'VDDQ'), ('U3', 'VDDQ'), ('U10', 'VDDQ'),
        ('W1', 'VDDQ'), ('W5', 'VDDQ'), ('W8', 'VDDQ'), ('W12', 'VDDQ'),
    ]

    power_pins = []
    for pin, name in vdd1_pins:
        power_pins.append((name, pin, 'power_in'))
    for pin, name in vdd2_pins:
        power_pins.append((name, pin, 'power_in'))
    for pin, name in vddq_pins:
        power_pins.append((name, pin, 'power_in'))

    num_pwr = len(power_pins)
    # Split between left and right sides
    half = (num_pwr + 1) // 2
    left_pwr = power_pins[:half]
    right_pwr = power_pins[half:]

    height3 = half * pin_spacing + 7.62
    y_start3 = height3 / 2

    lines.append(f'      (rectangle (start 0 {y_start3:.2f}) (end 25.40 {-y_start3:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Power (VDD)"')
    lines.append(f'        (at 12.70 {y_start3 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Left side
    y_pos = y_start3 - 5.08
    for name, pin_num, pin_type in left_pwr:
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    # Right side
    y_pos = y_start3 - 5.08
    for name, pin_num, pin_type in right_pwr:
        lines.append(f'      (pin {pin_type} line (at 27.94 {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 4 - Ground (VSS)
    lines.append('    (symbol "MT53D512M16D1DS_Aligned_4_1"')

    vss_pins = [
        'A3', 'A10', 'AB3', 'AB5', 'AB8', 'AB10',
        'C1', 'C5', 'C8', 'C12', 'D2', 'D4', 'D9', 'D11',
        'E1', 'E5', 'E8', 'E12', 'G1', 'G3', 'G5', 'G8', 'G10', 'G12',
        'J1', 'J3', 'J10', 'J12', 'K2', 'K4', 'K9', 'K11',
        'N2', 'N4', 'N9', 'N11', 'P1', 'P3', 'P10', 'P12',
        'T1', 'T3', 'T5', 'T8', 'T10', 'T12',
        'V1', 'V5', 'V8', 'V12', 'W2', 'W4', 'W9', 'W11',
        'Y1', 'Y5', 'Y8', 'Y12',
    ]

    gnd_pins = [('VSS', pin, 'power_in') for pin in vss_pins]
    num_gnd = len(gnd_pins)
    half_gnd = (num_gnd + 1) // 2
    left_gnd = gnd_pins[:half_gnd]
    right_gnd = gnd_pins[half_gnd:]

    height4 = half_gnd * pin_spacing + 7.62
    y_start4 = height4 / 2

    lines.append(f'      (rectangle (start 0 {y_start4:.2f}) (end 20.32 {-y_start4:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Ground (VSS)"')
    lines.append(f'        (at 10.16 {y_start4 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Left side
    y_pos = y_start4 - 5.08
    for name, pin_num, pin_type in left_gnd:
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    # Right side
    y_pos = y_start4 - 5.08
    for name, pin_num, pin_type in right_gnd:
        lines.append(f'      (pin {pin_type} line (at 22.86 {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 5 - NC and DNU pins
    lines.append('    (symbol "MT53D512M16D1DS_Aligned_5_1"')

    nc_pins = ['A8', 'G11', 'H3', 'J5', 'K5', 'K8', 'N5', 'N8', 'P5', 'R3']
    dnu_pins = ['A1', 'A2', 'A11', 'A12', 'AA1', 'AA12', 'AB1', 'AB2', 'AB11', 'AB12', 'B1', 'B12']

    misc_pins = []
    for pin in nc_pins:
        misc_pins.append(('NC', pin, 'no_connect'))
    for pin in dnu_pins:
        misc_pins.append(('DNU', pin, 'passive'))

    num_misc = len(misc_pins)
    height5 = num_misc * pin_spacing + 7.62
    y_start5 = height5 / 2

    lines.append(f'      (rectangle (start 0 {y_start5:.2f}) (end 20.32 {-y_start5:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "NC/DNU"')
    lines.append(f'        (at 10.16 {y_start5 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start5 - 5.08
    for name, pin_num, pin_type in misc_pins:
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    lines.append('  )')
    return '\n'.join(lines)


def main():
    script_dir = Path(__file__).parent
    output_path = script_dir / 'Aligned_Peripheral_Symbols.kicad_sym'

    print("Generating aligned peripheral symbols...")

    with open(output_path, 'w') as f:
        f.write(generate_symbol_header())
        f.write(generate_ft601q_aligned())
        f.write('\n')
        f.write(generate_adc12dl_aligned())
        f.write('\n')
        f.write(generate_ddr_aligned())
        f.write(')\n')

    print(f"Generated: {output_path}")
    print("\nSymbols created:")
    print("  - FT601Q_Aligned: FIFO interface pins on RIGHT to connect to FPGA on LEFT")
    print("  - ADC12DL500ACF_Aligned: LVDS outputs on LEFT to connect to FPGA on RIGHT")
    print("  - MT53D512M16D1DS_Aligned: Data/control pins on RIGHT to connect to FPGA on LEFT")


if __name__ == "__main__":
    main()
