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
    control_pins = [
        ('CLK', '58'),
        ('~{TXE}', '8'),
        ('~{RXF}', '9'),
        ('~{OE}', '13'),
        ('~{WR}', '11'),
        ('~{RD}', '12'),
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
    for name, pin_num in control_pins:
        pin_type = 'output' if 'TXE' in name or 'RXF' in name or 'CLK' in name else 'input'
        lines.append(f'      (pin {pin_type} line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 2 - USB and Control (other pins)
    lines.append('    (symbol "FT601Q_Aligned_2_1"')

    other_pins = [
        ('~{SIWU}', '10', 'input'),
        ('~{RESET}', '15', 'input'),
        ('~{WAKEUP}', '16', 'bidirectional'),
        ('GPIO0', '17', 'bidirectional'),
        ('GPIO1', '18', 'bidirectional'),
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

    lines.append('      (text "USB/Control"')
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
        ('GND', '26', 'passive'),
        ('GND', '29', 'passive'),
        ('GND', '36', 'passive'),
        ('GND', '77', 'passive'),
    ]

    num_pwr = 12  # Visible power pins
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
    seen_names = set()
    for name, pin_num, pin_type in power_pins:
        hide = 'yes' if name in seen_names else ''
        hide_str = '(hide yes)' if hide else ''
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54) {hide_str}')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        if name not in seen_names:
            seen_names.add(name)
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
        ('DACLK+', 'G9'), ('DACLK-', 'G10'),
        ('DCCLK+', 'G15'), ('DCCLK-', 'G16'),
        ('DBCLK+', 'K9'), ('DBCLK-', 'K10'),
        ('DDCLK+', 'K15'), ('DDCLK-', 'K16'),
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

    # Unit 6: Power and Ground (simplified - just key pins)
    lines.append('    (symbol "ADC12DL500ACF_Aligned_6_1"')

    power_pins = [
        ('VA11', 'E4', 'power_in'),
        ('VA19', 'G3', 'power_in'),
        ('VD11', 'E8', 'power_in'),
        ('VLVDS', 'G11', 'power_in'),
        ('AGND', 'A1', 'power_in'),
        ('DGND', 'A15', 'power_in'),
    ]

    num_pwr = len(power_pins)
    height5 = num_pwr * pin_spacing + 7.62
    y_start5 = height5 / 2

    lines.append(f'      (rectangle (start 0 {y_start5:.2f}) (end 20.32 {-y_start5:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Power"')
    lines.append(f'        (at 10.16 {y_start5 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start5 - 5.08
    for name, pin_num, pin_type in power_pins:
        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    lines.append('  )')
    return '\n'.join(lines)


def generate_ddr_aligned():
    """Generate MT53D512M16D1DS symbol with pins aligned to FPGA DDR unit.

    FPGA DDR4 Interface unit has pins on LEFT in this order:
    CS_N, CKE, CK, CK_N, RST_N, A[0-5], CAL,
    DQ[0-7], DQS[0], DQS_N[0], DM[0],
    DQ[8-15], DQS[1], DQS_N[1], DM[1], ...

    DDR_Aligned puts corresponding pins on RIGHT in same order.
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

    # Unit 1 - Control and Address pins
    lines.append('    (symbol "MT53D512M16D1DS_Aligned_1_1"')

    # Pins to match FPGA DDR unit order
    ctrl_pins = [
        ('CS0_A', 'H4', 'input'),
        ('CS0_B', 'R4', 'input'),
        ('CKE0_A', 'J4', 'input'),
        ('CKE0_B', 'P4', 'input'),
        ('CK_t_A', 'J8', 'input'),
        ('CK_t_B', 'P8', 'input'),
        ('CK_c_A', 'J9', 'input'),
        ('CK_c_B', 'P9', 'input'),
        ('RESET_n', 'T11', 'input'),
        ('ODT_CA_A', 'G2', 'input'),
        ('ODT_CA_B', 'T2', 'input'),
        ('CA0_A', 'H2', 'input'),
        ('CA0_B', 'R2', 'input'),
        ('CA1_A', 'J2', 'input'),
        ('CA1_B', 'P2', 'input'),
        ('CA2_A', 'H9', 'input'),
        ('CA2_B', 'R9', 'input'),
        ('CA3_A', 'H10', 'input'),
        ('CA3_B', 'R10', 'input'),
        ('CA4_A', 'H11', 'input'),
        ('CA4_B', 'R11', 'input'),
        ('CA5_A', 'J11', 'input'),
        ('CA5_B', 'P11', 'input'),
        ('ZQ0', 'A5', 'passive'),
    ]

    num_ctrl = len(ctrl_pins)
    pin_spacing = 2.54
    box_height = num_ctrl * pin_spacing + 7.62
    box_width = 25.4
    y_start = box_height / 2

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Control/Address"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Control pins on RIGHT to connect to FPGA on left
    y_pos = y_start - 5.08
    for name, pin_num, pin_type in ctrl_pins:
        lines.append(f'      (pin {pin_type} line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 2 - Data pins (DQ, DQS, DMI for both channels)
    lines.append('    (symbol "MT53D512M16D1DS_Aligned_2_1"')

    # Data pins - organized by byte lane to match FPGA
    data_pins = []

    # Byte lane 0 (DQ0-7)
    dq0_pins_a = [
        ('DQ0_A', 'B2'), ('DQ1_A', 'C2'), ('DQ2_A', 'E2'), ('DQ3_A', 'F2'),
        ('DQ4_A', 'F4'), ('DQ5_A', 'E4'), ('DQ6_A', 'C4'), ('DQ7_A', 'B4'),
    ]
    dq0_pins_b = [
        ('DQ0_B', 'AA2'), ('DQ1_B', 'Y2'), ('DQ2_B', 'V2'), ('DQ3_B', 'U2'),
        ('DQ4_B', 'U4'), ('DQ5_B', 'V4'), ('DQ6_B', 'Y4'), ('DQ7_B', 'AA4'),
    ]

    dqs0_pins = [
        ('DQS0_t_A', 'D3'), ('DQS0_c_A', 'E3'),
        ('DQS0_t_B', 'W3'), ('DQS0_c_B', 'V3'),
        ('DMI0_A', 'C3'), ('DMI0_B', 'Y3'),
    ]

    # Byte lane 1 (DQ8-15)
    dq1_pins_a = [
        ('DQ8_A', 'B11'), ('DQ9_A', 'C11'), ('DQ10_A', 'E11'), ('DQ11_A', 'F11'),
        ('DQ12_A', 'F9'), ('DQ13_A', 'E9'), ('DQ14_A', 'C9'), ('DQ15_A', 'B9'),
    ]
    dq1_pins_b = [
        ('DQ8_B', 'AA11'), ('DQ9_B', 'Y11'), ('DQ10_B', 'V11'), ('DQ11_B', 'U11'),
        ('DQ12_B', 'U9'), ('DQ13_B', 'V9'), ('DQ14_B', 'Y9'), ('DQ15_B', 'AA9'),
    ]

    dqs1_pins = [
        ('DQS1_t_A', 'D10'), ('DQS1_c_A', 'E10'),
        ('DQS1_t_B', 'W10'), ('DQS1_c_B', 'V10'),
        ('DMI1_A', 'C10'), ('DMI1_B', 'Y10'),
    ]

    # Combine all data pins
    for pin in dq0_pins_a + dq0_pins_b:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dqs0_pins:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dq1_pins_a + dq1_pins_b:
        data_pins.append((pin[0], pin[1], 'bidirectional'))
    for pin in dqs1_pins:
        data_pins.append((pin[0], pin[1], 'bidirectional'))

    num_data = len(data_pins)
    box_height2 = num_data * pin_spacing + 7.62
    y_start2 = box_height2 / 2

    lines.append(f'      (rectangle (start 0 {y_start2:.2f}) (end {box_width:.2f} {-y_start2:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "Data (DQ/DQS/DMI)"')
    lines.append(f'        (at {box_width/2:.2f} {y_start2 - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start2 - 5.08
    for name, pin_num, pin_type in data_pins:
        lines.append(f'      (pin {pin_type} line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{pin_num}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')

    # Unit 3 - Power
    lines.append('    (symbol "MT53D512M16D1DS_Aligned_3_1"')

    power_pins = [
        ('VDD1', 'F1', 'power_in'),
        ('VDD2', 'K1', 'power_in'),
        ('VDDQ', 'B3', 'power_in'),
        ('VSS', 'A3', 'power_in'),
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
