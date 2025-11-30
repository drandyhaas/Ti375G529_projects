#!/usr/bin/env python3
"""
Generate KiCad schematic symbol for Efinix Ti90G529 FPGA
organized by peripheral interface to align with connected chips:
- FT601Q USB3 FIFO
- ADC12DL500ACF dual ADCs (x2)
- MT53D512M16D1DS LPDDR4 memory

This script creates a multi-unit symbol where each unit's pins are arranged
to line up with the corresponding peripheral chip for easy schematic wiring.

Units:
1. FT601 Interface - pins on LEFT (connects to FT601Q on right)
2. ADC 1/2 LVDS Data - pins on RIGHT (connects to ADC on left)
3. ADC 3/4 LVDS Data - pins on RIGHT (connects to ADC on left)
4. ADC LVDS Clocks - pins on RIGHT (connects to ADC CLK outputs)
5. DDR4 Interface - pins on LEFT (connects to DDR on right)
6. Misc I/O - LEDs, buttons, etc.
7. CONFIG/JTAG
8. VCC Power
9. GND
10. Unassigned GPIO
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

from generate_ti90g529_symbol import (
    GPIOL_PINS, GPIOR_TR_PINS, GPIOR_BR_PINS,
    GPIOR_3A_PINS, GPIOR_3B_PINS, GPIOR_3C_PINS,
    GPIOT_2A_PINS, GPIOT_2B_PINS, GPIOT_2C_PINS,
    GPIOB_4A_PINS, GPIOB_4B_PINS, GPIOB_4C_PINS,
    DDR_PINS, PINOUT
)


def parse_peri_xml(xml_path):
    """Parse peri.xml to extract GPIO to signal name mapping"""
    gpio_to_signal = {}
    lvds_to_signal = {}

    tree = ET.parse(xml_path)
    root = tree.getroot()

    ns = {'efxpt': 'http://www.efinixinc.com/peri_design_db'}

    for gpio in root.findall('.//efxpt:comp_gpio', ns):
        signal_name = gpio.get('name')
        gpio_def = gpio.get('gpio_def')
        mode = gpio.get('mode')
        if signal_name and gpio_def:
            gpio_to_signal[gpio_def] = {'signal': signal_name, 'mode': mode}

    for lvds in root.findall('.//efxpt:lvds', ns):
        signal_name = lvds.get('name')
        lvds_def = lvds.get('lvds_def')
        ops_type = lvds.get('ops_type')
        if signal_name and lvds_def:
            lvds_to_signal[lvds_def] = {'signal': signal_name, 'mode': ops_type}

    return gpio_to_signal, lvds_to_signal


def normalize_gpio_name(pin_name):
    """Convert pin name to gpio_def format for matching"""
    name = re.sub(r'_PLLIN\d*$', '', pin_name)
    name = re.sub(r'_CLK\d+(_[PN])?$', '', name)
    name = re.sub(r'_EXTFB$', '', name)
    name = re.sub(r'_CDI\d+$', '', name)
    name = re.sub(r'_CCK$', '', name)
    name = re.sub(r'_CSI$', '', name)
    name = re.sub(r'_CSO$', '', name)
    name = re.sub(r'_SSL_N$', '', name)
    name = re.sub(r'_SSU_N$', '', name)
    name = re.sub(r'_CBSEL\d+$', '', name)
    name = re.sub(r'_NSTATUS$', '', name)
    name = re.sub(r'_TEST_N$', '', name)
    name = re.sub(r'_EXTSPICLK$', '', name)
    return name


def get_signal_for_pin(pin_name, ball, gpio_map, lvds_map):
    """Get the signal name for a given pin"""
    normalized = normalize_gpio_name(pin_name)

    if normalized in gpio_map:
        return gpio_map[normalized]['signal'], gpio_map[normalized]['mode']

    if pin_name in gpio_map:
        return gpio_map[pin_name]['signal'], gpio_map[pin_name]['mode']

    # Try LVDS mapping for positive pins
    lvds_name = re.sub(r'GPIO([TB])_P_(\d+)', r'GPIO\1_PN_\2', normalized)
    if lvds_name in lvds_map:
        return lvds_map[lvds_name]['signal'], lvds_map[lvds_name]['mode']

    # Try LVDS mapping for negative pins
    lvds_name_n = re.sub(r'GPIO([TB])_N_(\d+)', r'GPIO\1_PN_\2', normalized)
    if lvds_name_n in lvds_map:
        signal = lvds_map[lvds_name_n]['signal'] + '_N'
        return signal, lvds_map[lvds_name_n]['mode']

    return None, None


def categorize_pins(gpio_map, lvds_map):
    """Categorize all GPIO pins by their peripheral function"""
    ft601_pins = {}      # FT601 interface
    adc12_data_pins = {} # ADC 1/2 LVDS data (lvds_rx1_*, lvds_rx2_*)
    adc34_data_pins = {} # ADC 3/4 LVDS data (lvds_rx3_*, lvds_rx4_*)
    adc_clk_pins = {}    # ADC LVDS clock inputs (lvds_rx_top_clkin*, lvds_rx_bottom_clkin*)
    misc_io_pins = {}    # LEDs, buttons, etc.
    unassigned_pins = {} # Pins without function assignment

    # Combine all GPIO pin dictionaries
    all_gpio = {}
    all_gpio.update(GPIOL_PINS)
    all_gpio.update(GPIOR_TR_PINS)
    all_gpio.update(GPIOR_BR_PINS)
    all_gpio.update(GPIOR_3A_PINS)
    all_gpio.update(GPIOR_3B_PINS)
    all_gpio.update(GPIOR_3C_PINS)
    all_gpio.update(GPIOT_2A_PINS)
    all_gpio.update(GPIOT_2B_PINS)
    all_gpio.update(GPIOT_2C_PINS)
    all_gpio.update(GPIOB_4A_PINS)
    all_gpio.update(GPIOB_4B_PINS)
    all_gpio.update(GPIOB_4C_PINS)

    for pin_name, ball in all_gpio.items():
        signal, mode = get_signal_for_pin(pin_name, ball, gpio_map, lvds_map)

        if signal:
            if signal.startswith('ftdi_'):
                ft601_pins[pin_name] = (ball, signal, mode)
            elif signal.startswith('lvds_rx_top_clkin') or signal.startswith('lvds_rx_bottom_clkin'):
                # LVDS clock inputs - separate unit
                adc_clk_pins[pin_name] = (ball, signal, mode)
            elif signal.startswith('lvds_rx1_') or signal.startswith('lvds_rx2_'):
                adc12_data_pins[pin_name] = (ball, signal, mode)
            elif signal.startswith('lvds_rx3_') or signal.startswith('lvds_rx4_'):
                adc34_data_pins[pin_name] = (ball, signal, mode)
            else:
                misc_io_pins[pin_name] = (ball, signal, mode)
        else:
            unassigned_pins[pin_name] = (ball, pin_name, 'unassigned')

    return ft601_pins, adc12_data_pins, adc34_data_pins, adc_clk_pins, misc_io_pins, unassigned_pins


def sort_ft601_pins(ft601_pins):
    """Sort FT601 pins to match FT601Q symbol layout"""
    sorted_pins = []

    # Data pins in order
    data_pins = [(k, v) for k, v in ft601_pins.items() if 'ftdi_data[' in v[1]]
    data_pins.sort(key=lambda x: int(re.search(r'\[(\d+)\]', x[1][1]).group(1)))
    sorted_pins.extend(data_pins)

    # BE pins
    be_pins = [(k, v) for k, v in ft601_pins.items() if 'ftdi_be[' in v[1]]
    be_pins.sort(key=lambda x: int(re.search(r'\[(\d+)\]', x[1][1]).group(1)))
    sorted_pins.extend(be_pins)

    # Clock
    clk_pins = [(k, v) for k, v in ft601_pins.items() if v[1] == 'ftdi_clk']
    sorted_pins.extend(clk_pins)

    # Control signals
    control_order = ['ftdi_txe', 'ftdi_rxf', 'ftdi_oe', 'ftdi_wr', 'ftdi_rd']
    for ctrl in control_order:
        ctrl_pins = [(k, v) for k, v in ft601_pins.items() if v[1] == ctrl]
        sorted_pins.extend(ctrl_pins)

    # Any remaining
    remaining = [(k, v) for k, v in ft601_pins.items() if (k, v) not in sorted_pins]
    sorted_pins.extend(remaining)

    return sorted_pins


def sort_lvds_pins(lvds_pins):
    """Sort LVDS pins by channel then bit number, P followed by N"""
    sorted_pins = []

    # Get unique signal base names
    signal_bases = set()
    for pin_name, (ball, signal, mode) in lvds_pins.items():
        base = signal.rstrip('_N')
        signal_bases.add(base)

    def signal_sort_key(base):
        match = re.match(r'lvds_rx(\d+)_(\d+)', base)
        if match:
            return (int(match.group(1)), int(match.group(2)))
        return (100, 0)

    sorted_bases = sorted(signal_bases, key=signal_sort_key)

    for base in sorted_bases:
        # Find positive pin
        for pin_name, (ball, signal, mode) in lvds_pins.items():
            if signal == base:
                sorted_pins.append((pin_name, (ball, signal, mode)))
                break
        # Find negative pin
        for pin_name, (ball, signal, mode) in lvds_pins.items():
            if signal == base + '_N':
                sorted_pins.append((pin_name, (ball, signal, mode)))
                break

    return sorted_pins


def sort_adc_clk_pins(adc_clk_pins):
    """Sort ADC clock pins: clkin1, clkin1_N, clkin2, clkin2_N, clkin3, clkin3_N, clkin4, clkin4_N"""
    sorted_pins = []

    # Get unique signal base names
    signal_bases = set()
    for pin_name, (ball, signal, mode) in adc_clk_pins.items():
        base = signal.rstrip('_N')
        signal_bases.add(base)

    def clk_sort_key(base):
        # Extract clkin number
        match = re.search(r'clkin(\d+)', base)
        if match:
            return int(match.group(1))
        return 100

    sorted_bases = sorted(signal_bases, key=clk_sort_key)

    for base in sorted_bases:
        # Find positive pin
        for pin_name, (ball, signal, mode) in adc_clk_pins.items():
            if signal == base:
                sorted_pins.append((pin_name, (ball, signal, mode)))
                break
        # Find negative pin
        for pin_name, (ball, signal, mode) in adc_clk_pins.items():
            if signal == base + '_N':
                sorted_pins.append((pin_name, (ball, signal, mode)))
                break

    return sorted_pins


def sort_ddr_pins():
    """Sort DDR pins to match LPDDR4 symbol layout"""
    sorted_pins = []

    order = [
        'DDR_CS_N[0]', 'DDR_CS_N[1]', 'DDR_CS_N[2]', 'DDR_CS_N[3]',
        'DDR_CKE[0]', 'DDR_CKE[1]',
        'DDR_CK', 'DDR_CK_N',
        'DDR_RST_N',
        'DDR_A[0]', 'DDR_A[1]', 'DDR_A[2]', 'DDR_A[3]', 'DDR_A[4]', 'DDR_A[5]',
        'DDR_CAL',
    ]

    for name in order:
        if name in DDR_PINS:
            sorted_pins.append((name, DDR_PINS[name]))

    # Add DQ pins by byte lane
    for byte_lane in range(4):
        for bit in range(8):
            name = f'DDR_DQ[{byte_lane * 8 + bit}]'
            if name in DDR_PINS:
                sorted_pins.append((name, DDR_PINS[name]))
        dqs_name = f'DDR_DQS[{byte_lane}]'
        dqs_n_name = f'DDR_DQS_N[{byte_lane}]'
        dm_name = f'DDR_DM[{byte_lane}]'
        if dqs_name in DDR_PINS:
            sorted_pins.append((dqs_name, DDR_PINS[dqs_name]))
        if dqs_n_name in DDR_PINS:
            sorted_pins.append((dqs_n_name, DDR_PINS[dqs_n_name]))
        if dm_name in DDR_PINS:
            sorted_pins.append((dm_name, DDR_PINS[dm_name]))

    return sorted_pins


def generate_ddr_unit(unit_num, symbol_name):
    """Generate DDR4 Interface unit with data pins on LEFT and control on RIGHT.

    Control pins have gaps to align with MT53D512M16D1DS_Aligned symbol where
    each FPGA pin connects to two DDR pins (_A and _B channels).
    Gaps are left where FPGA pins don't connect (CS_N[2,3], CKE[1]).
    """
    lines = []
    lines.append(f'    (symbol "{symbol_name}_{unit_num}_1"')

    pin_spacing = 2.54

    # Data pins on LEFT - DQ, DQS, DM by byte lane
    data_pins = []
    for byte_lane in range(4):
        for bit in range(8):
            name = f'DDR_DQ[{byte_lane * 8 + bit}]'
            if name in DDR_PINS:
                data_pins.append((name, DDR_PINS[name], 'bidirectional'))
        dqs_name = f'DDR_DQS[{byte_lane}]'
        dqs_n_name = f'DDR_DQS_N[{byte_lane}]'
        dm_name = f'DDR_DM[{byte_lane}]'
        if dqs_name in DDR_PINS:
            data_pins.append((dqs_name, DDR_PINS[dqs_name], 'bidirectional'))
        if dqs_n_name in DDR_PINS:
            data_pins.append((dqs_n_name, DDR_PINS[dqs_n_name], 'bidirectional'))
        if dm_name in DDR_PINS:
            data_pins.append((dm_name, DDR_PINS[dm_name], 'bidirectional'))

    # Control pins on RIGHT with y_offset for gaps to align with MT53
    # Format: (name, ball, pin_type, y_offset)
    # Gaps at positions 2,3 (unused CS_N[2,3]) and 6 (unused CKE[1])
    ctrl_pins = [
        ('DDR_CS_N[0]', DDR_PINS.get('DDR_CS_N[0]', ''), 'output', 0),
        ('DDR_CS_N[1]', DDR_PINS.get('DDR_CS_N[1]', ''), 'output', 1),
        ('DDR_CS_N[2]', DDR_PINS.get('DDR_CS_N[2]', ''), 'output', 2),
        ('DDR_CS_N[3]', DDR_PINS.get('DDR_CS_N[3]', ''), 'output', 3),
        ('DDR_CKE[0]', DDR_PINS.get('DDR_CKE[0]', ''), 'output', 4),
        # Gap at 5 for CKE0_B on MT53 (connects to same DDR_CKE[0])
        ('DDR_CKE[1]', DDR_PINS.get('DDR_CKE[1]', ''), 'output', 6),
        ('DDR_CK', DDR_PINS.get('DDR_CK', ''), 'output', 7),
        # Gap at 8 for CK_t_B on MT53 (connects to same DDR_CK)
        ('DDR_CK_N', DDR_PINS.get('DDR_CK_N', ''), 'output', 9),
        # Gap at 10 for CK_c_B on MT53 (connects to same DDR_CK_N)
        ('DDR_RST_N', DDR_PINS.get('DDR_RST_N', ''), 'output', 11),
        ('DDR_A[0]', DDR_PINS.get('DDR_A[0]', ''), 'output', 12),
        # Gap at 13 for CA0_B on MT53 (connects to same DDR_A[0])
        ('DDR_A[1]', DDR_PINS.get('DDR_A[1]', ''), 'output', 14),
        # Gap at 15 for CA1_B on MT53
        ('DDR_A[2]', DDR_PINS.get('DDR_A[2]', ''), 'output', 16),
        # Gap at 17 for CA2_B on MT53
        ('DDR_A[3]', DDR_PINS.get('DDR_A[3]', ''), 'output', 18),
        # Gap at 19 for CA3_B on MT53
        ('DDR_A[4]', DDR_PINS.get('DDR_A[4]', ''), 'output', 20),
        # Gap at 21 for CA4_B on MT53
        ('DDR_A[5]', DDR_PINS.get('DDR_A[5]', ''), 'output', 22),
        # Gap at 23 for CA5_B on MT53
        ('DDR_CAL', DDR_PINS.get('DDR_CAL', ''), 'output', 24),
        # Gaps at 25,26 for ODT_CA_A, ODT_CA_B on MT53
    ]

    # Calculate box dimensions - use the larger of data or control pin heights
    num_data = len(data_pins)
    num_ctrl_rows = 27  # Total rows including gaps

    # Use whichever side needs more space
    num_rows = max(num_data, num_ctrl_rows)
    box_height = num_rows * pin_spacing + 7.62
    box_width = 30.48
    y_start = box_height / 2

    # Draw rectangle
    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    # Unit name
    lines.append('      (text "DDR4 Interface"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Data pins on LEFT
    y_pos = y_start - 5.08
    for name, ball, pin_type in data_pins:
        if ball:  # Only add pin if it exists
            lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
            lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
            lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
            lines.append('      )')
        y_pos -= pin_spacing

    # Control pins on RIGHT with explicit y_offset for gaps
    y_top = y_start - 5.08
    for name, ball, pin_type, y_offset in ctrl_pins:
        if ball:  # Only add pin if it exists
            y_pos = y_top - (y_offset * pin_spacing)
            lines.append(f'      (pin {pin_type} line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
            lines.append(f'        (name "{name}" (effects (font (size 1.016 1.016))))')
            lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
            lines.append('      )')

    lines.append('    )')
    return '\n'.join(lines)


def generate_symbol_header(name):
    return f'''(kicad_symbol_lib
  (version 20231120)
  (generator "kicad_symbol_editor")
  (generator_version "8.0")
  (symbol "{name}"
    (pin_names (offset 1.016))
    (exclude_from_sim no)
    (in_bom yes)
    (on_board yes)
'''


def generate_property(name, value, x, y, hide=True):
    hide_str = "hide" if hide else ""
    return f'''    (property "{name}" "{value}"
      (at {x} {y} 0)
      (effects (font (size 1.27 1.27)) {hide_str})
    )
'''


def get_pin_type(mode):
    if mode == 'input':
        return 'input'
    elif mode == 'output' or mode == 'clkout':
        return 'output'
    elif mode == 'inout':
        return 'bidirectional'
    elif mode == 'rx':
        return 'input'
    elif mode == 'tx':
        return 'output'
    else:
        return 'bidirectional'


def generate_unit(unit_num, symbol_name, unit_name, sorted_pins, pin_side='left', default_pin_type='bidirectional'):
    """Generate a symbol unit with pins"""
    lines = []
    lines.append(f'    (symbol "{symbol_name}_{unit_num}_1"')

    num_pins = len(sorted_pins)
    pin_spacing = 2.54
    box_height = max(num_pins * pin_spacing + 7.62, 25.4)
    box_width = 30.48

    y_start = box_height / 2

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append(f'      (text "{unit_name}"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start - 5.08
    for item in sorted_pins:
        pin_name = item[0]
        second = item[1]

        # Check if second element is a tuple (ball, signal, mode) or just a string (ball)
        if isinstance(second, tuple):
            ball, signal, mode = second
            display_name = signal
            pin_type = get_pin_type(mode) if mode != 'unassigned' else default_pin_type
        else:
            # Simple (name, ball) format for DDR
            ball = second
            display_name = pin_name
            if 'DQ' in pin_name or 'DM' in pin_name:
                pin_type = 'bidirectional'
            else:
                pin_type = 'output'

        if pin_side == 'left':
            lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        else:
            lines.append(f'      (pin {pin_type} line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')

        lines.append(f'        (name "{display_name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')
    return '\n'.join(lines)


def generate_config_unit(unit_num, symbol_name):
    """Generate JTAG/Config unit"""
    lines = []
    lines.append(f'    (symbol "{symbol_name}_{unit_num}_1"')

    config_pins = {
        'TDO': PINOUT['TDO'],
        'TMS': PINOUT['TMS'],
        'TCK': PINOUT['TCK'],
        'TDI': PINOUT['TDI'],
        'CRESET_N': PINOUT['CRESET_N'],
        'CDONE': PINOUT['CDONE'],
    }
    for name, ball in PINOUT.items():
        if name.startswith('REF_RES'):
            config_pins[name] = ball

    num_pins = len(config_pins)
    pin_spacing = 2.54
    box_height = max(num_pins * pin_spacing + 7.62, 25.4)
    box_width = 20.32
    y_start = box_height / 2

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "CONFIG/JTAG"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start - 5.08
    for pin_name, ball in sorted(config_pins.items()):
        if isinstance(ball, list):
            ball = ball[0]

        if pin_name == 'TDO':
            pin_type = 'output'
        elif pin_name in ['TDI', 'TMS', 'TCK', 'CRESET_N']:
            pin_type = 'input'
        elif pin_name == 'CDONE':
            pin_type = 'output'
        else:
            pin_type = 'passive'

        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')
    return '\n'.join(lines)


def generate_vcc_unit(unit_num, symbol_name):
    """Generate VCC power unit"""
    lines = []
    lines.append(f'    (symbol "{symbol_name}_{unit_num}_1"')

    vcc_pins = []
    for pwr_name, balls in PINOUT.items():
        if isinstance(balls, list):
            if pwr_name.startswith('VCC') or pwr_name.startswith('VDD') or pwr_name == 'VQPS':
                for ball in balls:
                    vcc_pins.append((pwr_name, ball))

    vcc_pins.sort(key=lambda x: (x[0], x[1]))

    num_pins = len(vcc_pins)
    pin_spacing = 2.54
    box_height = max(num_pins * pin_spacing + 7.62, 25.4)
    box_width = 25.4
    y_start = box_height / 2

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "VCC Power"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start - 5.08
    for pin_name, ball in vcc_pins:
        lines.append(f'      (pin power_in line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')
    return '\n'.join(lines)


def generate_gnd_unit(unit_num, symbol_name):
    """Generate GND power unit"""
    lines = []
    lines.append(f'    (symbol "{symbol_name}_{unit_num}_1"')

    gnd_pins = [('GND', ball) for ball in PINOUT.get('GND', [])]
    gnd_pins.sort(key=lambda x: x[1])

    num_pins = len(gnd_pins)
    pin_spacing = 2.54
    box_height = max(num_pins * pin_spacing + 7.62, 25.4)
    box_width = 20.32
    y_start = box_height / 2

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    lines.append('      (text "GND"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    y_pos = y_start - 5.08
    for pin_name, ball in gnd_pins:
        lines.append(f'      (pin power_in line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= pin_spacing

    lines.append('    )')
    return '\n'.join(lines)


def main():
    script_dir = Path(__file__).parent
    peri_xml_path = script_dir.parent / 'bsp' / 'TI375C529' / 'tools_core.peri.xml'

    print(f"Parsing peri.xml from: {peri_xml_path}")

    if not peri_xml_path.exists():
        print(f"Error: peri.xml not found at {peri_xml_path}")
        return

    gpio_map, lvds_map = parse_peri_xml(peri_xml_path)

    print(f"Found {len(gpio_map)} GPIO mappings")
    print(f"Found {len(lvds_map)} LVDS mappings")

    # Categorize pins by peripheral
    ft601_pins, adc12_data_pins, adc34_data_pins, adc_clk_pins, misc_io_pins, unassigned_pins = \
        categorize_pins(gpio_map, lvds_map)

    print(f"\nPin categorization:")
    print(f"  FT601 interface: {len(ft601_pins)} pins")
    print(f"  ADC 1/2 LVDS data: {len(adc12_data_pins)} pins")
    print(f"  ADC 3/4 LVDS data: {len(adc34_data_pins)} pins")
    print(f"  ADC LVDS clocks: {len(adc_clk_pins)} pins")
    print(f"  Misc I/O: {len(misc_io_pins)} pins")
    print(f"  Unassigned: {len(unassigned_pins)} pins")

    symbol_name = "Ti90G529_Functions"
    output_path = script_dir / 'Ti90G529_Functions.kicad_sym'

    with open(output_path, 'w') as f:
        f.write(generate_symbol_header(symbol_name))

        f.write(generate_property("Reference", "U", 0, 1.27, False))
        f.write(generate_property("Value", symbol_name, 0, -1.27, False))
        f.write(generate_property("Footprint", "haasoscope_pro_adc_fpga_board:BGA-529_23x23_19.0x19.0mm", 0, -3.81))
        f.write(generate_property("Datasheet", "", 0, -6.35))
        f.write(generate_property("Description", "Efinix Ti90G529 FPGA organized by peripheral interface", 0, -8.89))
        f.write(generate_property("ki_locked", "", 0, 0))
        f.write(generate_property("ki_keywords", "FPGA Efinix Ti90 G529", 0, 0))

        unit_num = 1

        # Unit 1: FT601 Interface (pins on LEFT)
        sorted_ft601 = sort_ft601_pins(ft601_pins)
        f.write(generate_unit(unit_num, symbol_name, "FT601 Interface", sorted_ft601, 'left'))
        print(f"  Unit {unit_num}: FT601 Interface ({len(sorted_ft601)} pins)")
        unit_num += 1

        # Unit 2: ADC 1/2 LVDS Data (pins on RIGHT)
        sorted_adc12 = sort_lvds_pins(adc12_data_pins)
        f.write(generate_unit(unit_num, symbol_name, "ADC 1/2 LVDS Data", sorted_adc12, 'right', 'input'))
        print(f"  Unit {unit_num}: ADC 1/2 LVDS Data ({len(sorted_adc12)} pins)")
        unit_num += 1

        # Unit 3: ADC 3/4 LVDS Data (pins on RIGHT)
        sorted_adc34 = sort_lvds_pins(adc34_data_pins)
        f.write(generate_unit(unit_num, symbol_name, "ADC 3/4 LVDS Data", sorted_adc34, 'right', 'input'))
        print(f"  Unit {unit_num}: ADC 3/4 LVDS Data ({len(sorted_adc34)} pins)")
        unit_num += 1

        # Unit 4: ADC LVDS Clocks (pins on RIGHT - connects to ADC CLK outputs)
        sorted_adc_clk = sort_adc_clk_pins(adc_clk_pins)
        f.write(generate_unit(unit_num, symbol_name, "ADC LVDS Clocks", sorted_adc_clk, 'right', 'input'))
        print(f"  Unit {unit_num}: ADC LVDS Clocks ({len(sorted_adc_clk)} pins)")
        unit_num += 1

        # Unit 5: DDR4 Interface (data on LEFT, control on RIGHT with gaps)
        f.write(generate_ddr_unit(unit_num, symbol_name))
        print(f"  Unit {unit_num}: DDR4 Interface (data LEFT, control RIGHT with gaps)")
        unit_num += 1

        # Unit 6: Misc I/O
        sorted_misc = sorted(misc_io_pins.items(), key=lambda x: x[1][1])
        f.write(generate_unit(unit_num, symbol_name, "Misc I/O", sorted_misc, 'left'))
        print(f"  Unit {unit_num}: Misc I/O ({len(sorted_misc)} pins)")
        unit_num += 1

        # Unit 7: Config/JTAG
        f.write(generate_config_unit(unit_num, symbol_name))
        print(f"  Unit {unit_num}: CONFIG/JTAG")
        unit_num += 1

        # Unit 8: VCC Power
        f.write(generate_vcc_unit(unit_num, symbol_name))
        print(f"  Unit {unit_num}: VCC Power")
        unit_num += 1

        # Unit 9: GND
        f.write(generate_gnd_unit(unit_num, symbol_name))
        print(f"  Unit {unit_num}: GND")
        unit_num += 1

        # Unit 10: Unassigned GPIO
        if unassigned_pins:
            sorted_unassigned = sorted(unassigned_pins.items(), key=lambda x: x[0])
            f.write(generate_unit(unit_num, symbol_name, "Unassigned GPIO", sorted_unassigned, 'left'))
            print(f"  Unit {unit_num}: Unassigned GPIO ({len(sorted_unassigned)} pins)")
            unit_num += 1

        f.write('  )\n')
        f.write(')\n')

    print(f"\nGenerated symbol: {output_path}")
    print(f"Total units: {unit_num - 1}")


if __name__ == "__main__":
    main()
