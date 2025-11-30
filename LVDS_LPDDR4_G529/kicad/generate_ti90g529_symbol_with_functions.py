#!/usr/bin/env python3
"""
Generate KiCad schematic symbol for Efinix Ti90G529 FPGA
with pin function names from peri.xml

This script creates a symbol where each pin shows:
- Pin name: The function/signal name (e.g., ftdi_data[0])
- Pin number: The ball number (e.g., F9)
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

# Import base pinout data from the original script
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

    # Define namespace
    ns = {'efxpt': 'http://www.efinixinc.com/peri_design_db'}

    # Find all comp_gpio elements
    for gpio in root.findall('.//efxpt:comp_gpio', ns):
        signal_name = gpio.get('name')
        gpio_def = gpio.get('gpio_def')
        mode = gpio.get('mode')
        if signal_name and gpio_def:
            gpio_to_signal[gpio_def] = {
                'signal': signal_name,
                'mode': mode
            }

    # Find all LVDS elements
    for lvds in root.findall('.//efxpt:lvds', ns):
        signal_name = lvds.get('name')
        lvds_def = lvds.get('lvds_def')
        ops_type = lvds.get('ops_type')
        if signal_name and lvds_def:
            lvds_to_signal[lvds_def] = {
                'signal': signal_name,
                'mode': ops_type
            }

    return gpio_to_signal, lvds_to_signal


def normalize_gpio_name(pin_name):
    """Convert pin name to gpio_def format for matching"""
    # Examples:
    # GPIOL_00_PLLIN1 -> GPIOL_00
    # GPIOR_P_00_PLLIN0 -> GPIOR_P_00
    # GPIOT_P_00_PLLIN0 -> GPIOT_P_00

    # Remove suffixes like _PLLIN1, _CLKxx, _EXTFB, etc
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
    # Try direct match first
    normalized = normalize_gpio_name(pin_name)

    if normalized in gpio_map:
        return gpio_map[normalized]['signal'], gpio_map[normalized]['mode']

    # Try with the full pin name
    if pin_name in gpio_map:
        return gpio_map[pin_name]['signal'], gpio_map[pin_name]['mode']

    # Try LVDS mapping (for differential pairs)
    # Convert GPIOT_P_xx to GPIOT_PN_xx format for positive pins
    lvds_name = re.sub(r'GPIO([TB])_P_(\d+)', r'GPIO\1_PN_\2', normalized)
    if lvds_name in lvds_map:
        return lvds_map[lvds_name]['signal'], lvds_map[lvds_name]['mode']

    # Convert GPIOT_N_xx to GPIOT_PN_xx format for negative pins
    # and append _N to signal name to indicate negative side of differential pair
    lvds_name_n = re.sub(r'GPIO([TB])_N_(\d+)', r'GPIO\1_PN_\2', normalized)
    if lvds_name_n in lvds_map:
        signal = lvds_map[lvds_name_n]['signal'] + '_N'
        return signal, lvds_map[lvds_name_n]['mode']

    # Return original pin name if no mapping found
    return None, None


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


def generate_property(name, value, id_num, x, y, hide=True):
    hide_str = "hide" if hide else ""
    return f'''    (property "{name}" "{value}"
      (at {x} {y} 0)
      (effects (font (size 1.27 1.27)) {hide_str})
    )
'''


def get_pin_type(mode):
    """Convert peri.xml mode to KiCad pin type"""
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


def generate_unit(unit_num, symbol_name, unit_name, pins, gpio_map, lvds_map, pin_side='left'):
    """Generate a unit with pins, showing function names"""
    lines = []
    lines.append(f'    (symbol "{symbol_name}_{unit_num}_1"')

    # Calculate box dimensions based on pin count
    num_pins = len(pins)
    pin_spacing = 2.54
    box_height = max(num_pins * pin_spacing + 5.08, 20.32)
    box_width = 25.4  # Wider to accommodate longer names

    y_start = box_height / 2

    # Draw rectangle
    if pin_side == 'left':
        lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    else:
        lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    # Add unit label
    lines.append(f'      (text "{unit_name}"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Add pins
    y_pos = y_start - 5.08

    # Sort pins by name for consistent ordering
    sorted_pins = sorted(pins.items(), key=lambda x: x[0])

    for pin_name, ball in sorted_pins:
        # Get signal name from peri.xml
        signal, mode = get_signal_for_pin(pin_name, ball, gpio_map, lvds_map)

        # Create display name: show signal if available, otherwise show GPIO name
        if signal:
            display_name = signal
            pin_type = get_pin_type(mode)
        else:
            display_name = pin_name
            pin_type = 'bidirectional'

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


def generate_power_unit(unit_num, symbol_name, gpio_map):
    """Generate power unit with grouped power pins"""
    lines = []
    lines.append(f'    (symbol "{symbol_name}_{unit_num}_1"')

    # Collect all power pins
    vcc_pins = []
    gnd_pins = []

    for pwr_name, balls in PINOUT.items():
        if isinstance(balls, list):
            if pwr_name == 'GND':
                for ball in balls:
                    gnd_pins.append((pwr_name, ball))
            elif pwr_name.startswith('VCC') or pwr_name.startswith('VDD') or pwr_name == 'VQPS':
                for ball in balls:
                    vcc_pins.append((pwr_name, ball))

    # Sort pins
    vcc_pins.sort(key=lambda x: (x[0], x[1]))
    gnd_pins.sort(key=lambda x: x[1])

    max_pins = max(len(vcc_pins), len(gnd_pins))
    height = max_pins * 2.54 + 5.08
    y_start = height / 2
    box_width = 30.48

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    # Add unit label
    lines.append('      (text "POWER"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Add VCC pins on left
    y_pos = y_start - 5.08
    for pin_name, ball in vcc_pins:
        lines.append(f'      (pin power_in line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= 2.54

    # Add GND pins on right
    y_pos = y_start - 5.08
    for pin_name, ball in gnd_pins:
        lines.append(f'      (pin power_in line (at {box_width + 2.54:.2f} {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= 2.54

    lines.append('    )')
    return '\n'.join(lines)


def generate_ddr_unit(unit_num, symbol_name):
    """Generate DDR interface unit"""
    lines = []
    lines.append(f'    (symbol "{symbol_name}_{unit_num}_1"')

    num_pins = len(DDR_PINS)
    height = num_pins * 2.54 + 5.08
    y_start = height / 2
    box_width = 20.32

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    # Add unit label
    lines.append('      (text "DDR"')
    lines.append(f'        (at {box_width/2:.2f} {y_start - 1.27:.2f} 0)')
    lines.append('        (effects (font (size 1.524 1.524) bold))')
    lines.append('      )')

    # Sort DDR pins by type then number
    sorted_pins = sorted(DDR_PINS.items(), key=lambda x: x[0])

    y_pos = y_start - 5.08
    for pin_name, ball in sorted_pins:
        # Determine pin type based on name
        if 'DQ' in pin_name or 'DM' in pin_name:
            pin_type = 'bidirectional'
        elif pin_name.startswith('DDR_A') or 'CK' in pin_name or 'CS' in pin_name or 'RST' in pin_name:
            pin_type = 'output'
        else:
            pin_type = 'bidirectional'

        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= 2.54

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
    # Add REF_RES pins
    for name, ball in PINOUT.items():
        if name.startswith('REF_RES'):
            config_pins[name] = ball

    num_pins = len(config_pins)
    height = num_pins * 2.54 + 5.08
    y_start = height / 2
    box_width = 20.32

    lines.append(f'      (rectangle (start 0 {y_start:.2f}) (end {box_width:.2f} {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    # Add unit label
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
        elif pin_name in ['TDI', 'TMS', 'TCK']:
            pin_type = 'input'
        elif pin_name == 'CRESET_N':
            pin_type = 'input'
        elif pin_name == 'CDONE':
            pin_type = 'output'
        else:
            pin_type = 'passive'

        lines.append(f'      (pin {pin_type} line (at -2.54 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.016 1.016))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.016 1.016))))')
        lines.append('      )')
        y_pos -= 2.54

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

    # Show some examples
    print("\nExample GPIO mappings:")
    for gpio_def, info in list(gpio_map.items())[:10]:
        print(f"  {gpio_def} -> {info['signal']} ({info['mode']})")

    print("\nExample LVDS mappings:")
    for lvds_def, info in list(lvds_map.items())[:5]:
        print(f"  {lvds_def} -> {info['signal']} ({info['mode']})")

    # Generate symbol
    symbol_name = "Ti90G529_Functions"
    output_path = script_dir / 'Ti90G529_Functions.kicad_sym'

    with open(output_path, 'w') as f:
        f.write(generate_symbol_header(symbol_name))

        # Add properties
        f.write(generate_property("Reference", "U", 0, 0, 1.27, False))
        f.write(generate_property("Value", symbol_name, 1, 0, -1.27, False))
        f.write(generate_property("Footprint", "haasoscope_pro_adc_fpga_board:BGA-529_23x23_19.0x19.0mm", 2, 0, -3.81))
        f.write(generate_property("Datasheet", "", 3, 0, -6.35))
        f.write(generate_property("Description", "Efinix Ti90G529 FPGA with function names from peri.xml", 4, 0, -8.89))
        f.write(generate_property("ki_locked", "", 5, 0, 0))
        f.write(generate_property("ki_keywords", "FPGA Efinix Ti90 G529", 6, 0, 0))

        unit_num = 1

        # Unit 1: GPIO Bank L
        f.write(generate_unit(unit_num, symbol_name, "BANK L (GPIO)", GPIOL_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 2: GPIO Bank R - TR/BR
        gpior_trbr = {**GPIOR_TR_PINS, **GPIOR_BR_PINS}
        f.write(generate_unit(unit_num, symbol_name, "BANK R (TR/BR)", gpior_trbr, gpio_map, lvds_map))
        unit_num += 1

        # Unit 3: GPIO Bank R - 3A
        f.write(generate_unit(unit_num, symbol_name, "BANK 3A", GPIOR_3A_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 4: GPIO Bank R - 3B
        f.write(generate_unit(unit_num, symbol_name, "BANK 3B", GPIOR_3B_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 5: GPIO Bank R - 3C
        f.write(generate_unit(unit_num, symbol_name, "BANK 3C", GPIOR_3C_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 6: GPIO Bank T - 2A
        f.write(generate_unit(unit_num, symbol_name, "BANK 2A (LVDS Top)", GPIOT_2A_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 7: GPIO Bank T - 2B
        f.write(generate_unit(unit_num, symbol_name, "BANK 2B (LVDS Top)", GPIOT_2B_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 8: GPIO Bank T - 2C
        f.write(generate_unit(unit_num, symbol_name, "BANK 2C (LVDS Top)", GPIOT_2C_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 9: GPIO Bank B - 4A
        f.write(generate_unit(unit_num, symbol_name, "BANK 4A (LVDS Bot)", GPIOB_4A_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 10: GPIO Bank B - 4B
        f.write(generate_unit(unit_num, symbol_name, "BANK 4B (LVDS Bot)", GPIOB_4B_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 11: GPIO Bank B - 4C
        f.write(generate_unit(unit_num, symbol_name, "BANK 4C (LVDS Bot)", GPIOB_4C_PINS, gpio_map, lvds_map))
        unit_num += 1

        # Unit 12: DDR Interface
        f.write(generate_ddr_unit(unit_num, symbol_name))
        unit_num += 1

        # Unit 13: Config/JTAG
        f.write(generate_config_unit(unit_num, symbol_name))
        unit_num += 1

        # Unit 14: Power
        f.write(generate_power_unit(unit_num, symbol_name, gpio_map))

        # Close symbol
        f.write('  )\n')
        f.write(')\n')

    print(f"\nGenerated symbol: {output_path}")
    print(f"Total units: {unit_num}")


if __name__ == "__main__":
    main()
