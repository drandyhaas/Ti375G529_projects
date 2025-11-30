#!/usr/bin/env python3
"""
Generate KiCad schematic symbol for Efinix Ti90G529 FPGA
Based on Ti90G529_pinout.pdf

Pin categories for multi-unit symbol:
- Unit A: Power (VCC, VCCIO, VCCA, VCCAUX, VDD_PHY, VDDQ_PHY, VDDQX_PHY, GND)
- Unit B: DDR interface
- Unit C: JTAG, Config, Clock (TDO, TMS, TCK, TDI, CRESET_N, CDONE, etc.)
- Unit D: GPIO Bank L (GPIOL)
- Unit E: GPIO Bank R (GPIOR) - part 1
- Unit F: GPIO Bank R (GPIOR) - part 2
- Unit G: GPIO Bank T (GPIOT) - BANK2A
- Unit H: GPIO Bank T (GPIOT) - BANK2B
- Unit I: GPIO Bank T (GPIOT) - BANK2C
- Unit J: GPIO Bank B (GPIOB) - BANK4A
- Unit K: GPIO Bank B (GPIOB) - BANK4B
- Unit L: GPIO Bank B (GPIOB) - BANK4C
"""

# Pinout data from Ti90G529_pinout.pdf
# Format: (pin_name, ball, bank/type)

PINOUT = {
    # ===== POWER PINS =====
    'VCC': ['J9', 'J11', 'J13', 'J15', 'K10', 'K12', 'K14', 'L9', 'L11', 'L13', 'L15',
            'M10', 'M12', 'M14', 'N9', 'N11', 'N13', 'N15', 'T8', 'T16', 'U8', 'U16'],
    'VCCA_BL': ['R7'],
    'VCCA_BR': ['J8'],
    'VCCA_TL': ['R17'],
    'VCCA_TR': ['H16'],
    'VCCAUX': ['K8', 'J16', 'R8', 'R16'],
    'VCCIO2A': ['P17', 'P22', 'R20'],
    'VCCIO2B': ['H21', 'K22', 'M17'],
    'VCCIO2C': ['F18', 'H17', 'K16'],
    'VCCIO33_BL': ['U4', 'V2'],
    'VCCIO33_BR': ['F8'],
    'VCCIO33_TL': ['U22', 'U19'],
    'VCCIO33_TR': ['F16'],
    'VCCIO3A': ['B17', 'D16'],
    'VCCIO3B': ['D13', 'F12', 'F14', 'G10'],
    'VCCIO3C': ['B8', 'D7'],
    'VCCIO4A': ['D4', 'F6', 'G4'],
    'VCCIO4B': ['K7', 'M5', 'M8'],
    'VCCIO4C': ['N7', 'P2', 'P4'],
    'VDD_PHY': ['P9', 'P11', 'P12', 'P14', 'P15', 'R10', 'R11', 'R13', 'R14',
                'T11', 'T13', 'T14', 'T15', 'T9', 'T10'],
    'VDDPLL_MCB_TOP_PHY': ['U12'],
    'VDDQ_CK_PHY': ['T12'],
    'VDDQ_PHY': ['AB6', 'AB10', 'AB14', 'AB18', 'W7', 'W12', 'W18', 'Y1', 'Y3',
                 'Y13', 'Y19', 'Y21', 'Y23', 'Y5', 'Y11'],
    'VDDQX_PHY': ['U9', 'U10', 'U13', 'V11', 'V12', 'V14', 'U15', 'V9'],
    'VQPS': ['U7'],

    # Ground
    'GND': ['A1', 'A19', 'A23', 'AA19', 'AB2', 'AC4', 'AB5', 'AB7', 'AC9', 'AB11',
            'AB13', 'AC15', 'AB17', 'AC20', 'AB22', 'AC1', 'AC23', 'B3', 'B6', 'B11',
            'B14', 'B21', 'D1', 'D10', 'D19', 'D22', 'F2', 'F20', 'F22', 'H1', 'H4',
            'H7', 'H23', 'J10', 'J12', 'J14', 'J19', 'K2', 'K5', 'K9', 'K11', 'K13',
            'K15', 'L10', 'L12', 'L14', 'M1', 'M3', 'M9', 'M11', 'M13', 'M15', 'M20',
            'M23', 'N8', 'N10', 'P10', 'P13', 'P16', 'R9', 'R12', 'R15', 'T1', 'T7',
            'T17', 'T21', 'T23', 'U11', 'U14', 'V7', 'V10', 'V13', 'V15', 'V17', 'Y2',
            'Y4', 'Y6', 'Y8', 'Y10', 'N12', 'N14', 'Y12', 'Y14', 'Y16', 'Y18', 'Y20', 'Y22'],

    # ===== JTAG & CONFIG =====
    'TDO': ['T4'],
    'TMS': ['T5'],
    'TCK': ['U5'],
    'TDI': ['U6'],
    'CRESET_N': ['R6'],
    'CDONE': ['T6'],

    # Reference resistor pins
    'REF_RES_2A': ['T22'],
    'REF_RES_2B': ['K23'],
    'REF_RES_2C': ['A22'],
    'REF_RES_3A': ['E18'],
    'REF_RES_3B': ['C14'],
    'REF_RES_3C': ['D6'],
    'REF_RES_4A': ['B5'],
    'REF_RES_4B': ['J1'],
    'REF_RES_4C': ['T2'],

    # NC pins
    'NC': ['W10', 'W11', 'W13', 'W14', 'V16', 'W15'],
}

# GPIO Bank L (BL region - directly on left)
GPIOL_PINS = {
    'GPIOL_00_PLLIN1': 'V6',
    'GPIOL_01': 'W6',
    'GPIOL_02': 'U3',
    'GPIOL_03_CLK24': 'W5',
    'GPIOL_04_CLK25': 'U2',
    'GPIOL_05_CLK26': 'T3',
    'GPIOL_06_CLK27': 'V1',
    'GPIOL_07': 'U1',
    'GPIOL_08': 'W1',
    'GPIOL_09': 'V3',
    'GPIOL_10_PLLIN1': 'W3',
    'GPIOL_20_PLLIN1': 'W4',
    'GPIOL_21': 'V5',
    'GPIOL_22': 'V4',
    'GPIOL_23': 'W2',
    # TL region GPIOL
    'GPIOL_26_PLLIN1': 'V18',
    'GPIOL_27_CLK28': 'T19',
    'GPIOL_28_CLK29': 'T20',
    'GPIOL_29_CLK30': 'V19',
    'GPIOL_30_CLK31': 'T18',
    'GPIOL_31': 'U20',
    'GPIOL_32_PLLIN1': 'U18',
    'GPIOL_33': 'V23',
    'GPIOL_34': 'V21',
    'GPIOL_35': 'V22',
    'GPIOL_36_PLLIN1': 'U23',
    'GPIOL_37': 'U21',
    'GPIOL_38': 'V20',
    'GPIOL_39': 'U17',
    'GPIOL_40': 'W22',
    'GPIOL_41': 'W23',
    'GPIOL_42': 'W21',
    'GPIOL_43': 'W19',
    'GPIOL_44': 'W20',
}

# GPIO Bank R - TR region (directly on right)
GPIOR_TR_PINS = {
    'GPIOR_56': 'E17',
    'GPIOR_57': 'F15',
    'GPIOR_58': 'H15',
    'GPIOR_59': 'E16',
    'GPIOR_60': 'G16',
    'GPIOR_61': 'G15',
    'GPIOR_63': 'E15',
}

# GPIO Bank R - BR region
GPIOR_BR_PINS = {
    'GPIOR_65': 'G9',
    'GPIOR_66': 'G8',
    'GPIOR_68': 'F7',
    'GPIOR_69': 'H8',
    'GPIOR_70': 'G7',
    'GPIOR_71': 'H9',
    'GPIOR_72': 'F9',
}

# GPIO Bank R - BANK3A
GPIOR_3A_PINS = {
    'GPIOR_P_36': 'B15', 'GPIOR_N_36': 'A15',
    'GPIOR_P_37': 'C15', 'GPIOR_N_37': 'D15',
    'GPIOR_P_38': 'A16', 'GPIOR_N_38': 'B16',
    'GPIOR_P_39': 'A17', 'GPIOR_N_39': 'A18',
    'GPIOR_P_40': 'C16', 'GPIOR_N_40': 'C17',
    'GPIOR_P_41': 'B18', 'GPIOR_N_41': 'C18',
    'GPIOR_P_42': 'A21', 'GPIOR_N_42': 'A20',
    'GPIOR_P_43': 'D17', 'GPIOR_N_43': 'D18',
    'GPIOR_P_44_EXTFB': 'C19', 'GPIOR_N_44': 'B19',
    'GPIOR_P_45_PLLIN0': 'C20', 'GPIOR_N_45': 'B20',
}

# GPIO Bank R - BANK3B
GPIOR_3B_PINS = {
    'GPIOR_P_16_PLLIN1': 'E10', 'GPIOR_N_16': 'F10',
    'GPIOR_P_17': 'C10', 'GPIOR_N_17': 'B10',
    'GPIOR_P_18': 'H11', 'GPIOR_N_18': 'H10',
    'GPIOR_P_19': 'A10', 'GPIOR_N_19': 'A11',
    'GPIOR_P_20_CLK15_P': 'C11', 'GPIOR_N_20_CLK15_N': 'D11',
    'GPIOR_P_21_CLK14_P': 'G11', 'GPIOR_N_21_CLK14_N': 'F11',
    'GPIOR_P_22_CLK13_P': 'B12', 'GPIOR_N_22_CLK13_N': 'A12',
    'GPIOR_P_23_CLK12_P': 'E11', 'GPIOR_N_23_CLK12_N': 'E12',
    'GPIOR_P_24_CLK11_P': 'G12', 'GPIOR_N_24_CLK11_N': 'H12',
    'GPIOR_P_25_CLK10_P': 'D12', 'GPIOR_N_25_CLK10_N': 'C12',
    'GPIOR_P_26_CLK9_P': 'A13', 'GPIOR_N_26_CLK9_N': 'A14',
    'GPIOR_P_27_CLK8_P': 'H13', 'GPIOR_N_27_CLK8_N': 'G13',
    'GPIOR_P_28': 'B13', 'GPIOR_N_28': 'C13',
    'GPIOR_P_29': 'F13', 'GPIOR_N_29': 'E13',
    'GPIOR_P_30': 'G14', 'GPIOR_N_30': 'H14',
    'GPIOR_P_31_PLLIN1': 'E14', 'GPIOR_N_31': 'D14',
}

# GPIO Bank R - BANK3C
GPIOR_3C_PINS = {
    'GPIOR_P_00_PLLIN0': 'C5', 'GPIOR_N_00': 'C6',
    'GPIOR_P_01_EXTFB': 'A5', 'GPIOR_N_01': 'A6',
    'GPIOR_P_02': 'E6', 'GPIOR_N_02': 'E7',
    'GPIOR_P_03': 'B7', 'GPIOR_N_03': 'A7',
    'GPIOR_P_04': 'C7', 'GPIOR_N_04': 'C8',
    'GPIOR_P_05': 'E8', 'GPIOR_N_05': 'D8',
    'GPIOR_P_06': 'A8', 'GPIOR_N_06': 'A9',
    'GPIOR_P_07': 'C9', 'GPIOR_N_07': 'B9',
    'GPIOR_P_08': 'E9', 'GPIOR_N_08': 'D9',
}

# GPIO Bank T - BANK2A
GPIOT_2A_PINS = {
    'GPIOT_P_00_PLLIN0': 'R22', 'GPIOT_N_00': 'R21',
    'GPIOT_P_01_EXTFB': 'R23', 'GPIOT_N_01': 'P23',
    'GPIOT_P_02': 'R18', 'GPIOT_N_02': 'R19',
    'GPIOT_P_03': 'P21', 'GPIOT_N_03': 'P20',
    'GPIOT_P_04': 'N22', 'GPIOT_N_04': 'N23',
    'GPIOT_P_05': 'P18', 'GPIOT_N_05': 'P19',
    'GPIOT_P_06': 'L23', 'GPIOT_N_06': 'L22',
    'GPIOT_P_07': 'N20', 'GPIOT_N_07': 'N21',
    'GPIOT_P_08': 'N16', 'GPIOT_N_08': 'N17',
    'GPIOT_P_09': 'N19', 'GPIOT_N_09': 'N18',
    'GPIOT_P_10': 'M21', 'GPIOT_N_10': 'M22',
}

# GPIO Bank T - BANK2B
GPIOT_2B_PINS = {
    'GPIOT_P_11_PLLIN0': 'J22', 'GPIOT_N_11': 'J23',
    'GPIOT_P_12_EXTFB': 'G22', 'GPIOT_N_12': 'H22',
    'GPIOT_P_13_CLK16_P': 'M19', 'GPIOT_N_13_CLK16_N': 'M18',
    'GPIOT_P_14_CLK17_P': 'L21', 'GPIOT_N_14_CLK17_N': 'K21',
    'GPIOT_P_15_CLK18_P': 'F23', 'GPIOT_N_15_CLK18_N': 'G23',
    'GPIOT_P_16_CLK19_P': 'L16', 'GPIOT_N_16_CLK19_N': 'M16',
    'GPIOT_P_17_CLK20_P': 'E23', 'GPIOT_N_17_CLK20_N': 'E22',
    'GPIOT_P_18_CLK21_P': 'J20', 'GPIOT_N_18_CLK21_N': 'J21',
    'GPIOT_P_19_CLK22_P': 'L18', 'GPIOT_N_19_CLK22_N': 'L17',
    'GPIOT_P_20_CLK23_P': 'G21', 'GPIOT_N_20_CLK23_N': 'F21',
    'GPIOT_P_21': 'K20', 'GPIOT_N_21': 'K19',
    'GPIOT_P_22': 'L20', 'GPIOT_N_22': 'L19',
}

# GPIO Bank T - BANK2C
GPIOT_2C_PINS = {
    'GPIOT_P_23_PLLIN0': 'D23', 'GPIOT_N_23': 'C23',
    'GPIOT_P_24_EXTFB': 'B23', 'GPIOT_N_24': 'B22',
    'GPIOT_P_25': 'K18', 'GPIOT_N_25': 'K17',
    'GPIOT_P_26': 'H20', 'GPIOT_N_26': 'G20',
    'GPIOT_P_27': 'E21', 'GPIOT_N_27': 'E20',
    'GPIOT_P_28': 'J18', 'GPIOT_N_28': 'J17',
    'GPIOT_P_29': 'F19', 'GPIOT_N_29': 'E19',
    'GPIOT_P_30': 'D21', 'GPIOT_N_30': 'D20',
    'GPIOT_P_31': 'H19', 'GPIOT_N_31': 'H18',
    'GPIOT_P_32': 'C21', 'GPIOT_N_32': 'C22',
    'GPIOT_P_33': 'F17', 'GPIOT_N_33': 'G17',
    'GPIOT_P_34': 'G19', 'GPIOT_N_34': 'G18',
}

# GPIO Bank B - BANK4A
GPIOB_4A_PINS = {
    'GPIOB_P_23_PLLIN0': 'E2', 'GPIOB_N_23_CDI12': 'E3',
    'GPIOB_P_24_EXTFB': 'B1', 'GPIOB_N_24_CDI13': 'C1',
    'GPIOB_P_25_CDI15': 'E4', 'GPIOB_N_25_CDI14': 'F4',
    'GPIOB_P_26_CDI16': 'D2', 'GPIOB_N_26_CDI17': 'D3',
    'GPIOB_P_27_CDI19': 'C3', 'GPIOB_N_27_CDI18': 'C2',
    'GPIOB_P_28_CDI21': 'J7', 'GPIOB_N_28_CDI20': 'J6',
    'GPIOB_P_29_CDI22': 'B2', 'GPIOB_N_29_CDI23': 'A2',
    'GPIOB_P_30_CDI25': 'D5', 'GPIOB_N_30_CDI24': 'E5',
    'GPIOB_P_31_CDI27': 'G6', 'GPIOB_N_31_CDI26': 'H6',
    'GPIOB_P_32_CDI28': 'C4', 'GPIOB_N_32_CDI29': 'B4',
    'GPIOB_P_33_CDI31': 'A4', 'GPIOB_N_33_CDI30': 'A3',
    'GPIOB_P_34': 'F5', 'GPIOB_N_34': 'G5',
}

# GPIO Bank B - BANK4B
GPIOB_4B_PINS = {
    'GPIOB_P_11_PLLIN0': 'G1', 'GPIOB_N_11': 'G2',
    'GPIOB_P_12_EXTFB': 'E1', 'GPIOB_N_12_SSU_N': 'F1',
    'GPIOB_P_13_CBSEL0_CLK0_P': 'M6', 'GPIOB_N_13_CBSEL1_CLK0_N': 'M7',
    'GPIOB_P_14_NSTATUS_CLK1_P': 'L3', 'GPIOB_N_14_TEST_N_CLK1_N': 'K3',
    'GPIOB_P_15_CLK2_P': 'J3', 'GPIOB_N_15_CLK2_N': 'J2',
    'GPIOB_P_16_EXTSPICLK_CLK3_P': 'L4', 'GPIOB_N_16_CLK3_N': 'L5',
    'GPIOB_P_17_CLK4_P': 'H2', 'GPIOB_N_17_CLK4_N': 'H3',
    'GPIOB_P_18_CLK5_P': 'F3', 'GPIOB_N_18_CLK5_N': 'G3',
    'GPIOB_P_19_CDI5_CLK6_P': 'L7', 'GPIOB_N_19_CDI4_CLK6_N': 'L8',
    'GPIOB_P_20_CDI6_CLK7_P': 'K4', 'GPIOB_N_20_CDI7_CLK7_N': 'J4',
    'GPIOB_P_21_CDI9': 'H5', 'GPIOB_N_21_CDI8': 'J5',
    'GPIOB_P_22_CDI11': 'K6', 'GPIOB_N_22_CDI10': 'L6',
}

# GPIO Bank B - BANK4C
GPIOB_4C_PINS = {
    'GPIOB_P_00_PLLIN0': 'R4', 'GPIOB_N_00': 'R5',
    'GPIOB_P_01_EXTFB': 'R2', 'GPIOB_N_01_CCK': 'R3',
    'GPIOB_P_02_CSI': 'P8', 'GPIOB_N_02_CSO': 'P7',
    'GPIOB_P_03': 'P1', 'GPIOB_N_03': 'R1',
    'GPIOB_P_04': 'P5', 'GPIOB_N_04': 'P6',
    'GPIOB_P_05': 'N5', 'GPIOB_N_05': 'N6',
    'GPIOB_P_06': 'N3', 'GPIOB_N_06': 'P3',
    'GPIOB_P_07': 'N1', 'GPIOB_N_07': 'N2',
    'GPIOB_P_08_SSL_N': 'N4', 'GPIOB_N_08': 'M4',
    'GPIOB_P_09_CDI0': 'L2', 'GPIOB_N_09_CDI1': 'M2',
    'GPIOB_P_10_CDI3': 'K1', 'GPIOB_N_10_CDI2': 'L1',
}

# DDR Pins
DDR_PINS = {
    'DDR_DQ[0]': 'AA22', 'DDR_DQ[1]': 'AA23', 'DDR_DQ[2]': 'AA20', 'DDR_DQ[3]': 'AC19',
    'DDR_DQ[4]': 'AB20', 'DDR_DQ[5]': 'AB19', 'DDR_DQ[6]': 'AC22', 'DDR_DQ[7]': 'AA21',
    'DDR_DQ[8]': 'AA17', 'DDR_DQ[9]': 'AC17', 'DDR_DQ[10]': 'AA18', 'DDR_DQ[11]': 'AC18',
    'DDR_DQ[12]': 'AB15', 'DDR_DQ[13]': 'AA15', 'DDR_DQ[14]': 'AC14', 'DDR_DQ[15]': 'AA14',
    'DDR_DQ[16]': 'AB9', 'DDR_DQ[17]': 'AA9', 'DDR_DQ[18]': 'AC10', 'DDR_DQ[19]': 'AA10',
    'DDR_DQ[20]': 'AC7', 'DDR_DQ[21]': 'AA7', 'DDR_DQ[22]': 'AC6', 'DDR_DQ[23]': 'AA6',
    'DDR_DQ[24]': 'AB4', 'DDR_DQ[25]': 'AA4', 'DDR_DQ[26]': 'AC5', 'DDR_DQ[27]': 'AA5',
    'DDR_DQ[28]': 'AB1', 'DDR_DQ[29]': 'AA1', 'DDR_DQ[30]': 'AA2', 'DDR_DQ[31]': 'AC2',
    'DDR_DQS[0]': 'AB21', 'DDR_DQS_N[0]': 'AC21',
    'DDR_DQS[1]': 'AB16', 'DDR_DQS_N[1]': 'AC16',
    'DDR_DQS[2]': 'AB8', 'DDR_DQS_N[2]': 'AC8',
    'DDR_DQS[3]': 'AB3', 'DDR_DQS_N[3]': 'AC3',
    'DDR_DM[0]': 'AB23', 'DDR_DM[1]': 'AA16', 'DDR_DM[2]': 'AA8', 'DDR_DM[3]': 'AA3',
    'DDR_A[0]': 'AC11', 'DDR_A[1]': 'AA11', 'DDR_A[2]': 'Y9', 'DDR_A[3]': 'W9',
    'DDR_A[4]': 'W8', 'DDR_A[5]': 'Y7',
    'DDR_CAL': 'V8',
    'DDR_CK': 'AC12', 'DDR_CK_N': 'AB12',
    'DDR_CKE[0]': 'AA12', 'DDR_CKE[1]': 'AA13',
    'DDR_RST_N': 'AC13',
    'DDR_CS_N[0]': 'Y15', 'DDR_CS_N[1]': 'W16', 'DDR_CS_N[2]': 'W17', 'DDR_CS_N[3]': 'Y17',
}


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

def generate_unit(unit_num, name, pins, x_offset, y_start, pin_type="bidirectional"):
    """Generate a unit with pins"""
    lines = []
    lines.append(f'    (symbol "{name}_{unit_num}_1"')

    # Draw rectangle
    height = len(pins) * 2.54 + 5.08
    lines.append(f'      (rectangle (start -10.16 {y_start}) (end 10.16 {y_start - height})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    # Add pins
    y_pos = y_start - 2.54
    for pin_name, ball in sorted(pins.items()):
        lines.append(f'      (pin {pin_type} line (at -12.7 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.27 1.27))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.27 1.27))))')
        lines.append('      )')
        y_pos -= 2.54

    lines.append('    )')
    return '\n'.join(lines)


def generate_power_unit(unit_num, name):
    """Generate power unit with grouped power pins"""
    lines = []
    lines.append(f'    (symbol "{name}_{unit_num}_1"')

    # Collect all power pins
    power_pins = []
    for pwr_name, balls in PINOUT.items():
        if pwr_name.startswith('VCC') or pwr_name.startswith('VDD') or pwr_name == 'VQPS':
            for ball in balls:
                power_pins.append((pwr_name, ball, 'power_in'))
        elif pwr_name == 'GND':
            for ball in balls:
                power_pins.append((pwr_name, ball, 'power_in'))

    height = len(power_pins) * 2.54 + 5.08
    y_start = height / 2

    lines.append(f'      (rectangle (start -15.24 {y_start:.2f}) (end 15.24 {-y_start:.2f})')
    lines.append('        (stroke (width 0.254) (type default))')
    lines.append('        (fill (type background))')
    lines.append('      )')

    # Add VCC pins on left, GND on right
    y_pos = y_start - 2.54
    vcc_pins = [(n, b) for n, b, t in power_pins if n != 'GND']
    gnd_pins = [(n, b) for n, b, t in power_pins if n == 'GND']

    for pin_name, ball in vcc_pins[:50]:  # Limit for readability
        lines.append(f'      (pin power_in line (at -17.78 {y_pos:.2f} 0) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.27 1.27))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.27 1.27))))')
        lines.append('      )')
        y_pos -= 2.54

    y_pos = y_start - 2.54
    for pin_name, ball in gnd_pins[:50]:
        lines.append(f'      (pin power_in line (at 17.78 {y_pos:.2f} 180) (length 2.54)')
        lines.append(f'        (name "{pin_name}" (effects (font (size 1.27 1.27))))')
        lines.append(f'        (number "{ball}" (effects (font (size 1.27 1.27))))')
        lines.append('      )')
        y_pos -= 2.54

    lines.append('    )')
    return '\n'.join(lines)


def main():
    symbol_name = "Ti375G529"

    print("Generating KiCad symbol for Ti375G529...")
    print(f"Total GPIO Bank L pins: {len(GPIOL_PINS)}")
    print(f"Total GPIO Bank R pins: {len(GPIOR_TR_PINS) + len(GPIOR_BR_PINS) + len(GPIOR_3A_PINS) + len(GPIOR_3B_PINS) + len(GPIOR_3C_PINS)}")
    print(f"Total GPIO Bank T pins: {len(GPIOT_2A_PINS) + len(GPIOT_2B_PINS) + len(GPIOT_2C_PINS)}")
    print(f"Total GPIO Bank B pins: {len(GPIOB_4A_PINS) + len(GPIOB_4B_PINS) + len(GPIOB_4C_PINS)}")
    print(f"Total DDR pins: {len(DDR_PINS)}")

    # For a simpler approach, just output the pin mapping as CSV
    print("\n--- PIN MAPPING (for KiCad import) ---")
    print("Pin Name,Ball,Type")

    all_pins = []

    # Add all GPIO pins
    for name, ball in GPIOL_PINS.items():
        all_pins.append((name, ball, 'BiDi'))
    for name, ball in {**GPIOR_TR_PINS, **GPIOR_BR_PINS, **GPIOR_3A_PINS, **GPIOR_3B_PINS, **GPIOR_3C_PINS}.items():
        all_pins.append((name, ball, 'BiDi'))
    for name, ball in {**GPIOT_2A_PINS, **GPIOT_2B_PINS, **GPIOT_2C_PINS}.items():
        all_pins.append((name, ball, 'BiDi'))
    for name, ball in {**GPIOB_4A_PINS, **GPIOB_4B_PINS, **GPIOB_4C_PINS}.items():
        all_pins.append((name, ball, 'BiDi'))
    for name, ball in DDR_PINS.items():
        all_pins.append((name, ball, 'BiDi'))

    # Add power pins
    for pwr_name, balls in PINOUT.items():
        if isinstance(balls, list):
            for ball in balls:
                if pwr_name == 'GND':
                    all_pins.append((pwr_name, ball, 'Power'))
                elif pwr_name.startswith('VCC') or pwr_name.startswith('VDD') or pwr_name == 'VQPS':
                    all_pins.append((pwr_name, ball, 'Power'))
                elif pwr_name in ['TDO', 'TMS', 'TCK', 'TDI', 'CRESET_N', 'CDONE']:
                    all_pins.append((pwr_name, ball, 'BiDi'))
                elif pwr_name.startswith('REF_RES'):
                    all_pins.append((pwr_name, ball, 'Passive'))
                elif pwr_name == 'NC':
                    all_pins.append((pwr_name, ball, 'NoConnect'))

    # Sort by ball name for easier verification
    all_pins.sort(key=lambda x: x[1])

    for name, ball, ptype in all_pins:
        print(f"{name},{ball},{ptype}")

    print(f"\nTotal pins: {len(all_pins)}")


if __name__ == "__main__":
    main()
