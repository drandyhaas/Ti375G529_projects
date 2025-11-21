#!/usr/bin/env python3
"""
USB3-based LPDDR4 Calibration Console
Replacement for JTAG-based console.py using USB3 register access
"""

import sys
import time
import logging
import argparse

from io_calibration import io_calibration
from read_leveling import read_leveling
from gate_leveling import gate_leveling
from write_leveling import write_leveling
from write_dq_leveling import write_dq_leveling
from ca_training import ca_training
from initialization import initialization
from usb_ddr_control import USBDDRControl

LOGGER = logging.getLogger('ddr_cali_tools')

def show_main_menu(dev, freq, axi0_freq, axi1_freq, cs_map, rank, width, ddr_type, size):
    """Show the main menu"""
    print('\n==================================================================')
    print('=============== EFINIX LPDDR4 Utility v3.0 (USB3) ================')
    print('==================================================================\n')
    LOGGER.info(f'Device: {dev}')
    LOGGER.info(f'Type: {ddr_type}')
    LOGGER.info(f'Data width: {width}')
    LOGGER.info(f'Density: {size} bit (Per Channel)')
    LOGGER.info(f'Frequency: {freq} MHz')
    LOGGER.info(f'Rank: {rank}')
    LOGGER.info(f'Chip Select Map: {hex(cs_map)}')
    LOGGER.info(f'AXI0: {axi0_freq:.2f} MHz')
    LOGGER.info(f'AXI1: {axi1_freq:.2f} MHz')
    print('\n')
    print('--> Calibration Menu:')
    print('----> init \tLPDDR4 initialization')
    print('----> io \tI/O Calibration')
    print('----> calvl \tCA Training')
    print('----> wrlvl \tWrite Leveling')
    print('----> gatlvl \tGate Leveling')
    print('----> rdlvl \tRead Leveling')
    print('----> wdqlvl \tWrite DQ Leveling')
    print('----> all \tRun all above processes')
    print('\n')
    print('--> Memory Test Menu:')
    print('----> mtest4 \t4MB    Size Memory Test')
    print('----> mtest16 \t16MB   Size Memory Test')
    print('----> mtest32 \t32MB   Size Memory Test')
    print('----> mtest64 \t64MB   Size Memory Test')
    print('----> mtest128 \t128MB  Size Memory Test')
    print('----> mtest256 \t256MB  Size Memory Test')
    print('----> mtest512 \t512MB  Size Memory Test')
    print('----> mtest1024 \t1024MB Size Memory Test')
    print('\n')
    print('--> Other Options:')
    print('----> help \tShow this menu')
    print('----> exit \tQuit program')
    print('\n')


def run_calibration_all(usb_ctrl, freq=800, cs_map=0x3, pcr_file='bsp/TI375C529/outflow/tools_core.pcr_write_pattern.v'):
    """Run all calibration steps in sequence"""
    LOGGER.info("="*70)
    LOGGER.info("Running full calibration sequence (all)")
    LOGGER.info("="*70)

    try:
        # 0. Initialize DDR controller access (same as JTAG console.py)
        LOGGER.info("\n[0/3] Initializing DDR controller access...")
        usb_ctrl.config_restart()      # Reset pulse on all DDR components
        usb_ctrl.config_ctrl_sel(1)    # Set config_sel=1 (required for register access)
        import time
        time.sleep(0.1)

        # 1. Initialization
        LOGGER.info("\n[1/3] Running LPDDR4 Initialization...")
        init = initialization(usb_ctrl)
        LOGGER.info("Created initialization instance")

        # Check CTL controller ID (matching JTAG console.py behavior)
        # Note: CTL controller may not respond until after initialization writes
        LOGGER.info("Reading CTL controller ID...")
        ctl_id = init.read_ctl_id()
        LOGGER.info(f"CTL Controller ID: 0x{ctl_id:04X}")
        if ctl_id == 0x2040:
            LOGGER.info("Read Memory controller ID PASS")
        elif ctl_id == 0x0000:
            LOGGER.warning("CTL Controller ID = 0x0000 (may be in reset, will initialize anyway)")
        else:
            LOGGER.warning(f"CTL Controller ID unexpected - Expected 0x2040, got 0x{ctl_id:04X}")

        # Check PI controller ID
        LOGGER.info("Reading PI controller ID...")
        pi_id = init.read_pi_id()
        LOGGER.info(f"PI Controller ID: 0x{pi_id:04X}")
        if pi_id == 0x2040:
            LOGGER.info("Read PI controller ID PASS")
        else:
            raise Exception(f"Read PI controller ID FAIL - Expected 0x2040, got 0x{pi_id:04X}")

        # Run initialization with PCR file
        import os
        if os.path.exists(pcr_file):
            LOGGER.info(f"Loading initialization from: {pcr_file}")
            init.initial(pcr_file, False, cs_map)
            LOGGER.info("DDR Initialization Done")
        else:
            LOGGER.warning(f"PCR file not found: {pcr_file}")
            LOGGER.warning("Skipping initialization - DDR may already be initialized by hardware")

        # 2. I/O Calibration
        LOGGER.info("\n[2/3] Running I/O Calibration...")
        io_cal = io_calibration(usb_ctrl)
        io_cal.run_io_calibration()

        LOGGER.info("\n" + "="*70)
        LOGGER.info("Calibration sequence completed successfully!")
        LOGGER.info("="*70 + "\n")

        return True

    except Exception as e:
        LOGGER.error(f"\nCalibration failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main console loop"""
    logging.basicConfig()
    stream_handler = logging.StreamHandler()
    formatter = logging.Formatter('%(message)s')
    stream_handler.setFormatter(formatter)
    LOGGER.addHandler(stream_handler)
    LOGGER.setLevel(logging.INFO)
    LOGGER.propagate = False

    # Parse arguments
    parser = argparse.ArgumentParser(description='USB3 LPDDR4 Console')
    parser.add_argument('--auto-all', action='store_true',
                       help='Automatically run "all" calibration and exit')
    args = parser.parse_args()

    # Initialize USB controller
    print("\nInitializing USB3 DDR Control...")
    try:
        usb_ctrl = USBDDRControl()
    except Exception as e:
        print(f"ERROR: Failed to initialize USB3 control: {e}")
        return 1

    # Default parameters (TODO: read from config file or hardware)
    dev = 'Ti375C529'
    freq = 800  # MHz
    axi0_freq = 100.0  # MHz
    axi1_freq = 100.0  # MHz
    cs_map = 0x3
    rank = 1
    width = 16
    ddr_type = 'LPDDR4'
    size = '4Gb'

    # If auto mode, run calibration and exit
    if args.auto_all:
        success = run_calibration_all(usb_ctrl)
        usb_ctrl.close()
        return 0 if success else 1

    # Show menu
    show_main_menu(dev, freq, axi0_freq, axi1_freq, cs_map, rank, width, ddr_type, size)

    # Interactive console loop
    while True:
        try:
            cmd = input("LPDDR4> ").strip().lower()

            if not cmd:
                continue

            if cmd == 'exit' or cmd == 'quit':
                break

            elif cmd == 'help':
                show_main_menu(dev, freq, axi0_freq, axi1_freq, cs_map, rank, width, ddr_type, size)

            elif cmd == 'init':
                LOGGER.info("Running LPDDR4 initialization...")
                usb_ctrl.config_restart()
                usb_ctrl.config_ctrl_sel(1)
                init = initialization(usb_ctrl)
                pcr_file = 'bsp/TI375C529/outflow/tools_core.pcr_write_pattern.v'

                # Check CTL controller ID (matching JTAG behavior)
                ctl_id = init.read_ctl_id()
                LOGGER.info(f"CTL Controller ID: 0x{ctl_id:04X}")
                if ctl_id == 0x2040:
                    LOGGER.info("Read Memory controller ID PASS")
                else:
                    raise Exception(f"Read Memory controller ID FAIL")

                # Check PI controller ID
                pi_id = init.read_pi_id()
                LOGGER.info(f"PI Controller ID: 0x{pi_id:04X}")
                if pi_id == 0x2040:
                    LOGGER.info("Read PI controller ID PASS")
                else:
                    raise Exception(f"Read PI controller ID FAIL")

                init.initial(pcr_file, False, cs_map=0x3)

            elif cmd == 'io':
                LOGGER.info("Running I/O Calibration...")
                io_cal = io_calibration(usb_ctrl)
                io_cal.execute()

            elif cmd == 'calvl':
                LOGGER.info("Running CA Training...")
                ca = ca_training(usb_ctrl)
                ca.execute()

            elif cmd == 'wrlvl':
                LOGGER.info("Running Write Leveling...")
                wrlvl = write_leveling(usb_ctrl)
                wrlvl.execute()

            elif cmd == 'gatlvl':
                LOGGER.info("Running Gate Leveling...")
                gatlvl = gate_leveling(usb_ctrl)
                gatlvl.execute()

            elif cmd == 'rdlvl':
                LOGGER.info("Running Read Leveling...")
                rdlvl = read_leveling(usb_ctrl)
                rdlvl.execute()

            elif cmd == 'wdqlvl':
                LOGGER.info("Running Write DQ Leveling...")
                wdqlvl = write_dq_leveling(usb_ctrl)
                wdqlvl.execute()

            elif cmd == 'all':
                run_calibration_all(usb_ctrl)

            elif cmd.startswith('mtest'):
                # Extract size from command (e.g., mtest4 -> 4)
                try:
                    size_mb = int(cmd.replace('mtest', ''))
                    LOGGER.info(f"Running {size_mb}MB memory test...")
                    result = usb_ctrl.memtest_run(size_mb=size_mb, lfsr_en=True, verbose=True)
                    if result:
                        LOGGER.info(f"Memory test PASSED")
                    else:
                        LOGGER.error(f"Memory test FAILED")
                except ValueError:
                    LOGGER.error(f"Invalid memory test command: {cmd}")

            else:
                LOGGER.warning(f"Unknown command: {cmd}")
                LOGGER.warning("Type 'help' for available commands")

        except KeyboardInterrupt:
            print("\nUse 'exit' to quit")
        except EOFError:
            break
        except Exception as e:
            LOGGER.error(f"Error: {e}")
            import traceback
            traceback.print_exc()

    print("\nClosing USB connection...")
    usb_ctrl.close()
    print("Goodbye!")
    return 0


if __name__ == '__main__':
    sys.exit(main())
