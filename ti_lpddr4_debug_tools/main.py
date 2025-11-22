import sys
import time
import math
import os
import re
import argparse
import json

import logging

LOGGER = logging.getLogger('ddr_cali_tools')
LOGGER.setLevel(logging.DEBUG)
#LOGGER.setLevel('ERROR')

from io_calibration import io_calibration
from read_leveling import read_leveling
from gate_leveling import gate_leveling
from write_leveling import write_leveling
from write_dq_leveling import write_dq_leveling
from ca_training import ca_training
from mode_register_wr_rd import mr_control
from pathlib import Path
from time import sleep
from contextlib import contextmanager
from typing import Mapping
from jtag_drv import jtag_drv
from timing_scan import timing_scan
from initialization import initialization

sys.path.append(str(Path(os.environ['EFINITY_HOME'], 'debugger', 'bin')))
sys.path.append(str(Path(os.environ['EFINITY_HOME'], 'pgm', 'bin')))

jtag_session = None
debug_session = None

from pyftdi.bits import BitSequence
from efx_dbg.define import DebugProfileBuilder
from efx_dbg.engine import DebugEngine
from efx_dbg.jtag import JtagManager, VioController,DebugSession
from efx_pgm.usb_resolver import UsbResolver

if __name__ == '__main__':
    logging.basicConfig()
    stream_handler = logging.StreamHandler()
    formatter = logging.Formatter('%(message)s')
    stream_handler.setFormatter(formatter)
    LOGGER.addHandler(stream_handler)
    LOGGER.setLevel(logging.INFO)
    LOGGER.propagate = False

    parser = argparse.ArgumentParser(description='DDR Calibration Tools : CS0 default with DQ[15:0],CS1/2/3 default with DQ[31:16].Support up to 2 CS')
    parser.add_argument('--init',default=False,action='store_true',help='DDR Initialization with PI calibration Auto flow')
    parser.add_argument('--io',default=False,action='store_true',help='Run I/O Calibration')
    parser.add_argument('--calvl',default=False,action='store_true',help='Run CA Leveling')
    parser.add_argument('--wrlvl',default=False,action='store_true',help='Run Write Leveling')
    parser.add_argument('--gatlvl',default=False,action='store_true',help='Run Gate Leveling')
    parser.add_argument('--rdlvl',default=False,action='store_true',help='Run Read Leveling')
    parser.add_argument('--wdqlvl',default=False,action='store_true',help='Run Write DQ Leveling')
    parser.add_argument('--mtest',default=False,type=str,help='Run hardware Memory Test : data')
    parser.add_argument('--mtest_lfsr',default=False, type=str,help='Run hardware Memory LFSR Test: data seed ')
    parser.add_argument('--scan',default=False, metavar='N', type=str, nargs='+',help='Timing Scan of Delay [0=write dq, 1=write dqs, 2=read dq rise, 3=read dq fall, 4=gate dqs] [data pattern 64bit] [start] [end]  example --scan 2 0x55555555AAAAAAAA 0 480')
    parser.add_argument('--scan_lfsr',default=False, metavar='N', type=str, nargs='+',help='Timing Scan of Delay [0=write dq, 1=write dqs, 2=read dq rise, 3=read dq fall, 4=gate dqs] with LSFR[data seed 64bit] [start] [end] example --scan_lfsr 0 0x55555555AAAAAAAA 0 480')
    parser.add_argument('--freq',default=1000, type=int,help='Freq for calculate vaild window default = 1000')
    parser.add_argument('--cs_map',default=1, type=int,help='CS Mask Enable for initialization [3:0] = {CS3,CS2,CS1,CS0}. 1=Enable,0=Disable  default = 1 for Efinix Board')
    parser.add_argument('--mrr',default=-1, type=int,help='Mode Register Read ')
    parser.add_argument('--mrw',default=False, metavar='N', type=int, nargs='+',help='Mode Register Write')
    parser.add_argument('--all',default=False,action='store_true',help='All Calibration Enable + DDR Initialization')
    parser.add_argument('--dev',default='TI180M484', type=str,help='Devive Type Default : TI180M484')
    parser.add_argument('--load_cali_file',default=False, metavar='N', type=str, nargs='+',help='Load Calibration Json file to PHY')
    parser.add_argument('input_file', nargs='?', default=f'bsp/TI180M484/outflow/tools_core.pcr_write_pattern.v', help='*.pcr_write_pattern.v verilog file path from interface designer')

    args = parser.parse_args()

    resolver = UsbResolver()

    usb_connects = resolver.get_usb_connections()

    #for target in usb_connects:
    #    LOGGER.info(f'target = {target}, URLS = {target.URLS}')

    # First USB Target, FTDI Channel B URL
    target_url = usb_connects[0].URLS[1]

    if args.dev != 'TI180M484':
        pcr_file = f'bsp/{args.dev}/outflow/tools_core.pcr_write_pattern.v'
    else:
        pcr_file = args.input_file

    LOGGER.info(pcr_file)
    #LOGGER.info(target_url)

    default_file = 'template_cali.json'

    with open(default_file) as file:
        json_file = json.load(file)

    f = open('cali.json', 'w')

    try:
        jtag_session = JtagManager().config_jtag(url=target_url, user='USER1', jcfs=None, chip_num=1, tap='efx_ti')
        debug_profile = DebugProfileBuilder.from_dict({
            "debug_cores": [
                {
                    "name": "axi0",
                    "type": "ddr_cali_axi"
                },
                {
                    "name": "axi1",
                    "type": "ddr_cali_axi"
                }
            ],
        })
        debug_session = JtagManager().config_debug(jtag_session, debug_profile)

        drv = jtag_drv(debug_session)
        init = initialization(drv)
        iopad = io_calibration(drv)
        calvl = ca_training(drv, int(args.freq))
        rdlvl = read_leveling(drv, int(args.freq))
        gatlvl = gate_leveling(drv, int(args.freq))
        wrlvl = write_leveling(drv, int(args.freq))
        wdqlvl = write_dq_leveling(drv, int(args.freq))
        mr = mr_control(drv)
        ts = timing_scan(drv, wrlvl, gatlvl, rdlvl, wdqlvl, int(args.freq))

        cs = 0

        for rank in range(4):
            if args.cs_map & (1 << rank):
                cs = cs+1

        if cs == 1:
            slice_mask = 0x3
        else:
            slice_mask = 0xF


        #LOGGER.info(f'This Software do not hard reset the controller, please toggle creset every --init/--all ')
        LOGGER.info(f'{cs} Rank Device')

        if args.init | args.all:

            drv.config_restart()

            if init.read_ctl_id() == 0x2040:
                LOGGER.info('Read Memory controller ID PASS')
            else:
                raise Exception('Read Memory controller ID FAIL')

            if init.read_pi_id() == 0x2040:
                LOGGER.info('Read pi controller ID PASS')
            else:
                raise Exception('Read pi controller ID FAIL')

            LOGGER.info('---DDR Initialization---')

            init.initial(pcr_file, False, args.cs_map)

        if args.io | args.all:
            LOGGER.info('---I/O Calibration---')
            iopad.run_io_calibration()

        if args.calvl | args.all:
            LOGGER.info('---CA Leveling---')
            json_file = calvl.run_ca_leveling(0, args.cs_map, json_file)
            json_file = mr.update_verf_ca_cali_file(0, json_file)
            mr.read_vref_ca(2)
            init.flush_phyreg_fifo()

        if args.wrlvl | args.all:
            LOGGER.info('---Write Leveling---')
            json_file = wrlvl.run_write_leveling(0, 0x3, json_file)

            for rank in range(1, 4):
                if args.cs_map & (1 << rank):
                    json_file = wrlvl.run_write_leveling(rank, 0xC, json_file)

        if args.gatlvl | args.all:
            LOGGER.info('---Gate Leveling---')
            json_file = gatlvl.run_gate_leveling(0, 0x3, json_file)

            for rank in range(1, 4):
                if args.cs_map & (1 << rank):
                    json_file = gatlvl.run_gate_leveling(rank, 0xC, json_file)

        if args.rdlvl | args.all:
            LOGGER.info('---Read Leveling---')
            json_file = rdlvl.run_read_leveling(0, 0x3, json_file)

            for rank in range(1, 4):
                if args.cs_map & (1 << rank):
                    json_file = rdlvl.run_read_leveling(rank, 0xC, json_file)

        if args.wdqlvl | args.all:
            LOGGER.info('---Write DQ Leveling---')

            json_file = wdqlvl.run_writedq_leveling(0, 0x3, json_file)
            json_file = mr.update_verf_dq_cali_file(0, json_file)
            init.flush_phyreg_fifo()
            for rank in range(1, 4):
                if args.cs_map & (1 << rank):
                    json_file = wdqlvl.run_writedq_leveling(
                        rank, 0xC, json_file)
                    mr.read_vref_dq(2)
            init.flush_phyreg_fifo()

        if args.scan:

            if args.scan[0] == '0':
                LOGGER.info('---Scan DQ output---')
                json_file = ts.pattern_scan_write_dqx(slice_mask, int(args.scan[1], 16), args.scan[2], args.scan[3], False , json_file)

            if args.scan[0] == '1':
                LOGGER.info('---Scan DQS output---')
                json_file = ts.pattern_scan_write_dqs(slice_mask, int(args.scan[1], 16), args.scan[2], args.scan[3], False , json_file)

            if args.scan[0] == '2':
                LOGGER.info('---Scan DQ rise input---')
                json_file = ts.pattern_scan_input_dqx_rise(
                    slice_mask, int(args.scan[1], 16), args.scan[2], args.scan[3], False , json_file)

            if args.scan[0] == '3':
                LOGGER.info('---Scan DQ Fall input---')
                json_file = ts.pattern_scan_input_dqx_fall(
                    slice_mask, int(args.scan[1], 16), args.scan[2], args.scan[3], False , json_file)

            if args.scan[0] == '4':
                LOGGER.info('---Scan DQS input---')
                json_file = ts.pattern_scan_gate_dqs(slice_mask, int(args.scan[1], 16), args.scan_lfsr[2], args.scan_lfsr[3], False , json_file)

        if args.scan_lfsr:
            if args.scan_lfsr[0] == '0':
                LOGGER.info('---Scan DQ output---')
                json_file = ts.pattern_scan_write_dqx(
                    slice_mask, int(args.scan_lfsr[1], 16), args.scan_lfsr[2], args.scan_lfsr[3], True, json_file)

            if args.scan_lfsr[0] == '1':
                LOGGER.info('---Scan DQS output---')
                json_file = ts.pattern_scan_write_dqs(
                    slice_mask, int(args.scan_lfsr[1], 16), args.scan_lfsr[2], args.scan_lfsr[3], True, json_file)

            if args.scan_lfsr[0] == '2':
                LOGGER.info('---Scan DQ rise input---')
                json_file = ts.pattern_scan_input_dqx_rise(
                    slice_mask, int(args.scan_lfsr[1], 16), args.scan_lfsr[2], args.scan_lfsr[3], True, json_file)

            if args.scan_lfsr[0] == '3':
                LOGGER.info('---Scan DQ Fall input---')
                json_file = ts.pattern_scan_input_dqx_fall(
                    slice_mask, int(args.scan_lfsr[1], 16), args.scan_lfsr[2], args.scan_lfsr[3], True, json_file)

            if args.scan_lfsr[0] == '4':
                LOGGER.info('---Scan DQS input---')
                json_file = ts.pattern_scan_gate_dqs(slice_mask, int(args.scan_lfsr[1], 16), args.scan_lfsr[2], args.scan_lfsr[3], True, json_file)

        if args.mtest:
            data = int(args.mtest, 16)
            drv.memtest_data(data)
            drv.memtest_restart(False)
            fail = drv.memtest_poll_done()

            if fail:
                output = drv.memtest_read_fail_dq()
                LOGGER.info(f'memtest done and Fail DQ = {hex(output)}')
            else:
                LOGGER.info('memtest done and Pass')

        if args.mtest_lfsr:
            data = int(args.mtest_lfsr, 16)
            drv.memtest_data(data)
            drv.memtest_restart(True)
            fail = drv.memtest_poll_done()

            if fail:
                output = drv.memtest_read_fail_dq()
                LOGGER.info(f'memtest done and Fail DQ = {hex(output)}')
            else:
                LOGGER.info('memtest done and Pass')

        if (args.mrr>=0):
            LOGGER.info('---Mode Register Read---')
            mode = int(args.mrr)
            rddata = mr.mode_register_read(0, mode)
            LOGGER.info(f'MRR{mode}= {hex(rddata)}')

        if args.mrw:
            LOGGER.info('---Mode Register Write---')
            mode = int(args.mrw[0])
            data = int(args.mrw[1])
            mr.mode_register_write(0, mode, data)
            LOGGER.info(f'MRW{mode}= {hex(data)}')

        if args.load_cali_file:
            calibration_file = args.load_cali_file[0]
            LOGGER.info(f'calibration_file path = {calibration_file}')
            LOGGER.info(f'Loading....')

            with open(calibration_file) as file:
                load_file = json.load(file)

            ca_delay = []

            for ca in range(6):
                ca_delay.append(int((load_file[f'cs{0}']['slice_ca']['left_edge']
                                [f'ca{ca}']+load_file[f'cs{0}']['slice_ca']['right_edge'][f'ca{ca}'])/2))

            calvl.write_phy_adrx_clk_wr_slave_delay(ca_delay)

            if load_file[f'cs{0}'][f'vrefca_range'] == 0:
                mr.write_vref_ca(0, 0, mr.vref_r0_dict.index(
                    load_file[f'cs{0}'][f'vrefca_val']))
            else:
                mr.write_vref_ca(0, 1, mr.vref_r1_dict.index(
                    load_file[f'cs{0}'][f'vrefca_val']))

            mr.read_vref_ca(0)

            dqs_output_delay = []

            for slicex in range(4):
                dqs_output_delay.append(int((load_file[f'cs{0}'][f'slice_{slicex}']['output']['left_edge']
                                        ['dqs']+load_file[f'cs{0}'][f'slice_{slicex}']['output']['right_edge']['dqs'])/2))

            wrlvl.update_phy_clk_wrdqs_slave_delay(0xF, dqs_output_delay)

            gate_input_delay = []

            for slicex in range(4):
                gate_input_delay.append(int((load_file[f'cs{0}'][f'slice_{slicex}']['input']['left_edge']
                                        ['dqs'] + load_file[f'cs{0}'][f'slice_{slicex}']['input']['right_edge']['dqs'])/2))

            gatlvl.write_phy_rddqs_gate_slave_delayX(0xF, gate_input_delay)

            dq_input_rise_delay = []
            dq_input_fall_delay = []

            for slicex in range(4):
                for dq in range(8):
                    dq_input_rise_delay.append(int((load_file[f'cs{0}'][f'slice_{slicex}']['input']['left_edge']['rise']
                                               [f'dq{dq}'] + load_file[f'cs{0}'][f'slice_{slicex}']['input']['right_edge']['rise'][f'dq{dq}'])/2))
                    dq_input_fall_delay.append(int((load_file[f'cs{0}'][f'slice_{slicex}']['input']['left_edge']['fall']
                                               [f'dq{dq}'] + load_file[f'cs{0}'][f'slice_{slicex}']['input']['right_edge']['fall'][f'dq{dq}'])/2))

            rdlvl.update_phy_rddqs_dqx_rise_slave_delay(
                0xF, dq_input_rise_delay)
            rdlvl.update_phy_rddqs_dqx_fall_slave_delay(
                0xF, dq_input_fall_delay)

            input_verf = []

            for slicex in range(4):
                rng = rdlvl.check_device_vref_range(0xF)

                if rng[slicex] == 0:
                    input_verf.append(rdlvl.vref_r0_list.index(
                        load_file[f'cs{0}'][f'slice_{slicex}']['vref']))
                else:
                    input_verf.append(rdlvl.vref_r1_list.index(
                        load_file[f'cs{0}'][f'slice_{slicex}']['vref']))

            rdlvl.set_phy_pad_vref_ctrl_dq(0xF, input_verf)

            dq_output_delay = []

            for slicex in range(4):
                for dq in range(8):
                    dq_output_delay.append(int((load_file[f'cs{0}'][f'slice_{slicex}']['output']['left_edge']
                                            [f'dq{dq}'] + load_file[f'cs{0}'][f'slice_{slicex}']['output']['right_edge'][f'dq{dq}'])/2))

                dq_output_delay.append(int((load_file[f'cs{0}'][f'slice_{slicex}']['output']['left_edge']['dm'] + load_file[f'cs{0}'][f'slice_{slicex}']['output']['right_edge']['dm'])/2))

            wdqlvl.write_phy_clk_wrdqx_slave_delay(0xF, dq_output_delay)

            wdqlvl.update_slave_delay()

            if load_file[f'cs{0}'][f'vrefdq_range'] == 0:
                mr.write_vref_dq(0, 0, mr.vref_r0_dict.index(load_file[f'cs{0}'][f'vrefdq_val']))
            else:
                mr.write_vref_dq(0, 1, mr.vref_r1_dict.index(load_file[f'cs{0}'][f'vrefdq_val']))

            mr.read_vref_dq(0)

            LOGGER.info(f'Done')

        init.clean_pi_interrupt_status()
        json.dump(json_file, f, indent=4)

    finally:
        if debug_session:
            JtagManager().unconfig_debug(debug_session)
            debug_session = None

        if jtag_session:
            JtagManager().unconfig_jtag(jtag_session)
            jtag_session = None
