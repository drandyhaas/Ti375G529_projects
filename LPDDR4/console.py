import sys
import time
import math
import os
import re
import argparse
import json
import xml.etree.ElementTree as ET
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
## set_efinity_user_dir_env For 2025.1
from efx_pgm.util.gen_util import set_efinity_user_dir_env

def show_main_menu(dev,freq,axi0_freq,axi1_freq,cs_map,rank,width,type,size):
    print('\n==================================================================')
    print('======================EFINIX LPDDR4 Uility v2.2===================')
    print('==================================================================\n')
    LOGGER.info(f'Device: {dev}')
    LOGGER.info(f'Type: {type}')
    LOGGER.info(f'Data width: {width}')
    LOGGER.info(f'Density: {size} bit (Per Channel)')
    LOGGER.info(f'Frequency: {freq} Mhz')
    LOGGER.info(f'Rank: {rank}')
    LOGGER.info(f'Chip Select Map: {hex(cs_map)}')
    LOGGER.info(f'AXI0: {axi0_freq:.2f} Mhz')
    LOGGER.info(f'AXI1: {axi1_freq:.2f} Mhz')
    print('\n')
    print('--> Calibration item Menu:')
    print('----> init \tLPDDR4 initialization')
    print('----> io \tI/O Calibration')
    print('----> calvl \tCA Training')
    print('----> wrlvl \tWrite Leveling')
    print('----> gatlvl \tGate Leveling')
    print('----> rdlvl \tRead Leveling')
    print('----> wdqlvl \tWrite DQ Leveling')
    print('----> all \tRun all above process')
    print('\n')
    print('--> Memory Quick Test Menu:')
    print('----> mtest4 \t4MB    Size Memory Test')
    print('----> mtest16 \t16MB   Size Memory Test')
    print('----> mtest32 \t32MB   Size Memory Test')
    print('----> mtest64 \t64MB   Size Memory Test')
    print('----> mtest128 \t128MB  Size Memory Test')
    print('----> mtest256 \t256MB  Size Memory Test')
    print('----> mtest512 \t512MB  Size Memory Test')
    print('----> mtest1024 1024MB Size Memory Test')
    print('\n')
    print('--> Memory efficiency Test Menu:')
    print('----> eff Default Memory efficiency Test')
    print('\n')
    print('--> Others Option:')
    print('----> mrw LPDDR4 Mode Register Write')
    print('----> mrr LPDDR4 Mode Register Read')
    print('----> help show menu')
    print('----> exit quit program')
    print('\n')

if __name__ == '__main__':
    logging.basicConfig()
    stream_handler = logging.StreamHandler()
    formatter = logging.Formatter('%(message)s')
    stream_handler.setFormatter(formatter)
    LOGGER.addHandler(stream_handler)
    LOGGER.setLevel(logging.INFO)
    LOGGER.propagate = False
    
    ## set_efinity_user_dir_env()For 2025.1
    set_efinity_user_dir_env()
    
    resolver        = UsbResolver()
    usb_connects    = resolver.get_usb_connections()
    target_url      = usb_connects[0].URLS[1]


    parser = argparse.ArgumentParser(description='LPDDR4 Utility')
    parser.add_argument('--dev',default='TI180J484', type=str,help='Devive Type Default : TI180J484')

    args = parser.parse_args()

    namespace = {'efxpt': 'http://www.efinixinc.com/peri_design_db'}
    # Parse the XML file
    tree = ET.parse(f'bsp/{args.dev}/tools_core.peri.xml')
    root = tree.getroot()

    # Find the 'efxpt:pll' element
    pll_element = root.find('.//efxpt:pll', namespaces=namespace)
    comp_output_clock_element = root.find(f'.//efxpt:comp_output_clock[@name="regACLK"]', namespaces=namespace)
    Divfeedback = int(comp_output_clock_element.get('out_divider'))

    # Extract the value of the 'ref_clock_freq' attribute
    ref_clock_freq  = float(pll_element.get('ref_clock_freq'))
    M      = int(pll_element.get('multiplier'))
    N      = int(pll_element.get('pre_divider'))
    O      = int(pll_element.get('post_divider'))

    Fpfd = ref_clock_freq/N
    Fvco = Fpfd*M*O*Divfeedback
    Fpll =Fvco/O

    comp_output_clock_element = root.find('.//efxpt:comp_output_clock[@name="ddr_clk"]', namespaces=namespace)
    out_divider = int(comp_output_clock_element.get('out_divider'))

    pcr_file = f'bsp/{args.dev}/outflow/tools_core.pcr_write_pattern.v'
    freq = int((Fpll/out_divider)*2)

    comp_output_clock_element2 = root.find('.//efxpt:comp_output_clock[@name="axi1_ACLK"]', namespaces=namespace)
    out_divider2 = int(comp_output_clock_element2.get('out_divider'))
    tester_freq = Fpll/out_divider2

    comp_output_clock_element2 = root.find('.//efxpt:comp_output_clock[@name="axi0_ACLK"]', namespaces=namespace)
    out_divider2 = int(comp_output_clock_element2.get('out_divider'))
    memtest_freq = Fpll/out_divider2

    adv_ddr_element = root.find('.//efxpt:adv_ddr', namespaces=namespace)
    physical_rank   = int(adv_ddr_element.get('physical_rank'))
    mem_type        = adv_ddr_element.get('mem_type')
    mem_density     = adv_ddr_element.get('mem_density')
    data_width      = int(adv_ddr_element.get('data_width'))

    if (physical_rank   == 2 and data_width == 32): cs_map = 0xF
    elif (physical_rank == 1 and data_width == 32): cs_map = 0x5
    elif (physical_rank == 2 and data_width == 16): cs_map = 0x3
    elif (physical_rank == 1 and data_width == 16): cs_map = 0x1
    else:                                           cs_map = 0x5
    
    print(pcr_file)
    
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

        drv     = jtag_drv(debug_session)
        init    = initialization(drv)
        iopad   = io_calibration(drv)
        calvl   = ca_training(drv, freq)
        rdlvl   = read_leveling(drv, freq)
        gatlvl  = gate_leveling(drv, freq)
        wrlvl   = write_leveling(drv, freq)
        wdqlvl  = write_dq_leveling(drv, freq)
        mr      = mr_control(drv)
        ts      = timing_scan(drv, wrlvl, gatlvl, rdlvl, wdqlvl, freq)

        default_file = 'template_cali.json'

        with open(default_file) as file:
            json_file = json.load(file)

        f = open('cali.json', 'w')

        cmd_list = ['init', 'io', 'calvl', 'wrlvl', 'gatlvl', 'rdlvl', 'wdqlvl', 'all', 'help', 'mtest4',
                    'mtest8', 'mtest16', 'mtest32', 'mtest64', 'mtest128', 'mtest256', 'mtest512', 'mtest1024', 'mrw', 'mrr','eff']

        show_main_menu(args.dev,freq,memtest_freq, tester_freq, cs_map, physical_rank, data_width, mem_type, mem_density)

        while(1):

            print('Select Option:',end='')
            opt=input()

            if opt in ['init', 'all']:

                drv.config_restart()
                drv.config_ctrl_sel(1)
                if init.read_ctl_id() == 0x2040:
                    LOGGER.info('Read Memory controller ID PASS')
                else:
                    raise Exception('Read Memory controller ID FAIL')

                if init.read_pi_id() == 0x2040:
                    LOGGER.info('Read pi controller ID PASS')
                else:
                    raise Exception('Read pi controller ID FAIL')

                LOGGER.info('---DDR Initialization---')

                init.initial(pcr_file, False, cs_map)

                LOGGER.info('---DDR Initialization Done---')

            if opt in ['io', 'all']:
                LOGGER.info('---I/O Calibration---')
                iopad.run_io_calibration()

            if opt in ['calvl', 'all']:
                LOGGER.info('---CA Leveling---')
                json_file = calvl.run_ca_leveling(0, cs_map, json_file)
                json_file = mr.update_verf_ca_cali_file(0, json_file)
                mr.read_vref_ca(2)
                init.flush_phyreg_fifo()

            if opt in ['wrlvl', 'all']:
                LOGGER.info('---Write Leveling---')
                if(cs_map == 0xF):
                    json_file = wrlvl.run_write_leveling(0, 0x1, json_file)
                    json_file = wrlvl.run_write_leveling(1, 0x2, json_file)
                    json_file = wrlvl.run_write_leveling(2, 0x4, json_file)
                    json_file = wrlvl.run_write_leveling(3, 0x8, json_file)
                elif(cs_map == 0x5):
                    json_file = wrlvl.run_write_leveling(0, 0x3, json_file)
                    json_file = wrlvl.run_write_leveling(2, 0xC, json_file)
                elif(cs_map == 0x3):
                    json_file = wrlvl.run_write_leveling(0, 0x1, json_file)
                    json_file = wrlvl.run_write_leveling(1, 0x2, json_file)
                elif(cs_map == 0x1):
                    json_file = wrlvl.run_write_leveling(0, 0x3, json_file)
                                        

            if opt in ['gatlvl', 'all']:
                LOGGER.info('---Gate Leveling---')

                if(cs_map == 0xF):
                    json_file = gatlvl.run_gate_leveling(0, 0x1, json_file)
                    json_file = gatlvl.run_gate_leveling(1, 0x2, json_file)
                    json_file = gatlvl.run_gate_leveling(2, 0x4, json_file)
                    json_file = gatlvl.run_gate_leveling(3, 0x8, json_file)
                elif(cs_map == 0x5):
                    json_file = gatlvl.run_gate_leveling(0, 0x3, json_file)
                    json_file = gatlvl.run_gate_leveling(2, 0xC, json_file)
                elif(cs_map == 0x3):
                    json_file = gatlvl.run_gate_leveling(0, 0x1, json_file)
                    json_file = gatlvl.run_gate_leveling(1, 0x2, json_file)
                elif(cs_map == 0x1):
                    json_file = gatlvl.run_gate_leveling(0, 0x3, json_file)

            if opt in ['rdlvl', 'all']:
                LOGGER.info('---Read Leveling---')

                if(cs_map == 0xF):
                    json_file = rdlvl.run_read_leveling(0, 0x1, json_file)
                    json_file = rdlvl.run_read_leveling(1, 0x2, json_file)
                    json_file = rdlvl.run_read_leveling(2, 0x4, json_file)
                    json_file = rdlvl.run_read_leveling(3, 0x8, json_file)
                elif(cs_map == 0x5):
                    json_file = rdlvl.run_read_leveling(0, 0x3, json_file)
                    json_file = rdlvl.run_read_leveling(2, 0xC, json_file)
                elif(cs_map == 0x3):
                    json_file = rdlvl.run_read_leveling(0, 0x1, json_file)
                    json_file = rdlvl.run_read_leveling(1, 0x2, json_file)
                elif(cs_map == 0x1):
                    json_file = rdlvl.run_read_leveling(0, 0x3, json_file)

            if opt in ['wdqlvl', 'all']:
                LOGGER.info('---Write DQ Leveling---')

                if(cs_map == 0xF):
                    json_file = wdqlvl.run_writedq_leveling(0, 0x1, json_file)
                    json_file = mr.update_verf_dq_cali_file(0, json_file)
                    init.flush_phyreg_fifo()
                    json_file = wdqlvl.run_writedq_leveling(1, 0x2, json_file)
                    json_file = mr.update_verf_dq_cali_file(1, json_file)
                    init.flush_phyreg_fifo()
                    json_file = wdqlvl.run_writedq_leveling(2, 0x4, json_file)
                    json_file = mr.update_verf_dq_cali_file(2, json_file)
                    init.flush_phyreg_fifo()
                    json_file = wdqlvl.run_writedq_leveling(3, 0x8, json_file)
                    json_file = mr.update_verf_dq_cali_file(3, json_file)
                    init.flush_phyreg_fifo()
                elif(cs_map == 0x5):
                    json_file = wdqlvl.run_writedq_leveling(0, 0x3, json_file)
                    json_file = mr.update_verf_dq_cali_file(0, json_file)
                    init.flush_phyreg_fifo()
                    json_file = wdqlvl.run_writedq_leveling(2, 0xC, json_file)
                    json_file = mr.update_verf_dq_cali_file(2, json_file)
                    init.flush_phyreg_fifo()
                elif(cs_map == 0x3):
                    json_file = wdqlvl.run_writedq_leveling(0, 0x1, json_file)
                    json_file = mr.update_verf_dq_cali_file(0, json_file)
                    init.flush_phyreg_fifo()
                    json_file = wdqlvl.run_writedq_leveling(1, 0x2, json_file)
                    json_file = mr.update_verf_dq_cali_file(1, json_file)
                    init.flush_phyreg_fifo()
                elif(cs_map == 0x1):
                    json_file = wdqlvl.run_writedq_leveling(0, 0x3, json_file)
                    json_file = mr.update_verf_dq_cali_file(0, json_file)
                    init.flush_phyreg_fifo()

            if opt in ['mtest4','mtest8','mtest16','mtest32','mtest64','mtest128','mtest256','mtest512','mtest1024']:

                drv.memtest_data(0x5555AAAA)

                if opt == 'mtest4':     size = 4
                elif opt == 'mtest8':   size = 8
                elif opt == 'mtest16':  size = 16
                elif opt == 'mtest32':  size = 32
                elif opt == 'mtest64':  size = 64
                elif opt == 'mtest128': size = 128
                elif opt == 'mtest256': size = 256
                elif opt == 'mtest512': size = 512
                elif opt == 'mtest1024':size = 1024
                else                   :size = 4
                drv.memtest_size(size)
                drv.memtest_restart(True)
                fail = drv.memtest_poll_done()

                if fail:
                    output = drv.memtest_read_fail_dq()
                    LOGGER.info(f'memtest done and Fail DQ = {hex(output)}')
                else:
                    LOGGER.info('memtest done and Pass')

            if opt in ['eff']:
                # Stop any running memtest on AXI0 to prevent interference with efficiency test on AXI1
                drv.memtest_stop()

                LOGGER.info('\nEfficiency Test :')
                LOGGER.info(f'||==== WRITE ===||==== READ ====||==EFFICIENCY==||== BANDWIDTH ==||=== ERROR ===||')
                drv.memtester_pattern(0x5555, 0xAAAA)
                drv.memtester_restart()
                err=drv.memtester_poll()
                len = drv.memtester_read_len()
                cnt = drv.memtester_read_cnt()
                eff = len/cnt * 100
                bw = (len/cnt)* tester_freq*512/1000
                LOGGER.info(f'||\t50%\t||\t50%\t||   {eff:.3f} %\t||  {bw:.3f} Gbps  ||\t{err}\t||')

                drv.memtester_pattern(0xFFFF, 0x0)
                drv.memtester_restart()
                err=drv.memtester_poll()
                err ='N/A'
                len = drv.memtester_read_len()
                cnt = drv.memtester_read_cnt()
                eff = len/cnt * 100
                bw = (len/cnt)* tester_freq*512/1000
                LOGGER.info(f'||\t100%\t||\t0%\t||   {eff:.3f} %\t||  {bw:.3f} Gbps  ||\t{err}\t||')

                drv.memtester_pattern(0x0, 0xFFFF)
                drv.memtester_restart()
                err=drv.memtester_poll()
                err ='N/A'
                len = drv.memtester_read_len()
                cnt = drv.memtester_read_cnt()
                eff = len/cnt * 100
                bw = (len/cnt)* tester_freq*512/1000
                LOGGER.info(f'||\t0%\t||\t100%\t||   {eff:.3f} %\t||  {bw:.3f} Gbps  ||\t{err}\t||')

            if opt in ['mrw']:
                LOGGER.info('---Mode Register Write---')
                LOGGER.info(f'input Mode Register Write Address:')
                mrw_addr=input()
                LOGGER.info(f'input Mode Register Write Data:')
                mrw_data=input()
                mode = int(mrw_addr)
                data = int(mrw_data)
                mr.mode_register_write(0, mode, data)
                LOGGER.info(f'MRW{mode}= {hex(data)}')

            if opt in ['mrr']:
                LOGGER.info('---Mode Register Read---')
                LOGGER.info(f'input Mode Register Read Address:')
                mrr_addr=input()
                rddata = mr.mode_register_read(0, int(mrr_addr))
                LOGGER.info(f'MRR{mrr_addr}= {hex(rddata)}')

            if opt in ['help']:
                show_main_menu(args.dev,freq,memtest_freq, tester_freq, cs_map, physical_rank, data_width, mem_type, mem_density)

            if opt in ['exit']:
                if debug_session:
                    JtagManager().unconfig_debug(debug_session)
                    debug_session = None

                if jtag_session:
                    JtagManager().unconfig_jtag(jtag_session)
                    jtag_session = None
                break

            if opt not in cmd_list:
                LOGGER.info('invaild command')

            LOGGER.info('\n')
    finally:
        if debug_session:
            JtagManager().unconfig_debug(debug_session)
            debug_session = None

        if jtag_session:
            JtagManager().unconfig_jtag(jtag_session)
            jtag_session = None
