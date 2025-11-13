from __future__ import annotations
from typing import TYPE_CHECKING

import logging
import time

LOGGER = logging.getLogger('ddr_cali_tools')


if TYPE_CHECKING:
    from efx_dbg.jtag import DebugSession


class jtag_drv:

    def __init__(self, debug_session: DebugSession):
        self.axi_controllers = [
            debug_session.get_controller_by_name('axi0'),
            debug_session.get_controller_by_name('axi1')
        ]
        self.debug_session = debug_session

    def jtag2axi_write(self, addr: int, data: int):

        temp = []
        temp.append(data)
        self.axi_controllers[0].write(addr, temp)
        temp.clear()

        return

    def jtag2axi_read(self, addr: int):

        temp = self.axi_controllers[0].read(addr, 1)
        #print("jtag2axi_read",temp)
        return temp[0]

    def jtag2axi1_write(self, addr: int, data: int):

        temp = []
        temp.append(data)
        self.axi_controllers[1].write(addr, temp)
        temp.clear()

        return

    def jtag2axi1_read(self, addr: int):

        temp = self.axi_controllers[1].read(addr, 1)

        return temp[0]

    def lpddr4_ctrl_write(self, type: str, addr: int, data: int):

        if type == 'CTL':
            self.jtag2axi_write(addr*4, data)
        elif type == 'PI':
            self.jtag2axi_write((addr*4)+0x2000, data)
        elif type == 'PHY':
            self.jtag2axi_write((addr*4)+0x4000, data)
        else:
            return None

        return

    def lpddr4_ctrl_read(self, type: str, addr: int):

        if type == 'CTL':
            return self.jtag2axi_read(addr*4)
        elif type == 'PI':
            return self.jtag2axi_read((addr*4)+0x2000)
        elif type == 'PHY':
            return self.jtag2axi_read((addr*4)+0x4000)
        else:
            return None

    def memtest_ctrl_write(self, addr: int, data: int):

        self.jtag2axi1_write(addr*4, data)

        return

    def memtest_ctrl_read(self, addr: int):

        return self.jtag2axi1_read(addr*4)

    def memtest_restart(self, lsfr_en):

        if lsfr_en == True:
            self.memtest_ctrl_write(6, 0x1)
        else:
            self.memtest_ctrl_write(6, 0x0)

        self.memtest_ctrl_write(2, 0x00)
        self.memtest_ctrl_write(2, 0x03)

    def memtest_stop(self):
        '''
        Stop the memory test module (AXI0) to prevent interference
        REG2[0] = memtest_start
        REG2[1] = memtest_rstn
        '''
        self.memtest_ctrl_write(2, 0x00)

    def memtest_data(self, data):

        data0 = data & 0xFFFFFFFF
        data1 = (data >> 32) & 0xFFFFFFFF

        self.memtest_ctrl_write(4, data0)
        self.memtest_ctrl_write(5, data1)

    def memtest_size(self, size):

        data0 = ((size*1024*1024)-4096) & 0xFFFFFFFF
        self.memtest_ctrl_write(9, data0)

    def memtest_poll_done(self):

        while (1):
            output = self.memtest_ctrl_read(1)

            if (output & 0x01):
                # LOGGER.info(f'{n} / {math.floor(end/step)}  ', end="\r")
                break

        if output & 0x02:  # Test FAIL
            return True
        else:  # Test PASS
            return False

    def memtest_read_fail_dq(self):

        return self.memtest_ctrl_read(0)

    def config_restart(self):
        '''
        bit[0]  = CFG_RESET
        bit[1]  = CFG_PHY_RSTN
        bit[2]  = CTRL_RSTN
        bit[3]  = CR_ARESETN
        bit[4]  = ARSTN_0
        bit[5]  = ARSTN_1
        '''
        #CFG_RESET = 0
        CFG_PHY_RSTN = 0
        CTRL_RSTN = 0
        CR_ARESETN = 0
        ARSTN_0 = 0
        ARSTN_1 = 0

        output = ((ARSTN_1 << 4) | (ARSTN_0 << 3) | (CR_ARESETN << 2) | (CTRL_RSTN << 1)
                  | (CFG_PHY_RSTN << 0))

        self.memtest_ctrl_write(3, output)
        time.sleep(0.1)

        #CFG_RESET = 1
        CFG_PHY_RSTN = 1
        CTRL_RSTN = 1
        CR_ARESETN = 1
        ARSTN_0 = 1
        ARSTN_1 = 1

        output = ((ARSTN_1 << 4) | (ARSTN_0 << 3) | (CR_ARESETN << 2) | (CTRL_RSTN << 1)
                  | (CFG_PHY_RSTN << 0))

        self.memtest_ctrl_write(3, output)

    def config_ctrl_start(self):
        '''
        bit[0]  = CFG_RESET
        bit[1]  = CFG_SEL
        bit[2]  = CFG_START
        bit[3]  = CFG_DONE
        '''
        CFG_RESET = 0
        CFG_SEL = 0
        CFG_START = 0

        output = ((CFG_START << 2) | (CFG_SEL << 1) | (CFG_RESET))

        self.memtest_ctrl_write(10, output)
        time.sleep(0.1)
        done = self.memtest_ctrl_read(10)
        print(done)

        CFG_RESET = 0
        CFG_SEL = 0
        CFG_START = 1

        output = ((CFG_START << 2) | (CFG_SEL << 1) | (CFG_RESET))

        self.memtest_ctrl_write(10, output)

        while (1):
            done = self.memtest_ctrl_read(10)
            print(done)
            if done & 0x8:
                break

    def config_ctrl_sel(self,sel):

        '''
        bit[0]  = CFG_RESET
        bit[1]  = CFG_SEL
        bit[2]  = CFG_START
        bit[3]  = CFG_DONE
        '''
        CFG_RESET = 1
        CFG_SEL = sel
        CFG_START = 0

        output = ((CFG_START << 2) | ((CFG_SEL & 0x1) << 1) | (CFG_RESET))

        self.memtest_ctrl_write(10, output)

        CFG_RESET = 0
        CFG_SEL = sel
        CFG_START = 0

        output = ((CFG_START << 2) | ((CFG_SEL & 0x1) << 1) | (CFG_RESET))

        self.memtest_ctrl_write(10, output)

    def x16_mode_test_enable(self):

        self.memtest_ctrl_write(7, 1)

        return

    def x16_mode_test_disable(self):

        self.memtest_ctrl_write(7, 0)

        return

    def memtester_restart(self):

        self.memtest_ctrl_write(16, 0x00)
        self.memtest_ctrl_write(16, 0x01)

    def memtester_poll(self):

        while (1):
            done = self.memtest_ctrl_read(15)

            if done & 0x1:
                break
        
        err = (done >>1) & 0x01

        return err

    def memtester_read_len(self):

        lower = self.memtest_ctrl_read(11)
        upper = self.memtest_ctrl_read(12)

        output = upper << 32 | lower

        return output
    
    def memtester_read_cnt(self):

        lower = self.memtest_ctrl_read(13)
        upper = self.memtest_ctrl_read(14)

        output = upper << 32 | lower

        return output
    
    def memtester_pattern(self, wr, rd):

        patt = wr | (rd << 16)

        self.memtest_ctrl_write(17, patt)
