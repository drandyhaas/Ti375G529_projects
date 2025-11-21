import sys
import time
import math
import logging
from excp import FatalException

LOGGER = logging.getLogger('ddr_cali_tools')


class initialization:

    def __init__(self, drv_obj):
        self.drv_obj = drv_obj

    def extract_pcr_write_pattern(self, file: str):
        phy_data = []
        pi_data = []
        ctl_data = []

        ctl_addr = []
        phy_addr = []
        pi_addr = []

        phy_reg = []
        pi_reg = []
        ctl_reg = []

        line_buff = []
        index_cnt = 0
        list_cnt = 0
        out_hex = 0

        with open(file, 'r') as f:  # open file for reading
            for line in f:

                index_addr_ctl = line.find("//DENALI_CTL_")
                index_addr_pi = line.find("//DENALI_PI_")
                index_addr_phy = line.find("//DENALI_PHY_")

                if index_addr_ctl != -1:
                    # extract the text after "8'b" and remove semicolon and any whitespace
                    extracted_text = line[index_addr_ctl+13:].strip()
                    extracted_text = int(extracted_text.replace('_DATA', ''))
                    ctl_addr.append(extracted_text)

                if index_addr_pi != -1:
                    # extract the text after "8'b" and remove semicolon and any whitespace
                    extracted_text = line[index_addr_pi+12:].strip()
                    extracted_text = int(extracted_text.replace('_DATA', ''))
                    pi_addr.append(extracted_text)

                if index_addr_phy != -1:
                    # extract the text after "8'b" and remove semicolon and any whitespace
                    extracted_text = line[index_addr_phy+13:].strip()
                    extracted_text = int(extracted_text.replace('_DATA', ''))
                    phy_addr.append(extracted_text)

                index = line.find("8'b")  # find the index of "8'b" in the line

                if index != -1:

                    # extract the text after "8'b" and remove semicolon and any whitespace
                    extracted_text = line[index+3:].strip()
                    extracted_text = extracted_text.replace(';', '')
                    extracted_text = int(extracted_text, 2)
                    line_buff.append(extracted_text)
                    index_cnt = index_cnt+1

                    if ((index_cnt % 4) == 0):
                        out_hex = ((line_buff[3] << 24) | (line_buff[2] << 16) | (
                            line_buff[1] << 8) | (line_buff[0]))

                        if list_cnt < len(ctl_addr):
                            ctl_data.append(out_hex)
                        elif list_cnt < (len(ctl_addr)+len(pi_addr)):
                            pi_data.append(out_hex)
                        else:
                            phy_data.append(out_hex)

                        line_buff.clear()
                        list_cnt = list_cnt+1
                        index_cnt = 0
        f.close()

        # wr_data_temp=[]

        for i in range(len(ctl_addr)):
            ctl_reg.append([ctl_addr[i], ctl_data[i]])
            # LOGGER.info(f'CTL address = {ctl_addr[i]} data = {hex(ctl_data[i])}')

        for i in range(len(pi_addr)):
            pi_reg.append([pi_addr[i], pi_data[i]])
            # LOGGER.info(f'PI address = {pi_addr[i]} data = {hex(pi_data[i])}')

        for i in range(len(phy_addr)):
            phy_reg.append([phy_addr[i], phy_data[i]])
            # LOGGER.info(f'PHY address = {phy_addr[i]} data = {hex(phy_data[i])}')

        return ctl_reg, pi_reg, phy_reg

    def dump_current_config(self, ctl_reg: list, pi_reg: list, phy_reg: list):

        for i in range(len(ctl_reg)):
            temp = self.drv_obj.lpddr4_ctrl_read('CTL', ctl_reg[i][0])
            LOGGER.info(f'DENALI_CTL_{ctl_reg[i][0]} = {hex(temp)}')

        for i in range(len(pi_reg)):
            temp = self.drv_obj.lpddr4_ctrl_read('PI', pi_reg[i][0])
            LOGGER.info(f'DENALI_PI_{pi_reg[i][0]} = {hex(temp)}')

        for i in range(len(phy_reg)):
            temp = self.drv_obj.lpddr4_ctrl_read('PHY', phy_reg[i][0])
            LOGGER.info(f'DENALI_PHY_{phy_reg[i][0]} = {hex(temp)}')

        return

    def dump_current_ctl_config(self, ctl_reg: list):

        for i in range(len(ctl_reg)):
            temp = self.drv_obj.lpddr4_ctrl_read('CTL', ctl_reg[i][0])
            LOGGER.info(f'DENALI_CTL_{ctl_reg[i][0]} = {hex(temp)}')

        return

    def dump_current_pi_config(self, pi_reg: list):

        for i in range(len(pi_reg)):
            temp = self.drv_obj.lpddr4_ctrl_read('PI', pi_reg[i][0])
            LOGGER.info(f'DENALI_PI_{pi_reg[i][0]} = {hex(temp)}')

        return

    def dump_current_phy_config(self, phy_reg: list):

        list_buf = []

        for i in range(len(phy_reg)):
            temp = self.drv_obj.lpddr4_ctrl_read('PHY', phy_reg[i][0])
            list_buf.append(temp)
            LOGGER.info(f'DENALI_PHY_{phy_reg[i][0]} = {hex(temp)}')

        return list_buf

    def init_config_file(self, ctl_reg: list, pi_reg: list, phy_reg: list):
        import time

        LOGGER.info('Write CTL Register')
        start_time = time.time()
        for i in range(len(ctl_reg)):
            write_start = time.time()
            self.drv_obj.lpddr4_ctrl_write('CTL', ctl_reg[i][0], ctl_reg[i][1])
            write_time = (time.time() - write_start) * 1000  # Convert to ms
            if write_time > 10.0:  # Report any write that takes > 10ms
                LOGGER.info(f'  SLOW: CTL[{i}] addr=0x{ctl_reg[i][0]:04X} took {write_time:.1f}ms')
            elif i % 50 == 0:
                LOGGER.info(f'  CTL[{i}/{len(ctl_reg)}] last write: {write_time:.1f}ms')
        ctl_time = time.time() - start_time
        LOGGER.info(f'Wrote {len(ctl_reg)} CTL registers in {ctl_time:.2f}s')

        LOGGER.info('Write PHY Register')
        start_time = time.time()
        for i in range(len(phy_reg)):
            if i % 50 == 0:
                LOGGER.info(f'  Writing PHY register {i}/{len(phy_reg)}...')
            self.drv_obj.lpddr4_ctrl_write('PHY', phy_reg[i][0], phy_reg[i][1])
        phy_time = time.time() - start_time
        LOGGER.info(f'Wrote {len(phy_reg)} PHY registers in {phy_time:.2f}s')

        LOGGER.info('Write PI Register')
        start_time = time.time()
        for i in range(len(pi_reg)):
            if i % 50 == 0:
                LOGGER.info(f'  Writing PI register {i}/{len(pi_reg)}...')
            self.drv_obj.lpddr4_ctrl_write('PI', pi_reg[i][0], pi_reg[i][1])
        pi_time = time.time() - start_time
        LOGGER.info(f'Wrote {len(pi_reg)} PI registers in {pi_time:.2f}s')

        total_time = ctl_time + phy_time + pi_time
        LOGGER.info(f'Total initialization time: {total_time:.2f}s')

        return True

    class mask:

        ca_training_cs = 0xF0FFFFFF
        ca_training_en = 0xFFFCFCFC
        wr_leveling_cs = 0xF0FFFFFF
        wr_leveling_en_f01 = 0xFCFCFFFF
        wr_leveling_en_f2 = 0xFFFFFFFC
        gate_leveling_cs = 0xF0FFFFFF
        gate_leveling_en_f01 = 0xFCFFFCFF
        gate_leveling_en_f2 = 0xFFFFFCFF
        read_leveling_cs = 0xFFF0FFFF
        read_leveling_en_f01 = 0xFFFCFFFC
        read_leveling_en_f2 = 0xFFFFFFFC
        writedq_leveling_cs = 0xFFF0FFFF
        writedq_leveling_en_f0 = 0xFFFFFCFF
        writedq_leveling_en_f1 = 0xFCFFFFFF
        writedq_leveling_en_f2 = 0xFFFFFCFF
        pi_cs_map = 0xFFFFFFF0
        ctrl_cs_sel = 0xFFFFFCFF
        ctrl_no_mrs_init = 0xFFFEFFFF
        ctrl_indep_init = 0xFFFEFFFF
        ctrl_indep_tran = 0xFFFFFFFE
        start_pi = 0xFFFFFFFE
        start_mem = 0xFFFFFFFE
        mc_init_status = 0xFFFFFFEF
        pi_init_status = 0xFFFFFFFE
        phy_pll_lock = 0xFFFEFFFE

    def ca_training_en(self, cs_map: int, en: bool):

        # DENALI_PI_57 PI_CALVL_CS_MAP:RW:24:4:=0x0f PI_CALVL_ROTATE:RW:16:1:=0x00 PI_CALVL_DISABLE_DFS:RW:8:1:=0x00 PI_CALVL_ON_SREF_EXIT:RW:0:1:=0x00
        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 57) & self.mask.ca_training_cs

        if cs_map & 0x01:
            output = output | 0x01000000
        if cs_map & 0x02:
            output = output | 0x02000000
        if cs_map & 0x04:
            output = output | 0x04000000
        if cs_map & 0x08:
            output = output | 0x08000000

        self.drv_obj.lpddr4_ctrl_write('PI', 57, output)

        # DENALI_PI_127 PI_TMRZ_F0:RW:24:5:=0x01 PI_CALVL_EN_F2:RW:16:2:=0x01 PI_CALVL_EN_F1:RW:8:2:=0x01 PI_CALVL_EN_F0:RW:0:2:=0x01
        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 137) & self.mask.ca_training_en

        if en:
            output = output | ((~self.mask.ca_training_en) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 137, output)

        return

    def wr_leveling_en(self, cs_map: int, en: bool):

        # DENALI_PI_31 PI_WRLVL_CS_MAP:RW:24:4:=0x0f PI_WRLVL_ROTATE:RW:16:1:=0x00 PI_WRLVL_RESP_MASK:RW:8:4:=0x00 PI_WRLVL_DISABLE_DFS:RW:0:1:=0x00
        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 31) & self.mask.wr_leveling_cs

        if cs_map & 0x01:
            output = output | 0x01000000
        if cs_map & 0x02:
            output = output | 0x02000000
        if cs_map & 0x04:
            output = output | 0x04000000
        if cs_map & 0x08:
            output = output | 0x08000000

        self.drv_obj.lpddr4_ctrl_write('PI', 31, output)

        # DENALI_PI_122 PI_WRLVL_EN_F1:RW:24:2:=0x01 PI_WRLVL_EN_F0:RW:16:2:=0x01 PI_TDFI_CTRL_DELAY_F2:RW_D:8:4:=0x04 PI_TDFI_CTRL_DELAY_F1:RW_D:0:4:=0x04
        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 122) & self.mask.wr_leveling_en_f01

        if en:
            output = output | ((~self.mask.wr_leveling_en_f01) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 122, output)

        # DENALI_PI_123 PI_TDFI_WRLVL_WW_F0:RW:8:10:=0x001e PI_WRLVL_EN_F2:RW:0:2:=0x01
        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 123) & self.mask.wr_leveling_en_f2

        if en:
            output = output | ((~self.mask.wr_leveling_en_f2) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 123, output)

        return

    def gate_leveling_en(self, cs_map: int, en: bool):

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 47) & self.mask.gate_leveling_cs

        if cs_map & 0x01:
            output = output | 0x01000000
        if cs_map & 0x02:
            output = output | 0x02000000
        if cs_map & 0x04:
            output = output | 0x04000000
        if cs_map & 0x08:
            output = output | 0x08000000

        self.drv_obj.lpddr4_ctrl_write('PI', 47, output)

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 128) & self.mask.gate_leveling_en_f01

        if en:
            output = output | ((~self.mask.gate_leveling_en_f01) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 128, output)

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 129) & self.mask.gate_leveling_en_f2

        if en:
            output = output | ((~self.mask.gate_leveling_en_f2) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 129, output)

        return

    def read_leveling_en(self, cs_map: int, en: bool):

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 47) & self.mask.read_leveling_cs

        if cs_map & 0x01:
            output = output | 0x00010000
        if cs_map & 0x02:
            output = output | 0x00020000
        if cs_map & 0x04:
            output = output | 0x00040000
        if cs_map & 0x08:
            output = output | 0x00080000

        self.drv_obj.lpddr4_ctrl_write('PI', 47, output)

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 128) & self.mask.read_leveling_en_f01

        if en:
            output = output | ((~self.mask.read_leveling_en_f01) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 128, output)

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 129) & self.mask.read_leveling_en_f2

        if en:
            output = output | ((~self.mask.read_leveling_en_f2) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 129, output)

        return

    def writedq_leveling_en(self, cs_map: int, en: bool):

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 67) & self.mask.writedq_leveling_cs

        if cs_map & 0x01:
            output = output | 0x00010000
        if cs_map & 0x02:
            output = output | 0x00020000
        if cs_map & 0x04:
            output = output | 0x00040000
        if cs_map & 0x08:
            output = output | 0x00080000

        self.drv_obj.lpddr4_ctrl_write('PI', 67, output)

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 157) & self.mask.writedq_leveling_en_f0

        if en:
            output = output | (
                (~self.mask.writedq_leveling_en_f0) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 157, output)

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 159) & self.mask.writedq_leveling_en_f1

        if en:
            output = output | (
                (~self.mask.writedq_leveling_en_f1) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 159, output)

        output = self.drv_obj.lpddr4_ctrl_read(
            'PI', 162) & self.mask.writedq_leveling_en_f2

        if en:
            output = output | (
                (~self.mask.writedq_leveling_en_f2) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('PI', 162, output)

        return

    def pi_cs_map(self, cs_map: int):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 11) & self.mask.pi_cs_map

        if cs_map & 0x01:
            output = output | 0x00000001
        if cs_map & 0x02:
            output = output | 0x00000002
        if cs_map & 0x04:
            output = output | 0x00000004
        if cs_map & 0x08:
            output = output | 0x00000008

        self.drv_obj.lpddr4_ctrl_write('PI', 11, output)

        return

    def ctrl_cs_select(self, cs: int):

        cs &= 0x03
        output = self.drv_obj.lpddr4_ctrl_read(
            'CTL', 223) & self.mask.ctrl_cs_sel
        output = output | (cs << 8)
        self.drv_obj.lpddr4_ctrl_write('CTL', 223, output)

        return

    def no_mrw_init(self, en: bool):

        output = self.drv_obj.lpddr4_ctrl_read(
            'CTL', 18) & self.mask.ctrl_no_mrs_init
        if en:
            output = output | ((~self.mask.ctrl_no_mrs_init) & 0xFFFFFFFF)
        self.drv_obj.lpddr4_ctrl_write('CTL', 18, output)

        return

    def mc_phy_indep_mode(self, trainning: bool, init: bool):

        output = self.drv_obj.lpddr4_ctrl_read(
            'CTL', 19) & self.mask.ctrl_indep_init & self.mask.ctrl_indep_tran

        if init:
            output = output | ((~self.mask.ctrl_indep_init) & 0xFFFFFFFF)
        if trainning:
            output = output | ((~self.mask.ctrl_indep_tran) & 0xFFFFFFFF)

        self.drv_obj.lpddr4_ctrl_write('CTL', 19, output)

        return

    def start_pi_controller(self, enable):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 0) & self.mask.start_pi
        if enable:
            output = output | ((~self.mask.start_pi) & 0xFFFFFFFF)
        else:
            output = output & 0xFFFFFFFF
        self.drv_obj.lpddr4_ctrl_write('PI', 0, output)

        return

    def start_mem_controller(self, enable):

        output = self.drv_obj.lpddr4_ctrl_read('CTL', 0) & self.mask.start_mem
        if enable:
            output = output | ((~self.mask.start_mem) & 0xFFFFFFFF)
        else:
            output = output & 0xFFFFFFFF

        self.drv_obj.lpddr4_ctrl_write('CTL', 0, output)

        return

    def mmc_init_status(self):

        output = self.drv_obj.lpddr4_ctrl_read('CTL', 235)

        if output & ((~self.mask.mc_init_status) & 0xFFFFFFFF):
            return True
        else:
            return False

    def pi_init_status(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 77)

        if output & ((~self.mask.pi_init_status) & 0xFFFFFFFF):
            return True
        else:
            return False

    DENALI_PI_77 = [
        'PI_INIT_DONE_BIT', 'PI_RDLVL_ERROR_BIT', 'PI_RDLVL_GATE_ERROR_BIT', 'PI_WRLVL_ERROR_BIT', 'PI_CALVL_ERROR_BIT', 'PI_WDQLVL_ERROR_BIT', 'PI_UPDATE_ERROR_BIT', 'PI_RDLVL_REQ_BIT', 'PI_RDLVL_GATE_REQ_BIT', 'PI_WRLVL_REQ_BIT', 'PI_CALVL_REQ_BIT', 'PI_WDQLVL_REQ_BIT', 'PI_LVL_DONE_BIT', 'PI_BIST_DONE_BIT', 'PI_TDFI_INIT_TIME_OUT_BIT', 'PI_DLL_LOCK_STATE_CHANGE_BIT', 'PI_RDLVL_GATE_DONE_BIT', 'PI_RDLVL_DONE_BIT', 'PI_WRLVL_DONE_BIT', 'PI_CALVL_DONE_BIT', 'PI_WDQLVL_DONE_BIT', 'ANY_VALID_BIT'
    ]

    def pi_interrupt_status(self):

        status = []

        output = self.drv_obj.lpddr4_ctrl_read('PI', 77)

        for index, value in enumerate(self.DENALI_PI_77):
            if output & (1 << index):
                status.append(True)
            else:
                status.append(False)
        #    LOGGER.info(f'{value} \t\t ={status[index]}')

        return status

    def clean_pi_interrupt_status(self):
        self.drv_obj.lpddr4_ctrl_write('PI', 78, 0xFFFFFFFF)

    def check_phy_pll_lock(self):

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1314)

        if output & ((~self.mask.phy_pll_lock) & 0xFFFFFFFF):
            return True
        else:
            return False

    def check_phy_pi_lock(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 103)

        if output & 0x01:
            return True
        else:
            return False

    def reset_pi_controller(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 10)
        output = output & 0xFFFEFFFF
        self.drv_obj.lpddr4_ctrl_write('PI', 10, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 10)
        output = (output & 0xFFFEFFFF) | (1 << 16)
        self.drv_obj.lpddr4_ctrl_write('PI', 10, output)

    def init_controller(self, cali_enable: bool, cs_map: int):

        self.start_pi_controller(0)
        self.start_mem_controller(0)

        self.ca_training_en(cs_map, cali_enable)
        self.wr_leveling_en(cs_map, cali_enable)
        self.gate_leveling_en(cs_map, cali_enable)
        self.read_leveling_en(cs_map, cali_enable)
        self.writedq_leveling_en(cs_map, cali_enable)

        self.pi_cs_map(cs_map)
        self.ctrl_cs_select(cs_map)
        self.no_mrw_init(0)
        self.mc_phy_indep_mode(1, 1)
        self.start_pi_controller(1)
        self.start_mem_controller(1)

        time_out = 0
        while True:
            if self.check_phy_pll_lock():
                LOGGER.info('PLL locked')
                time_out = 0
                break

            if time_out > 30:
                time_out = 0
                LOGGER.info('PLL lock time out')
                raise FatalException("PLL lock time out")
                break

            time_out = time_out+1
            time.sleep(0.1)

        while True:
            if self.mmc_init_status():
                LOGGER.info('MC Initialization Done')
                time_out = 0
                break

            if time_out > 30:
                LOGGER.info('Memory controller initial time out')
                raise FatalException("Memory controller initial time out")
                time_out = 0
                break

            time_out = time_out+1
            time.sleep(0.1)

            if self.pi_init_status():
                LOGGER.info('PI Initialization Done')
                time_out = 0
                break

            if time_out > 30:
                LOGGER.info('PI controller initial time out')
                raise FatalException("PI controller initial time out")
                time_out = 0
                break

            time_out = time_out+1
            time.sleep(0.1)

        LOGGER.info(f'PI PLL = {self.check_phy_pi_lock()}')

        status = self.pi_interrupt_status()

        if (status[0] is False):
            raise FatalException("INIT Done Fail")

        return status

    def read_ctl_id(self):

        output = (self.drv_obj.lpddr4_ctrl_read('CTL', 0))

        out = ((output >> 16) & 0xFFFF)

        return out

    def read_pi_id(self):

        output = (self.drv_obj.lpddr4_ctrl_read('PI', 0))
        output = ((output >> 16) & 0xFFFF)

        return output

    def flush_phyreg_fifo(self):

        self.drv_obj.memtest_ctrl_write(8, 255)

        for n in range(8):
            self.drv_obj.lpddr4_ctrl_read('PHY', n)

        self.drv_obj.memtest_ctrl_write(8, 0)

    def initial(self, file: str, cali_enable: bool, cs_map: int):

        ctl_reg = []
        pi_reg = []
        phy_reg = []
        LOGGER.info(f'cs mapping = {cs_map}')
        # LOGGER.info('extract config file')
        [ctl_reg, pi_reg, phy_reg] = self.extract_pcr_write_pattern(file)
        # LOGGER.info('init config')

        self.init_config_file(ctl_reg, pi_reg, phy_reg)

        self.reset_pi_controller()
        # LOGGER.info('init ctrl')

        if (cs_map == 0x1) | (cs_map == 0x2) | (cs_map == 0x4) | (cs_map == 0x8):
            self.drv_obj.x16_mode_test_enable()
        else:
            self.drv_obj.x16_mode_test_disable()

        status = self.init_controller(cali_enable, cs_map)
        self.flush_phyreg_fifo()

        return status
