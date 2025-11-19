import logging
import time
from excp import FatalException

LOGGER = logging.getLogger('ddr_cali_tools')


class ca_training:
    '''
    Class for CA Training feature
    '''

    def __init__(self, drv_obj, freq: int):
        self.drv_obj = drv_obj
        self.freq = freq
        self.step = 1/freq*1000000/512

    def ca_leveling_enable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 137)
        output = output | (0x01 << 1)  # PI_CALVL_EN_F0
        output = output | (0x01 << 9)  # PI_CALVL_EN_F1
        output = output | (0x01 << 17)  # PI_CALVL_EN_F2
        self.drv_obj.lpddr4_ctrl_write('PI', 137, output)

    def ca_leveling_disable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 137)
        output = output & 0xFFFCFCFC
        self.drv_obj.lpddr4_ctrl_write('PI', 137, output)

    def cs_leveling_enable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1285)
        output = output | (0x01 << 24)  # PHY_CSLVL_EN
        self.drv_obj.lpddr4_ctrl_write('PHY', 1285, output)

    def cs_leveling_disable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1285)
        output = output & 0xFEFFFFFF
        self.drv_obj.lpddr4_ctrl_write('PHY', 1285, output)

    def ca_leveling_cs(self, cs):
        cs_map = cs  # [1:0] for target of CS[x], e.g:cs=1 cs1 enable
        output = self.drv_obj.lpddr4_ctrl_read('PI', 55)
        output = output & 0xFCFFFFFF
        output = output | ((cs_map & 0x3) << 24)  # PI_WRLVL_CS
        self.drv_obj.lpddr4_ctrl_write('PI', 55, output)

    def ca_leveling_cs_map(self, cs_map):
        #cs_map = 0x01 << cs  # [1:0] CS mask [0] for CS[0],[1] for CS[1]
        output = self.drv_obj.lpddr4_ctrl_read('PI', 57)
        output = output & 0xFF0FFFFF
        output = output | ((cs_map & 0xF) << 24)  # PI_CALVL_CS_MAP
        self.drv_obj.lpddr4_ctrl_write('PI', 57, output)

    def ca_leveling_req(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 55)
        output = output | (0x1 << 16)  # PI_CALVL_REQ
        self.drv_obj.lpddr4_ctrl_write('PI', 55, output)

    def poll_ca_leveling_status(self):

        timeout = 0

        while (1):
            PI_INT_STATUS = self.drv_obj.lpddr4_ctrl_read('PI', 77)
            read_lvl_done = (PI_INT_STATUS >> 19) & 0x01

            if read_lvl_done:
                return

            if timeout > 6:
                raise FatalException("CA leveling time out")

            time.sleep(0.5)
            timeout = timeout + 1

    def clean_ca_leveling_status(self):

        PI_INT_ACK = self.drv_obj.lpddr4_ctrl_read('PI', 78)

        PI_INT_ACK = PI_INT_ACK | (0x01 << 4)  # PI_CALVL_ERROR_BIT
        PI_INT_ACK = PI_INT_ACK | (0x01 << 10)  # PI_CALVL_REQ_BIT
        PI_INT_ACK = PI_INT_ACK | (0x01 << 19)  # PI_CALVL_DONE_BIT

        self.drv_obj.lpddr4_ctrl_write('PI', 78, PI_INT_ACK)

    def read_ca_leveling_obs(self):

        # ca_output_delay=[]
        ca_le = [0]*6
        ca_re = [0]*6

        for ca in range(6):
            output = self.drv_obj.lpddr4_ctrl_read('PHY', 1040)
            output = output & 0xFFF8FFFF
            output = output | (ca << 16)

            self.drv_obj.lpddr4_ctrl_write('PHY', 1040, output)

            output = self.drv_obj.lpddr4_ctrl_read('PHY', 1041)
            ca_le[ca] = (output >> 16) & 0x7FF
            ca_re[ca] = (output) & 0x7FF
            # valid_window = round((ca_re[ca]-ca_le[ca])*self.step,2)

            # LOGGER.info(f'CA{ca} Left edge = {ca_le[ca]} Right edge = {ca_re[ca]} valid_window = {valid_window} ps'.ljust(70) + '[')

            # left_found  = (output >>28)&0x01
            # right_found = (output >>12)&0x01

            # for q in range(0,2048,24):
            #    if (q > ca_le[ca]) & (q< ca_re[ca]):
            #        LOGGER.info('o')
            #    else:
            #        LOGGER.info('-')

            # if (left_found) & (right_found):
            #    ca_output_delay.append(int((ca_le[ca]+ca_re[ca])/2))
            # else:
            #    ca_output_delay.append(None)

            # LOGGER.info(']')

        return [ca_le, ca_re]

    def write_phy_adrx_clk_wr_slave_delay(self, addr_delay):

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1063)
        output = ((output & 0xFFF800FF) | (addr_delay[0] << 8))
        self.drv_obj.lpddr4_ctrl_write('PHY', 1063, output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1064)
        output = ((output & 0xFFFFF800) | addr_delay[1])
        self.drv_obj.lpddr4_ctrl_write('PHY', 1064, output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1065)
        output = ((output & 0xFFFFF800) | addr_delay[2])
        self.drv_obj.lpddr4_ctrl_write('PHY', 1065, output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1066)
        output = ((output & 0xFFFFF800) | addr_delay[3])
        self.drv_obj.lpddr4_ctrl_write('PHY', 1066, output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1067)
        output = ((output & 0xFFFFF800) | addr_delay[4])
        self.drv_obj.lpddr4_ctrl_write('PHY', 1067, output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1068)
        output = ((output & 0xFFFFF800) | addr_delay[5])
        self.drv_obj.lpddr4_ctrl_write('PHY', 1068, output)

        return addr_delay

    def read_phy_adrx_clk_wr_slave_delay(self):

        addr_delay = []

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1063)
        output = (output >> 8) & 0x7FF
        addr_delay.append(output)
        LOGGER.info(f'ADDR0 = {(output)}')

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1064)
        output = (output >> 0) & 0x7FF
        addr_delay.append(output)
        LOGGER.info(f'ADDR1 = {(output)}')

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1065)
        output = (output >> 0) & 0x7FF
        addr_delay.append(output)
        LOGGER.info(f'ADDR2 = {(output)}')

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1066)
        output = (output >> 0) & 0x7FF
        addr_delay.append(output)
        LOGGER.info(f'ADDR3 = {(output)}')

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1067)
        output = (output >> 0) & 0x7FF
        addr_delay.append(output)
        LOGGER.info(f'ADDR4 = {(output)}')

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1068)
        output = (output >> 0) & 0x7FF
        addr_delay.append(output)
        LOGGER.info(f'ADDR5 = {(output)}')

        return addr_delay

    def read_cs_leveling_obs(self):

        cs_output_delay = []

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1290)
        ca_le = (output >> 16) & 0x7FF
        # +0x410 # magic offset number 0x410 from PHY ug P.95 "reset cslvl_delay to 'h410"
        ca_re = ((output) & 0x7FF)
        valid_window = round((ca_re-ca_le)*self.step, 2)

        left_found = (output >> 28) & 0x01
        right_found = (output >> 12) & 0x01

        mini_chart = '['
        for q in range(0, 2048, 24):
            # if (q > ca_le) & (q< ca_re):
            if (q > ca_re) & (q < ca_le):
                mini_chart += 'o'
            else:
                mini_chart += '-'

        if (left_found) & (right_found):
            cs_output_delay.append(int((ca_le+ca_re)/2))
        else:
            cs_output_delay.append(None)

        mini_chart += ']'
        LOGGER.info(f'CS Left edge = {ca_le} Right edge = {ca_re} valid_window = {valid_window} ps'.ljust(
            10) + mini_chart)

        return cs_output_delay

    def write_phy_grp_slave_delay(self, cs_output_delay):

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1378)
        output = (output & 0xFFFFF800) | cs_output_delay[0]
        self.drv_obj.lpddr4_ctrl_write('PHY', 1378, output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1378)
        output = ((output & 0xF800FFFF) | (cs_output_delay[0] << 16))
        self.drv_obj.lpddr4_ctrl_write('PHY', 1378, output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1379)
        output = (output & 0xFFFFF800) | cs_output_delay[0]
        self.drv_obj.lpddr4_ctrl_write('PHY', 1379, output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1379)
        output = ((output & 0xF800FFFF) | (cs_output_delay[0] << 16))

        self.drv_obj.lpddr4_ctrl_write('PHY', 1379, output)

        return

    def read_phy_grp_slave_delay(self):
        cs_output_delay = []

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1378)
        LOGGER.info(f'phy_grp_slave_delay_0 = {output &0x7FF}')
        cs_output_delay.append(output & 0x7FF)

        LOGGER.info(f'phy_grp_slave_delay_1 = {(output>>16) &0x7FF}')
        cs_output_delay.append((output >> 16) & 0x7FF)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1379)
        LOGGER.info(f'phy_grp_slave_delay_2 = {output &0x7FF}')
        cs_output_delay.append(output & 0x7FF)

        LOGGER.info(f'phy_grp_slave_delay_3 = {(output>>16) &0x7FF}')
        cs_output_delay.append((output >> 16) & 0x7FF)

        return cs_output_delay

    def ca_training_verf_enable(self):
        output = self.drv_obj.lpddr4_ctrl_read('PI', 63)
        output = output | (0x1 << 8)
        self.drv_obj.lpddr4_ctrl_write('PI', 63, output)

    def ca_training_verf_disable(self):
        output = self.drv_obj.lpddr4_ctrl_read('PI', 63)
        output = output & 0xFFFFFEFF
        self.drv_obj.lpddr4_ctrl_write('PI', 63, output)

    def ca_training_number_pattern(self, number):
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1039)
        output = output & 0xFFFFFFFC
        output = output | (number & 0x3)
        self.drv_obj.lpddr4_ctrl_write('PHY', 1039, output)

    def ca_training_set_pattern(self, number, data):
        number = number & 0x3
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1046+(2*number))
        output = output & 0xFFFFFFC0
        output = output | (data & 0x3F)
        self.drv_obj.lpddr4_ctrl_write('PHY', 1046+(2*number), output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1047+(2*number))
        output = output & 0xFFFFFFC0
        output = output | ((~data) & 0x3F)
        self.drv_obj.lpddr4_ctrl_write('PHY', 1047+(2*number), output)

    def update_calvl_cali_file(self, cs, file, le_delay, re_delay):

        cs = cs & 0x3

        for ca in range(6):
            file[f'cs{0}']['slice_ca']['left_edge'][f'ca{ca}'] = le_delay[ca]
            file[f'cs{0}']['slice_ca']['right_edge'][f'ca{ca}'] = re_delay[ca]

        return file

    def run_ca_leveling(self,cs, cs_map, cali_file):

        ca_le = [0]*6
        ca_re = [0]*6
        ca_output_delay = []
        window = []

        # CA Trarining
        self.clean_ca_leveling_status()
        self.ca_training_number_pattern(0)
        self.ca_training_set_pattern(0, 0x55)
        self.ca_training_verf_enable()
        self.cs_leveling_disable()
        self.ca_leveling_enable()
        self.ca_leveling_cs(cs)
        self.ca_leveling_cs_map(cs_map)
        self.ca_leveling_req()
        self.poll_ca_leveling_status()
        self.clean_ca_leveling_status()
        [ca_le0, ca_re0] = self.read_ca_leveling_obs()

        self.ca_training_set_pattern(0, 0xAA)
        self.ca_leveling_req()
        self.poll_ca_leveling_status()
        self.clean_ca_leveling_status()
        [ca_le1, ca_re1] = self.read_ca_leveling_obs()
        LOGGER.info(f'CA Training result in (ps)')
        LOGGER.info(f'||===== CA =====||===== MIN =====||===== MAX ====||=== CENTER ===||== EYE WIDTH ==||== EYE WIDTH% ==||')

        for ca in range(6):
            if ca_le0[ca] > ca_le1[ca]:
                ca_le[ca] = ca_le0[ca]
            else:
                ca_le[ca] = ca_le1[ca]

            if ca_re0[ca] < ca_re1[ca]:
                ca_re[ca] = ca_re0[ca]
            else:
                ca_re[ca] = ca_re1[ca]

            window.append(ca_re[ca]-ca_le[ca])
            valid_window = round((window[ca]*self.step), 2)

            mini_chart = '['
            for q in range(0, 2048, 32):
                if (q >= ca_le[ca]) & (q <= ca_re[ca]):
                    # LOGGER.info('o', extra={'end': ''})
                    mini_chart += 'o'
                else:
                    mini_chart += '-'

            ca_output_delay.append(int((ca_le[ca]+ca_re[ca])/2))
            mini_chart += ']'

            start_delay = round((ca_le[ca]*self.step), 2)
            end_delay = round((ca_re[ca]*self.step), 2)
            center = round(((start_delay+end_delay)/2), 2)

            #LOGGER.info(f'CA{ca} Start = {start_delay} ps End = {end_delay} ps valid_window = {valid_window} ps'.ljust(
            #    70) + mini_chart)
            LOGGER.info(f'||   CA{ca} \t||\t{start_delay}\t ||\t{end_delay}\t ||\t{center}\t ||\t{valid_window}\t  ||\t  {int(((ca_re[ca]-ca_le[ca])/512)*100)}\t    ||'.ljust(50)+mini_chart)
        self.update_calvl_cali_file(cs, cali_file, ca_le, ca_re)

        for ca in range(6):
            if ((window[ca] <= 0) | (window[ca] > 512)):
                raise FatalException(f"CA{ca} Training result fail")

        self.write_phy_adrx_clk_wr_slave_delay(ca_output_delay)

        return cali_file
