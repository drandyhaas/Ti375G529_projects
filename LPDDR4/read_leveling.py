import logging
import time

from excp import FatalException

LOGGER = logging.getLogger('ddr_cali_tools')


class read_leveling:

    vref_r1_list = [21.20, 21.50, 21.80, 22.10,
                    22.40, 22.70, 23.00, 23.30,
                    23.60, 23.90, 24.20, 24.50,
                    24.80, 25.10, 25.40, 25.70,
                    26.00, 26.30, 26.60, 26.90,
                    27.20, 27.50, 27.80, 28.10,
                    28.40, 28.70, 29.00, 29.30,
                    29.60, 29.90, 30.20, 30.50,
                    30.80, 31.10, 31.40, 31.70,
                    32.00, 32.30, 32.60, 32.90,
                    33.20, 33.50, 33.80, 34.10,
                    34.40, 34.70, 35.00, 35.30,
                    35.60, 35.90, 36.20, 36.50,
                    36.80, 37.10, 37.40, 37.70,
                    38.00, 38.30, 38.60, 38.90,
                    39.20, 39.50, 39.80, 40.10,
                    40.40, 40.70, 41.00, 41.30,
                    41.60, 41.90, 42.20, 42.50,
                    42.80, 43.10, 43.40, 43.70,
                    44.00, 44.30, 44.60, 44.90,
                    45.20, 45.50, 45.80, 46.10,
                    46.40, 46.70, 47.00, 47.30,
                    47.60, 47.90, 48.20, 48.50,
                    48.80, 49.10, 49.40, 49.70,
                    50.00, 50.30, 50.60, 50.90,
                    51.20, 51.50, 51.80, 52.10,
                    52.40, 52.70, 53.00, 53.30,
                    53.60, 53.90, 54.20, 54.50,
                    54.80, 55.10, 55.40, 55.70,
                    56.00, 56.30, 56.60, 56.90,
                    57.20, 57.50, 57.80, 58.10,
                    58.40, 58.70, 59.00, 59.30]

    vref_r0_list = [11.60, 11.90, 12.20, 12.50,
                    12.80, 13.10, 13.40, 13.70,
                    14.00, 14.30, 14.60, 14.90,
                    15.20, 15.50, 15.80, 16.10,
                    16.40, 16.70, 17.00, 17.30,
                    17.60, 17.90, 18.20, 18.50,
                    18.80, 19.10, 19.40, 19.70,
                    20.00, 20.30, 20.60, 20.90,
                    21.20, 21.50, 21.80, 22.10,
                    22.40, 22.70, 23.00, 23.30,
                    23.60, 23.90, 24.20, 24.50,
                    24.80, 25.10, 25.40, 25.70,
                    26.00, 26.30, 26.60, 26.90,
                    27.20, 27.50, 27.80, 28.10,
                    28.40, 28.70, 29.00, 29.30,
                    29.60, 29.90, 30.20, 30.50,
                    30.80, 31.10, 31.40, 31.70,
                    32.00, 32.30, 32.60, 32.90,
                    33.20, 33.50, 33.80, 34.10,
                    34.40, 34.70, 35.00, 35.30,
                    35.60, 35.90, 36.20, 36.50,
                    36.80, 37.10, 37.40, 37.70,
                    38.00, 38.30, 38.60, 38.90,
                    39.20, 39.50, 39.80, 40.10,
                    40.40, 40.70, 41.00, 41.30,
                    41.60, 41.90, 42.20, 42.50,
                    42.80, 43.10, 43.40, 43.70,
                    44.00, 44.30, 44.60, 44.90,
                    45.20, 45.50, 45.80, 46.10,
                    46.40, 46.70, 47.00, 47.30,
                    47.60, 47.90, 48.20, 48.50,
                    48.80, 49.10, 49.40, 49.70]

    def __init__(self, drv_obj, freq: int):
        self.drv_obj = drv_obj
        self.freq = freq
        self.step = 1/freq*1000000/512

    def read_leveling_enable(self):

        # Enable the PI data eye training module.
        # Bit(1) represents the support when non-initialization.
        # Bit(0)represents the support when initialization. Set to 1 to enable.

        output = self.drv_obj.lpddr4_ctrl_read('PI', 128)
        output = output | (0x01 << 1)  # PI_RDLVL_EN_F0
        output = output | (0x01 << 17)  # PI_RDLVL_EN_F1

        self.drv_obj.lpddr4_ctrl_write('PI', 128, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 129)
        output = output | (0x01 << 1)  # PI_RDLVL_EN_F2
        self.drv_obj.lpddr4_ctrl_write('PI', 129, output)

    def read_leveling_cs(self, cs):
        cs_map = cs
        output = self.drv_obj.lpddr4_ctrl_read('PI', 36)
        output = output | ((cs_map & 0x3) << 24)  # PI_RDLVL_CS
        self.drv_obj.lpddr4_ctrl_write('PI', 36, output)

    def read_leveling_cs_map(self, cs):
        cs_map = 0x01 << cs
        output = self.drv_obj.lpddr4_ctrl_read('PI', 47)
        output = output | ((cs_map & 0xF) << 16)  # PI_RDLVL_CS
        self.drv_obj.lpddr4_ctrl_write('PI', 47, output)

    def read_leveling_req(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 36)
        output = output | (0x1 << 8)  # PI_RDLVL_REQ
        self.drv_obj.lpddr4_ctrl_write('PI', 36, output)

    def read_leveling_start_pattern(self, val):
        output = self.drv_obj.lpddr4_ctrl_read('PI', 53)
        output = output & 0xFFF0FFFF
        output = output | (val & 0xF << 16)  # PI_RDLVL_PATTERN_START
        self.drv_obj.lpddr4_ctrl_write('PI', 53, output)

    def read_leveling_number_pattern(self, val):
        output = self.drv_obj.lpddr4_ctrl_read('PI', 53)
        output = output & 0xF0FFFFFF
        output = output | (val & 0xF << 24)  # PI_RDLVL_PATTERN_NUM
        self.drv_obj.lpddr4_ctrl_write('PI', 53, output)

    def read_leveling_multi_pattern_enable(self):
        output = self.drv_obj.lpddr4_ctrl_read('PI', 131)
        output = output & 0xFFFDFFFF
        output = output | (0x01 << 17)  # PI_RDLVL_MULTI_EN_F2
        self.drv_obj.lpddr4_ctrl_write('PI', 131, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 130)
        output = output & 0xFFFDFFFD
        output = output | (0x01 << 1)  # PI_RDLVL_MULTI_EN_F0
        output = output | (0x01 << 17)  # PI_RDLVL_MULTI_EN_F1
        self.drv_obj.lpddr4_ctrl_write('PI', 130, output)

    def read_leveling_pattern0_enable(self):
        output = self.drv_obj.lpddr4_ctrl_read('PI', 129)
        output = (output & 0xFFFDFFFF) | (0x01 << 17)  # PI_RDLVL_PAT0_EN_F0
        self.drv_obj.lpddr4_ctrl_write('PI', 129, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 130)
        output = (output & 0xFFFFFDFF) | (0x01 << 9)  # PI_RDLVL_PAT0_EN_F1
        self.drv_obj.lpddr4_ctrl_write('PI', 130, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 131)
        output = (output & 0xFFFFFFFD) | (0x01 << 1)  # PI_RDLVL_PAT0_EN_F2
        self.drv_obj.lpddr4_ctrl_write('PI', 131, output)

    def read_leveling_pattern_data(self, pattern, pat_data: int):
        self.drv_obj.lpddr4_ctrl_write('PI', (37+pattern), pat_data)

    def poll_read_leveling_status(self):

        timeout = 0

        while (1):
            PI_INT_STATUS = self.drv_obj.lpddr4_ctrl_read('PI', 77)
            read_lvl_done = (PI_INT_STATUS >> 17) & 0x01

            if read_lvl_done:
                return

            if timeout > 6:
                raise FatalException("Read leveling time out")

            time.sleep(0.5)
            timeout = timeout + 1

    def clean_read_leveling_status(self):

        PI_INT_ACK = self.drv_obj.lpddr4_ctrl_read('PI', 78)

        PI_INT_ACK = PI_INT_ACK | (0x01 << 1)
        PI_INT_ACK = PI_INT_ACK | (0x01 << 7)
        PI_INT_ACK = PI_INT_ACK | (0x01 << 17)

        self.drv_obj.lpddr4_ctrl_write('PI', 78, PI_INT_ACK)

    def check_read_leveling_status(self):
        PI_INT_STATUS = self.drv_obj.lpddr4_ctrl_read('PI', 77)

        read_lvl_err = (PI_INT_STATUS >> 1) & 0x01
        read_lvl_req = (PI_INT_STATUS >> 7) & 0x01
        read_lvl_done = (PI_INT_STATUS >> 17) & 0x01

        LOGGER.info(f'read_lvl_err = {read_lvl_err}')
        LOGGER.info(f'read_lvl_req = {read_lvl_req}')
        LOGGER.info(f'read_lvl_done ={read_lvl_done}')

    def read_read_leveling_obs(self):
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 34)
        LOGGER.info(f'DENALI_PHY_34 = {hex(output)}')
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 290)
        LOGGER.info(f'DENALI_PHY_290 = {hex(output)}')
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 546)
        LOGGER.info(f'DENALI_PHY_546 = {hex(output)}')
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 802)
        LOGGER.info(f'DENALI_PHY_802 = {hex(output)}')

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 60)
        LOGGER.info(f'DENALI_PHY_60 = {hex(output)}')
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 316)
        LOGGER.info(f'DENALI_PHY_316 = {hex(output)}')
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 572)
        LOGGER.info(f'DENALI_PHY_572 = {hex(output)}')
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 828)
        LOGGER.info(f'DENALI_PHY_828 = {hex(output)}')

    def read_read_leveling_rddqs_rise_delay_obs(self, cs, slice_mask, cali_file):
        '''
        # Selects which DQ/DM bit is the source for the phy_rdlvl_rddqs_le/te_dly_obs parameters
        # 5’h0 – DQ0 rising edge
        # 5’h1 – DQ1 rising edge
        # 5’h2 – DQ2 rising edge
        # 5’h3 – DQ3 rising edge
        # 5’h4 – DQ4 rising edge
        # 5’h5 – DQ5 rising edge
        # 5’h6 – DQ6 rising edge
        # 5’h7 – DQ7 rising edge
        # 5’h8 – DQ0 falling edge
        # 5’h9 – DQ1 falling edge
        # 5’hA – DQ2 falling edge
        # 5’hB – DQ3 falling edge
        # 5’hC – DQ4 falling edge
        # 5’hD – DQ5 falling edge
        # 5’hE – DQ6 falling edge
        # 5’hF – DQ7 falling edge
        # 5’h10 – DM rising edge
        # 5’h18 – DM falling edge
        '''
        dqs_input_delay = []
        DQ_le_list = []
        DQ_te_list = []
        window = []
        LOGGER.info(f'Rising Read leveling result in (ps)')
        LOGGER.info(f'||===== DQ =====||===== MIN =====||===== MAX ====||=== CENTER ===||== EYE WIDTH ==||== EYE WIDTH% ==||')

        for n in range(4):
            if slice_mask & (0x01 << n):
                for x in range(8):
                    output = self.drv_obj.lpddr4_ctrl_read('PHY', 34+(n*256))
                    output = output & 0xFFE0FFFF
                    output = output | (x << 16)

                    self.drv_obj.lpddr4_ctrl_write('PHY', 34+(n*256), output)
                    output = self.drv_obj.lpddr4_ctrl_read('PHY', 58+(n*256))
                    DQ_te = (output >> 16) & 0x3FF
                    DQ_le = (output) & 0x3FF
                    window.append(DQ_te-DQ_le)
                    valid_window = round((window[x+(n*8)]*self.step), 2)

                    mini_chart = '['
                    for q in range(0, 1023, 16):
                        if (q >= DQ_le) & (q <= DQ_te):
                            mini_chart += 'o'
                        else:
                            mini_chart += '-'

                    dqs_input_delay.append(int((DQ_te+DQ_le)/2))
                    DQ_le_list.append(DQ_le)
                    DQ_te_list.append(DQ_te)

                    start_delay = round((DQ_le*self.step), 2)
                    end_delay = round((DQ_te*self.step), 2)
                    center = round(((start_delay+end_delay)/2), 2)

                    mini_chart += ']'
                    
                    LOGGER.info(f'||   DQ{x+(n*8)} \t||\t{start_delay}\t ||\t{end_delay}\t ||\t{center}\t ||\t{valid_window}\t  ||\t  {int(((DQ_te-DQ_le)/256)*100)}\t    ||'.ljust(50)+mini_chart)
                    #LOGGER.info(
                    #    f'DQ{x+(n*8)} Rising Start = {start_delay} ps End = {end_delay} ps valid_window = {valid_window} ps'.ljust(90)+mini_chart)

            else:
                for x in range(8):
                    dqs_input_delay.append(int(0))
                    DQ_le_list.append(int(0))
                    DQ_te_list.append(int(0))
                    window.append(int(0))

        cali_file = self.update_rdlvl_rise_cali_file(cs, slice_mask, cali_file, DQ_le_list, DQ_te_list)

        for n in range(4):
            if slice_mask & (0x01 << n):
                for x in range(8):
                    if ((window[x+(n*8)] <= 0) | (window[x+(n*8)] > 320)):
                        raise Exception(f"Read Leveling result Rising DQ{x+(n*8)} fail")

        return [dqs_input_delay, cali_file]

    def read_read_leveling_rddqs_fall_delay_obs(self, cs, slice_mask, cali_file):
        '''
        # Selects which DQ/DM bit is the source for the phy_rdlvl_rddqs_le/te_dly_obs parameters
        # 5’h0 – DQ0 rising edge
        # 5’h1 – DQ1 rising edge
        # 5’h2 – DQ2 rising edge
        # 5’h3 – DQ3 rising edge
        # 5’h4 – DQ4 rising edge
        # 5’h5 – DQ5 rising edge
        # 5’h6 – DQ6 rising edge
        # 5’h7 – DQ7 rising edge
        # 5’h8 – DQ0 falling edge
        # 5’h9 – DQ1 falling edge
        # 5’hA – DQ2 falling edge
        # 5’hB – DQ3 falling edge
        # 5’hC – DQ4 falling edge
        # 5’hD – DQ5 falling edge
        # 5’hE – DQ6 falling edge
        # 5’hF – DQ7 falling edge
        # 5’h10 – DM rising edge
        # 5’h18 – DM falling edge
        '''
        dqs_input_delay = []
        DQ_le_list = []
        DQ_te_list = []
        window = []

        LOGGER.info(f'Falling Read leveling result in (ps)')
        LOGGER.info(f'||===== DQ =====||===== MIN =====||===== MAX ====||=== CENTER ===||== EYE WIDTH ==||== EYE WIDTH% ==||')

        for n in range(4):
            if slice_mask & (0x01 << n):
                for x in range(8):
                    output = self.drv_obj.lpddr4_ctrl_read('PHY', 34+(n*256))
                    output = output & 0xFFE0FFFF
                    output = output | ((x+8) << 16)

                    self.drv_obj.lpddr4_ctrl_write('PHY', 34+(n*256), output)
                    output = self.drv_obj.lpddr4_ctrl_read('PHY', 58+(n*256))
                    DQ_te = (output >> 16) & 0x3FF
                    DQ_le = (output) & 0x3FF
                    window.append(DQ_te-DQ_le)
                    valid_window = round((window[x+(n*8)]*self.step), 2)

                    mini_chart = '['
                    for q in range(0, 1023, 16):
                        if (q >= DQ_le) & (q <= DQ_te):
                            mini_chart += 'o'
                        else:
                            mini_chart += '-'

                    dqs_input_delay.append(int((DQ_te+DQ_le)/2))
                    DQ_le_list.append(DQ_le)
                    DQ_te_list.append(DQ_te)

                    start_delay = round((DQ_le*self.step), 2)
                    end_delay = round((DQ_te*self.step), 2)
                    center = round(((start_delay+end_delay)/2), 2)

                    mini_chart += ']'
                    #LOGGER.info(
                    #    f'DQ{x+(n*8)} Falling Start = {start_delay} ps End = {end_delay} ps valid_window = {valid_window} ps'.ljust(90)+mini_chart)
                    LOGGER.info(f'||   DQ{x+(n*8)} \t||\t{start_delay}\t ||\t{end_delay}\t ||\t{center}\t ||\t{valid_window}\t  ||\t  {int(((DQ_te-DQ_le)/256)*100)}\t    ||'.ljust(50)+mini_chart)
            else:
                for x in range(8):
                    dqs_input_delay.append(int(0))
                    DQ_le_list.append(int(0))
                    DQ_te_list.append(int(0))
                    window.append(int(0))

        cali_file = self.update_rdlvl_fall_cali_file(cs, slice_mask, cali_file, DQ_le_list, DQ_te_list)

        for n in range(4):
            if slice_mask & (0x01 << n):
                for x in range(8):
                    if ((window[x+(n*8)] <= 0) | (window[x+(n*8)] > 320)):
                        raise Exception(f"Read Leveling result falling DQ{x+(n*8)} fail")


        return [dqs_input_delay, cali_file]

    def read_read_leveling_rddqs_rise_delay(self, slice_mask):
        '''
        # Selects which DQ/DM bit is the source for the phy_rdlvl_rddqs_le/te_dly_obs parameters
        # 5’h0 – DQ0 rising edge
        # 5’h1 – DQ1 rising edge
        # 5’h2 – DQ2 rising edge
        # 5’h3 – DQ3 rising edge
        # 5’h4 – DQ4 rising edge
        # 5’h5 – DQ5 rising edge
        # 5’h6 – DQ6 rising edge
        # 5’h7 – DQ7 rising edge
        # 5’h8 – DQ0 falling edge
        # 5’h9 – DQ1 falling edge
        # 5’hA – DQ2 falling edge
        # 5’hB – DQ3 falling edge
        # 5’hC – DQ4 falling edge
        # 5’hD – DQ5 falling edge
        # 5’hE – DQ6 falling edge
        # 5’hF – DQ7 falling edge
        # 5’h10 – DM rising edge
        # 5’h18 – DM falling edge
        '''
        dqs_input_delay = []

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                # DQ0
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 132+(slice*256))
                output = (output >> 8) & 0x03FF
                dqs_input_delay.append(output)
                # LOGGER.info(f'phy_rddqs_dq0_rise_slave_delay_{slice} Delay = {output}')

                # DQ1 to DQ7
                for dq in range(7):
                    output = self.drv_obj.lpddr4_ctrl_read(
                        'PHY', 133+dq+(slice*256))
                    output = (output >> 16) & 0x03FF
                    dqs_input_delay.append(output)
                    # LOGGER.info(f'phy_rddqs_dq{dq+1}_rise_slave_delay_{slice} Delay = {output}')

        return dqs_input_delay

    def read_read_leveling_rddqs_fall_delay(self, slice_mask):
        '''
        # Selects which DQ/DM bit is the source for the phy_rdlvl_rddqs_le/te_dly_obs parameters
        # 5’h0 – DQ0 rising edge
        # 5’h1 – DQ1 rising edge
        # 5’h2 – DQ2 rising edge
        # 5’h3 – DQ3 rising edge
        # 5’h4 – DQ4 rising edge
        # 5’h5 – DQ5 rising edge
        # 5’h6 – DQ6 rising edge
        # 5’h7 – DQ7 rising edge
        # 5’h8 – DQ0 falling edge
        # 5’h9 – DQ1 falling edge
        # 5’hA – DQ2 falling edge
        # 5’hB – DQ3 falling edge
        # 5’hC – DQ4 falling edge
        # 5’hD – DQ5 falling edge
        # 5’hE – DQ6 falling edge
        # 5’hF – DQ7 falling edge
        # 5’h10 – DM rising edge
        # 5’h18 – DM falling edge
        '''
        dqs_input_delay = []

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                for dq in range(8):
                    output = self.drv_obj.lpddr4_ctrl_read(
                        'PHY', 133+dq+(slice*256))
                    output = output & 0x03FF
                    dqs_input_delay.append(output)
                    # LOGGER.info(f'phy_rddqs_dq{dq}_fall_slave_delay_{slice} Delay = {output}')

        return dqs_input_delay

    def update_phy_rddqs_dqx_rise_slave_delay(self, slice_mask, delay_list):

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                # DQ0
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 132+(slice*256))
                output = output & ((~(0x03FF << 8)) & 0xFFFFFFFF)
                output = output | (delay_list[0+(slice*8)] << 8)
                self.drv_obj.lpddr4_ctrl_write('PHY', 132+(slice*256), output)

                # DQ1 to DQ7
                for dq in range(7):
                    output = self.drv_obj.lpddr4_ctrl_read(
                        'PHY', 133+dq+(slice*256))
                    output = output & ((~(0x03FF << 16)) & 0xFFFFFFFF)
                    output = output | (delay_list[dq+1+(slice*8)] << 16)
                    self.drv_obj.lpddr4_ctrl_write(
                        'PHY', 133+dq+(slice*256), output)

        self.update_slave_delay()

    def update_phy_rddqs_dqx_fall_slave_delay(self, slice_mask, delay_list):

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                # DQ0 to DQ7
                for dq in range(8):
                    output = self.drv_obj.lpddr4_ctrl_read(
                        'PHY', 133+dq+(slice*256))
                    output = output & ((~0x03FF) & 0xFFFFFFFF)
                    output = output | (delay_list[dq+(slice*8)])
                    self.drv_obj.lpddr4_ctrl_write(
                        'PHY', 133+dq+(slice*256), output)

        self.update_slave_delay()

    def update_rddqY_slave_delay(self, slice_mask, dq_input_delay):
        for n in range(4):
            if slice_mask & (1 << n):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 103+(256*n))
                output = output & 0xFC00FFFF
                output = output | ((dq_input_delay[0] & 0x3FF) << 16)
                self.drv_obj.lpddr4_ctrl_write('PHY', 103+(256*n), output)

                output = self.drv_obj.lpddr4_ctrl_read('PHY', 104+(256*n))
                output = output & 0xFFFFFC00
                output = output | ((dq_input_delay[1] & 0x3FF) << 0)
                self.drv_obj.lpddr4_ctrl_write('PHY', 104+(256*n), output)

                output = self.drv_obj.lpddr4_ctrl_read('PHY', 104+(256*n))
                output = output & 0xFC00FFFF
                output = output | ((dq_input_delay[2] & 0x3FF) << 16)
                self.drv_obj.lpddr4_ctrl_write('PHY', 104+(256*n), output)

                output = self.drv_obj.lpddr4_ctrl_read('PHY', 105+(256*n))
                output = output & 0xFFFFFC00
                output = output | ((dq_input_delay[3] & 0x3FF) << 0)
                self.drv_obj.lpddr4_ctrl_write('PHY', 105+(256*n), output)

                output = self.drv_obj.lpddr4_ctrl_read('PHY', 105+(256*n))
                output = output & 0xFC00FFFF
                output = output | ((dq_input_delay[4] & 0x3FF) << 16)
                self.drv_obj.lpddr4_ctrl_write('PHY', 105+(256*n), output)

                output = self.drv_obj.lpddr4_ctrl_read('PHY', 106+(256*n))
                output = output & 0xFFFFFC00
                output = output | ((dq_input_delay[5] & 0x3FF) << 0)
                self.drv_obj.lpddr4_ctrl_write('PHY', 106+(256*n), output)

                output = self.drv_obj.lpddr4_ctrl_read('PHY', 106+(256*n))
                output = output & 0xFC00FFFF
                output = output | ((dq_input_delay[6] & 0x3FF) << 16)
                self.drv_obj.lpddr4_ctrl_write('PHY', 106+(256*n), output)

                output = self.drv_obj.lpddr4_ctrl_read('PHY', 107+(256*n))
                output = output & 0xFFFFFC00
                output = output | ((dq_input_delay[7] & 0x3FF) << 0)
                self.drv_obj.lpddr4_ctrl_write('PHY', 107+(256*n), output)

        self.update_slave_delay()

    def read_rddqY_slave_delay(self, slice):

        delay = []

        output = (self.drv_obj.lpddr4_ctrl_read(
            'PHY', 103+(256*slice)) >> 16) & 0x3FF
        delay.append(output)
        LOGGER.info(f'DQ0 Delay = {output}')
        output = (self.drv_obj.lpddr4_ctrl_read(
            'PHY', 104+(256*slice))) & 0x3FF
        delay.append(output)
        LOGGER.info(f'DQ1 Delay = {output}')
        output = (self.drv_obj.lpddr4_ctrl_read(
            'PHY', 104+(256*slice)) >> 16) & 0x3FF
        delay.append(output)
        LOGGER.info(f'DQ2 Delay = {output}')
        output = (self.drv_obj.lpddr4_ctrl_read(
            'PHY', 105+(256*slice))) & 0x3FF
        delay.append(output)
        LOGGER.info(f'DQ3 Delay = {output}')
        output = (self.drv_obj.lpddr4_ctrl_read(
            'PHY', 105+(256*slice)) >> 16) & 0x3FF
        delay.append(output)
        LOGGER.info(f'DQ4 Delay = {output}')
        output = (self.drv_obj.lpddr4_ctrl_read(
            'PHY', 106+(256*slice))) & 0x3FF
        delay.append(output)
        LOGGER.info(f'DQ5 Delay = {output}')
        output = (self.drv_obj.lpddr4_ctrl_read(
            'PHY', 106+(256*slice)) >> 16) & 0x3FF
        delay.append(output)
        LOGGER.info(f'DQ6 Delay = {output}')
        output = (self.drv_obj.lpddr4_ctrl_read(
            'PHY', 107+(256*slice))) & 0x3FF
        delay.append(output)
        LOGGER.info(f'DQ7 Delay = {output}')

        return delay

    def update_slave_delay(self):

        # [8] Manual update selection of all slave delay line settings. Set 1 to assert phyupd req and wait phyupd ack to update delay line, set 0 to update delay line directly.
        # [0] Manual update of all slave delay line settings. Set to 1 to trigger. WRITE-ONLY

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1285)
        output = (output | 0x11)
        self.drv_obj.lpddr4_ctrl_write('PHY', 1285, output)

    def set_phy_vref_training_ctrl_enable(self, slice_mask):
        '''
        # Controls the enable of VREF training. There is a phy_vref_training_ctrl_X parameter for each of the slices of data sent on the DFI data bus.
        # • Bit [1] = Frequency of VREF training, only valid when bit [0] is set to 1
        # -- 'b0 = Perform VREF training the first time dfi_lvl_pattern = 0 is encountered
        # -- 'b1 = Perform VREF training every time dfi_lvl_pattern = 0 is encountered
        # • Bit [0] = Enables automatic VREF training when dfi_lvl_pattern = 0
        '''
        for slice in range(4):
            if slice_mask == (1 << slice):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 98+(256*slice))
                output = (output & 0xFFFFFCFF) | (0x01 << 8)
                self.drv_obj.lpddr4_ctrl_write('PHY', 98+(256*slice), output)

        return

    def set_phy_vref_training_ctrl_disable(self, slice_mask):
        '''
        # Controls the enable of VREF training. There is a phy_vref_training_ctrl_X parameter for each of the slices of data sent on the DFI data bus.
        # • Bit [1] = Frequency of VREF training, only valid when bit [0] is set to 1
        # -- 'b0 = Perform VREF training the first time dfi_lvl_pattern = 0 is encountered
        # -- 'b1 = Perform VREF training every time dfi_lvl_pattern = 0 is encountered
        # • Bit [0] = Enables automatic VREF training when dfi_lvl_pattern = 0
        '''
        for slice in range(4):
            if slice_mask == (1 << slice):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 98+(256*slice))
                output = (output & 0xFFFFFCFF)
                self.drv_obj.lpddr4_ctrl_write('PHY', 98+(256*slice), output)

        return

    def set_phy_vref_initial_start_stop_point(self, start, stop):
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 97+(256*slice))
        output = (output & 0x80FFFFFF) | (start << 24)
        self.drv_obj.lpddr4_ctrl_write('PHY', 97+(256*slice), output)

        output = self.drv_obj.lpddr4_ctrl_read('PHY', 98+(256*slice))
        output = (output & 0xFFFFFF80) | (stop << 0)
        self.drv_obj.lpddr4_ctrl_write('PHY', 98+(256*slice), output)

        return

    def read_phy_vref_train_obs(self, slice_mask):

        vref_list = []

        LOGGER.info('Vref Training Observation')

        rng = self.check_device_vref_range(slice_mask)

        for slice in range(4):
            if slice_mask & (1 << slice):
                output = ((self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 13+(256*slice))) >> 16) & 0x7F

                if rng[slice] == 0:
                    LOGGER.info(
                        f'Slice{slice} FPGA Vref Result = {self.vref_r0_list[output]}% Range ={rng[slice]}')
                else:
                    LOGGER.info(
                        f'Slice{slice} FPGA Vref Result = {self.vref_r1_list[output]}% Range ={rng[slice]}')
                vref_list.append(output)
            else:
                vref_list.append(0)

        return vref_list

    def set_phy_pad_vref_ctrl_dq(self, slice_mask, verf_list):

        for slice in range(4):
            if slice_mask & (1 << slice):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 111+(256*slice))
                output = ((output & 0xFF80FFFF) | (
                    (verf_list[slice] & 0x7F) << 16))
                self.drv_obj.lpddr4_ctrl_write('PHY', 111+(256*slice), output)

        return

    def check_device_vref_range(self, slice_mask):

        dev_list = []
        rng_list = []

        for slice in range(4):
            if slice_mask & (1 << slice):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 111+(256*slice))
                output = (output >> 24) & 0xF
                if output == 0x7:  # 0111
                    device = "LPDDR4"
                    rng = 0
                elif output == 0x6:  # 0110
                    device = "LPDDR4"
                    rng = 1
                elif output == 0x9:  # 1001
                    device = "LPDDR4x"
                    rng = 0
                elif output == 0xA:  # 1010
                    device = "LPDDR4x"
                    rng = 1
                else:
                    device = None
                    rng = None

                dev_list.append(device)
                rng_list.append(rng)
            else:
                dev_list.append(None)
                rng_list.append(None)

        return rng_list

    def update_rdlvl_rise_cali_file(self, cs, slice_mask, file, le_delay, re_delay):

        cs = cs & 0x3

        for slice in range(4):
            if (slice_mask & (0x1 << slice)):
                for dq in range(8):
                    file[f'cs{0}'][f'slice_{slice}']['input']['left_edge']['rise'][f'dq{dq}'] = le_delay[dq+(
                        slice*8)]
                    file[f'cs{0}'][f'slice_{slice}']['input']['right_edge']['rise'][f'dq{dq}'] = re_delay[dq+(
                        slice*8)]

        return file

    def update_rdlvl_fall_cali_file(self, cs, slice_mask, file, le_delay, re_delay):

        cs = cs & 0x3

        for slice in range(4):
            if (slice_mask & (0x1 << slice)):
                for dq in range(8):
                    file[f'cs{0}'][f'slice_{slice}']['input']['left_edge']['fall'][f'dq{dq}'] = le_delay[dq+(
                        slice*8)]
                    file[f'cs{0}'][f'slice_{slice}']['input']['right_edge']['fall'][f'dq{dq}'] = re_delay[dq+(
                        slice*8)]

        return file

    def update_rdlvl_fpga_vref_cali_file(self, cs, slice_mask, file, vref):

        cs = cs & 0x3

        rng = self.check_device_vref_range(slice_mask)

        for slice in range(4):
            if (slice_mask & (0x1 << slice)):
                if rng[slice] == 0:
                    file[f'cs{0}'][f'slice_{slice}']['vref'] = self.vref_r0_list[vref[slice]]
                else:
                    file[f'cs{0}'][f'slice_{slice}']['vref'] = self.vref_r1_list[vref[slice]]

        return file

    def run_read_leveling(self, cs, slice_mask, cali_file):

        self.set_phy_vref_training_ctrl_enable(slice_mask)  # FPGA Vref Cali with Rdlvl Enable
        self.clean_read_leveling_status()
        self.read_leveling_start_pattern(0)
        self.read_leveling_number_pattern(8)
        self.read_leveling_pattern0_enable()
        self.read_leveling_multi_pattern_enable()
        self.read_leveling_enable()
        self.read_leveling_cs(cs)
        self.read_leveling_cs_map(cs)
        self.read_leveling_req()
        self.poll_read_leveling_status()

        [rise_delay, cali_file] = self.read_read_leveling_rddqs_rise_delay_obs(cs, slice_mask, cali_file)
        [fall_delay, cali_file] = self.read_read_leveling_rddqs_fall_delay_obs(cs, slice_mask, cali_file)

        self.update_phy_rddqs_dqx_rise_slave_delay(slice_mask, rise_delay)
        self.update_phy_rddqs_dqx_fall_slave_delay(slice_mask, fall_delay)

        vref_list = self.read_phy_vref_train_obs(slice_mask)
        self.set_phy_pad_vref_ctrl_dq(slice_mask, vref_list)

        cali_file = self.update_rdlvl_fpga_vref_cali_file(
            cs, slice_mask, cali_file, vref_list)

        LOGGER.info('')

        return cali_file
