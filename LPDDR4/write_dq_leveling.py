import logging
import time

LOGGER = logging.getLogger('ddr_cali_tools')


class write_dq_leveling:

    def __init__(self, drv_obj, freq: int):
        self.drv_obj = drv_obj
        self.freq = freq
        self.step = 1/freq*1000000/512

    def writedq_leveling_enable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 157)
        output = output | (0x01 << 9)  # PI_WDQLVL_EN_F0
        self.drv_obj.lpddr4_ctrl_write('PI', 157, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 159)
        output = output | (0x01 << 25)  # PI_WDQLVL_EN_F1
        self.drv_obj.lpddr4_ctrl_write('PI', 159, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 162)
        output = output | (0x01 << 9)  # PI_WDQLVL_EN_F2
        self.drv_obj.lpddr4_ctrl_write('PI', 162, output)

    def writedq_leveling_disable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 157)
        output = output & 0xFFFFFCFF  # PI_WDQLVL_EN_F0
        self.drv_obj.lpddr4_ctrl_write('PI', 157, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 159)
        output = output & 0xFCFFFFFF  # PI_WDQLVL_EN_F1
        self.drv_obj.lpddr4_ctrl_write('PI', 159, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 162)
        output = output & 0xFFFFFCFF  # PI_WDQLVL_EN_F2
        self.drv_obj.lpddr4_ctrl_write('PI', 162, output)

    def writedq_leveling_verf_enable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 66)
        output = output | (0x01 << 16)  # PI_WDQLVL_VREF_EN
        self.drv_obj.lpddr4_ctrl_write('PI', 66, output)

    def writedq_leveling_verf_disable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 66)
        output = output & 0xFFFEFFFF  # PI_WDQLVL_VREF_EN
        self.drv_obj.lpddr4_ctrl_write('PI', 66, output)

    def writedq_leveling_cs(self, cs):
        cs_map = cs  # [1:0] for target of CS[x], e.g:cs=1 cs1 enable
        output = self.drv_obj.lpddr4_ctrl_read('PI', 68)
        output = output & 0xFCFFFFFF
        output = output | ((cs_map & 0x3) << 24)  # PI_WRLVL_CS
        self.drv_obj.lpddr4_ctrl_write('PI', 68, output)

    def writedq_leveling_cs_map(self, cs):
        cs_map = 0x01 << cs  # [3:0] CS mask [0] for CS[0],[1] for CS[1]
        output = self.drv_obj.lpddr4_ctrl_read('PI', 67)
        output = output & 0xFFF0FFFF
        output = output | ((cs_map & 0xF) << 16)  # PI_WRLVL_CS_MAP
        self.drv_obj.lpddr4_ctrl_write('PI', 67, output)

    def writedq_leveling_req(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 68)
        output = output | (0x1 << 16)  # PI_WRLVL_REQ
        self.drv_obj.lpddr4_ctrl_write('PI', 68, output)

    def poll_writedq_leveling_status(self):

        timeout = 0

        while (1):
            PI_INT_STATUS = self.drv_obj.lpddr4_ctrl_read('PI', 77)
            read_lvl_done = (PI_INT_STATUS >> 20) & 0x01

            if read_lvl_done:
                return

            if timeout > 6:
                raise FatalException("Write DQ leveling time out")

            time.sleep(0.5)
            timeout = timeout + 1

    def clean_writedq_leveling_status(self):

        PI_INT_ACK = self.drv_obj.lpddr4_ctrl_read('PI', 78)

        PI_INT_ACK = PI_INT_ACK | (0x01 << 5)  # PI_WDQLVL_ERROR_BIT
        PI_INT_ACK = PI_INT_ACK | (0x01 << 11)  # PI_WDQLVL_REQ_BIT
        PI_INT_ACK = PI_INT_ACK | (0x01 << 20)  # PI_WDQLVL_DONE_BIT

        self.drv_obj.lpddr4_ctrl_write('PI', 78, PI_INT_ACK)

    def training_multicast(self, en):
        for slice in range(4):
            output = self.drv_obj.lpddr4_ctrl_read('PHY', 0x06+(slice*256))

            if en:
                output = output | (1 << 8)
            else:
                output = output & 0xFFFFFEFF

            self.drv_obj.lpddr4_ctrl_write('PHY', 0x06+(slice*256), output)

    def training_prerank_index(self, cs):
        for slice in range(4):
            output = self.drv_obj.lpddr4_ctrl_read('PHY', 0x06+(slice*256))

            output = ((output & 0xFFFCFFFF) | (cs << 16))

            self.drv_obj.lpddr4_ctrl_write('PHY', 0x06+(slice*256), output)

    def read_writedq_leveling_dqdm_delay_obs(self, cs, slice_mask, cali_file):

        dq_output_delay = []
        le_list = []
        te_list = []
        window = []
        LOGGER.info(f'Write DQ Leveling result in (ps)')
        LOGGER.info(f'||===== DQ =====||===== MIN =====||===== MAX ====||=== CENTER ===||== EYE WIDTH ==||== EYE WIDTH% ==||')

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                for dq in range(9):
                    output = self.drv_obj.lpddr4_ctrl_read(
                        'PHY', 37+(slice*256))
                    output = output & 0xFFFFF0FF
                    output = output | (dq << 8)

                    self.drv_obj.lpddr4_ctrl_write(
                        'PHY', 37+(slice*256), output)

                    output = self.drv_obj.lpddr4_ctrl_read(
                        'PHY', 61+(slice*256))
                    DQ_te = (output >> 16) & 0x3FF
                    DQ_le = (output) & 0x3FF

                    # if DQ_te  < DQ_le :
                    #    DQ_te = DQ_te +1024

                    if DQ_te < DQ_le:
                        window.append(DQ_te-DQ_le+1024)
                    else:
                        window.append(DQ_te-DQ_le)

                    valid_window = round((window[dq+(9*slice)]*self.step), 2)

                    mini_chart = '['

                    if DQ_te < DQ_le:
                        for q in range(0, 1023, 24):
                            if (q > DQ_le) | (q < DQ_te):
                                mini_chart += 'o'
                            else:
                                mini_chart += '-'
                    else:
                        for q in range(0, 1023, 24):
                            if (q > DQ_le) & (q < DQ_te):
                                mini_chart += 'o'
                            else:
                                mini_chart += '-'

                    if DQ_te < DQ_le:
                        DQ_te = int(DQ_te+1024)

                    dq_output_delay.append(int((DQ_te+DQ_le)/2))
                    le_list.append(int(DQ_le))
                    te_list.append(int(DQ_te))

                    mini_chart += ']'

                    start_delay = round((DQ_le*self.step), 2)
                    end_delay = round((DQ_te*self.step), 2)
                    center = round(((start_delay + end_delay)/2), 2)

                    if dq == 8:
                        #LOGGER.info(
                        #    f'DM{slice} Start = {start_delay} ps End = {end_delay} ps valid_window = {valid_window} ps'.ljust(80)+mini_chart)
                            LOGGER.info(f'||   DM{slice} \t||\t{start_delay}\t ||\t{end_delay}\t ||\t{center}\t ||\t{valid_window}\t  ||\t  {int(((DQ_te-DQ_le)/256)*100)}\t    ||'.ljust(50)+mini_chart)
                    else:
                        #LOGGER.info(
                            #f'DQ{dq+(slice*8)} Start = {start_delay} ps End = {end_delay} ps valid_window = {valid_window} ps'.ljust(80)+mini_chart)
                            LOGGER.info(f'||   DQ{dq+(slice*8)} \t||\t{start_delay}\t ||\t{end_delay}\t ||\t{center}\t ||\t{valid_window}\t  ||\t  {int(((DQ_te-DQ_le)/256)*100)}\t    ||'.ljust(50)+mini_chart)
            else:
                for dq in range(9):
                    dq_output_delay.append(int(0))
                    le_list.append(int(0))
                    te_list.append(int(0))
                    window.append(0)

        cali_file = self.update_wdqlvl_cali_file(cs, slice_mask, cali_file, le_list, te_list)

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                for dq in range(8):
                    if ((window[dq+(9*slice)] <= 0) | (window[dq+(9*slice)] > 320)):
                        raise Exception(
                            f"Write DQ Leveling result DQ{slice*8+dq} fail")

                if ((window[(9*slice)] <= 0) | (window[(9*slice)] > 320)):
                    raise Exception(
                        f"Write DQ Leveling result DM{slice} fail")

        return [dq_output_delay, cali_file]

    def read_phy_clk_wrdqx_slave_delay(self, slice_mask):

        dq_output_delay = []

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                output = self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 127+(slice*256))  # DQ0-1
                dq_output_delay.append(output & 0x7FF)
                dq_output_delay.append((output >> 16) & 0x7FF)

                output = self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 128+(slice*256))  # DQ2-3
                dq_output_delay.append(output & 0x7FF)
                dq_output_delay.append((output >> 16) & 0x7FF)

                output = self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 129+(slice*256))  # DQ4-5
                dq_output_delay.append(output & 0x7FF)
                dq_output_delay.append((output >> 16) & 0x7FF)

                output = self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 130+(slice*256))  # DQ6-7
                dq_output_delay.append(output & 0x7FF)
                dq_output_delay.append((output >> 16) & 0x7FF)

                output = self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 131+(slice*256))  # DM
                dq_output_delay.append(output & 0x7FF)

        # for dq in range(len(dq_output_delay)):
        #    LOGGER.info(f'phy_clk_wrdq{dq}_slave_delay = {dq_output_delay[dq]}')

        return dq_output_delay

    # main fuction for shift delay of output write dq
    def write_phy_clk_wrdqx_slave_delay(self, slice_mask, dq_output_delay):

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                self.drv_obj.lpddr4_ctrl_write(
                    'PHY', 127+(slice*256), dq_output_delay[1+(slice*9)] << 16 | dq_output_delay[0+(slice*9)])  # DQ0-1
                self.drv_obj.lpddr4_ctrl_write(
                    'PHY', 128+(slice*256), dq_output_delay[3+(slice*9)] << 16 | dq_output_delay[2+(slice*9)])  # DQ2-3
                self.drv_obj.lpddr4_ctrl_write(
                    'PHY', 129+(slice*256), dq_output_delay[5+(slice*9)] << 16 | dq_output_delay[4+(slice*9)])  # DQ4-5
                self.drv_obj.lpddr4_ctrl_write(
                    'PHY', 130+(slice*256), dq_output_delay[7+(slice*9)] << 16 | dq_output_delay[6+(slice*9)])  # DQ6-7
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 131+(slice*256))
                output = ((output & 0xFFFFF800) | dq_output_delay[8+(slice*9)])
                self.drv_obj.lpddr4_ctrl_write(
                    'PHY', 131+(slice*256), output)  # DM

    def update_slave_delay(self):
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1285)
        output = (output | 0x11)
        self.drv_obj.lpddr4_ctrl_write('PHY', 1285, output)

    def write_wdqlvl_datadm_mask(self, slice_mask, mask):
        '''
        # Defines the per-bit mask for write DQ training. There is a phy_wdqlvl_datadm_mask_X parameter for each of the slices of data sent on the DFI data bus.
        # Bit [8] = DM
        # Bit [7] = DQ7
        # Bit [6] = DQ6
        # Bit [5] = DQ5
        # Bit [4] = DQ4
        # Bit [3] = DQ3
        # Bit [2] = DQ2
        # Bit [1] = DQ1
        # Bit [0] = DQ0
        # For each bit:
        # b0 = Level this bit
        # b1 = Mask this bit from the leveling process
        '''
        mask = mask & 0x1FF

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 38+(slice*256))
                output = output & 0xFE00FFFF
                output = output | (mask << 16)
                self.drv_obj.lpddr4_ctrl_write('PHY', 38+(slice*256), output)

    def write_wdqlvl_patt(self, slice_mask, enable):
        '''
       # Defines the training patterns to be used during the write DQ training sequence. If multiple bits are set
       # the training for each of the chosen patterns will be executed and the settings that give the smallest data valid window eye will bechosen.
       # There is a phy_wdqlvl_patt_X parameter for each of the slices of data sent on the DFI data bus.

        # • Bit [2] = User-defined data pattern training
        # -- ’b0 = Training pattern will NOT be executed
        # -- ’b1 = Training pattern will be executed
        # • Bit [1] = CLK data training pattern
        # -- ’b0 = Training pattern will NOT be executed
        # -- ’b1 = Training pattern will be executed
        # • Bit [0] = LFSR data training pattern
        # -- ’b0 = Training pattern will NOT be executed
        # -- ’b1 = Training pattern will be executed
        '''
        enable = enable & 0x7

        for slice in range(4):
            if slice_mask & (0x01 << slice):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 36+(slice*256))
                output = output & 0xFFFFF8FF
                output = output | (enable << 8)
                self.drv_obj.lpddr4_ctrl_write('PHY', 36+(slice*256), output)

    def update_wdqlvl_cali_file(self, cs, slice_mask, file, le_delay, re_delay):

        cs = cs & 0x3
        for slice in range(4):
            if (slice_mask & (0x1 << slice)):
                for dq in range(8):
                    file[f'cs{0}'][f'slice_{slice}']['output']['left_edge'][f'dq{dq}'] = le_delay[dq+(
                        slice*9)]
                    file[f'cs{0}'][f'slice_{slice}']['output']['right_edge'][f'dq{dq}'] = re_delay[dq+(
                        slice*9)]

                file[f'cs{0}'][f'slice_{slice}']['output']['left_edge']['dm'] = le_delay[(
                    slice*9)+8]
                file[f'cs{0}'][f'slice_{slice}']['output']['right_edge']['dm'] = re_delay[(
                    slice*9)+8]

        return file

    def run_writedq_leveling(self, cs, slice_mask, cali_file):

        # self.training_multicast(True)
        # self.training_prerank_index(cs)
        self.write_wdqlvl_datadm_mask(slice_mask, 0)
        self.write_wdqlvl_patt(slice_mask, 0x7)
        self.clean_writedq_leveling_status()
        self.writedq_leveling_enable()
        self.writedq_leveling_verf_enable()
        self.writedq_leveling_cs(cs)
        self.writedq_leveling_cs_map(cs)
        self.writedq_leveling_req()
        self.poll_writedq_leveling_status()
        [delay, cali_file] = self.read_writedq_leveling_dqdm_delay_obs(cs, slice_mask, cali_file)
        self.write_phy_clk_wrdqx_slave_delay(slice_mask, delay)
        self.update_slave_delay()
        self.read_phy_clk_wrdqx_slave_delay(slice_mask)

        return cali_file
