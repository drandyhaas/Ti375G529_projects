import logging
import time

from excp import FatalException

LOGGER = logging.getLogger('ddr_cali_tools')


class write_leveling:

    def __init__(self, drv_obj, freq: int):
        self.drv_obj = drv_obj
        self.freq = freq
        self.step = 1/freq*1000000/512

    phy_wrlvl_status_obs = [
        'return data from DRAM',
        'write zero found',
        'write one found',
        'hard0',
        'hard1',
        'error',
        'wrlvl_state'
    ]

    def write_leveling_enable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 122)
        output = output | (0x01 << 17)  # PI_WRLVL_EN_F0
        output = output | (0x01 << 25)  # PI_WRLVL_EN_F1

        self.drv_obj.lpddr4_ctrl_write('PI', 122, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 123)
        output = output | (0x01 << 1)  # PI_WRLVL_EN_F2
        self.drv_obj.lpddr4_ctrl_write('PI', 123, output)

    def write_leveling_disable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 122)
        output = output & 0xFFFCFFFF  # PI_WRLVL_EN_F0
        output = output & 0xFCFFFFFF  # PI_WRLVL_EN_F1

        self.drv_obj.lpddr4_ctrl_write('PI', 122, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 123)
        output = output & 0xFFFFFFFC  # PI_WRLVL_EN_F2
        self.drv_obj.lpddr4_ctrl_write('PI', 123, output)

    def write_leveling_cs(self, cs):
        cs_map = cs  # [1:0] for target of CS[x], e.g:cs=1 cs1 enable
        output = self.drv_obj.lpddr4_ctrl_read('PI', 29)
        output = output | ((cs_map & 0x3) << 8)  # PI_WRLVL_CS
        self.drv_obj.lpddr4_ctrl_write('PI', 29, output)

    def write_leveling_cs_map(self, cs):
        cs_map = 0x01 << cs  # [3:0] CS mask [0] for CS[0],[1] for CS[1]
        output = self.drv_obj.lpddr4_ctrl_read('PI', 31)
        output = output | ((cs_map & 0xF) << 24)  # PI_WRLVL_CS_MAP
        self.drv_obj.lpddr4_ctrl_write('PI', 31, output)

    def write_leveling_req(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 29)
        output = output | (0x1 << 0)  # PI_WRLVL_REQ
        self.drv_obj.lpddr4_ctrl_write('PI', 29, output)

    def poll_write_leveling_status(self):

        timeout = 0

        while (1):
            PI_INT_STATUS = self.drv_obj.lpddr4_ctrl_read('PI', 77)
            read_lvl_done = (PI_INT_STATUS >> 18) & 0x01

            if read_lvl_done:
                return

            if timeout > 6:
                raise FatalException("Write leveling time out")

            time.sleep(0.5)
            timeout = timeout + 1

    def clean_write_leveling_status(self):

        PI_INT_ACK = self.drv_obj.lpddr4_ctrl_read('PI', 78)

        PI_INT_ACK = PI_INT_ACK | (0x01 << 3)  # PI_WRLVL_ERROR_BIT
        PI_INT_ACK = PI_INT_ACK | (0x01 << 9)  # PI_WRLVL_REQ_BIT
        PI_INT_ACK = PI_INT_ACK | (0x01 << 18)  # PI_WRLVL_DONE_BIT

        self.drv_obj.lpddr4_ctrl_write('PI', 78, PI_INT_ACK)

    def read_write_leveling_wrdqs_delay(self, slice_mask):

        delay_list = []

        for x in range(4):
            if (slice_mask & (1 << x)):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 131+(x*256))
                output = (output >> 16) & 0x3FF
                delay_list.append(output)
                # LOGGER.info(f'PHY_CLK_WRDQS_SLAVE_DELAY_{x} = {output}')

                LOGGER.info(
                    f'Write DQS Delay Slice{x} = {round(output*self.step,2)}ps')

        return delay_list

    def update_phy_clk_wrdqs_slave_delay(self, slice_mask, delay_list):

        for x in range(4):
            if (slice_mask & (1 << x)):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 131+(x*256))
                output = output & ((~(0x3FF << 16)) & 0xFFFFFFFF)
                output = output | (int(delay_list[x]) << 16)
                self.drv_obj.lpddr4_ctrl_write('PHY', 131+(x*256), output)

        self.update_slave_delay()

    def read_write_leveling_wrdqs_delay_bypass(self, slice_mask):
        for x in range(4):
            if (slice_mask & (1 << x)):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 1+(x*256))
                output = (output >> 16) & 0x3FF
                LOGGER.info(f'PHY_CLK_WRDQS_SLAVE_DELAY_BYPASS{x} = {output}')

    def read_write_leveling_path_latency(self, slice_mask):
        for x in range(4):
            if (slice_mask & (1 << x)):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 151+(x*256))
                output = (output >> 8) & 0x7
                LOGGER.info(f'PHY_WRITE_PATH_LAT_ADD_{x} = {output}')

    def read_write_leveling_error_obs(self, slice_mask):
        for x in range(4):
            if (slice_mask & (1 << x)):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 55+(x*256))
                output = (output >> 0) & 0xFFFF
                LOGGER.info(f'PHY_WRLVL_ERROR_OBS{x} = {output}')

    def read_write_leveling_hard_delay_obs(self, slice_mask):

        delay = []

        for x in range(4):
            wrdqs_delay = 0

            if (slice_mask & (1 << x)):
                find0 = self.drv_obj.lpddr4_ctrl_read('PHY', 51+(x*256))
                find0 = (find0 >> 16) & 0x3FF
                # LOGGER.info(f'PHY_WRLVL_HARD0_DELAY_OBS_{x} = {find0}')

                find1 = self.drv_obj.lpddr4_ctrl_read('PHY', 52+(x*256))
                find1 = (find1 >> 0) & 0x3FF
                # LOGGER.info(f'PHY_WRLVL_HARD1_DELAY_OBS_{x} = {find1}')
                wrdqs_delay = ((find0+find1)/2)

            delay.append(int(wrdqs_delay))

        return delay

    def read_write_leveling_status_obs(self, slice_mask):
        for x in range(4):
            if (slice_mask & (1 << x)):
                output = self.drv_obj.lpddr4_ctrl_read('PHY', 53+(x*256))

                LOGGER.info(
                    f'slice{x} {self.phy_wrlvl_status_obs[0]} = {output & 0xFF}')
                LOGGER.info(
                    f'slice{x} {self.phy_wrlvl_status_obs[1]} = {(output>>8) & 0x01}')
                LOGGER.info(
                    f'slice{x} {self.phy_wrlvl_status_obs[2]} = {(output>>9) & 0x01}')
                LOGGER.info(
                    f'slice{x} {self.phy_wrlvl_status_obs[3]} = {(output>>10) & 0x01}')
                LOGGER.info(
                    f'slice{x} {self.phy_wrlvl_status_obs[4]} = {(output>>11) & 0x01}')
                LOGGER.info(
                    f'slice{x} {self.phy_wrlvl_status_obs[5]} = {(output>>12) & 0x01}')
                LOGGER.info(
                    f'slice{x} {self.phy_wrlvl_status_obs[6]} = {(output>>13) & 0x0F}')

    def update_slave_delay(self):
        output = self.drv_obj.lpddr4_ctrl_read('PHY', 1285)
        output = (output | 0x11)
        self.drv_obj.lpddr4_ctrl_write('PHY', 1285, output)

    def write_leveling_training_multicast(self, en):
        for x in range(4):
            output = self.drv_obj.lpddr4_ctrl_read('PHY', 0x06+(x*256))

            if en:
                output = output | (1 << 8)
            else:
                output = output & 0xFFFFFEFF

            self.drv_obj.lpddr4_ctrl_write('PHY', 0x06+(x*256), output)

    def write_leveling_training_prerank_index(self, cs):
        for x in range(4):
            output = self.drv_obj.lpddr4_ctrl_read('PHY', 0x06+(x*256))

            output = ((output & 0xFFFCFFFF) | (cs << 16))

            self.drv_obj.lpddr4_ctrl_write('PHY', 0x06+(x*256), output)

    def update_wrlvl_cali_file(self, cs, slice_mask, file, le_delay, re_delay):

        for slice in range(4):
            if (slice_mask & (0x1 << slice)):
                file[f'cs{0}'][f'slice_{slice}']['output']['left_edge']['dqs'] = le_delay[slice]
                file[f'cs{0}'][f'slice_{slice}']['output']['right_edge']['dqs'] = re_delay[slice]
                
        return file

    def run_write_leveling(self, cs, slice_mask, cali_file):
        self.write_leveling_training_multicast(1)
        self.clean_write_leveling_status()
        # self.read_write_leveling_wrdqs_delay(slice_mask)
        # self.read_write_leveling_path_latency(slice_mask)
        self.write_leveling_cs_map(cs)
        self.write_leveling_cs(cs)
        self.write_leveling_disable()
        self.write_leveling_enable()
        self.write_leveling_req()
        self.poll_write_leveling_status()
        delay = self.read_write_leveling_hard_delay_obs(slice_mask)
        # self.read_write_leveling_status_obs(slice_mask)
        self.update_phy_clk_wrdqs_slave_delay(slice_mask, delay)
        self.read_write_leveling_wrdqs_delay(slice_mask)
        cali_file = self.update_wrlvl_cali_file(cs, slice_mask, cali_file, delay, delay)

        return cali_file
