import logging
import time
from excp import FatalException

LOGGER = logging.getLogger('ddr_cali_tools')


class gate_leveling:

    '''
    PHY 57
    '''
    phy_gtlvl_status_obs_ = [
        'gate_sample',
        'Reserved',
        'gate_zero_found',
        'gate_one_found',
        'hard0',
        'hard1',
        'gtlvl_error_min',
        'gtlvl_error_max'
    ]

    def __init__(self, drv_obj, freq: int):
        self.drv_obj = drv_obj
        self.freq = freq
        self.step = 1/freq*1000000/512

    def gate_leveling_enable(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 128)
        output = output | (0x01 << 9)  # PI_RDLVL_GATE_EN_F0
        output = output | (0x01 << 25)  # PI_RDLVL_GATE_EN_F1

        self.drv_obj.lpddr4_ctrl_write('PI', 128, output)

        output = self.drv_obj.lpddr4_ctrl_read('PI', 129)
        output = output | (0x01 << 9)  # PI_RDLVL_GATE_EN_F2
        self.drv_obj.lpddr4_ctrl_write('PI', 129, output)

    def gate_leveling_cs(self, cs):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 36)
        output = output | ((cs & 0x3) << 24)  # PI_RDLVL_CS
        self.drv_obj.lpddr4_ctrl_write('PI', 36, output)

    def gate_leveling_cs_map(self, cs):
        cs_map = 0x01 << cs
        output = self.drv_obj.lpddr4_ctrl_read('PI', 47)
        output = output | ((cs_map & 0xF) << 24)  # PI_RDLVL_CS_MAP
        self.drv_obj.lpddr4_ctrl_write('PI', 47, output)

    def gate_leveling_req(self):

        output = self.drv_obj.lpddr4_ctrl_read('PI', 36)
        output = output | (0x1 << 16)  # PI_RDLVL_REQ
        self.drv_obj.lpddr4_ctrl_write('PI', 36, output)

    def clean_gate_leveling_status(self):

        PI_INT_ACK = self.drv_obj.lpddr4_ctrl_read('PI', 78)

        PI_INT_ACK = PI_INT_ACK | (0x01 << 2)  # PI_RDLVL_GATE_ERROR_BIT
        PI_INT_ACK = PI_INT_ACK | (0x01 << 8)  # PI_RDLVL_GATE_REQ_BIT
        PI_INT_ACK = PI_INT_ACK | (0x01 << 16)  # PI_RDLVL_DONE_BIT

        self.drv_obj.lpddr4_ctrl_write('PI', 78, PI_INT_ACK)

    def read_gate_leveling_slave_delay(self, slice_mask):

        delay = []
        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                output = (self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 150+(slicex*256)) >> 16) & 0x3FF
                delay.append(output)
            else:
                delay.append(0)

        return delay

    def read_gate_leveling_slave_lat(self, slice_mask):

        latency = []
        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                output = (self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 151+(slicex*256)) >> 0) & 0xF
                latency.append(output)
            else:
                latency.append(0)

        return latency

    def read_gate_leveling_status_obs(self):
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 57) >> 0) & 0x3FFFF

        for i in range(len(self.phy_gtlvl_status_obs_)):
            LOGGER.info(
                f'{self.phy_gtlvl_status_obs_[i]} = {(output >> i) & 0x01}')

    def read_phy_gate_leveling_delay_obs(self, slice_mask):
        delay = []

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):

                buf0 = ((self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 55+(256*slicex))) >> 16) & 0x3FFF
                # LOGGER.info(f'PHY_GTLVL_HARD0_DELAY_OBS_{slicex} = {buf0}')

                buf1 = (self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 56+(256*slicex)) >> 0) & 0x3FFF
                # LOGGER.info(f'PHY_GTLVL_HARD1_DELAY_OBS_{slicex} = {buf1}')

                rising = 0
                final_step = self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 118+(256*slicex)) & 0x3ff
                # magic equation from PHY ug p.125 'set rdlvl_gate_delay to curr_rise_edge - final_step'
                result = buf1-final_step

                obs = self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 57+(256*slicex)) & 0x3FFFF

                hard0 = (obs >> 4) & 0x1
                hard1 = (obs >> 5) & 0x1

                if (hard0) & (hard1):
                    rising = True
                    delay.append(result)
                else:
                    delay.append(0)

                LOGGER.info(
                    f'Gate DQS Delay slice {slicex} = {round(result*self.step,2)} ps')
            else:
                delay.append(0)

        return delay

    def write_gate_leveling_slave_delay(self, slice_mask, delay):

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                output = (self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 150+(256*slicex))) & 0xFC00FFFF
                self.drv_obj.lpddr4_ctrl_write(
                    'PHY', 150+(slicex*256), output | (delay[slicex] << 16))

    def write_gate_leveling_slave_lat(self, slice_mask, latency):

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                output = (self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 151+(256*slicex))) & 0xFFFFFFF0
                self.drv_obj.lpddr4_ctrl_write(
                    'PHY', 151+(slicex*256), output | latency[slicex])

    def write_phy_rddqs_gate_slave_delayX(self, slice_mask, delay):

        dqs_delay = []
        latency = []

        for slicex in range(4):
            dqs_delay.append(int(delay[slicex] % 512))
            latency.append((int(delay[slicex] // 512) & 0xF))

        self.write_gate_leveling_slave_delay(slice_mask, dqs_delay)
        self.write_gate_leveling_slave_lat(slice_mask, latency)

        # for slice in range(4):
        #    if(slice_mask & (1<<slice)):

        #        result=delay[slice]

        #        output = (self.drv_obj.lpddr4_ctrl_read('PHY',151+(256*slice))) & 0xFFFFFFF0
        #        latency = (result //512)&0xF
        #        self.drv_obj.lpddr4_ctrl_write('PHY',151+(slice*256),output | latency)

        #        dqs_delay = result%512
        #        output = (self.drv_obj.lpddr4_ctrl_read('PHY',150+(256*slice))) & 0xFC00FFFF
        #        self.drv_obj.lpddr4_ctrl_write('PHY',150+(slice*256),output | (dqs_delay<<16))

    def read_phy_rddqs_gate_slave_delayX(self, slice_mask):

        dqs_delay = []
        latency = []
        delay = []

        dqs_delay = self.read_gate_leveling_slave_delay(slice_mask)
        latency = self.read_gate_leveling_slave_lat(slice_mask)

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                delay.append((latency[slicex]*512)+dqs_delay[slicex])
            else:
                delay.append(0)

        return delay

    def poll_gate_leveling_status(self):

        timeout = 0

        while (1):
            PI_INT_STATUS = self.drv_obj.lpddr4_ctrl_read('PI', 77)
            gate_lvl_done = (PI_INT_STATUS >> 16) & 0x01

            if gate_lvl_done:
                return

            if timeout > 6:
                raise FatalException("Gate leveling time out")

            time.sleep(0.5)
            timeout = timeout + 1

    def gate_leveling_debug_enable(self, slicex, enable):
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 32+(slicex*256))
                  & 0xFFFFFFFF)  # PHY LVL DEBUG MODE 0
        if enable:
            self.drv_obj.lpddr4_ctrl_write(
                'PHY', 32+(slicex*256), output | 0x01)
        else:
            self.drv_obj.lpddr4_ctrl_write(
                'PHY', 32+(slicex*256), output & 0xFFFFFFFE)

    def gate_leveling_debug_step_continue(self, slicex):
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 32+(slicex*256))
                  & 0xFFFFFFFF)  # SC_PHY_LVL_DEBUG_CONT_0

        self.drv_obj.lpddr4_ctrl_write('PHY', 32+(slicex*256), output | 0x100)

    def gate_leveling_debug_sample_message(self, slicex):
        output = (self.drv_obj.lpddr4_ctrl_read(
            'PHY', 57+(256*slicex)) >> 0) & 0x1
        LOGGER.info(f'sample = {output}')

    def gate_leveling_adj_latency_start(self, slice_mask, latency):
        '''
        # Defines the initial value for the read DQS gate cycle delay from the slice's base dfi_rddata_en_p0 / dfi_rddata_en_p1 signal during gate training.
        # There is a phy_gtlvl_lat_adj_start_X parameter for each of the slices of data sent on the DFI data bus.
        # Bits [3:0] = Number of cycles delayed from slice base dfi_rddata_en_p0 / dfi_rddata_en_p1.
        '''
        for x in range(4):
            if slice_mask & (1 << x):
                output = (self.drv_obj.lpddr4_ctrl_read(
                    'PHY', 153+(x*256))) & 0xFFF0FFFF
                output = output | ((latency & 0x0F) << 16)
                self.drv_obj.lpddr4_ctrl_write('PHY', 153+(x*256), output)

    def update_gate_leveling_slave_delay(self, cs, data_width):

        return

    def update_gatlvl_cali_file(self, cs, slice_mask, file, le_delay, re_delay):

        cs = cs & 0x3

        for slicex in range(4):
            if (slice_mask & (0x1 << slicex)):
                file[f'cs{0}'][f'slice_{slicex}']['input']['left_edge']['dqs'] = le_delay[slicex]
                file[f'cs{0}'][f'slice_{slicex}']['input']['right_edge']['dqs'] = re_delay[slicex]

        return file

    def run_gate_leveling(self, cs, slice_mask, cail_file):

        self.clean_gate_leveling_status()
        self.gate_leveling_enable()
        self.gate_leveling_cs(cs)
        self.gate_leveling_cs_map(cs)
        self.gate_leveling_req()
        self.poll_gate_leveling_status()
        delay = self.read_phy_gate_leveling_delay_obs(slice_mask)
        self.write_phy_rddqs_gate_slave_delayX(slice_mask, delay)
        self.clean_gate_leveling_status()
        cail_file = self.update_gatlvl_cali_file(cs, slice_mask, cail_file, delay, delay)
        LOGGER.info('')
        return cail_file
