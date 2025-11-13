import logging

LOGGER = logging.getLogger('ddr_cali_tools')


class mr_control:

    def __init__(self, drv_obj):
        self.drv_obj = drv_obj

    vref_r0_dict = [10.0, 10.4, 10.8, 11.2, 11.6,
                    12.0, 12.4, 12.8, 13.2, 13.6,
                    14.0, 14.4, 14.8, 15.2, 15.6,
                    16.0, 16.4, 16.8, 17.2, 17.6,
                    18.0, 18.4, 18.8, 19.2, 19.6,
                    20.0, 20.4, 20.8, 21.2, 21.6,
                    22.0, 22.4, 22.8, 23.2, 23.6,
                    24.0, 24.4, 24.8, 25.2, 25.6,
                    26.0, 26.4, 26.8, 27.2, 27.6,
                    28.0, 28.4, 28.8, 29.2, 29.6, 30.0]

    vref_r1_dict = [22.0, 22.4, 22.8, 23.2, 23.6,
                    24.0, 24.4, 24.8, 25.2, 25.6,
                    26.0, 26.4, 26.8, 27.2, 27.6,
                    28.0, 28.4, 28.8, 29.2, 29.6,
                    30.0, 30.4, 30.8, 31.2, 31.6,
                    32.0, 32.4, 32.8, 33.2, 33.6,
                    34.0, 34.4, 34.8, 35.2, 35.6,
                    36.0, 36.4, 36.8, 37.2, 37.6,
                    38.0, 38.4, 38.8, 39.2, 39.6,
                    40.0, 40.4, 40.8, 41.2, 41.6, 42.0]

    def mode_register_read(self, cs, mr):

        output = self.drv_obj.lpddr4_ctrl_read('CTL', 142)
        output = output & 0xFE0000FF

        trigger = 1
        READ_MODEREG = ((trigger << 16) | (cs << 8) | mr) & 0x1FFFF
        output = output | (READ_MODEREG << 8)

        self.drv_obj.lpddr4_ctrl_write('CTL', 142, output)

        while (1):
            done = (self.drv_obj.lpddr4_ctrl_read('CTL', 235) >> 25) & 0x01
            if done:
                break

        err = (self.drv_obj.lpddr4_ctrl_read('CTL', 235) >> 16) & 0x1

        if err:
            LOGGER.info('MRR Error')

        rddata = self.drv_obj.lpddr4_ctrl_read('CTL', 143) & 0xFF

        return rddata

    def mode_register_write(self, cs, mr, op):

        if cs == 1:
            output = self.drv_obj.lpddr4_ctrl_read('CTL', 165)
            output = ((output & 0xFFFF00FF) | (op << 8))
            self.drv_obj.lpddr4_ctrl_write('CTL', 165, output)
        else:
            output = self.drv_obj.lpddr4_ctrl_read('CTL', 158)
            output = ((output & 0xFF00FFFF) | (op << 16))
            self.drv_obj.lpddr4_ctrl_write('CTL', 158, output)

        output = self.drv_obj.lpddr4_ctrl_read('CTL', 141)
        output = output & 0xFC7F0000
        all_cs = 0  # Bit [24] = Write all chip selects.
        trigger = 1  # Bit [25] = Trigger the MRW sequence.
        bMRW = 1  # Bit [23] = Write a single MRz

        # Bits [15:8] = Chip select number to be written
        # Bits [7:0] = Mode register number to be written

        READ_MODEREG = ((trigger << 25) | (all_cs << 24)
                        | (bMRW << 23) | (cs << 8) | mr)
        output = (output | READ_MODEREG)
        self.drv_obj.lpddr4_ctrl_write('CTL', 141, output)

        while (1):
            done = (self.drv_obj.lpddr4_ctrl_read('CTL', 235) >> 28) & 0x01
            if done:
                break

        err = (self.drv_obj.lpddr4_ctrl_read('CTL', 142) >> 0) & 0x1

        if err:
            LOGGER.info('MRR Error')

        return err

    def read_vref_ca(self, cs):

        rddata = self.mode_register_read(cs, 12)
        LOGGER.info(f'MR12 = {hex(rddata)}')
        vref = rddata & 0x3F
        vref_rng = (rddata >> 6) & 0x1
        if vref_rng == 0:
            vref = self.vref_r0_dict[vref]
        else:
            vref = self.vref_r1_dict[vref]

        LOGGER.info(f'CS{cs} Vref CA = {vref}% Range = {vref_rng}')

        return [vref_rng, vref]

    def read_vref_dq(self, cs):

        rddata = self.mode_register_read(cs, 14)
        LOGGER.info(f'MR14 = {hex(rddata)}')
        vref = rddata & 0x3F
        vref_rng = (rddata >> 6) & 0x1
        if vref_rng == 0:
            vref = self.vref_r0_dict[vref]
        else:
            vref = self.vref_r1_dict[vref]

        LOGGER.info(f'CS{cs} Vref DQ = {vref}% Range = {vref_rng}')

        return [vref_rng, vref]

    def write_vref_ca(self, cs, vref_rng, vref):

        op = (vref_rng << 6) | vref

        self.mode_register_write(cs, 12, op)

        return

    def write_vref_dq(self, cs, vref_rng, vref):

        op = (vref_rng << 6) | vref

        self.mode_register_write(cs, 14, op)

        return

    def update_verf_ca_cali_file(self, cs, file):

        [vref_rng, vref] = self.read_vref_ca(cs)

        file[f'cs{0}'][f'vrefca_range'] = vref_rng
        file[f'cs{0}'][f'vrefca_val'] = vref

        return file

    def update_verf_dq_cali_file(self, cs, file):

        [vref_rng, vref] = self.read_vref_dq(cs)

        file[f'cs{0}'][f'vrefdq_range'] = vref_rng
        file[f'cs{0}'][f'vrefdq_val'] = vref

        return file
