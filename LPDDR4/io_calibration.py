import logging
import time

LOGGER = logging.getLogger('ddr_cali_tools')


class io_calibration:

    def __init__(self, drv_obj):
        self.drv_obj = drv_obj

    #PHY 1334    
    phy_cal_result_obs_0 =[
        'I/O pad PVTP setting'
        ,'I/O pad PVTN setting'
        ,'I/O pad PVTR setting'
        ,'Calibration complete'
    ]

    #PHY 1335
    phy_cal_result_obs_2 =[
        'I/O pad PVTP setting'
        ,'I/O pad PVTN setting'
        ,'I/O pad PVTR setting'
        ,'Calibration complete'
    ]
    #PHY 1342
    phy_cal_result_obs_3 =[
        'first_hard1'
        ,'last_hard0'
        ,'first_hard0'
        ,'last_hard1'
        ,'cur_hard0'
        ,'cur_hard1'
        ,'Reserved'
        ,'state'
    ]
    #PHY 1336
    phy_cal_result_obs_4 =[
        'dfi_phyupd_ack_R'
        ,'phy_phyupd_req'
        ,'holdoff_phyupd_req'
        ,'pass_num'
        ,'shadow_cal'
        ,'Reserved'
    ]
    #PHY 1337
    phy_cal_result_obs_5 =[
        'flag_fore_req'
        ,'flag_donw'
        ,'shadow_cal_pass2'
        ,'Reserved'
    ]
    #PHY 1338
    phy_cal_result_obs_6 =[
        'pass1_pu_within_range'
        ,'pass1_pd_within_range'
        ,'pass1_rx_within_range'
        ,'pass1_within_range'
        ,'pass1_pvtp_pre'
        ,'pass1_pvtn_pre'
        ,'pass1_pvtr_pre'
        ,'Reserved'
    ]
    #PHY 1339
    phy_cal_result_obs_7 =[
        'pass2_pu_within_range'
        ,'pass2_pd_within_range'
        ,'pass2_rx_within_range'
        ,'pass2_within_range'
        ,'pass2_pvtp_pre'
        ,'pass2_pvtn_pre'
        ,'pass2_pvtr_pre'
        ,'Reserved'
    ]

    def read_result_obs_0(self):

        temp = []
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 1334))

        temp.append(output & 0x3F)
        temp.append((output >> 6) & 0x3F)
        temp.append((output >> 12) & 0x3F)
        temp.append((output >> 23) & 0x1)

        for x in range(len(temp)):
            LOGGER.info(f'{self.phy_cal_result_obs_0[x]} = {hex(temp[x])}')

        return output

    def read_result_obs_2(self):

        temp = []
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 1335))

        temp.append(output & 0x3F)
        temp.append((output >> 6) & 0x3F)
        temp.append((output >> 12) & 0x3F)
        temp.append((output >> 23) & 0x1)

        for x in range(len(temp)):
            LOGGER.info(f'{self.phy_cal_result_obs_2[x]} = {hex(temp[x])}')

        return output

    def read_result_obs_3(self):

        temp = []
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 1342))

        temp.append(output & 0x3F)
        temp.append((output >> 6) & 0x3F)
        temp.append((output >> 12) & 0x3F)
        temp.append((output >> 18) & 0x3F)
        temp.append((output >> 24) & 0x01)
        temp.append((output >> 25) & 0x01)
        temp.append((output >> 26) & 0x03)
        temp.append((output >> 28) & 0x0F)

        for x in range(len(temp)):
            LOGGER.info(f'{self.phy_cal_result_obs_3[x]} = {hex(temp[x])}')

        return output

    def read_result_obs_4(self):

        temp = []
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 1336))

        temp.append(output & 0x01)
        temp.append((output >> 1) & 0x01)
        temp.append((output >> 2) & 0x01)
        temp.append((output >> 3) & 0x01)
        temp.append((output >> 4) & 0x1FF)
        temp.append((output >> 21) & 0x07)

        for x in range(len(temp)):
            LOGGER.info(f'{self.phy_cal_result_obs_4[x]} = {hex(temp[x])}')

        return output

    def read_result_obs_5(self):

        temp = []
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 1336))

        temp.append((output >> 2) & 0x01)
        temp.append((output >> 3) & 0x01)
        temp.append((output >> 4) & 0x1FF)
        temp.append((output >> 21) & 0x07)

        for x in range(len(temp)):
            LOGGER.info(f'{self.phy_cal_result_obs_5[x]} = {hex(temp[x])}')

        return output

    def read_result_obs_6(self):

        temp = []
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 1342))

        temp.append(output & 0x01)
        temp.append((output >> 1) & 0x01)
        temp.append((output >> 2) & 0x01)
        temp.append((output >> 3) & 0x01)
        temp.append((output >> 4) & 0x3F)
        temp.append((output >> 10) & 0x3F)
        temp.append((output >> 16) & 0x1F)
        temp.append((output >> 21) & 0x07)

        for x in range(len(temp)):
            LOGGER.info(f'{self.phy_cal_result_obs_6[x]} = {hex(temp[x])}')

        return output

    def read_result_obs_7(self):

        temp = []
        output = (self.drv_obj.lpddr4_ctrl_read('PHY', 1342))

        temp.append(output & 0x01)
        temp.append((output >> 1) & 0x01)
        temp.append((output >> 2) & 0x01)
        temp.append((output >> 3) & 0x01)
        temp.append((output >> 4) & 0x3F)
        temp.append((output >> 10) & 0x3F)
        temp.append((output >> 16) & 0x1F)
        temp.append((output >> 21) & 0x07)

        for x in range(len(temp)):
            LOGGER.info(f'{self.phy_cal_result_obs_7[x]} = {hex(temp[x])}')

        return output

    io_cal_reg = [
        1331,
        1332,
        1399,
        1333,
        1339,
        1340,
        1343,
        1344,
        1345,
        1334,
        1335,
        1342
    ]

    def read_io_cal_register(self):

        for x in self.io_cal_reg:
            output = hex(self.drv_obj.lpddr4_ctrl_read('PHY', x))
            LOGGER.info(f'register {x} = {output}')

    def calibration_clean(self):
        self.drv_obj.lpddr4_ctrl_write(
            'PHY', 1331, (self.drv_obj.lpddr4_ctrl_read('PHY', 1331) & 0xFFF) | 0x10000)

    def calibration_start(self):
        self.drv_obj.lpddr4_ctrl_write(
            'PHY', 1331, (self.drv_obj.lpddr4_ctrl_read('PHY', 1331) & 0xFFF) | 0x1000000)

    def poll_calibartion_done(self):

        done1 = 0
        done0 = 0
        timeout = 0

        while (1):
            if self.drv_obj.lpddr4_ctrl_read('PHY', 1334) & 0x800000:
                LOGGER.info('IO pass 1 calibration done')
                done0 = 1

            if self.drv_obj.lpddr4_ctrl_read('PHY', 1335) & 0x800000:
                LOGGER.info('IO pass 2 calibration done')
                done1 = 1

            if (done0 & done1):
                break

            if timeout > 10:
                raise FatalException("IO calibration time out")

            time.sleep(0.5)
            timeout = timeout + 1

    def run_io_calibration(self):

        self.calibration_clean()
        self.calibration_start()
        self.poll_calibartion_done()
        #self.read_result_obs_0()
        #self.read_result_obs_2()
