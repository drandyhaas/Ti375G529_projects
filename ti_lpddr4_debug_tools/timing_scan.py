import sys
import logging

LOGGER = logging.getLogger('ddr_cali_tools')

class timing_scan:

    def __init__(self, drv_obj, wrlvl, gatlvl, rdlvl, wdqlvl, freq: int):
        self.drv_obj = drv_obj
        self.wrlvl = wrlvl
        self.gatlvl = gatlvl
        self.rdlvl = rdlvl
        self.wdqlvl = wdqlvl
        self.freq = freq
        self.step = 1/freq*1000000/512

    def query_yes_no(self, question, default="yes"):

        #return True

        valid = {"yes": True, "y": True, "ye": True, "no": False, "n": False}
        if default is None:
            prompt = " [y/n] "
        elif default == "yes":
            prompt = " [Y/n] "
        elif default == "no":
            prompt = " [y/N] "
        else:
            raise ValueError("invalid default answer: '%s'" % default)

        while True:
            sys.stdout.write(question + prompt)
            choice = input().lower()
            if default is not None and choice == "":
                return valid[default]
            elif choice in valid:
                return valid[choice]
            else:
                sys.stdout.write(
                    "Please respond with 'yes' or 'no' " "(or 'y' or 'n').\n")

    def pattern_scan_write_dqx(self, slice_mask, data, start, end, lsfr_en, cali_file):

        dq_pass_list = []
        n = 0
        step = 8
        #start = 800
        #end = 1100
        start = int(start)
        end = int(end)
        dq_first_pass = [-1]*32
        dq_last_pass = [-1]*32
        temp_delay_list = []
        result_list = []
        le_delay = []
        re_delay = []
        window = []

        temp_delay_list = self.wdqlvl.read_phy_clk_wrdqx_slave_delay(0xF)

        for x in range(start, end, step):

            delay = [x]*36
            self.wdqlvl.write_phy_clk_wrdqx_slave_delay(slice_mask, delay)

            self.drv_obj.memtest_data(data)
            self.drv_obj.memtest_restart(lsfr_en)

            fail = self.drv_obj.memtest_poll_done()

            if fail:
                mini_chart = f'FAIL {x}:'.ljust(12)
            else:
                mini_chart = f'PASS {x}:'.ljust(12)

            output = self.drv_obj.memtest_read_fail_dq()

            dq_pass_list.append((~output) & 0xFFFFFFFF)

            for dq in range(32):
                if (dq_pass_list[n] & (0x80000000 >> dq)):
                    mini_chart += '1'
                else:
                    mini_chart += '0'

            LOGGER.info(mini_chart)

            n = n+1

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                for dq in range(8):
                    for m in range(n):
                        if (dq_pass_list[m] & 1 << (8*slicex+dq)):
                            if (dq_first_pass[8*slicex+dq] == -1):
                                dq_first_pass[8*slicex+dq] = (m*step)+start
                            else:
                                dq_last_pass[8*slicex+dq] = (m*step)+start

                    window.append(dq_last_pass[8*slicex+dq]-dq_first_pass[8*slicex+dq])

                    start_delay = round((dq_first_pass[8*slicex+dq]*self.step),2)
                    end_delay = round((dq_last_pass[8*slicex+dq]*self.step),2)

                    LOGGER.info(f'DQ[{8*slicex+dq}] start = {start_delay} ps end = {end_delay} ps window = {round((window[8*slicex+dq]*self.step),2)} ps')
            else:
                for dq in range(8):
                    window.append(int(0))

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                for dq in range(8):
                    result = int((dq_last_pass[8*slicex+dq]+dq_first_pass[8*slicex+dq])/2)
                    result_list.append(result)
                    le_delay.append(dq_first_pass[8*slicex+dq])
                    re_delay.append(dq_last_pass[8*slicex+dq])

                result_list.append(temp_delay_list[(8*slicex)+8])  # DM
                le_delay.append(temp_delay_list[(8*slicex)+8])
                re_delay.append(temp_delay_list[(8*slicex)+8])
            else:
                for dq in range(8):
                    result_list.append(int(0))
                    le_delay.append(int(0))
                    re_delay.append(int(0))

                result_list.append(int(0))  # DM
                le_delay.append(int(0))
                re_delay.append(int(0))

        cali_file = self.wdqlvl.update_wdqlvl_cali_file(0, slice_mask, cali_file, le_delay, re_delay)

        save = self.query_yes_no(
            'Do you save update calibration result to PHY?')

        if save:
            self.wdqlvl.write_phy_clk_wrdqx_slave_delay(slice_mask, result_list)
            LOGGER.info('Updated with Calibrated Result')
        else:
            self.wdqlvl.write_phy_clk_wrdqx_slave_delay(0xF, temp_delay_list)
            LOGGER.info('Rstore to Default value')

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                for dq in range(8):
                    if ((window[8*slicex+dq] <= 0) | (window[8*slicex+dq] > 320)):
                        self.wdqlvl.write_phy_clk_wrdqx_slave_delay(0xF, temp_delay_list)
                        raise Exception(f"pattern_scan_write_dq{8*slicex+dq}fail")

        return cali_file

    def pattern_scan_write_dqs(self, slice_mask, data, start, end, lsfr_en: bool, cali_file):

        dq_pass_list = []
        dq_first_pass = [-1]*4
        dq_last_pass = [-1]*4
        result_list = []
        temp_delay_list = []
        n = 0
        step = 32
        #start = 0
        #end = 0x3FF
        start = int(start)
        end = int(end)
        window = []
        temp_delay_list = self.wrlvl.read_write_leveling_wrdqs_delay(0xF)

        for x in range(start, end, step):

            delay = [x]*4
            self.wrlvl.update_phy_clk_wrdqs_slave_delay(slice_mask, delay)
            self.drv_obj.memtest_data(data)
            self.drv_obj.memtest_restart(lsfr_en)

            fail = self.drv_obj.memtest_poll_done()

            if fail:
                mini_chart = f'FAIL {x}:'.ljust(12)
            else:
                mini_chart = f'PASS {x}:'.ljust(12)

            output = self.drv_obj.memtest_read_fail_dq()

            dq_pass_list.append((~output) & 0xFFFFFFFF)

            for dq in range(32):
                if (output & (0x80000000 >> dq)):
                    mini_chart += '1'
                else:
                    mini_chart += '0'

            LOGGER.info(mini_chart)

            n = n+1

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                for m in range(n):
                    if (((dq_pass_list[m] >> (slicex*8)) & 0xFF) == 0xFF):
                        if (dq_first_pass[slicex] == -1):
                            dq_first_pass[slicex] = (m*step)+start
                        else:
                            dq_last_pass[slicex] = (m*step)+start

                result_list.append(
                    int((dq_last_pass[slicex]+dq_first_pass[slicex])/2))

                window.append(dq_last_pass[slicex]-dq_first_pass[slicex])

                start_delay = round((dq_first_pass[slicex]*self.step),2)
                end_delay = round((dq_last_pass[slicex]*self.step),2)

                LOGGER.info(f'Slice[{slicex}] start = {start_delay} ps end = {end_delay} ps window = {round((window[slicex]*self.step),2)} ps')

            else:
                result_list.append(int(0))
                window.append(int(0))

        cali_file = self.wrlvl.update_wrlvl_cali_file(0, slice_mask, cali_file, dq_first_pass, dq_last_pass)

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                if ((window[slicex] <= 0)):
                    self.wrlvl.update_phy_clk_wrdqs_slave_delay(0xF, temp_delay_list)
                    raise Exception(f"pattern_scan_write_dqs{slicex}fail")

        save = self.query_yes_no(
            'Do you save update calibration result to PHY?')

        if save:
            self.wrlvl.update_phy_clk_wrdqs_slave_delay(slice_mask, result_list)
            LOGGER.info('Updated with Calibrated Result')
        else:
            self.wrlvl.update_phy_clk_wrdqs_slave_delay(0xF, temp_delay_list)
            LOGGER.info('Rstore to Default value')

        return cali_file

    def pattern_scan_gate_dqs(self, slice_mask, data, start, end, lsfr_en: bool, cali_file):

        dq_pass_list = []
        dqs_delay = [0, 0, 0, 0]
        latency = [0, 0, 0, 0]
        dq_first_pass = [-1]*4
        dq_last_pass = [-1]*4
        result_list = []
        temp_delay_list = []
        n = 0
        step = 64
        #start = 0
        #end = 4096
        start = int(start)
        end = int(end)
        window = []

        temp_delay_list = self.gatlvl.read_phy_rddqs_gate_slave_delayX(0xF)
        for x in range(start, end, step):

            for slicex in range(4):
                dqs_delay[slicex] = (int(x % 512))
                latency[slicex] = ((int(x//512) & 0xF))

            self.gatlvl.write_gate_leveling_slave_delay(0xF, dqs_delay)
            self.gatlvl.write_gate_leveling_slave_lat(0xF, latency)
            self.drv_obj.memtest_data(data)
            self.drv_obj.memtest_restart(lsfr_en)

            fail = self.drv_obj.memtest_poll_done()

            if fail:
                mini_chart = f'FAIL {x}:'.ljust(12)
            else:
                mini_chart = f'PASS {x}:'.ljust(12)

            output = self.drv_obj.memtest_read_fail_dq()

            dq_pass_list.append((~output) & 0xFFFFFFFF)

            for dq in range(32):
                if (output & (0x80000000 >> dq)):
                    mini_chart += '1'
                else:
                    mini_chart += '0'

            LOGGER.info(mini_chart)

            n = n+1

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                for m in range(n):
                    if (((dq_pass_list[m] >> (slicex*8)) & 0xFF) == 0xFF):
                        if (dq_first_pass[slicex] == -1):
                            dq_first_pass[slicex] = (m*step)+start
                        else:
                            dq_last_pass[slicex] = (m*step)+start

                result_list.append(
                    int((dq_last_pass[slicex]+dq_first_pass[slicex])/2))

                window.append(dq_last_pass[slicex]-dq_first_pass[slicex])

                start_delay = round((dq_first_pass[slicex]*self.step),2)
                end_delay = round((dq_last_pass[slicex]*self.step),2)

                LOGGER.info(f'Slice[{slicex}] start = {start_delay} ps end = {end_delay} ps window = {round((window[slicex]*self.step),2)} ps')

                LOGGER.info(f'Slice[{slicex}] start = {dq_first_pass[slicex]} end = {dq_last_pass[slicex]} result = {result_list[slicex]} window = {round((window[slicex]*self.step),2)} ps')

            else:
                result_list.append(int(0))

        cali_file = self.gatlvl.update_gatlvl_cali_file(0, slice_mask, cali_file, dq_first_pass, dq_last_pass)

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                if ((window[slicex] <= 0)):
                    self.gatlvl.write_phy_rddqs_gate_slave_delayX(0xF, temp_delay_list)
                    raise Exception(f"pattern_scan_gate_dqs{slicex}fail")

        save = self.query_yes_no(
            'Do you save update calibration result to PHY?')

        if save:
            self.gatlvl.write_phy_rddqs_gate_slave_delayX(slice_mask, result_list)
            LOGGER.info('Updated with Calibrated Result')
        else:
            self.gatlvl.write_phy_rddqs_gate_slave_delayX(0xF, temp_delay_list)
            LOGGER.info('Rstore to Default value')

        return cali_file

    def pattern_scan_input_dqx_rise(self, slice_mask, data, start, end, lsfr_en: bool, cali_file):

        dq_pass_list = []
        dq_first_pass = [-1]*32
        dq_last_pass = [-1]*32
        temp_delay_list = []
        result_list = []
        n = 0
        step = 8
        #start = 0
        #end = 480
        start = int(start)
        end = int(end)
        window = []

        temp_delay_list = self.rdlvl.read_read_leveling_rddqs_rise_delay(0xF)

        for x in range(start, end, step):

            delay = [x]*8*4
            self.rdlvl.update_phy_rddqs_dqx_rise_slave_delay(slice_mask, delay)
            self.drv_obj.memtest_data(data)
            self.drv_obj.memtest_restart(lsfr_en)

            fail = self.drv_obj.memtest_poll_done()

            if fail:
                mini_chart = f'FAIL {x}:'.ljust(12)
            else:
                mini_chart = f'PASS {x}:'.ljust(12)

            output = self.drv_obj.memtest_read_fail_dq()

            dq_pass_list.append((~output) & 0xFFFFFFFF)

            for dq in range(32):
                if (dq_pass_list[n] & (0x80000000 >> dq)):
                    mini_chart += '1'
                else:
                    mini_chart += '0'

            LOGGER.info(mini_chart)

            n = n+1

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                for dq in range(8):
                    for m in range(n):
                        if (dq_pass_list[m] & 1 << (8*slicex+dq)):
                            if (dq_first_pass[8*slicex+dq] == -1):
                                dq_first_pass[8*slicex+dq] = (m*step)+start
                            else:
                                dq_last_pass[8*slicex+dq] = (m*step)+start

                    result_list.append(int((dq_last_pass[8*slicex+dq]+dq_first_pass[8*slicex+dq])/2))

                    window.append(dq_last_pass[8*slicex+dq]-dq_first_pass[8*slicex+dq])

                    start_delay = round((dq_first_pass[8*slicex+dq]*self.step),2)
                    end_delay = round((dq_last_pass[8*slicex+dq]*self.step),2)

                    LOGGER.info(f'DQ[{8*slicex+dq}] start = {start_delay} ps end = {end_delay} ps window = {round((window[8*slicex+dq]*self.step),2)} ps')
            else:
                for dq in range(8):
                    result_list.append(int(0))
                    window.append(int(0))

        cali_file = self.rdlvl.update_rdlvl_rise_cali_file(0, slice_mask, cali_file, dq_first_pass, dq_last_pass)

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                for dq in range(8):
                    if ((window[8*slicex+dq] <= 0)):
                        self.rdlvl.update_phy_rddqs_dqx_rise_slave_delay(0xF, temp_delay_list)
                        raise Exception(f"pattern_scan_input_dq{8*slicex+dq}_rise fail")

        save = self.query_yes_no(
            'Do you save update calibration result to PHY?')

        if save:
            self.rdlvl.update_phy_rddqs_dqx_rise_slave_delay(slice_mask, result_list)

            LOGGER.info('Updated with Calibrated Result')
        else:
            self.rdlvl.update_phy_rddqs_dqx_rise_slave_delay(
                0xF, temp_delay_list)
            LOGGER.info('Rstore to Default value')

        return cali_file

    def pattern_scan_input_dqx_fall(self, slice_mask, data, start, end, lsfr_en: bool, cali_file):

        dq_pass_list = []
        dq_first_pass = [-1]*32
        dq_last_pass = [-1]*32
        result_list = []
        temp_delay_list = []
        n = 0
        step = 8
        #start = 0
        #end = 480
        start = int(start)
        end = int(end)
        window = []

        temp_delay_list = self.rdlvl.read_read_leveling_rddqs_fall_delay(0xF)

        for x in range(start, end, step):

            delay = [x]*8*4
            self.rdlvl.update_phy_rddqs_dqx_fall_slave_delay(slice_mask, delay)
            self.drv_obj.memtest_data(data)
            self.drv_obj.memtest_restart(lsfr_en)

            fail = self.drv_obj.memtest_poll_done()

            if fail:
                mini_chart = f'FAIL {x}:'.ljust(12)
            else:
                mini_chart = f'PASS {x}:'.ljust(12)

            output = self.drv_obj.memtest_read_fail_dq()

            dq_pass_list.append((~output) & 0xFFFFFFFF)

            for dq in range(32):
                if (dq_pass_list[n] & (0x80000000 >> dq)):
                    mini_chart += '1'
                else:
                    mini_chart += '0'

            LOGGER.info(mini_chart)

            n = n+1

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                for dq in range(8):
                    for m in range(n):
                        if (dq_pass_list[m] & 1 << (8*slicex+dq)):
                            if (dq_first_pass[8*slicex+dq] == -1):
                                dq_first_pass[8*slicex+dq] = (m*step)+start
                            else:
                                dq_last_pass[8*slicex+dq] = (m*step)+start

                    result_list.append(int((dq_last_pass[8*slicex+dq]+dq_first_pass[8*slicex+dq])/2))

                    window.append(dq_last_pass[8*slicex+dq]-dq_first_pass[8*slicex+dq])

                    start_delay = round((dq_first_pass[8*slicex+dq]*self.step),2)
                    end_delay = round((dq_last_pass[8*slicex+dq]*self.step),2)

                    LOGGER.info(f'DQ[{8*slicex+dq}] start = {start_delay} ps end = {end_delay} ps window = {round((window[8*slicex+dq]*self.step),2)} ps')
            else:
                for dq in range(8):
                    result_list.append(int(0))
                    window.append(int(0))

        cali_file = self.rdlvl.update_rdlvl_fall_cali_file(0, slice_mask, cali_file, dq_first_pass, dq_last_pass)

        for slicex in range(4):
            if (slice_mask & (1 << slicex)):
                for dq in range(8):
                    if ((window[8*slicex+dq] <= 0)):
                        self.rdlvl.update_phy_rddqs_dqx_fall_slave_delay(0xF, temp_delay_list)
                        raise Exception(f"pattern_scan_input_dq{8*slicex+dq}_fall fail")

        save = self.query_yes_no(
            'Do you save update calibration result to PHY?')

        if save:
            self.rdlvl.update_phy_rddqs_dqx_fall_slave_delay(slice_mask, result_list)

            LOGGER.info('Updated with Calibrated Result')
        else:
            self.rdlvl.update_phy_rddqs_dqx_fall_slave_delay(
                0xF, temp_delay_list)
            LOGGER.info('Rstore to Default value')

        return cali_file
