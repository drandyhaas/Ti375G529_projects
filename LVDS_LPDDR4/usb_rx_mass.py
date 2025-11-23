# -*- coding:utf-8 -*-
# Python3
#
# This program is a test of FPGA+FTDI USB chips (FT232H, FT600, or FT601)
# It sends a command byte + 4 bytes (length) to FTDI chip.
# The FPGA interprets the command and length, then sends back the requested data.
#
# Protocol: [CMD][LENGTH(4B,LE)]
# Commands:
#   0x01 - TX_MASS: Receive length, send back that many bytes
#

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode  # see USB_FTX232H_FT60X.py
#from random import randint
import time

TEST_COUNT = 50

# Command codes
CMD_TX_MASS = 0x01

if __name__ == '__main__':

    usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=(
        ('FT60X', 'Haasoscope USB3'   ),           # If the chip's name has been modified, you can use FT_Prog software to look up it.
        ('FT60X', 'FTDI SuperSpeed-FIFO Bridge')) # secondly try to open FT60X (FT600 or FT601) device named 'FTDI SuperSpeed-FIFO Bridge'. Note that 'FTDI SuperSpeed-FIFO Bridge' is the default name of FT600 or FT601 chip unless the user has modified it.
        )

    total_rx_len = 0
    expect_len = 10 * 1000 * 1000  # randint(1, 10000000) # random a length
    # Protocol: [CMD][LENGTH(4B,LE)]
    txdata = bytes([CMD_TX_MASS,
                    expect_len & 0xff, (expect_len >> 8) & 0xff, (expect_len >> 16) & 0xff,
                    (expect_len >> 24) & 0xff])  # command byte + length (4 bytes, little-endian)

    time_start = time.time()
    for i in range(TEST_COUNT):
        usb.send(txdata)  # send the command + 4 bytes to usb
        data = usb.recv(expect_len) # recv from usb
        rx_len = len(data)
        if i==0: print(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7])
        total_rx_len += rx_len
        time_total = time.time() - time_start
        data_rate = total_rx_len / (time_total + 0.001) / 1e6
        if i==TEST_COUNT-1: print('[%d/%d]   rx_len=%e   total_rx_len=%e   data_rate=%.3f MB/s' % (i + 1, TEST_COUNT, rx_len, total_rx_len, data_rate))
        if expect_len != rx_len:
            print('*** expect_len (%d) and rx_len (%d) mismatch' % (expect_len, rx_len))
            break

    usb.close()
