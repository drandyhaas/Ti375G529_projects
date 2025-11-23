# -*- coding:utf-8 -*-
# Python3
#
# USB3 DDR Control Library
# Provides register read/write and memory testing over FT601 USB3 interface
#
# ARCHITECTURE NOTES:
# ===================
# The USB3 interface connects to axi_lite_slave which provides:
#   - Memory test control registers (start, stop, configure, read status)
#   - Configuration control (config_restart, config_ctrl_sel)
#   - System reset controls
#
# The USB3 interface does NOT have direct access to:
#   - DDR Controller (CTL) registers - these are on AXI0 bus
#   - PI registers - these are on AXI0 bus
#   - PHY registers - these are on AXI0 bus
#
# These DDR controller registers were previously accessed via JTAG (jtag2axi_debugger).
# For full DDR calibration (init, io, calvl, wrlvl, gatlvl, rdlvl, wdqlvl) you would
# need to add a second AXI master to the USB command handler to access AXI0.
#
# CURRENT CAPABILITIES:
# =====================
# Via USB3, you can:
#   - Read/write to axi_lite_slave registers
#   - Run memory tests with configurable size and patterns
#   - Control system resets
#   - Monitor test results and failures
#
# This is sufficient for memory testing after the DDR has been calibrated via JTAG
# or after power-on auto-calibration.
#

import sys
from pathlib import Path

# Add ftdi245fifo python directory to path
sys.path.insert(0, str(Path(__file__).parent))

from USB_FTX232H_FT60X import USB_FTX232H_FT60X_sync245mode
import time

# USB Command codes
CMD_TX_MASS   = 0x01
CMD_REG_WRITE = 0x02
CMD_REG_READ  = 0x03

# Register map (from axi_lite_slave.v)
# These are byte addresses that map to slaveReg[] array
# The hardware uses slaveReg[address[ADDR_WIDTH-1:2]] so address must be multiple of 4
REG_0_DQ_FAIL      = 0x00  # slaveReg[0]  - [RO] Failed DQ bits (dq_fail input)
REG_1_STATUS       = 0x04  # slaveReg[1]  - [RO] bit0=memtest_done, bit1=memtest_fail
REG_2_CONTROL      = 0x08  # slaveReg[2]  - [WO] bit0=memtest_start, bit1=memtest_rstn
REG_3_RESET        = 0x0C  # slaveReg[3]  - [WO] bit0=phy_rstn, bit1=ctrl_rstn, bit2=reg_axi_rstn, bit3=axi0_rstn, bit4=axi1_rstn
REG_4_DATA_L       = 0x10  # slaveReg[4]  - [WO] memtest_data[31:0]
REG_5_DATA_H       = 0x14  # slaveReg[5]  - [WO] memtest_data[63:32]
REG_6_LFSR         = 0x18  # slaveReg[6]  - [WO] bit0=memtest_lfsr_en
REG_7_X16          = 0x1C  # slaveReg[7]  - [WO] bit0=memtest_x16_en
REG_8_ARLEN        = 0x20  # slaveReg[8]  - [WO] reg_axi_arlen[7:0]
REG_9_SIZE         = 0x24  # slaveReg[9]  - [WO] memtest_size
REG_10_CONFIG      = 0x28  # slaveReg[10] - [RW] bit0=config_rst, bit1=config_sel, bit2=config_start, bit3=config_done[RO]
REG_11_TESTER_LEN_L= 0x2C  # slaveReg[11] - [RO] tester_loop_len[31:0]
REG_12_TESTER_LEN_H= 0x30  # slaveReg[12] - [RO] tester_loop_len[63:32]
REG_13_TESTER_CNT_L= 0x34  # slaveReg[13] - [RO] tester_loop_cnt[31:0]
REG_14_TESTER_CNT_H= 0x38  # slaveReg[14] - [RO] tester_loop_cnt[63:32]
REG_15_TESTER_STAT = 0x3C  # slaveReg[15] - [RO] bit0=tester_loop_done, bit1=tester_error
REG_16_TESTER_RST  = 0x40  # slaveReg[16] - [WO] bit0=tester_rst
REG_17_TESTER_PAT  = 0x44  # slaveReg[17] - [WO] tester_pattern[31:0]

# DDR Controller register bases (from jtag_drv.py, accessed via AXI0 not via axi_lite_slave)
# NOTE: These registers are NOT accessed through axi_lite_slave!
# They connect directly to AXI0 bus which goes to the DDR controller
# We'll need a different approach to access these - likely they're not accessible via USB yet
DDR_CTL_BASE  = 0x0080  # Controller registers (CTL type) at DDR addr 0x0000
DDR_PI_BASE   = 0x2080  # PI registers (PI type) at DDR addr 0x2000
DDR_PHY_BASE  = 0x4080  # PHY registers (PHY type) at DDR addr 0x4000


class USBDDRControl:
    """USB3 interface for DDR control and testing"""

    def __init__(self, device_list=(('FT60X', 'Haasoscope USB3'),
                                     ('FT60X', 'FTDI SuperSpeed-FIFO Bridge'))):
        """Initialize USB connection"""
        self.usb = USB_FTX232H_FT60X_sync245mode(device_to_open_list=device_list)
        print("USB3 DDR Control initialized")

    def close(self):
        """Close USB connection"""
        self.usb.close()

    def reg_write(self, addr, data):
        """
        Write to AXI-Lite register

        Args:
            addr: Register address (32-bit)
            data: Data to write (32-bit)
        """
        # Protocol: [CMD][ADDR(4B,LE)][DATA(4B,LE)]
        txdata = bytes([
            CMD_REG_WRITE,
            addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF, (addr >> 24) & 0xFF,
            data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF
        ])
        self.usb.send(txdata)
        # No response expected for write

    def reg_read(self, addr, timeout=2.0, verbose=False):
        """
        Read from AXI-Lite register

        Args:
            addr: Register address (32-bit)
            timeout: Timeout in seconds (default 2.0)
            verbose: Print debug info

        Returns:
            data: 32-bit value read from register
        """
        # Protocol: [CMD][ADDR(4B,LE)]
        txdata = bytes([
            CMD_REG_READ,
            addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF, (addr >> 24) & 0xFF
        ])

        if verbose:
            print(f"  Sending read command: {txdata.hex()}")

        self.usb.send(txdata)

        # Receive 4 bytes response with timeout
        if verbose:
            print(f"  Waiting for 4 bytes response (timeout={timeout}s)...")

        start_time = time.time()
        rxdata = bytes()

        while len(rxdata) < 4:
            # Check timeout on every iteration, not just when rxdata is empty
            elapsed = time.time() - start_time
            if elapsed > timeout:
                raise TimeoutError(f"Register read timeout after {elapsed:.2f}s (addr=0x{addr:04X}, received {len(rxdata)}/4 bytes)")

            chunk = self.usb.recv(4 - len(rxdata))
            rxdata += chunk

            if len(chunk) == 0:
                time.sleep(0.001)  # Short delay before retry

        if verbose:
            print(f"  Received: {rxdata.hex()}")

        if len(rxdata) != 4:
            raise Exception(f"Expected 4 bytes, got {len(rxdata)}")

        # Convert little-endian bytes to 32-bit value
        value = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
        return value

    def get_status(self):
        """
        Get hardware status using GET_STATUS command (0x05)

        Returns:
            status: 32-bit status word
                bit[0] = ddr_pll_lock
                bit[1] = axi_arready
                bit[2] = axi_rvalid
        """
        # Send GET_STATUS command
        self.usb.send(bytes([0x05]))

        # Read 4-byte response
        rxdata = bytes()
        start_time = time.time()
        timeout = 2.0

        while len(rxdata) < 4:
            chunk = self.usb.recv(4 - len(rxdata))
            rxdata += chunk

            if len(rxdata) == 0 and (time.time() - start_time) > timeout:
                raise TimeoutError("No response from GET_STATUS")

            if len(chunk) == 0:
                time.sleep(0.001)

        # Convert to 32-bit value
        value = rxdata[0] | (rxdata[1] << 8) | (rxdata[2] << 16) | (rxdata[3] << 24)
        return value

    # ========================================================================
    # DDR Controller Access Functions (from jtag_drv.py)
    # ========================================================================

    def lpddr4_ctrl_write(self, reg_type, addr, data):
        """
        Write to LPDDR4 controller register

        Args:
            reg_type: 'CTL', 'PI', or 'PHY'
            addr: Register address (word address, will be multiplied by 4)
            data: 32-bit data to write
        """
        if reg_type == 'CTL':
            self.reg_write(DDR_CTL_BASE + (addr * 4), data)
        elif reg_type == 'PI':
            self.reg_write(DDR_PI_BASE + (addr * 4), data)
        elif reg_type == 'PHY':
            self.reg_write(DDR_PHY_BASE + (addr * 4), data)
        else:
            raise ValueError(f"Invalid register type: {reg_type}")

    def lpddr4_ctrl_read(self, reg_type, addr, max_retries=3):
        """
        Read from LPDDR4 controller register with retry on timeout

        Args:
            reg_type: 'CTL', 'PI', or 'PHY'
            addr: Register address (word address, will be multiplied by 4)
            max_retries: Maximum number of retry attempts (default 3)

        Returns:
            data: 32-bit value read from register
        """
        # Calculate full address
        if reg_type == 'CTL':
            full_addr = DDR_CTL_BASE + (addr * 4)
        elif reg_type == 'PI':
            full_addr = DDR_PI_BASE + (addr * 4)
        elif reg_type == 'PHY':
            full_addr = DDR_PHY_BASE + (addr * 4)
        else:
            raise ValueError(f"Invalid register type: {reg_type}")

        # Retry loop for read operations
        last_exception = None
        for attempt in range(max_retries):
            try:
                return self.reg_read(full_addr)
            except TimeoutError as e:
                last_exception = e
                if attempt < max_retries - 1:
                    print(f"Warning: {reg_type} register read timeout (addr={addr}, attempt {attempt+1}/{max_retries}), retrying...")
                    time.sleep(0.1)  # Brief delay before retry
                else:
                    print(f"Error: {reg_type} register read failed after {max_retries} attempts (addr={addr})")
                    raise

        # Should never reach here, but just in case
        if last_exception:
            raise last_exception

    def config_restart(self):
        """
        Reset DDR configuration (from jtag_drv.py config_restart)

        REG_3_RESET bits:
        bit[0] = phy_rstn
        bit[1] = ctrl_rstn
        bit[2] = reg_axi_rstn
        bit[3] = axi0_rstn
        bit[4] = axi1_rstn
        """
        # Reset all (set all bits to 0)
        self.reg_write(REG_3_RESET, 0x00)
        time.sleep(0.1)

        # Release reset (set all bits to 1)
        self.reg_write(REG_3_RESET, 0x1F)

    def config_ctrl_sel(self, sel):
        """
        Select configuration controller (from jtag_drv.py config_ctrl_sel)

        Args:
            sel: 0 or 1 to select configuration source

        REG_10_CONFIG bits:
        bit[0] = config_rst
        bit[1] = config_sel
        bit[2] = config_start
        bit[3] = config_done (read-only)
        """
        # Assert config_rst with selected config_sel
        output = (0 << 2) | ((sel & 0x1) << 1) | (1 << 0)
        self.reg_write(REG_10_CONFIG, output)

        # Deassert config_rst
        output = (0 << 2) | ((sel & 0x1) << 1) | (0 << 0)
        self.reg_write(REG_10_CONFIG, output)

    def read_ctl_id(self):
        """
        Read memory controller ID (should return 0x2040)

        NOTE: This requires access to DDR controller registers via AXI0,
        which is currently only available via JTAG (jtag2axi_debugger).
        USB only has access to axi_lite_slave registers.
        """
        return self.lpddr4_ctrl_read('CTL', 0)

    def read_pi_id(self):
        """
        Read PI controller ID (should return 0x2040)

        NOTE: This requires access to DDR controller registers via AXI0,
        which is currently only available via JTAG (jtag2axi_debugger).
        USB only has access to axi_lite_slave registers.
        """
        return self.lpddr4_ctrl_read('PI', 0)

    def memtest_data(self, data):
        """Set 64-bit test data pattern"""
        data_l = data & 0xFFFFFFFF
        data_h = (data >> 32) & 0xFFFFFFFF
        self.reg_write(REG_4_DATA_L, data_l)
        self.reg_write(REG_5_DATA_H, data_h)

    def memtest_size(self, size_mb):
        """
        Set memory test size in MB

        Args:
            size_mb: Size in megabytes (4, 8, 16, 32, 64, 128, 256, 512, 1024)
        """
        size_bytes = ((size_mb * 1024 * 1024) - 4096) & 0xFFFFFFFF
        self.reg_write(REG_9_SIZE, size_bytes)

    def memtest_restart(self, lfsr_en=True):
        """
        Restart memory test

        Args:
            lfsr_en: Enable LFSR mode (True) or use fixed pattern (False)
        """
        # Set LFSR enable
        self.reg_write(REG_6_LFSR, 1 if lfsr_en else 0)

        # Clear start/rstn
        self.reg_write(REG_2_CONTROL, 0x00)

        # Set start and rstn
        self.reg_write(REG_2_CONTROL, 0x03)

    def memtest_stop(self):
        """
        Stop the memory test

        REG_2_CONTROL[0] = memtest_start
        REG_2_CONTROL[1] = memtest_rstn
        """
        self.reg_write(REG_2_CONTROL, 0x00)

    def memtest_poll_done(self, timeout=30.0):
        """
        Poll until memory test completes

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            fail: True if test failed, False if passed

        Raises:
            TimeoutError: If test doesn't complete within timeout
        """
        start_time = time.time()

        while True:
            status = self.reg_read(REG_1_STATUS)

            # Check if done (bit 0)
            if status & 0x01:
                # Check if failed (bit 1)
                return bool(status & 0x02)

            # Check timeout
            if time.time() - start_time > timeout:
                raise TimeoutError(f"Memory test did not complete within {timeout}s")

            time.sleep(0.01)  # Poll every 10ms

    def memtest_read_fail_dq(self):
        """Read which DQ bits failed"""
        return self.reg_read(REG_0_DQ_FAIL)

    def memtest_run(self, size_mb=4, pattern=0x5555AAAA, lfsr_en=True, verbose=True):
        """
        Run complete memory test

        Args:
            size_mb: Memory size to test in MB
            pattern: Test pattern (used if lfsr_en=False)
            lfsr_en: Use LFSR pattern generator
            verbose: Print status messages

        Returns:
            pass: True if test passed, False if failed
        """
        if verbose:
            print(f"Running {size_mb}MB memory test...")
            print(f"Pattern: {'LFSR' if lfsr_en else f'0x{pattern:016X}'}")

        # Stop any running test first
        self.memtest_stop()
        time.sleep(0.1)

        # Configure test
        if not lfsr_en:
            self.memtest_data(pattern)
        self.memtest_size(size_mb)

        # Start test
        start_time = time.time()
        self.memtest_restart(lfsr_en)

        # Wait for completion
        try:
            failed = self.memtest_poll_done()
            elapsed = time.time() - start_time

            if failed:
                fail_dq = self.memtest_read_fail_dq()
                if verbose:
                    print(f"X Test FAILED in {elapsed:.2f}s - Failed DQ bits: 0x{fail_dq:08X}")
                return False
            else:
                if verbose:
                    print(f"+ Test PASSED in {elapsed:.2f}s")
                return True

        except TimeoutError as e:
            if verbose:
                print(f"X Test TIMEOUT: {e}")
            return False

    def memtest_ctrl_write(self, addr, data):
        """
        Write to memtest control register (maps to axi_lite_slave registers)

        Args:
            addr: Register address (will be multiplied by 4 for byte addressing)
            data: 32-bit data to write
        """
        self.reg_write(addr * 4, data)

    def memtest_ctrl_read(self, addr):
        """
        Read from memtest control register

        Args:
            addr: Register address (will be multiplied by 4 for byte addressing)

        Returns:
            32-bit register value
        """
        return self.reg_read(addr * 4)

    def x16_mode_test_enable(self):
        """Enable x16 mode testing (writes 1 to register 7)"""
        self.memtest_ctrl_write(7, 1)

    def x16_mode_test_disable(self):
        """Disable x16 mode testing (writes 0 to register 7)"""
        self.memtest_ctrl_write(7, 0)

    def release_ddr_resets(self):
        """
        Release DDR controller resets (REG_3)
        Bit 0: phy_rstn
        Bit 1: ctrl_rstn
        Bit 2: reg_axi_rstn
        Bit 3: axi0_rstn
        Bit 4: axi1_rstn
        """
        # Release all DDR resets
        self.reg_write(0x000C, 0x0000001F)  # Set bits 0-4 to 1

    def assert_ddr_resets(self):
        """Assert DDR controller resets (put all DDR components in reset)"""
        self.reg_write(0x000C, 0x00000000)

    def config_restart(self):
        """
        Restart configuration - performs reset pulse on all DDR components
        This is required before accessing DDR controller registers!

        REG_3 bits:
        bit[0] = phy_rstn
        bit[1] = ctrl_rstn
        bit[2] = reg_axi_rstn
        bit[3] = axi0_rstn
        bit[4] = axi1_rstn
        """
        # Assert all resets
        self.reg_write(0x000C, 0x00000000)
        time.sleep(0.1)

        # Release all resets
        self.reg_write(0x000C, 0x0000001F)

    def config_ctrl_sel(self, sel):
        """
        Configure control selection

        REG_10 bits:
        bit[0] = config_rst
        bit[1] = config_sel
        bit[2] = config_start
        bit[3] = config_done (read-only)
        """
        # CFG_RESET=1, CFG_SEL=sel, CFG_START=0
        value = (0 << 2) | ((sel & 0x1) << 1) | 1
        self.reg_write(0x0028, value)  # REG_10 at offset 0x28

    def ddr_auto_init(self, timeout=5.0):
        """
        Trigger DDR auto-initialization using built-in configuration

        This uses cfg_sel=0 to load configuration from bitstream,
        then triggers initialization automatically.

        Args:
            timeout: Maximum time to wait for init completion (seconds)

        Returns:
            True if initialization succeeded, False otherwise
        """
        print("Starting DDR auto-initialization...")

        # Step 1: Select built-in configuration (cfg_sel=0)
        # REG_10_CONFIG: bit0=cfg_rst, bit1=cfg_sel, bit2=cfg_start

        # Assert config reset with cfg_sel=0
        self.reg_write(REG_10_CONFIG, 0x01)  # cfg_rst=1, cfg_sel=0, cfg_start=0
        time.sleep(0.01)

        # Deassert config reset
        self.reg_write(REG_10_CONFIG, 0x00)  # cfg_rst=0, cfg_sel=0, cfg_start=0
        time.sleep(0.01)

        # Step 2: Pulse cfg_start to trigger initialization
        self.reg_write(REG_10_CONFIG, 0x04)  # cfg_rst=0, cfg_sel=0, cfg_start=1
        time.sleep(0.01)
        self.reg_write(REG_10_CONFIG, 0x00)  # cfg_rst=0, cfg_sel=0, cfg_start=0

        # Step 3: Wait for cfg_done (bit 3 of REG_10_CONFIG)
        print("Waiting for DDR initialization to complete...")
        start_time = time.time()

        while True:
            config_reg = self.reg_read(REG_10_CONFIG)
            cfg_done = (config_reg >> 3) & 0x1

            if cfg_done:
                elapsed = time.time() - start_time
                print(f"✓ DDR initialization complete in {elapsed:.2f}s")
                return True

            if time.time() - start_time > timeout:
                print(f"✗ DDR initialization timeout after {timeout}s")
                print(f"  REG_10_CONFIG = 0x{config_reg:08X}")
                return False

            time.sleep(0.1)


# Simple test functions
def test_register_access():
    """Test basic register read/write"""
    print("\n=== Testing Register Access ===")
    usb = USBDDRControl()

    try:
        # Test register write/read using REG_17_TESTER_PAT (a general purpose register)
        test_addr = REG_17_TESTER_PAT
        test_value = 0xDEADBEEF

        print(f"Writing 0x{test_value:08X} to register 0x{test_addr:04X}...")
        usb.reg_write(test_addr, test_value)

        print(f"Reading from register 0x{test_addr:04X}...")
        read_value = usb.reg_read(test_addr, verbose=True, timeout=2.0)
        print(f"Read value: 0x{read_value:08X}")

        if read_value == test_value:
            print("✓ Register access test PASSED")
        else:
            print(f"✗ Register access test FAILED (expected 0x{test_value:08X}, got 0x{read_value:08X})")

    except Exception as e:
        print(f"✗ Test FAILED with exception: {e}")

    finally:
        usb.close()


def test_memtest(size_mb=4):
    """Test memory with specified size"""
    print(f"\n=== Testing {size_mb}MB Memory ===")
    usb = USBDDRControl()

    try:
        result = usb.memtest_run(size_mb=size_mb, lfsr_en=True, verbose=True)
        return result
    finally:
        usb.close()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='USB3 DDR Control')
    parser.add_argument('--test-reg', action='store_true', help='Test register access')
    parser.add_argument('--memtest', type=int, metavar='SIZE_MB', help='Run memory test (size in MB)')

    args = parser.parse_args()

    if args.test_reg:
        test_register_access()
    elif args.memtest:
        test_memtest(args.memtest)
    else:
        # Default: run both tests
        test_register_access()
        test_memtest(4)
