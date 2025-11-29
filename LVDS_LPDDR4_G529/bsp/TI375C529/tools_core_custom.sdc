# Custom SDC constraints for LPDDR4 debug tools
# This file supplements the auto-generated bsp/TI375C529/tools_core.sdc
# Add this file to your project constraints
#
# Based on the working Ti375C529 OOB v2.1 example running at 1650 MHz
# Reference: C:\Users\ahaas\.efinity\project\efx_ti375c529_oob_v2.1\bsp\TI375C529_DK\constraints.sdc

# USB FT600 Clock Definition
# The FT600 provides a 100 MHz clock (10 ns period)
create_clock -period 10.000 [get_ports {ftdi_clk}]

# Clock Domain Crossing Constraints
# All clocks are asynchronous to each other:
# - axi0_ACLK: DDR controller AXI interface
# - regACLK: DDR register interface (CDC via axi_lite_cdc with 2-FF synchronizers)
# - clk_command: USB/command processor (CDC via axi_lite_cdc and async FIFOs)
# - clk50: General purpose 50MHz clock
# - ftdi_clk: External USB FT60X clock (async FIFOs handle CDC)
# - lvds_clk_slow_clkin1/lvds_clk_fast_clkin1: LVDS clocks from PLL
set_clock_groups -asynchronous \
    -group {axi0_ACLK} \
    -group {regACLK} \
    -group {clk_command} \
    -group {clk50} \
    -group {ftdi_clk} \
    -group {lvds_clk_slow_clkin1} \
    -group {lvds_clk_fast_clkin1}

# Override Auto-Generated I/O Delay Constraints
# The auto-generated BSP file uses fixed delay values (2.310/2.625 ns) that were
# appropriate for slower AXI clocks but consume 98%+ of the 5ns period at 200 MHz.
# These scaled values are based on maintaining ~15% of clock period margin,
# similar to the working reference design approach.
#
# For axi0_ACLK at 5 ns period (200 MHz):
# - Target max output delay: ~0.75 ns (15% of period, down from 2.310 ns)
# - Target max input delay: ~0.85 ns (17% of period, down from 2.625 ns)
#
# NOTE: These values override the auto-generated constraints in bsp/TI375C529/tools_core.sdc
# Comment out these overrides if they cause issues, and instead try Option 2 below.

# AXI0 Output Delays (from FPGA to DDR controller)
#set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_ARADDR[*] axi0_ARAPCMD axi0_ARBURST[*] axi0_ARID[*] axi0_ARLEN[*] axi0_ARSIZE[*] axi0_ARVALID axi0_ARLOCK axi0_ARQOS}]
#set_output_delay -clock axi0_ACLK -min -0.140 [get_ports {axi0_ARADDR[*] axi0_ARAPCMD axi0_ARBURST[*] axi0_ARID[*] axi0_ARLEN[*] axi0_ARSIZE[*] axi0_ARVALID axi0_ARLOCK axi0_ARQOS}]

#set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_AWADDR[*] axi0_AWALLSTRB axi0_AWBURST[*] axi0_AWCOBUF axi0_AWID[*] axi0_AWLEN[*] axi0_AWSIZE[*] axi0_AWVALID axi0_AWLOCK axi0_AWCACHE[*] axi0_AWQOS}]
#set_output_delay -clock axi0_ACLK -min -0.140 [get_ports {axi0_AWADDR[*] axi0_AWALLSTRB axi0_AWBURST[*] axi0_AWCOBUF axi0_AWID[*] axi0_AWLEN[*] axi0_AWSIZE[*] axi0_AWVALID axi0_AWLOCK axi0_AWCACHE[*] axi0_AWQOS}]
#set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_AWAPCMD}]
#set_output_delay -clock axi0_ACLK -min -0.350 [get_ports {axi0_AWAPCMD}]

#set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_WDATA[*] axi0_WLAST axi0_WSTRB[*] axi0_WVALID}]
#set_output_delay -clock axi0_ACLK -min -0.140 [get_ports {axi0_WDATA[*] axi0_WLAST axi0_WSTRB[*] axi0_WVALID}]

#set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_BREADY axi0_RREADY}]
#set_output_delay -clock axi0_ACLK -min -0.140 [get_ports {axi0_BREADY axi0_RREADY}]

# AXI0 Input Delays (from DDR controller to FPGA)
# Note: BID, RID, BRESP, RRESP signals are ALL removed by synthesis (unconnected)
# Only constrain the signals that actually exist in the netlist
#set_input_delay -clock axi0_ACLK -max 0.850 [get_ports {axi0_ARREADY axi0_AWREADY axi0_WREADY}]
#set_input_delay -clock axi0_ACLK -min 0.600 [get_ports {axi0_ARREADY axi0_AWREADY axi0_WREADY}]

#set_input_delay -clock axi0_ACLK -max 0.850 [get_ports {axi0_BVALID axi0_RVALID}]
#set_input_delay -clock axi0_ACLK -min 0.600 [get_ports {axi0_BVALID axi0_RVALID}]

#set_input_delay -clock axi0_ACLK -max 0.850 [get_ports {axi0_RDATA[*] axi0_RLAST}]
#set_input_delay -clock axi0_ACLK -min 0.600 [get_ports {axi0_RDATA[*] axi0_RLAST}]

# For debugging timing issues: Uncomment to see detailed timing reports
# set_max_delay 20 -from [get_clocks {axi0_ACLK}] -to [all_outputs]

# ============================================================================
# Notes on Clock Domain Crossings
# ============================================================================
# All CDC paths are handled by the set_clock_groups -asynchronous constraint above.
# The design uses proper CDC mechanisms:
# - clk_command ↔ regACLK: axi_lite_cdc module with 2-FF synchronizers
# - clk_command ↔ ftdi_clk: Async FIFOs in ftdi_245_sync_fifo
# - clk_command ↔ lvds_clk_slow_clkin1: Dual-port RAM (sample_ram)
# - lvds_clk_fast_clkin1 → clk_command: 2-FF synchronizer on phase_diff
# - lvds_clk_slow_clkin1 → lvds_clk_fast_clkin1: Phase detector designed for async input
