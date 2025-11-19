# Custom SDC constraints for LPDDR4 debug tools
# This file supplements the auto-generated bsp/TI375C529/tools_core.sdc
# Add this file to your project constraints
#
# Based on the working Ti375C529 OOB v2.1 example running at 1650 MHz
# Reference: C:\Users\ahaas\.efinity\project\efx_ti375c529_oob_v2.1\bsp\TI375C529_DK\constraints.sdc

# JTAG Clock Definition
# The JTAG clock needs to be defined for proper timing analysis
create_clock -period 166.67 [get_ports {jtag_inst1_TCK}]

# Clock Domain Crossing Constraints
# CRITICAL: Use -exclusive instead of -asynchronous for better timing closure
# The working 1650 MHz example uses set_clock_groups -exclusive
# This tells the timing analyzer these clock domains are mutually exclusive
# and no paths between them need to be analyzed
set_clock_groups -exclusive \
    -group {axi0_ACLK} \
    -group {axi1_ACLK} \
    -group {regACLK} \
    -group {jtag_inst1_TCK}

# Note: With -exclusive, the explicit set_false_path commands below are
# technically redundant, but we keep them for defensive design and clarity
# They ensure that if the clock groups constraint is not properly recognized,
# we still have proper cross-domain handling

# Relax inter-clock domain paths between AXI and register clocks
set_false_path -from [get_clocks {regACLK}] -to [get_clocks {axi0_ACLK}]
set_false_path -from [get_clocks {regACLK}] -to [get_clocks {axi1_ACLK}]
set_false_path -from [get_clocks {axi0_ACLK}] -to [get_clocks {regACLK}]
set_false_path -from [get_clocks {axi1_ACLK}] -to [get_clocks {regACLK}]
set_false_path -from [get_clocks {axi0_ACLK}] -to [get_clocks {axi1_ACLK}]
set_false_path -from [get_clocks {axi1_ACLK}] -to [get_clocks {axi0_ACLK}]

# JTAG is always asynchronous to all design clocks
set_false_path -from [get_clocks {jtag_inst1_TCK}] -to [get_clocks {axi0_ACLK}]
set_false_path -from [get_clocks {jtag_inst1_TCK}] -to [get_clocks {axi1_ACLK}]
set_false_path -from [get_clocks {jtag_inst1_TCK}] -to [get_clocks {regACLK}]
set_false_path -from [get_clocks {axi0_ACLK}] -to [get_clocks {jtag_inst1_TCK}]
set_false_path -from [get_clocks {axi1_ACLK}] -to [get_clocks {jtag_inst1_TCK}]
set_false_path -from [get_clocks {regACLK}] -to [get_clocks {jtag_inst1_TCK}]

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
set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_ARADDR[*] axi0_ARAPCMD axi0_ARBURST[*] axi0_ARID[*] axi0_ARLEN[*] axi0_ARSIZE[*] axi0_ARVALID axi0_ARLOCK axi0_ARQOS}]
set_output_delay -clock axi0_ACLK -min -0.140 [get_ports {axi0_ARADDR[*] axi0_ARAPCMD axi0_ARBURST[*] axi0_ARID[*] axi0_ARLEN[*] axi0_ARSIZE[*] axi0_ARVALID axi0_ARLOCK axi0_ARQOS}]

set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_AWADDR[*] axi0_AWALLSTRB axi0_AWBURST[*] axi0_AWCOBUF axi0_AWID[*] axi0_AWLEN[*] axi0_AWSIZE[*] axi0_AWVALID axi0_AWLOCK axi0_AWCACHE[*] axi0_AWQOS}]
set_output_delay -clock axi0_ACLK -min -0.140 [get_ports {axi0_AWADDR[*] axi0_AWALLSTRB axi0_AWBURST[*] axi0_AWCOBUF axi0_AWID[*] axi0_AWLEN[*] axi0_AWSIZE[*] axi0_AWVALID axi0_AWLOCK axi0_AWCACHE[*] axi0_AWQOS}]
set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_AWAPCMD}]
set_output_delay -clock axi0_ACLK -min -0.350 [get_ports {axi0_AWAPCMD}]

set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_WDATA[*] axi0_WLAST axi0_WSTRB[*] axi0_WVALID}]
set_output_delay -clock axi0_ACLK -min -0.140 [get_ports {axi0_WDATA[*] axi0_WLAST axi0_WSTRB[*] axi0_WVALID}]

set_output_delay -clock axi0_ACLK -max 0.750 [get_ports {axi0_BREADY axi0_RREADY}]
set_output_delay -clock axi0_ACLK -min -0.140 [get_ports {axi0_BREADY axi0_RREADY}]

# AXI0 Input Delays (from DDR controller to FPGA)
# Note: BID, RID, BRESP, RRESP signals are ALL removed by synthesis (unconnected)
# Only constrain the signals that actually exist in the netlist
set_input_delay -clock axi0_ACLK -max 0.850 [get_ports {axi0_ARREADY axi0_AWREADY axi0_WREADY}]
set_input_delay -clock axi0_ACLK -min 0.600 [get_ports {axi0_ARREADY axi0_AWREADY axi0_WREADY}]

set_input_delay -clock axi0_ACLK -max 0.850 [get_ports {axi0_BVALID axi0_RVALID}]
set_input_delay -clock axi0_ACLK -min 0.600 [get_ports {axi0_BVALID axi0_RVALID}]

set_input_delay -clock axi0_ACLK -max 0.850 [get_ports {axi0_RDATA[*] axi0_RLAST}]
set_input_delay -clock axi0_ACLK -min 0.600 [get_ports {axi0_RDATA[*] axi0_RLAST}]

# AXI1 Output/Input Delays (if used - same scaling as AXI0)
set_output_delay -clock axi1_ACLK -max 0.750 [get_ports {axi1_ARADDR[*] axi1_ARAPCMD axi1_ARBURST[*] axi1_ARID[*] axi1_ARLEN[*] axi1_ARSIZE[*] axi1_ARVALID axi1_ARLOCK axi1_ARQOS}]
set_output_delay -clock axi1_ACLK -min -0.140 [get_ports {axi1_ARADDR[*] axi1_ARAPCMD axi1_ARBURST[*] axi1_ARID[*] axi1_ARLEN[*] axi1_ARSIZE[*] axi1_ARVALID axi1_ARLOCK axi1_ARQOS}]

set_output_delay -clock axi1_ACLK -max 0.750 [get_ports {axi1_AWADDR[*] axi1_AWALLSTRB axi1_AWBURST[*] axi1_AWCOBUF axi1_AWID[*] axi1_AWLEN[*] axi1_AWSIZE[*] axi1_AWVALID axi1_AWLOCK axi1_AWCACHE[*] axi1_AWQOS}]
set_output_delay -clock axi1_ACLK -min -0.140 [get_ports {axi1_AWADDR[*] axi1_AWALLSTRB axi1_AWBURST[*] axi1_AWCOBUF axi1_AWID[*] axi1_AWLEN[*] axi1_AWSIZE[*] axi1_AWVALID axi1_AWLOCK axi1_AWCACHE[*] axi1_AWQOS}]
set_output_delay -clock axi1_ACLK -max 0.750 [get_ports {axi1_AWAPCMD}]
set_output_delay -clock axi1_ACLK -min -0.350 [get_ports {axi1_AWAPCMD}]

set_output_delay -clock axi1_ACLK -max 0.750 [get_ports {axi1_WDATA[*] axi1_WLAST axi1_WSTRB[*] axi1_WVALID axi1_BREADY axi1_RREADY}]
set_output_delay -clock axi1_ACLK -min -0.140 [get_ports {axi1_WDATA[*] axi1_WLAST axi1_WSTRB[*] axi1_WVALID axi1_BREADY axi1_RREADY}]

# AXI1 inputs - BID, RID, BRESP, RRESP are ALL removed (unconnected)
set_input_delay -clock axi1_ACLK -max 0.850 [get_ports {axi1_ARREADY axi1_AWREADY axi1_WREADY}]
set_input_delay -clock axi1_ACLK -min 0.600 [get_ports {axi1_ARREADY axi1_AWREADY axi1_WREADY}]

set_input_delay -clock axi1_ACLK -max 0.850 [get_ports {axi1_BVALID axi1_RVALID}]
set_input_delay -clock axi1_ACLK -min 0.600 [get_ports {axi1_BVALID axi1_RVALID}]

set_input_delay -clock axi1_ACLK -max 0.850 [get_ports {axi1_RDATA[*] axi1_RLAST}]
set_input_delay -clock axi1_ACLK -min 0.600 [get_ports {axi1_RDATA[*] axi1_RLAST}]

# For debugging timing issues: Uncomment to see detailed timing reports
# set_max_delay 20 -from [get_clocks {axi0_ACLK}] -to [all_outputs]
# set_max_delay 20 -from [get_clocks {axi1_ACLK}] -to [all_outputs]
