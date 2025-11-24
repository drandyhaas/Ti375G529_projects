
module top (

// LVDS
input  [9:0]    lvds_rx_inst1_RX_DATA,
input           lvds_clk_slow_clkin,
input           lvds_clk_fast_clkin,  // Fast clock for phase detector
output          lvds_rx_inst1_RX_RST,

// LVDS Trigger signals for board-to-board communication
input           lvdsin_trig,
output          lvdsout_trig,
input           lvdsin_trig_b,
output          lvdsout_trig_b,
input           lvdsin_spare,
output          lvdsout_spare,

// External trigger input and aux output
input           exttrigin,
output          auxout,

// LEDs
output [3:0]    LED,
output          led2,  // LED2 controlled by triggerer

// USB3 FT601 Interface
input           ftdi_clk,
input           ftdi_rxf_n,
input           ftdi_txe_n,
output          ftdi_oe_n,
output          ftdi_rd_n,
output          ftdi_wr_n,
input  [31:0]   ftdi_data_IN,
output [31:0]   ftdi_data_OUT,
output [31:0]   ftdi_data_OE,
input  [3:0]    ftdi_be_IN,
output [3:0]    ftdi_be_OUT,
output [3:0]    ftdi_be_OE,

// 100MHz clock for USB processing
input           clk_100,

// SPI interface
output [7:0]    spitx,
input  [7:0]    spirx,
input           spitxready,
output          spitxdv,
input           spirxdv,
output [7:0]    spics,
output [2:0]    spimisossel,
output [1:0]    spi_mode,
output          spireset_L,

// Clock control
output          pllreset,
output [2:0]    phasecounterselect,
output          phaseupdown,
output [3:0]    phasestep,
output          scanclk,
output          clkswitch,
output          clkout_ena,
output          clk_over_4,

// Flash interface
output [23:0]   flash_addr,
output          flash_bulk_erase,
output [7:0]    flash_datain,
output          flash_rden,
output          flash_read,
output          flash_write,
output          flash_reset,
input           flash_busy,
input           flash_data_valid,
input  [7:0]    flash_dataout,

// Board I/O
output [11:0]   debugout,
input  [3:0]    overrange,
input  [7:0]    boardin,
output [7:0]    boardout,
input  [3:0]    lockinfo,

// RGB LED control
output [23:0]   neo_color0,
output [23:0]   neo_color1,
output          send_color,

// Reload flash signal
output          reloadflash,

// DDR Interface
input           axi0_ACLK,
output          axi0_ARESETn,
output          axi0_ARQOS,
output          axi0_AWQOS,
output [5:0]    axi0_AWID,
output [32:0]   axi0_AWADDR,
output [7:0]    axi0_AWLEN,
output [2:0]    axi0_AWSIZE,
output [1:0]    axi0_AWBURST,
output          axi0_AWVALID,
output [3:0]    axi0_AWCACHE,
output          axi0_AWCOBUF,
output          axi0_AWLOCK,
output          axi0_AWAPCMD,
output          axi0_AWALLSTRB,
output [5:0]    axi0_ARID,
output [32:0]   axi0_ARADDR,
output [7:0]    axi0_ARLEN,
output [2:0]    axi0_ARSIZE,
output [1:0]    axi0_ARBURST,
output          axi0_ARVALID,
output          axi0_ARLOCK,
output          axi0_ARAPCMD,
output          axi0_WLAST,
output          axi0_WVALID,
output [511:0]  axi0_WDATA,
output [63:0]   axi0_WSTRB,
output          axi0_BREADY,
output          axi0_RREADY,
input           axi0_AWREADY,
input           axi0_ARREADY,
input           axi0_WREADY,
input [5:0]     axi0_BID,
input [1:0]     axi0_BRESP,
input           axi0_BVALID,
input [5:0]     axi0_RID,
input           axi0_RLAST,
input           axi0_RVALID,
input [511:0]   axi0_RDATA,
input [1:0]     axi0_RRESP,

// axi1 interface removed - no longer needed (only using axi0 for memory_checker_lfsr)

output          cfg_sel,
output          cfg_start,
output          cfg_reset,
input           cfg_done,
output          phy_rstn,
output          ctrl_rstn,
input           ddr_pll_lock,
output          ddr_pll_rstn,
//config
input           regACLK,
output [14:0]   regARADDR,
output [5:0]    regARID,
output [7:0]    regARLEN,
output [2:0]    regARSIZE,
output [1:0]    regARBURST,
output          regARVALID,
input           regARREADY,
input [31:0]    regRDATA,
input           regRVALID,
input           regRLAST,
input [1:0]     regRRESP,
input [5:0]     regRID,
output          regRREADY,
output [14:0]   regAWADDR,
output [5:0]    regAWID,
output [7:0]    regAWLEN,
output [2:0]    regAWSIZE,
output [1:0]    regAWBURST,
output          regAWVALID,
input           regAWREADY,

output [31:0]   regWDATA,
output [3:0]    regWSTRB,
output          regWLAST,
output          regWVALID,
input           regWREADY,

output          regBREADY,
input [5:0]     regBID,
input [1:0]     regBRESP,
input           regBVALID,

output          regARESETn
);

// LVDS reset output
assign lvds_rx_inst1_RX_RST = 1'b0;

// Reset signals (tied to not-reset for now)
wire rstn;
assign rstn = 1'b1;

// Internal wires for downsampler connections
wire [559:0] lvdsbitsout;
wire signed [11:0] samplevalue[40];

// Wires between triggerer and downsampler
wire integer downsamplecounter;

// Wires from command_processor to both triggerer and downsampler
wire [7:0] channeltype;
wire [7:0] downsamplemerging;
wire highres;
wire [4:0] downsample;

// Wires from command_processor to triggerer
wire signed [23:0] lowerthresh;
wire signed [23:0] upperthresh;
wire [15:0] lengthtotake;
wire [15:0] prelengthtotake;
wire triggerlive;
wire didreadout;
wire [7:0] triggertype;
wire [7:0] triggerToT;
wire triggerchan;
wire dorolling;
wire [3:0] auxoutselector;
wire [1:0] firstlast;
wire [7:0] trigger_options [2];

// Wires from triggerer to command_processor
wire [7:0] acqstate;
wire [31:0] eventcounter;
wire [9:0] ram_address_triggered;
wire [19:0] sample_triggered;
wire [7:0] downsamplemergingcounter_triggered;
wire [8:0] triggerphase;
wire [31:0] eventtime;
wire [19:0] sample1_triggered;
wire [19:0] sample2_triggered;
wire [19:0] sample3_triggered;
wire [19:0] sample4_triggered;

// RAM interface wires
wire ram_wr;
wire [9:0] ram_wr_address;
wire [9:0] ram_rd_address;
wire [559:0] ram_rd_data;

// Phase detector wires
wire [15:0] phase_diff;
wire [15:0] phase_diff_b;

// Neo color array to individual signals
wire [23:0] neo_color [2];
assign neo_color0 = neo_color[0];
assign neo_color1 = neo_color[1];

// Buffer registers for LVDS data deserialization (14 cycles × 10 bits = 140 bits)
// According to the downsampler comments, each lvdsbits input represents deserialized data
// where each of the 10 bits in lvds_rx_inst1_RX_DATA represents one of 4 channels
reg [139:0] lvds1bits, lvds2bits, lvds3bits, lvds4bits;

// Simple assignment: replicate the 10-bit LVDS data across all four channels
// TODO: Verify the correct mapping between lvds_rx_inst1_RX_DATA bits and the four channels
always @(posedge lvds_clk_slow_clkin) begin
   // Direct assignment - each bit of lvds_rx_inst1_RX_DATA feeds all four channels
   // This is a placeholder and may need adjustment based on actual LVDS protocol
   lvds1bits <= {14{lvds_rx_inst1_RX_DATA}};
   lvds2bits <= {14{lvds_rx_inst1_RX_DATA}};
   lvds3bits <= {14{lvds_rx_inst1_RX_DATA}};
   lvds4bits <= {14{lvds_rx_inst1_RX_DATA}};
end

// Dual-port RAM for storing LVDS samples
sample_ram sample_ram_inst (
   .clk_wr(lvds_clk_slow_clkin),
   .wr_en(ram_wr),
   .wr_addr(ram_wr_address),
   .wr_data(lvdsbitsout),
   .clk_rd(clk_100),
   .rd_addr(ram_rd_address),
   .rd_data(ram_rd_data)
);

// Downsampler instantiation
downsampler downsampler_inst (
   .clklvds(lvds_clk_slow_clkin),
   .lvds1bits(lvds1bits),
   .lvds2bits(lvds2bits),
   .lvds3bits(lvds3bits),
   .lvds4bits(lvds4bits),
   .lvdsbitsout(lvdsbitsout),
   .downsamplecounter(downsamplecounter),
   .samplevalue(samplevalue),
   .channeltype(channeltype),
   .downsamplemerging(downsamplemerging),
   .highres(highres),
   .downsample(downsample)
);

// Phase detector for forward trigger path
phase_detector phase_det_fwd (
   .clk_fast(lvds_clk_fast_clkin),
   .start(lvdsout_trig),
   .stop(lvdsin_trig_b),
   .phase_diff(phase_diff)
);

// Phase detector for backward trigger path
phase_detector phase_det_bwd (
   .clk_fast(lvds_clk_fast_clkin),
   .start(lvdsout_trig_b),
   .stop(lvdsin_trig),
   .phase_diff(phase_diff_b)
);

// Triggerer instantiation
triggerer triggerer_inst (
   .clklvds(lvds_clk_slow_clkin),
   .rstn(rstn),
   .ram_wr(ram_wr),
   .ram_wr_address(ram_wr_address),
   .samplevalue(samplevalue),
   .lvdsin_trig(lvdsin_trig),
   .lvdsout_trig(lvdsout_trig),
   .lvdsin_trig_b(lvdsin_trig_b),
   .lvdsout_trig_b(lvdsout_trig_b),
   .exttrigin(exttrigin),
   .auxout(auxout),
   .led(led2),
   .acqstate(acqstate),
   .eventcounter(eventcounter),
   .ram_address_triggered(ram_address_triggered),
   .sample_triggered(sample_triggered),
   .downsamplemergingcounter_triggered(downsamplemergingcounter_triggered),
   .triggerphase(triggerphase),
   .downsamplecounter(downsamplecounter),
   .eventtime(eventtime),
   .sample1_triggered(sample1_triggered),
   .sample2_triggered(sample2_triggered),
   .sample3_triggered(sample3_triggered),
   .sample4_triggered(sample4_triggered),
   .lowerthresh(lowerthresh),
   .upperthresh(upperthresh),
   .lengthtotake(lengthtotake),
   .prelengthtotake(prelengthtotake),
   .triggerlive(triggerlive),
   .didreadout(didreadout),
   .triggertype(triggertype),
   .triggerToT(triggerToT),
   .triggerchan(triggerchan),
   .dorolling(dorolling),
   .auxoutselector(auxoutselector),
   .channeltype(channeltype),
   .downsamplemerging(downsamplemerging),
   .downsample(downsample),
   .firstlast(firstlast),
   .trigger_options(trigger_options)
);

// Command processor instantiation
// Note: USB AXI-stream interface removed - needs to be replaced with simple interface
// TODO: Create simple command/data interface to replace AXI-stream
command_processor cmd_proc_inst (
   .rstn(rstn),
   .clk(clk_100),

   // USB interface - STUBBED OUT (to be replaced)
   .i_tready(),      // TODO: Connect to USB3 interface
   .i_tvalid(1'b0),  // TODO: Connect to USB3 interface
   .i_tdata(8'd0),   // TODO: Connect to USB3 interface
   .o_tready(1'b1),  // TODO: Connect to USB3 interface
   .o_tvalid(),      // TODO: Connect to USB3 interface
   .o_tdata(),       // TODO: Connect to USB3 interface
   .o_tkeep(),       // TODO: Connect to USB3 interface
   .o_tlast(),       // TODO: Connect to USB3 interface

   .pllreset(pllreset),

   // SPI interface
   .spitx(spitx),
   .spirx(spirx),
   .spitxready(spitxready),
   .spitxdv(spitxdv),
   .spirxdv(spirxdv),
   .spics(spics),
   .spimisossel(spimisossel),
   .spi_mode(spi_mode),
   .spireset_L(spireset_L),

   .lockinfo(lockinfo),

   // RAM interface
   .ram_rd_address(ram_rd_address),
   .lvdsbitsin(ram_rd_data),

   // PLL phase control
   .phasecounterselect(phasecounterselect),
   .phaseupdown(phaseupdown),
   .phasestep(phasestep),
   .scanclk(scanclk),

   .debugout(debugout),
   .overrange(overrange),
   .boardin(boardin),
   .boardout(boardout),
   .clkswitch(clkswitch),
   .lvdsin_spare(lvdsin_spare),
   .lvdsout_spare(lvdsout_spare),
   .clk50(clk_100),  // Using clk_100 as requested
   .clk_over_4(clk_over_4),

   // Flash interface
   .flash_addr(flash_addr),
   .flash_bulk_erase(flash_bulk_erase),
   .flash_datain(flash_datain),
   .flash_rden(flash_rden),
   .flash_read(flash_read),
   .flash_write(flash_write),
   .flash_reset(flash_reset),
   .flash_busy(flash_busy),
   .flash_data_valid(flash_data_valid),
   .flash_dataout(flash_dataout),

   .clkout_ena(clkout_ena),

   // RGB LED control
   .neo_color(neo_color),
   .send_color(send_color),

   // Outputs to triggerer
   .lowerthresh(lowerthresh),
   .upperthresh(upperthresh),
   .lengthtotake(lengthtotake),
   .prelengthtotake(prelengthtotake),
   .triggerlive(triggerlive),
   .didreadout(didreadout),
   .triggertype(triggertype),
   .triggerToT(triggerToT),
   .triggerchan(triggerchan),
   .dorolling(dorolling),
   .auxoutselector(auxoutselector),
   .channeltype(channeltype),
   .downsamplemerging(downsamplemerging),
   .highres(highres),
   .downsample(downsample),
   .firstlast(firstlast),
   .trigger_options(trigger_options),

   .reloadflash(reloadflash),

   // Inputs from triggerer
   .acqstate(acqstate),
   .eventcounter(eventcounter),
   .ram_address_triggered(ram_address_triggered),
   .sample_triggered(sample_triggered),
   .downsamplemergingcounter_triggered(downsamplemergingcounter_triggered),
   .triggerphase(triggerphase),
   .eventtime(eventtime),
   .phase_diff(phase_diff),
   .phase_diff_b(phase_diff_b),
   .sample1_triggered(sample1_triggered),
   .sample2_triggered(sample2_triggered),
   .sample3_triggered(sample3_triggered),
   .sample4_triggered(sample4_triggered)
);

tools_core core0(

// LEDs
.LED(LED),

// USB3 Interface
.ftdi_clk(ftdi_clk),
.ftdi_rxf_n(ftdi_rxf_n),
.ftdi_txe_n(ftdi_txe_n),
.ftdi_oe_n(ftdi_oe_n),
.ftdi_rd_n(ftdi_rd_n),
.ftdi_wr_n(ftdi_wr_n),
.ftdi_data_IN(ftdi_data_IN),
.ftdi_data_OUT(ftdi_data_OUT),
.ftdi_data_OE(ftdi_data_OE),
.ftdi_be_IN(ftdi_be_IN),
.ftdi_be_OUT(ftdi_be_OUT),
.ftdi_be_OE(ftdi_be_OE),

// DDR Interface
.axi0_ACLK(axi0_ACLK),
.axi0_ARESETn(axi0_ARESETn),
.axi0_ARQOS(axi0_ARQOS),
.axi0_AWQOS(axi0_AWQOS),
.axi0_AWID(axi0_AWID),
.axi0_AWADDR(axi0_AWADDR),
.axi0_AWLEN(axi0_AWLEN),
.axi0_AWSIZE(axi0_AWSIZE),
.axi0_AWBURST(axi0_AWBURST),
.axi0_AWVALID(axi0_AWVALID),
.axi0_AWCACHE(axi0_AWCACHE),
.axi0_AWCOBUF(axi0_AWCOBUF),
.axi0_AWLOCK(axi0_AWLOCK),
.axi0_AWAPCMD(axi0_AWAPCMD),
.axi0_AWALLSTRB(axi0_AWALLSTRB),
.axi0_ARID(axi0_ARID),
.axi0_ARADDR(axi0_ARADDR),
.axi0_ARLEN(axi0_ARLEN),
.axi0_ARSIZE(axi0_ARSIZE),
.axi0_ARBURST(axi0_ARBURST),
.axi0_ARVALID(axi0_ARVALID),
.axi0_ARLOCK(axi0_ARLOCK),
.axi0_ARAPCMD(axi0_ARAPCMD),
.axi0_WLAST(axi0_WLAST),
.axi0_WVALID(axi0_WVALID),
.axi0_WDATA(axi0_WDATA),
.axi0_WSTRB(axi0_WSTRB),
.axi0_BREADY(axi0_BREADY),
.axi0_RREADY(axi0_RREADY),
.axi0_AWREADY(axi0_AWREADY),
.axi0_ARREADY(axi0_ARREADY),
.axi0_WREADY(axi0_WREADY),
.axi0_BID(axi0_BID),
.axi0_BRESP(axi0_BRESP),
.axi0_BVALID(axi0_BVALID),
.axi0_RID(axi0_RID),
.axi0_RLAST(axi0_RLAST),
.axi0_RVALID(axi0_RVALID),
.axi0_RDATA(axi0_RDATA),
.axi0_RRESP(axi0_RRESP),

// axi1 interface connections removed

.cfg_sel(cfg_sel),
.cfg_start(cfg_start),
.cfg_reset(cfg_reset),
.cfg_done(cfg_done),
.phy_rstn(phy_rstn),
.ctrl_rstn(ctrl_rstn),
.ddr_pll_lock(ddr_pll_lock),
.ddr_pll_rstn(ddr_pll_rstn),

.regACLK(regACLK),
.regARADDR(regARADDR),
.regARID(regARID),
.regARLEN(regARLEN),
.regARSIZE(regARSIZE),
.regARBURST(regARBURST),
.regARVALID(regARVALID),
.regARREADY(regARREADY),
.regRDATA(regRDATA),
.regRVALID(regRVALID),
.regRLAST(regRLAST),
.regRRESP(regRRESP),
.regRID(regRID),
.regRREADY(regRREADY),
.regAWADDR(regAWADDR),
.regAWID(regAWID),
.regAWLEN(regAWLEN),
.regAWSIZE(regAWSIZE),
.regAWBURST(regAWBURST),
.regAWVALID(regAWVALID),
.regAWREADY(regAWREADY),

.regWDATA(regWDATA),
.regWSTRB(regWSTRB),
.regWLAST(regWLAST),
.regWVALID(regWVALID),
.regWREADY(regWREADY),

.regBREADY(regBREADY),
.regBID(regBID),
.regBRESP(regBRESP),
.regBVALID(regBVALID),

.regARESETn(regARESETn)
);

endmodule