
//--------------------------------------------------------------------------------------------------------
// Module  : fpga_top_ft600_tx_mass
// Type    : synthesizable, FPGA's top, IP's example design
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: an example of ftdi_245fifo_top
//           the pins of this module should connect to FT600 chip
//           This design will receive 4 bytes from FTDI chip,
//           and then regard the 4 bytes as a length, send length of bytes to FTDI chip
//--------------------------------------------------------------------------------------------------------

module fpga_top_ft600_tx_mass (
    input  wire         clk_100,            // main clock, connect to on-board crystal oscillator
    
    output wire  [ 3:0] LED,
    
    // USB3.0 (FT600 chip) ------------------------------------------------------------
    output wire         ftdi_resetn,    // to FT600's pin10 (RESET_N) , !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!
    output wire         ftdi_wakeupn,   // to FT600's pin11 (WAKEUP_N), !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!
    output wire         ftdi_gpio0,     // to FT600's pin12 (GPIO0)   , !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!
    output wire         ftdi_gpio1,     // to FT600's pin13 (GPIO1)   , !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!
    output wire         ftdi_siwu,      // to FT600's pin6  (SIWU_N)  , !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!
    input  wire         ftdi_clk,       // to FT600's pin43 (CLK)
    input  wire         ftdi_rxf_n,     // to FT600's pin5  (RXF_N)
    input  wire         ftdi_txe_n,     // to FT600's pin4  (TXE_N)
    output wire         ftdi_oe_n,      // to FT600's pin9  (OE_N)
    output wire         ftdi_rd_n,      // to FT600's pin8  (RD_N)
    output wire         ftdi_wr_n,      // to FT600's pin7  (WR_N)
    
    input      [31:0] ftdi_data_IN,      // to FT600's pin56~53 (DATA_15~DATA_12) , pin48~45 (DATA_11~DATA_8) , pin42~39 (DATA_7~DATA4) and pin36~33 (DATA_3~DATA_0)
    output     [31:0] ftdi_data_OUT,
    output     [31:0] ftdi_data_OE,
    
    input  wire      [ 3:0] ftdi_be_IN,         // to FT600's pin3 (BE_1) and pin2 (BE_0)
    output  wire     [ 3:0] ftdi_be_OUT,
    output  wire     [ 3:0] ftdi_be_OE,
    
    
    output  [7:0] 	    ddr_inst1_DDR_AID_0,
    output  [31:0] 	    ddr_inst1_DDR_AADDR_0,
    output  [7:0]   	ddr_inst1_DDR_ALEN_0,
    output  [2:0]   	ddr_inst1_DDR_ASIZE_0,
    output  [1:0]   	ddr_inst1_DDR_ABURST_0,
    output  [1:0]   	ddr_inst1_DDR_ALOCK_0,
    output 			    ddr_inst1_DDR_AVALID_0,
    input 			    ddr_inst1_DDR_AREADY_0,
    output  		    ddr_inst1_DDR_ATYPE_0,

    output [7:0] 		ddr_inst1_DDR_WID_0,
    output [255:0] 	    ddr_inst1_DDR_WDATA_0,
    output [31:0] 		ddr_inst1_DDR_WSTRB_0,
    output  			ddr_inst1_DDR_WLAST_0,
    output  			ddr_inst1_DDR_WVALID_0,
    input 				ddr_inst1_DDR_WREADY_0,

    input   [7:0] 		ddr_inst1_DDR_RID_0,
    input   [255:0] 	ddr_inst1_DDR_RDATA_0,
    input				ddr_inst1_DDR_RLAST_0,
    input				ddr_inst1_DDR_RVALID_0,
    output  			ddr_inst1_DDR_RREADY_0,
    input   [1:0] 		ddr_inst1_DDR_RRESP_0,

    input   [7:0] 		ddr_inst1_DDR_BID_0,
    input 				ddr_inst1_DDR_BVALID_0,
    output  			ddr_inst1_DDR_BREADY_0,

    output              ddr_inst1_RSTN,
    output              ddr_inst1_CFG_SEQ_RST,
    output              ddr_inst1_CFG_SEQ_START,


    input               jtag_inst1_DRCK,
    input               jtag_inst1_RUNTEST,
    input               jtag_inst1_TCK,
    input               jtag_inst1_TDI,
    output              jtag_inst1_TDO,
    input               jtag_inst1_SEL,
    input               jtag_inst1_CAPTURE,
    input               jtag_inst1_SHIFT,
    input               jtag_inst1_UPDATE,
    input               jtag_inst1_RESET

    
);



assign ftdi_resetn = 1'b1;  // 1=normal operation          , !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!
assign ftdi_wakeupn= 1'b0;  // 0=wake up                   , !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!
assign ftdi_gpio0  = 1'b0;  // GPIO[1:0]=00 = 245fifo mode , !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!
assign ftdi_gpio1  = 1'b0;  //                             , !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!
assign ftdi_siwu   = 1'b0;  // 0=send immidiently          , !!!!!! UnComment this line if this signal is connected to FPGA. !!!!!!




//-----------------------------------------------------------------------------------------------------------------------------
// user AXI-stream signals (loopback)
//-----------------------------------------------------------------------------------------------------------------------------
wire        rx_tready;
wire        rx_tvalid;
wire [ 7:0] rx_tdata;

wire        tx_tready;
wire        tx_tvalid;
wire [31:0] tx_tdata;
wire [ 3:0] tx_tkeep;
wire        tx_tlast;




//-----------------------------------------------------------------------------------------------------------------------------
// FTDI USB chip's 245fifo mode controller
//-----------------------------------------------------------------------------------------------------------------------------
ftdi_245fifo_top #(
    .TX_EW                 ( 2                  ),   // TX data stream width,  0=8bit, 1=16bit, 2=32bit, 3=64bit, 4=128bit ...
    .TX_EA                 ( 14                 ),   // TX FIFO depth = 2^TX_AEXP = 2^10 = 1024
    .RX_EW                 ( 0                  ),   // RX data stream width,  0=8bit, 1=16bit, 2=32bit, 3=64bit, 4=128bit ...
    .RX_EA                 ( 8                  ),   // RX FIFO depth = 2^RX_AEXP = 2^10 = 1024
    .CHIP_TYPE             ( "FT601"            )
) u_ftdi_245fifo_top (
    .rstn_async            ( 1'b1               ),
    .tx_clk                ( clk_100                ),
    .tx_tready             ( tx_tready          ),
    .tx_tvalid             ( tx_tvalid          ),
    .tx_tdata              ( tx_tdata           ),
    .tx_tkeep              ( tx_tkeep           ),
    .tx_tlast              ( tx_tlast           ),
    .rx_clk                ( clk_100                ),
    .rx_tready             ( rx_tready          ),
    .rx_tvalid             ( rx_tvalid          ),
    .rx_tdata              ( rx_tdata           ),
    .rx_tkeep              (                    ),
    .rx_tlast              (                    ),
    .ftdi_clk              ( ftdi_clk           ),
    .ftdi_rxf_n            ( ftdi_rxf_n         ),
    .ftdi_txe_n            ( ftdi_txe_n         ),
    .ftdi_oe_n             ( ftdi_oe_n          ),
    .ftdi_rd_n             ( ftdi_rd_n          ),
    .ftdi_wr_n             ( ftdi_wr_n          ),
    .ftdi_data_IN             ( ftdi_data_IN          ),
    .ftdi_data_OUT             ( ftdi_data_OUT          ),
    .ftdi_data_OE             ( ftdi_data_OE          ),
    .ftdi_be_IN               ( ftdi_be_IN            ),
    .ftdi_be_OUT               ( ftdi_be_OUT            ),
    .ftdi_be_OE               ( ftdi_be_OE            )
);




//-----------------------------------------------------------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------------------------------------------------------
tx_specified_len u_tx_specified_len (
    .rstn                  ( 1'b1               ),
    .clk                   ( clk_100                ),
    .i_tready              ( rx_tready          ),
    .i_tvalid              ( rx_tvalid          ),
    .i_tdata               ( rx_tdata           ),
    .o_tready              ( tx_tready          ),
    .o_tvalid              ( tx_tvalid          ),
    .o_tdata               ( tx_tdata           ),
    .o_tkeep               ( tx_tkeep           ),
    .o_tlast               ( tx_tlast           )
);




//-----------------------------------------------------------------------------------------------------------------------------
// show the low 2-bit of the last received data on LED
//-----------------------------------------------------------------------------------------------------------------------------
reg  [1:0] tdata_d = 2'h0;

always @ (posedge clk_100)
    if (rx_tvalid)
        tdata_d <= rx_tdata[1:0];

assign LED[1:0] = tdata_d;




//-----------------------------------------------------------------------------------------------------------------------------
// if ftdi_clk continuous run, then beat will blink. The function of this module is to observe whether ftdi_clk is running
//-----------------------------------------------------------------------------------------------------------------------------

clock_beat # (
    .CLK_FREQ              ( 100000000          ),
    .BEAT_FREQ             ( 5                  )
) u_clk_beat (
    .clk                   ( clk_100           ),
    .beat                  ( LED[2]             )
);

clock_beat # (
    .CLK_FREQ              ( 100000000          ),
    .BEAT_FREQ             ( 5                  )
) u_ftdi_clk_beat (
    .clk                   ( ftdi_clk           ),
    .beat                  ( LED[3]             )
);


assign fan_speed_control = 0;



localparam      AXI_SPEED_MHZ	=	100;


reg     [31:0]  r_counter;
reg             r_divide_start;
reg             r_led;

wire    [31:0]  w_band_width;
wire    [63:0]  w_divide_out;
wire            w_async_rst;


wire            w_trig_out_debug;
wire    [3:0]   w_pattern_num;

wire    [31:0]  w_loop_n;
wire            w_decode_trig;
wire    [3:0]   w_states_debug;

wire    [31:0]  w_pattern;
wire    [15:0]  w_o_pattern;
wire    [6:0]   w_pattern_cnt;
wire            w_pattern_done;

wire            w_Start;
wire    [63:0]  w_timer_counter;
wire    [7:0]   w_write_cnt;
wire    [255:0] w_i_pattern_len;
wire    [511:0] w_i_pattern_addr;
wire    [63:0]	w_total_len;
wire            w_ddr_init_done;
wire            w_axi_ctrl_rstn;

wire            w_pause;
wire            w_err;
wire [32:0]     w_loop_cnt_n;
wire [127:0]    w_lfsr;

assign  w_decode_trig =1'b1;
assign  pll_rst = ftdi_resetn;
assign  test_led = r_led;
assign  w_axi_ctrl_rstn = w_ddr_init_done & w_async_rst;
assign  w_band_width = w_divide_out[31:0];

always@(posedge clk_100 or negedge ftdi_resetn)
begin
    if(~ftdi_resetn)
    begin
        r_led     <=1'b0;
        r_counter <=32'b0;
    end
    else
    begin
        if(r_counter > 12500000)
        begin
            r_led <= (~r_led);
            r_counter <=32'b0;
        end
        else
        begin
            r_counter <=r_counter +1'b1;    
        end
    end
end


always@(posedge clk_100 or negedge ftdi_resetn)
begin
	if(~ftdi_resetn)
    begin
		r_divide_start <=1'b0;
	end
	else
	begin
		if(w_pattern_done)
			r_divide_start <=1'b1;
		else
			r_divide_start <=1'b0;
	end
end


Divide 
#(
    .WIDTH(64)
)
division0
(
	.clk    (clk_100),  
	.rstn   (w_axi_ctrl_rstn),  
	.start  (r_divide_start),  
	.N      ((w_total_len<<5)*AXI_SPEED_MHZ),  //mutiple 256 bit/8 =32byte*100Mhz
	.D      (w_timer_counter),  
	.Q      (w_divide_out),
    .R      ()
);



ddr_reset_sequencer ddr_reset_sequencer0(

 .ddr_rstn_i        (w_async_rst & ddr_pll_lock),				// main user DDR reset, active low
 .clk               (clk_100),						// user clock

/* Connect these three signals to DDR reset interface */
 .ddr_rstn          (ddr_inst1_RSTN),				// Master Reset
 .ddr_cfg_seq_rst   (ddr_inst1_CFG_SEQ_RST),			// Sequencer Reset
 .ddr_cfg_seq_start (ddr_inst1_CFG_SEQ_START),	// Sequencer Start

/* optional status monitor for user logic */
 .ddr_init_done (w_ddr_init_done)		// Done status

);



axi_ctrl axi_ctrl0
(
    .axi_clk    	(clk_100),
    .rstn       	(w_axi_ctrl_rstn), 
	.INADDR     	(w_i_pattern_addr),    
    
    .TRIGGER       	(w_decode_trig),
	.LOOP_N			(w_loop_n),
    .PATTERN_NUMBER (w_pattern_num),
    .IN_PATTERN 	(w_o_pattern),

 	.DDR_AID_0      (ddr_inst1_DDR_AID_0),
  	.DDR_AADDR_0    (ddr_inst1_DDR_AADDR_0),
  	.DDR_ALEN_0     (ddr_inst1_DDR_ALEN_0),
  	.DDR_ASIZE_0    (ddr_inst1_DDR_ASIZE_0),
   	.DDR_ABURST_0   (ddr_inst1_DDR_ABURST_0),
    .DDR_ALOCK_0    (ddr_inst1_DDR_ALOCK_0),
    .DDR_AVALID_0   (ddr_inst1_DDR_AVALID_0),
    .DDR_AREADY_0   (ddr_inst1_DDR_AREADY_0),
    .DDR_ATYPE_0    (ddr_inst1_DDR_ATYPE_0),

	.DDR_WID_0      (ddr_inst1_DDR_WID_0),
    .DDR_WDATA_0    (ddr_inst1_DDR_WDATA_0),
    .DDR_WSTRB_0    (ddr_inst1_DDR_WSTRB_0),
    .DDR_WLAST_0    (ddr_inst1_DDR_WLAST_0),
    .DDR_WVALID_0   (ddr_inst1_DDR_WVALID_0),
    .DDR_WREADY_0   (ddr_inst1_DDR_WREADY_0),

    .DDR_RID_0      (ddr_inst1_DDR_RID_0),
	.DDR_RDATA_0    (ddr_inst1_DDR_RDATA_0),
    .DDR_RLAST_0    (ddr_inst1_DDR_RLAST_0),
    .DDR_RVALID_0   (ddr_inst1_DDR_RVALID_0),
    .DDR_RREADY_0   (ddr_inst1_DDR_RREADY_0),
    .DDR_RRESP_0    (ddr_inst1_DDR_RRESP_0),

    .DDR_BID_0      (ddr_inst1_DDR_BID_0),
    .DDR_BVALID_0   (ddr_inst1_DDR_BVALID_0),
    .DDR_BREADY_0   (ddr_inst1_DDR_BREADY_0),

    .o_states           (w_states_debug),
    .o_out_trig         (w_trig_out_debug),
	.i_pattern_len	    (w_i_pattern_len),
    .o_pattern_cnt	    (w_pattern_cnt),
    .o_pattern_done	    (w_pattern_done),
    .o_Start		    (w_Start),
    .o_time_counter	    (w_timer_counter),
	.o_write_cnt	    (w_write_cnt),
	.o_total_len	    (w_total_len),
    .o_loop_n           (w_loop_cnt_n),
    .i_lfsr_seed        (128'h5555AAAA5555AAAA5555AAAA5555AAAA),
    .r_lfsr_1P          (w_lfsr),
    .i_pause            (w_pause),
    .o_compare_error    (w_err)
    
);




edb_top debug0(
    // debug core ports
    .bscan_CAPTURE  (jtag_inst1_CAPTURE),
    .bscan_DRCK     (jtag_inst1_DRCK),
    .bscan_RESET    (jtag_inst1_RESET),
    .bscan_RUNTEST  (jtag_inst1_RUNTEST),
    .bscan_SEL      (jtag_inst1_SEL),
    .bscan_SHIFT    (jtag_inst1_SHIFT),
    .bscan_TCK      (jtag_inst1_TCK),
    .bscan_TDI      (jtag_inst1_TDI),
    .bscan_TMS      (jtag_inst1_TMS),
    .bscan_UPDATE   (jtag_inst1_UPDATE),
    .bscan_TDO      (jtag_inst1_TDO),


    .la0_clk            (clk_100),
    .la0_rstn           (w_async_rst),
    .la0_ddr_rst_done   (w_ddr_init_done),

    .la0_DDR_WSTRB_0    (ddr_inst1_DDR_WSTRB_0),
    .la0_DDR_BVALID_0   (ddr_inst1_DDR_BVALID_0),
    .la0_DDR_RRESP_0    (ddr_inst1_DDR_RRESP_0),
    .la0_DDR_RLAST_0    (ddr_inst1_DDR_RLAST_0),
    .la0_DDR_WVALID_0   (ddr_inst1_DDR_WVALID_0),    
    .la0_DDR_BREADY_0   (ddr_inst1_DDR_BREADY_0), 
    .la0_DDR_AVALID_0   (ddr_inst1_DDR_AVALID_0),
    .la0_DDR_ALOCK_0    (ddr_inst1_DDR_ALOCK_0),    
    .la0_DDR_ABURST_0   (ddr_inst1_DDR_ABURST_0),    
	
    .la0_DDR_WDATA_0    (ddr_inst1_DDR_WDATA_0[31:0]),
	.la0_DDR_WDATA_1    (ddr_inst1_DDR_WDATA_0[63:32]),
	.la0_DDR_WDATA_2    (ddr_inst1_DDR_WDATA_0[95:64]),
	.la0_DDR_WDATA_3    (ddr_inst1_DDR_WDATA_0[127:96]),
	.la0_DDR_WDATA_4    (ddr_inst1_DDR_WDATA_0[159:128]),
	.la0_DDR_WDATA_5    (ddr_inst1_DDR_WDATA_0[191:160]),
	.la0_DDR_WDATA_6    (ddr_inst1_DDR_WDATA_0[223:192]),
	.la0_DDR_WDATA_7    (ddr_inst1_DDR_WDATA_0[255:224]),
	
    .la0_DDR_AREADY_0   (ddr_inst1_DDR_AREADY_0),
    .la0_DDR_WID_0      (ddr_inst1_DDR_WID_0),
    .la0_DDR_WREADY_0   (ddr_inst1_DDR_WREADY_0),
    .la0_DDR_RDATA_0    (ddr_inst1_DDR_RDATA_0[31:0]),
	.la0_DDR_RDATA_1    (ddr_inst1_DDR_RDATA_0[63:32]),
	.la0_DDR_RDATA_2    (ddr_inst1_DDR_RDATA_0[95:64]),
	.la0_DDR_RDATA_3    (ddr_inst1_DDR_RDATA_0[127:96]),
	.la0_DDR_RDATA_4    (ddr_inst1_DDR_RDATA_0[159:128]),
	.la0_DDR_RDATA_5    (ddr_inst1_DDR_RDATA_0[191:160]),
	.la0_DDR_RDATA_6    (ddr_inst1_DDR_RDATA_0[223:192]),
	.la0_DDR_RDATA_7    (ddr_inst1_DDR_RDATA_0[255:224]),
    .la0_DDR_RID_0      (ddr_inst1_DDR_RID_0),
    .la0_DDR_RVALID_0   (ddr_inst1_DDR_RVALID_0),
    .la0_DDR_ASIZE_0    (ddr_inst1_DDR_ASIZE_0),    
    .la0_DDR_AID_0      (ddr_inst1_DDR_AID_0),
    .la0_DDR_ALEN_0     (ddr_inst1_DDR_ALEN_0),    
    .la0_DDR_ATYPE_0    (ddr_inst1_DDR_ATYPE_0),
    .la0_DDR_BID_0      (ddr_inst1_DDR_BID_0),
    .la0_DDR_AADDR_0    (ddr_inst1_DDR_AADDR_0),    
    .la0_DDR_WLAST_0    (ddr_inst1_DDR_WLAST_0),
    .la0_DDR_RREADY_0   (ddr_inst1_DDR_RREADY_0),

    .la0_TRIG           ( w_trig_out_debug ),
    .la0_STATE          ( w_states_debug ),
    .la0_PATTERN        ( w_pattern ),
    .la0_PATTERN_NUM    ( w_pattern_num ),
    .la0_OUT_PATTERN    ( w_o_pattern ),
    .la0_PATTERN_DONE   ( w_pattern_done ),
    .la0_CURRENT_PATTERN( w_pattern_cnt ),
    .la0_START          ( w_Start ),
    .la0_Timer_counter  (w_timer_counter),
	.la0_WR_Counte      (w_write_cnt),
    .la0_total_len      (w_total_len),
    .la0_LOOP_N         (w_loop_cnt_n),
    .la0_lfsr1          (w_lfsr),
    .la0_pause          (w_pause),
    .la0_ERROR          (w_err),


    .vio0_clk              (clk_100),            
    
	.vio0_LOOP_N            ( w_loop_n ),
	.vio0_PATTERN_DEPTH     ( w_pattern_num ),
    .vio0_PATTERN           ( w_o_pattern ),
	
	.vio0_PATTERN_LEN0    ( w_i_pattern_len[7:0]),
	.vio0_PATTERN_LEN1    ( w_i_pattern_len[15:8]),
	.vio0_PATTERN_LEN2    ( w_i_pattern_len[23:16]),
	.vio0_PATTERN_LEN3    ( w_i_pattern_len[31:24]),
	.vio0_PATTERN_LEN4    ( w_i_pattern_len[39:32]),
	.vio0_PATTERN_LEN5    ( w_i_pattern_len[47:40]),
	.vio0_PATTERN_LEN6    ( w_i_pattern_len[55:48]),
	.vio0_PATTERN_LEN7    ( w_i_pattern_len[63:56]),
	.vio0_PATTERN_LEN8    ( w_i_pattern_len[71:64]),
	.vio0_PATTERN_LEN9    ( w_i_pattern_len[79:72]),
	.vio0_PATTERN_LEN10   ( w_i_pattern_len[87:80]),
	.vio0_PATTERN_LEN11   ( w_i_pattern_len[95:88]),
	.vio0_PATTERN_LEN12   ( w_i_pattern_len[103:96]),
	.vio0_PATTERN_LEN13   ( w_i_pattern_len[111:104]),
	.vio0_PATTERN_LEN14   ( w_i_pattern_len[119:112]),
	.vio0_PATTERN_LEN15   ( w_i_pattern_len[127:120]),
	
	
	.vio0_PATTERN_ADDR0   ( w_i_pattern_addr[31:0]),
	.vio0_PATTERN_ADDR1   ( w_i_pattern_addr[63:32]),
	.vio0_PATTERN_ADDR2   ( w_i_pattern_addr[95:64]),
	.vio0_PATTERN_ADDR3   ( w_i_pattern_addr[127:96]),
	.vio0_PATTERN_ADDR4   ( w_i_pattern_addr[159:128]),
	.vio0_PATTERN_ADDR5   ( w_i_pattern_addr[191:160]),
	.vio0_PATTERN_ADDR6   ( w_i_pattern_addr[223:192]),
	.vio0_PATTERN_ADDR7   ( w_i_pattern_addr[255:224]),
	.vio0_PATTERN_ADDR8   ( w_i_pattern_addr[287:256]),
	.vio0_PATTERN_ADDR9   ( w_i_pattern_addr[319:288]),
	.vio0_PATTERN_ADDR10  ( w_i_pattern_addr[351:320]),
	.vio0_PATTERN_ADDR11  ( w_i_pattern_addr[383:352]),
	.vio0_PATTERN_ADDR12  ( w_i_pattern_addr[415:384]),
	.vio0_PATTERN_ADDR13  ( w_i_pattern_addr[447:416]),
	.vio0_PATTERN_ADDR14  ( w_i_pattern_addr[479:448]),
	.vio0_PATTERN_ADDR15  ( w_i_pattern_addr[511:480]),
	
    .vio0_Reset             (w_async_rst),
    .vio0_Pause             (w_pause),
	.vio0_Total_ALEN        (w_total_len),
    .vio0_Cycle_Counter     (w_timer_counter),
    .vio0_Test_Done         (w_pattern_done),
	.vio0_Band_Width_MBs    (w_band_width),	
    .vio0_ERROR             (w_err)
    
);


endmodule
