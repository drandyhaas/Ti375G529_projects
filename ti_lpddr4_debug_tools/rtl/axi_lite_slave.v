module axi_lite_slave #(
	parameter ADDR_WIDTH = 32,
	parameter DATA_WIDTH = 32
) (
    input			        axi_aclk,
	input			        axi_resetn,
	//AW
	input [ADDR_WIDTH-1:0]	axi_awaddr,
	output			        axi_awready,
	input 			        axi_awvalid,

    //W
    output			        axi_wready,
	input [DATA_WIDTH-1:0]	axi_wdata,
    input			        axi_wvalid,
    input 			        axi_wlast,
	input [(DATA_WIDTH/8)-1:0] axi_wstrb,

    //B
	output [7:0]	        axi_bid,        //not use
	output [1:0]	        axi_bresp,      //not use
	output 			        axi_bvalid,
	input			        axi_bready,

    //AR
    input [ADDR_WIDTH-1:0]	axi_araddr,
	input 			        axi_arvalid,
	output			        axi_arready,

    //R
	output [7:0]		    axi_rid,        //not use
	output [1:0]		    axi_rresp,      //not use
    input			        axi_rready,
    output [DATA_WIDTH-1:0]	axi_rdata,
    output			        axi_rvalid,
    output 			        axi_rlast,

	output[31:0]				db_reg0,
	output[31:0]				db_reg1,
	output[31:0]				db_reg2,
	output[31:0]				db_reg3,

	output[31:0]				db_reg4,
	output[31:0]				db_reg5,
	output[31:0]				db_reg6,
	output[31:0]				db_reg7,

	output memtest_start,
	output memtest_rstn,
	input memtest_fail,
	input memtest_done,
	output ctrl_rstn,
	output phy_rstn,
	output reg_axi_rstn,
	output axi0_rstn,
	output axi1_rstn,
	input [31:0]dq_fail,

	output[63:0] memtest_data,
	output 		 memtest_lfsr_en,
	output		 memtest_x16_en,

    output[7:0]  reg_axi_arlen,
    output[31:0] memtest_size,

	output config_rst,
	output config_sel,
	output config_start,
	input config_done,

	input [63:0]tester_loop_len,
    input [63:0]tester_loop_cnt,
    input tester_loop_done,
	input tester_error,
	output tester_rst,
	output[31:0]tester_pattern
);

assign axi_bid      =8'b0;
assign axi_bresp    =8'b0;

assign axi_rid      =8'b0;
assign axi_rresp    =8'b0;

assign axi_wready   =1'b1;
assign axi_awready   =1'b1;
assign axi_arready   =1'b1;

reg [DATA_WIDTH-1:0]	slaveReg [17:0];

reg [ADDR_WIDTH-1:0]	slaveAWAddr;
reg [ADDR_WIDTH-1:0]	slaveARAddr;
reg rd_flag;
reg wr_flag;
reg r_axi_wvalid;
reg r_axi_wready;
reg r_axi_wlast;
reg [DATA_WIDTH-1:0]	r_axi_wdata;

reg [DATA_WIDTH-1:0]	r_axi_rdata;
reg			            r_axi_rvalid;
reg 			        r_axi_rlast;
reg 			        r_axi_bvalid;

assign axi_rdata    = r_axi_rdata;
assign axi_rvalid   = r_axi_rvalid;
assign axi_rlast    = r_axi_rlast;
assign axi_bvalid   = r_axi_bvalid;

assign db_reg0 = slaveReg[0];
assign db_reg1 = slaveReg[1];
assign db_reg2 = slaveReg[2];
assign db_reg3 = slaveReg[3];

assign db_reg4 = slaveReg[4];
assign db_reg5 = slaveReg[5];
assign db_reg6 = slaveReg[6];
assign db_reg7 = slaveReg[7];

assign memtest_start	= slaveReg[2][0];	//REG2
assign memtest_rstn		= slaveReg[2][1];

assign phy_rstn			= slaveReg[3][0];	//REG3
assign ctrl_rstn		= slaveReg[3][1];	//REG3
assign reg_axi_rstn		= slaveReg[3][2];	//REG3
assign axi0_rstn		= slaveReg[3][3];	//REG3
assign axi1_rstn		= slaveReg[3][4];	//REG3

wire[31:0] memtest_data0;
wire[31:0] memtest_data1;

assign memtest_data0		= slaveReg[4];		//REG4
assign memtest_data1		= slaveReg[5];		//REG5
assign memtest_lfsr_en		= slaveReg[6][0];		//REG6
assign memtest_x16_en		= slaveReg[7][0];		//REG7
assign reg_axi_arlen        = slaveReg[8][7:0];  		//REG8
assign memtest_size		    = slaveReg[9];		//REG9

assign memtest_data[31:0] 	= memtest_data0;
assign memtest_data[63:32] 	= memtest_data1;

assign config_rst		= slaveReg[10][0];	//REG10
assign config_sel		= slaveReg[10][1];	//REG10
assign config_start		= slaveReg[10][2];	//REG10

assign tester_rst		= slaveReg[16][0];	//REG16
assign tester_pattern	= slaveReg[17];	//REG17

always@ (posedge axi_aclk or negedge axi_resetn)
	begin
		if (!axi_resetn)
		begin
			slaveAWAddr <= {ADDR_WIDTH{1'b0}};
			slaveARAddr <= {ADDR_WIDTH{1'b0}};
			slaveReg[0]	<= {DATA_WIDTH{1'b0}};
			slaveReg[1]	<= {DATA_WIDTH{1'b0}};
			slaveReg[2]	<= {DATA_WIDTH{1'b0}};
			slaveReg[3]	<= {DATA_WIDTH{1'b0}};

			slaveReg[4]	<= {DATA_WIDTH{1'b0}};
			slaveReg[5]	<= {DATA_WIDTH{1'b0}};
			slaveReg[6]	<= {DATA_WIDTH{1'b0}};
			slaveReg[7]	<= {DATA_WIDTH{1'b0}};

            slaveReg[8]	<= {DATA_WIDTH{1'b0}};
            slaveReg[9]	<= {DATA_WIDTH{1'b0}};
			slaveReg[10]	<= {DATA_WIDTH{1'b0}};
            slaveReg[11]	<= {DATA_WIDTH{1'b0}};

			slaveReg[12]	<= {DATA_WIDTH{1'b0}};
            slaveReg[13]	<= {DATA_WIDTH{1'b0}};
			slaveReg[14]	<= {DATA_WIDTH{1'b0}};
            slaveReg[15]	<= {DATA_WIDTH{1'b0}};

			slaveReg[16]	<= {DATA_WIDTH{1'b0}};
			slaveReg[17]	<= {DATA_WIDTH{1'b0}};

			rd_flag		<=1'b0;
            wr_flag     <=1'b0;

			r_axi_wready	<=1'b0;
			r_axi_wvalid	<=1'b0;
			r_axi_wdata		<={DATA_WIDTH{1'b0}};
            r_axi_bvalid    <=1'b0;
		end
		else
		begin
			r_axi_wready	<=axi_wready;
			r_axi_wvalid	<=axi_wvalid;
            r_axi_wlast	    <=axi_wlast;
			r_axi_wdata		<=axi_wdata;
            r_axi_bvalid    <=1'b0;
			r_axi_rvalid	<=1'b0;
			r_axi_rlast	    <=1'b0;

			if((axi_awready) && (axi_awvalid))
				slaveAWAddr <= axi_awaddr;
			else
				slaveAWAddr <= slaveAWAddr;

			if((axi_arready) && (axi_arvalid))
			begin
				slaveARAddr <= axi_araddr;
				rd_flag		<=1'b1;
			end
			else
				slaveARAddr <= slaveARAddr;

			if(r_axi_wready && r_axi_wvalid && r_axi_wlast)
			begin
				slaveReg[slaveAWAddr[ADDR_WIDTH-1:2]]   <=r_axi_wdata;
                wr_flag                                 <=1'b1;
			end

			if(axi_rready && rd_flag)
			begin
				r_axi_rdata     <=slaveReg[slaveARAddr[ADDR_WIDTH-1:2]];

				if (slaveARAddr[ADDR_WIDTH-1:2] == 32'd0)	//REG0
				begin
					r_axi_rdata	<=dq_fail;
				end
				else if (slaveARAddr[ADDR_WIDTH-1:2] == 32'd1)	//REG1
				begin
					r_axi_rdata[0]		<=memtest_done;
					r_axi_rdata[1]		<=memtest_fail;
					r_axi_rdata[31:2]	<=30'b0;
				end
				else if (slaveARAddr[ADDR_WIDTH-1:2] == 32'd10)	//REG10
				begin
					r_axi_rdata[3]		<=config_done;
				end
				else if (slaveARAddr[ADDR_WIDTH-1:2] == 32'd11)	//REG11
				begin
					r_axi_rdata		<=tester_loop_len[31:0];
				end
				else if (slaveARAddr[ADDR_WIDTH-1:2] == 32'd12)	//REG12
				begin
					r_axi_rdata		<=tester_loop_len[63:32];
				end
				else if (slaveARAddr[ADDR_WIDTH-1:2] == 32'd13)	//REG13
				begin
					r_axi_rdata		<=tester_loop_cnt[31:0];
				end
				else if (slaveARAddr[ADDR_WIDTH-1:2] == 32'd14)	//REG14
				begin
					r_axi_rdata		<=tester_loop_cnt[63:32];
				end
				else if (slaveARAddr[ADDR_WIDTH-1:2] == 32'd15)	//REG15
				begin
					r_axi_rdata[0]		<=tester_loop_done;
					r_axi_rdata[1]		<=tester_error;
				end

				r_axi_rvalid	<=1'b1;
				r_axi_rlast	    <=1'b1;
				rd_flag		    <=1'b0;
			end

            if(axi_bready && wr_flag)
			begin
				wr_flag		    <=1'b0;
                r_axi_bvalid    <=1'b1;
			end
		end
	end
endmodule