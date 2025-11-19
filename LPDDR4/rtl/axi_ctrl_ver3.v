/////////////////////////////////////////////////////////////////////////////
//           _____       
//          / _______    Copyright (C) 2013-2020 Efinix Inc. All rights reserved.
//         / /       \   
//        / /  ..    /   axi_ctrl.v
//       / / .'     /    
//    __/ /.'      /     Description:
//   __   \       /      Include file for generic fifo required functions
//  /_/ /\ \_____/ /     
// ____/  \_______/      
//
// *******************************
// Ver 2 
// FEATURE
// - Split pattern to 2 for read/write (allow issue of read/write request at same cycle)
// - Convert FSM from Binary encodng to One Hots encoding
// - Split the address channel into 2 for read write
// - Add support to both AXI3 & 4 interface 
//		*AXI3 & 4 will be determined by the Aready signal, if 1, axi3, else if 0, axi4
//		*AXI3 will skip read/write request at the same cycle(Treated as idle in the operation)
// - Add parameter for ASIZE, Data bussize, address width and id width
// -----------------------------------------------------------------------
// | R_IN_PATTERN[n] | W_IN_PATTERN[n] | AXI3 Operation | AXI4 Operation |
// |---------------------------------------------------------------------|
// |        0        |        0        |      IDLE      |      IDLE      |
// |        0        |        1        |      WRITE     |      WRITE     |
// |        1        |        0        |      READ      |      READ      |
// |        1        |        1        |      IDLE      |   READ/WRITE   |
// -----------------------------------------------------------------------
// 
// *******************************

module axi_ctrl_ver3(

input	axi_clk,
input	rstn,

input	[528:0]		W_INADDR,
input	[528:0]		R_INADDR,//new
input				TRIGGER,
input	[31:0]		LOOP_N,
input	[3:0]		PATTERN_NUMBER,
input	[15:0]		W_IN_PATTERN,
input	[15:0]		R_IN_PATTERN,//new

//==========================================================
//AXI 3
output	reg	[IDWIDTH-1:0]	DDR_AID_0,
output	reg [ADDRWIDTH-1:0]	DDR_AADDR_0,
output	reg [7:0]	DDR_ALEN_0,
output	reg [2:0]	DDR_ASIZE_0,
output	reg [1:0]	DDR_ABURST_0,
output	reg [1:0]	DDR_ALOCK_0,
output	reg		 	DDR_AVALID_0,
input				DDR_AREADY_0,
output	reg			DDR_ATYPE_0,

//AXI 4
output	reg	[IDWIDTH-1:0]	DDR_ARID_0,
output	reg	[ADDRWIDTH-1:0]	DDR_ARADDR_0,
output	reg	[7:0]	DDR_ARLEN_0,
output	reg	[2:0]	DDR_ARSIZE_0,
output	reg	[1:0]	DDR_ARBURST_0,
output	reg			DDR_ARLOCK_0,
output	reg			DDR_ARVALID_0,
input				DDR_ARREADY_0,
output	reg			DDR_ARQOS_0,
output	reg			DDR_ARAPCMD_0,

output	reg	[IDWIDTH-1:0]	DDR_AWID_0,
output	reg	[ADDRWIDTH-1:0]	DDR_AWADDR_0,
output	reg	[7:0]	DDR_AWLEN_0,
output	reg	[2:0]	DDR_AWSIZE_0,
output	reg	[1:0]	DDR_AWBURST_0,
output	reg			DDR_AWLOCK_0,
output	reg			DDR_AWVALID_0,
input				DDR_AWREADY_0,
output	reg 		DDR_AWQOS_0,
output	reg			DDR_AWAPCMD_0,
output	reg	[3:0]	DDR_AWCACHE_0,
output	reg			DDR_AWALLSTRB_0,
output	reg			DDR_AWCOBUF_0,

//==========================================================
output	reg	[7:0]	DDR_WID_0,
output	reg	[DWIDTH-1:0]	DDR_WDATA_0,
output	reg	[STRBLEN-1:0]	DDR_WSTRB_0,
output	reg			DDR_WLAST_0,
output	reg			DDR_WVALID_0,
input				DDR_WREADY_0,

input	[IDWIDTH-1:0]		DDR_RID_0,
input	[DWIDTH-1:0]	DDR_RDATA_0,
input				DDR_RLAST_0,
input				DDR_RVALID_0,
output	reg			DDR_RREADY_0,
input	[1:0]		DDR_RRESP_0,

input	[IDWIDTH-1:0]	DDR_BID_0,
input				DDR_BVALID_0,
output	reg			DDR_BREADY_0,

//==========================================================

output	[4:0]		o_states,
output				o_out_trig,

input		[255:0]	i_pattern_len,
output	reg	[6:0]	o_pattern_cnt,
output				o_pattern_done,
output	reg			o_Start,
output	reg	[63:0]	o_time_counter,
output	reg	[7:0]	o_write_cnt,
output	reg	[63:0]	o_total_len,
output	reg	[32:0]	o_loop_n,
input		[127:0]	i_lfsr_seed,
input		[7:0]	i_active_factor,
input		[255:0]	i_fix_data_pattern,
input				i_pause,
output loop_done,
output [63:0]loop_cnt,
output [63:0]loop_len,

output	reg			o_compare_error
);
parameter		IDWIDTH				= 6;
parameter		DWIDTH				= 512;
parameter		ADDRWIDTH			= 33;
parameter		ASIZE				= 6;
parameter       TEST_DATA           = 128'h5555AAAA5555AAAA5555AAAA5555AAAA;
parameter       NUM_AXI_IDS         = 4;  // Number of outstanding transaction IDs (2-16)

localparam		STRBLEN				= DWIDTH/8;
//FSM
localparam		IDLE				= 5'b00000;
localparam		WRITE				= 5'b00001;
localparam		WRITE_W_ADDR		= 5'b00010;
localparam		POST_WRITE			= 5'b00100;

localparam		WRITE_R_ADDR		= 5'b01000;
localparam		READ				= 5'b10000;

//reg r_DDR_ARREADY_0;
reg r_DDR_RVALID_0;
reg r_DDR_RLAST_0;
reg [DWIDTH-1:0]r_DDR_RDATA_0;


reg	[4:0]		r_states;
reg				r_pattern_done;
reg				r_trig_state0;
reg				r_trig_state1;

reg	[127:0]		r_lfsr_1P;
reg	[127:0]		r_lfsr_2P;
reg [127:0]		r_lfsr_buffer;//new

reg				r_state_WR_RD0;
reg				r_state_WR_RD1;

reg	[DWIDTH-1:0]		r_rd_buff;
reg	[DWIDTH-1:0]		r_rd_buff_p0;
reg r_loop_done;
reg r_loop_done_p0;
reg [63:0]r_loop_len;
reg [63:0] r_loop_cnt;
reg [15:0]r_nop_cnt;
reg [IDWIDTH-1:0]   r_axi_id;  // Transaction ID counter for multiple outstanding transactions

wire	w_fb_0P;
//wire	[DWIDTH-1:0]	w_compare_buff;
reg		[DWIDTH-1:0]	r_compare_buff;


assign	o_pattern_done	= r_pattern_done;
assign	o_states		= r_states;
assign	o_out_trig		= ~r_trig_state1 & r_trig_state0;

assign	w_fb_0P			= r_lfsr_1P[98] ^~ r_lfsr_1P[100] ^~ r_lfsr_1P[125] ^~ r_lfsr_1P[127];	//XNOR max length tap LSFR 128 
//assign	w_compare_buff	= (i_active_factor == 8'b0)?{DWIDTH/128{r_lfsr_2P}}:{DWIDTH/256{i_fix_data_pattern}};


always@(posedge axi_clk or negedge rstn)
begin
	if (~rstn) begin
		r_trig_state0	<= 1'b0;
		r_trig_state1	<= 1'b0;
	end
	else begin
		if (TRIGGER)
			r_trig_state0	<= 1'b1;
		else begin 
			r_trig_state0	<= 1'b0;
			r_trig_state1	<= 1'b0;
		end
		r_trig_state1	<= r_trig_state0;
	end
end

always @ (posedge axi_clk or negedge rstn) //for divider counter
begin
	if (~rstn) begin
		o_time_counter	<= 64'b0;
		r_loop_cnt		<= 64'b0;
		r_loop_len		<= 64'b0;
		r_loop_done_p0	<= 1'b0;
	end
	else begin
		if (o_Start) begin							// start counter when high
			if(i_pause)								//pause counter
				o_time_counter	<= o_time_counter;
			else
				o_time_counter	<= o_time_counter +1'b1;
		end

		r_loop_done_p0 <= r_loop_done;

		if(r_loop_done)
		begin
			r_loop_cnt	<= o_time_counter;
			r_loop_len	<= o_total_len;
		end
	end
end

assign loop_len	= r_loop_len;
assign loop_cnt	= r_loop_cnt;
assign loop_done = r_loop_done_p0;

always @ (posedge axi_clk or negedge rstn) // start ddr fsm
begin
	if (~rstn) begin
		DDR_AID_0		<= {IDWIDTH{1'b0}};
		DDR_AADDR_0		<= {ADDRWIDTH{1'b0}};
		DDR_ALEN_0		<= 8'b0;
		DDR_ASIZE_0		<= 3'b0;
		DDR_ABURST_0	<= 2'b0;
		DDR_ALOCK_0		<= 1'b0;
		DDR_AVALID_0	<= 1'b0;
		DDR_ATYPE_0		<= 1'b0;
		
		DDR_ARID_0		<= {IDWIDTH{1'b0}};
		DDR_ARADDR_0	<= {ADDRWIDTH{1'b0}};
		DDR_ARLEN_0		<= 8'b0;
		DDR_ARSIZE_0	<= 3'b0;
		DDR_ARBURST_0	<= 2'b0;
		DDR_ARLOCK_0	<= 1'b0;
		DDR_ARVALID_0	<= 1'b0;
		DDR_ARQOS_0		<= 1'b0;
		DDR_ARAPCMD_0	<= 1'b0;

		DDR_AWID_0		<= {IDWIDTH{1'b0}};
		DDR_AWADDR_0	<= {ADDRWIDTH{1'b0}};
		DDR_AWLEN_0		<= 8'b0;
		DDR_AWSIZE_0	<= 3'b0;
		DDR_AWBURST_0	<= 2'b0;
		DDR_AWLOCK_0	<= 1'b0;
		DDR_AWVALID_0	<= 1'b0;
		DDR_AWQOS_0		<= 1'b0;
		DDR_AWAPCMD_0	<= 1'b0;
		DDR_AWCACHE_0	<= 4'b0;
		DDR_AWALLSTRB_0	<= 1'b0;
		DDR_AWCOBUF_0	<= 1'b0;
		
		DDR_WID_0		<= 8'b0;
		DDR_WDATA_0		<= {DWIDTH{1'b0}};
		DDR_WSTRB_0		<= {STRBLEN{1'b1}};
		DDR_WLAST_0		<= 1'b0;
		DDR_WVALID_0	<= 1'b0;
		DDR_RREADY_0	<= 1'b0;
		DDR_BREADY_0	<= 1'b0;
		//r_DDR_ARREADY_0	<=1'b0;
		r_DDR_RVALID_0	<= 1'b0;
		r_DDR_RDATA_0	<= {DWIDTH{1'b0}};
		r_DDR_RLAST_0	<=1'b0;
		
		o_pattern_cnt	<= 7'b0;
		r_pattern_done	<= 1'b0;
		r_lfsr_1P		<= i_lfsr_seed;
		r_lfsr_2P		<= i_lfsr_seed;
		r_rd_buff		<= {(DWIDTH){1'b0}};
		r_states		<= IDLE;
		o_Start			<= 1'b0;
		o_total_len		<= 64'b0;
		o_loop_n		<= 33'b0;
		r_state_WR_RD0	<= 1'b0;
		r_state_WR_RD1	<= 1'b0;
		r_lfsr_buffer	<= i_lfsr_seed;//new
		r_loop_done		<= 1'b0;
		r_nop_cnt		<= 16'b0;
		o_compare_error	<= 1'b0;
		r_compare_buff	<={(DWIDTH){1'b0}};
		r_axi_id		<= {IDWIDTH{1'b0}};  // Initialize transaction ID counter
	end
	else begin

		if(r_compare_buff!=r_rd_buff) begin
			o_compare_error	<= 1'b1; // check if value and rdata same
        end

        r_loop_done		<= 1'b0;
		//r_DDR_ARREADY_0	<= DDR_ARREADY_0;
		r_DDR_RVALID_0	<= DDR_RVALID_0;
		r_DDR_RDATA_0	<= DDR_RDATA_0;
		r_DDR_RLAST_0	<= DDR_RLAST_0;

		if (o_loop_n <= LOOP_N) begin 
			if (o_pattern_cnt <= PATTERN_NUMBER) begin 
				r_pattern_done<= 1'b0;
				
				if (r_states == IDLE) begin
					DDR_WID_0		<= 8'b0;
					DDR_WDATA_0		<= {DWIDTH{1'b0}};
					DDR_WSTRB_0		<= {STRBLEN{1'b1}};
					DDR_WVALID_0	<= 1'b0;
					DDR_RREADY_0	<= 1'b0;
					DDR_BREADY_0	<= 1'b0;
					r_state_WR_RD0	<= W_IN_PATTERN[o_pattern_cnt]; 
					r_nop_cnt		<= 16'b0;
                    
					if(r_state_WR_RD0 != W_IN_PATTERN[o_pattern_cnt]) begin		
                        r_lfsr_1P	<= r_lfsr_buffer; //Write to read transition reset back to seed to compare pseudorandom value
                        r_lfsr_buffer	<= r_lfsr_1P;                        
					end
                    else												
					r_lfsr_1P	<= r_lfsr_1P;

					if (o_out_trig || o_Start) begin
						o_Start				<= 1'b1;
						o_write_cnt		<= i_pattern_len[(o_pattern_cnt*8)+:8];

						if (i_pause) begin
							r_states		<= IDLE;
							DDR_AVALID_0	<= 1'b0;
							DDR_ARVALID_0	<= 1'b0;
							DDR_AWVALID_0	<= 1'b0;
						end
						else if ((~DDR_AREADY_0 & W_IN_PATTERN[o_pattern_cnt]) | (R_IN_PATTERN[o_pattern_cnt] ^ W_IN_PATTERN[o_pattern_cnt])) begin
							r_states		<= r_states	| (W_IN_PATTERN[o_pattern_cnt] ? WRITE : 5'b0)
														| (R_IN_PATTERN[o_pattern_cnt] ? WRITE_R_ADDR : 5'b0);

							// Increment AXI ID for multiple outstanding transactions
							// This allows memory controller to reorder and pipeline transactions
							if (r_axi_id >= (NUM_AXI_IDS - 1))
								r_axi_id <= {IDWIDTH{1'b0}};  // Wrap around
							else
								r_axi_id <= r_axi_id + 1'b1;

							// axi4
                            if (R_IN_PATTERN[o_pattern_cnt]) begin //PREREAD
                                DDR_ARVALID_0	<= 1'b1;
                                DDR_ARID_0		<= r_axi_id;  // Use unique ID for pipelining
                                DDR_ARADDR_0	<= R_INADDR[(o_pattern_cnt*(ADDRWIDTH))+:(ADDRWIDTH)];
                                DDR_ARLEN_0		<= i_pattern_len[(o_pattern_cnt*8)+:8];
                                DDR_ARSIZE_0	<= ASIZE;
                                DDR_ARBURST_0	<= 2'b01;
                            end

                            if (W_IN_PATTERN[o_pattern_cnt]) begin //PREWRITE
                                DDR_WVALID_0	<= 1'b1;
                                DDR_AWVALID_0   <= 1'b1;
                                DDR_AWID_0		<= r_axi_id;  // Use unique ID for pipelining
                                DDR_AWADDR_0	<= W_INADDR[(o_pattern_cnt*(ADDRWIDTH))+:(ADDRWIDTH)];
                                DDR_AWLEN_0		<= i_pattern_len[(o_pattern_cnt*8)+:8];
                                DDR_AWSIZE_0	<= ASIZE;
                                DDR_AWBURST_0	<= 2'b01;
                                
                               // r_lfsr_1P		<= i_lfsr_seed;
								if(i_active_factor == 8'b0)
								begin
							   	r_lfsr_1P		<= {r_lfsr_1P[126:0], w_fb_0P};
                                DDR_WDATA_0		<= {DWIDTH/128{r_lfsr_1P}};
								end
								else
								begin
									DDR_WDATA_0		<= {DWIDTH/256{i_fix_data_pattern}};
								end
                                
                                if(i_pattern_len[(o_pattern_cnt*8)+:8] == 1'b0) begin
                                        DDR_WLAST_0	<= 1'b1;
                                        DDR_BREADY_0 <= 1'b1;
                                        r_states	<= r_states	| WRITE_W_ADDR;
                                 end
                                 else begin
                                        DDR_WLAST_0	<= 1'b0;
                                 end
                            end


							//axi3
							DDR_ATYPE_0		<= W_IN_PATTERN[o_pattern_cnt];
							DDR_AID_0		<= r_axi_id;  // Use unique ID for pipelining (AXI3)
							DDR_AVALID_0	<= 1'b1;
							DDR_AADDR_0		<= W_INADDR[(o_pattern_cnt*(ADDRWIDTH))+:(ADDRWIDTH)];
							DDR_ALEN_0		<= i_pattern_len[(o_pattern_cnt*8)+:8];
							DDR_ASIZE_0		<= ASIZE;
							DDR_ABURST_0	<= 2'b01;
						end
						else begin
							r_nop_cnt		<= r_nop_cnt +1'b1;
						
							if(r_nop_cnt == i_pattern_len[(o_pattern_cnt*8)+:8])
							begin
								r_states		<= IDLE;
								o_pattern_cnt	<= o_pattern_cnt +1'b1;
								r_nop_cnt		<=16'b0;
							end							
						end
					end
					else begin
						DDR_AADDR_0		<= {ADDRWIDTH{1'b0}};
						DDR_ALEN_0		<= 8'b0;
						DDR_ASIZE_0		<= 3'b0;
						DDR_ABURST_0	<= 2'b0;
						DDR_AVALID_0	<= 1'b0;
						DDR_ATYPE_0		<= 1'b0;

						DDR_ARADDR_0	<= {ADDRWIDTH{1'b0}};
						DDR_ARLEN_0		<= 8'b0;
						DDR_ARSIZE_0	<= 3'b0;
						DDR_ARBURST_0	<= 2'b0;
						DDR_ARVALID_0	<= 1'b0;

						DDR_AWADDR_0	<= {ADDRWIDTH{1'b0}};
						DDR_AWLEN_0		<= 8'b0;
						DDR_AWSIZE_0	<= 3'b0;
						DDR_AWBURST_0	<= 2'b0;
						DDR_AWVALID_0	<= 1'b0;						
						r_states		<= IDLE;
					end
				end
                
                if (r_states[0]) begin //WRITE
					if (DDR_WREADY_0) begin
                        DDR_BREADY_0    <= 1'b1;
						o_write_cnt		<= o_write_cnt -1'b1;
						//r_lfsr_1P		<= i_lfsr_seed;
						
						if(i_active_factor == 8'b0)
						begin
							r_lfsr_1P		<= {r_lfsr_1P[126:0], w_fb_0P};
							DDR_WDATA_0		<= {DWIDTH/128{r_lfsr_1P}};
						end
						else
						begin							
							DDR_WDATA_0		<= {DWIDTH/256{i_fix_data_pattern}};
						end

						if (o_write_cnt <= 1'b1) begin
							DDR_WLAST_0		<= 1'b1;
							r_states		<= r_states & (~WRITE) | WRITE_W_ADDR;
						end
						else begin
							DDR_WLAST_0		<= 1'b0;
							r_states		<= r_states;
						end                                                  
					end
					else begin
						o_write_cnt		<= o_write_cnt;
					end

					if(DDR_AWREADY_0 || DDR_AREADY_0) begin
						DDR_AVALID_0	<= 1'b0;
						DDR_AWVALID_0   <= 1'b0;
					end
			//		else begin
			//			DDR_AWVALID_0	<= 1'b1;
			//			DDR_AVALID_0	<= 1'b1;
		//			end
				end                
                
                if (r_states[1]) begin //WRITE_W_ADDR

					if(DDR_WREADY_0) begin
                        DDR_WLAST_0		<= 1'b0;
                        DDR_WVALID_0	<= 1'b0;
                        r_states	<= r_states & (~WRITE_W_ADDR) | POST_WRITE;
                    end
                    else begin
                        DDR_WVALID_0	<= DDR_WVALID_0;
						DDR_WLAST_0		<= DDR_WLAST_0;
					end
                    
                    if(DDR_AWREADY_0 || DDR_AREADY_0) begin
						DDR_AVALID_0	<= 1'b0;
                        DDR_AWVALID_0   <= 1'b0;
                    end

				end
				
				if (r_states[2]) begin //POST_WRITE
					if(DDR_BVALID_0) begin
						DDR_BREADY_0	<= 1'b0;
						r_states		<= r_states & (~POST_WRITE) ;
						o_total_len		<= o_total_len+(i_pattern_len[(o_pattern_cnt*8)+:8]+1'b1);
						o_pattern_cnt	<= r_states == POST_WRITE ? o_pattern_cnt +1'b1 : o_pattern_cnt;					
                    end
					else begin
						DDR_BREADY_0	<= 1'b1;
						o_pattern_cnt	<= o_pattern_cnt;
					end
				end
				
				if (r_states[3]) begin //WRITE_R_ADDR
					if (DDR_ARREADY_0 || DDR_AREADY_0) begin
						DDR_ARVALID_0	<= 1'b0;
						DDR_AVALID_0	<= 1'b0;
						
						DDR_RREADY_0	<= 1'b1;
						if(r_DDR_RVALID_0) begin
							//r_lfsr_1P		<= i_lfsr_seed;
							r_lfsr_1P		<= {r_lfsr_1P[126:0], w_fb_0P};							
							r_lfsr_2P		<= r_lfsr_1P;
							
							if(i_active_factor == 8'b0)
								r_compare_buff	<= {DWIDTH/128{r_lfsr_1P}};
							else
								r_compare_buff	<= {DWIDTH/256{i_fix_data_pattern}};

							r_rd_buff		<= r_DDR_RDATA_0;
						end
						r_states			<= r_states & (~WRITE_R_ADDR) | READ;
					end
					else begin
						DDR_ARVALID_0	<= 1'b1;
						DDR_AVALID_0	<= 1'b1;
					end
				end	
				
				if (r_states[4]) begin //READ
					if (r_DDR_RVALID_0) begin
						//r_lfsr_1P		<= i_lfsr_seed;
						r_lfsr_1P		<= {r_lfsr_1P[126:0], w_fb_0P};
						r_lfsr_2P		<= r_lfsr_1P;
						
						if(i_active_factor == 8'b0)
							r_compare_buff	<= {DWIDTH/128{r_lfsr_1P}};
						else
							r_compare_buff	<= {DWIDTH/256{i_fix_data_pattern}};

						r_rd_buff		<= r_DDR_RDATA_0;

						if (r_DDR_RLAST_0) begin
							DDR_RREADY_0	<= 1'b0;
							r_states		<= r_states & (~READ);
							o_total_len		<= o_total_len+(i_pattern_len[(o_pattern_cnt*8)+:8]+1'b1); // total burst size
							o_pattern_cnt	<= r_states == READ ? o_pattern_cnt +1'b1 : o_pattern_cnt;
                        end
						else begin
							DDR_RREADY_0	<= 1'b1;
						end
					end
				end
			end
			else begin 
				o_pattern_cnt		<= 7'b0;
				o_loop_n			<= o_loop_n+1'b1;
				r_loop_done			<= 1'b1;
			end
		end
		else begin
			r_lfsr_1P			<= i_lfsr_seed;
			r_pattern_done		<= 1'b1;
			o_Start				<= 1'b0;
		end
	end
end

endmodule // axi_ctrl
