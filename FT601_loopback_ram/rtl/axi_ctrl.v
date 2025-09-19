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
// Revisions:
// 1.0 Initial rev
//
// *******************************

module axi_ctrl(

input   axi_clk,
input   rstn,

input   [511:0]     INADDR,
input               TRIGGER,
input	[31:0]		LOOP_N,
input   [3:0]       PATTERN_NUMBER,
input   [15:0]      IN_PATTERN,


//==========================================================
output  reg [7:0]	DDR_AID_0,
output  reg [31:0]	DDR_AADDR_0,
output  reg [7:0]	DDR_ALEN_0,
output  reg [2:0]	DDR_ASIZE_0,
output  reg [1:0]	DDR_ABURST_0,
output  reg [1:0]	DDR_ALOCK_0,
output 	reg		 	DDR_AVALID_0,
input 		      	DDR_AREADY_0,
output  reg	       	DDR_ATYPE_0,

output reg [7:0] 	DDR_WID_0,
output reg [255:0]	DDR_WDATA_0,
output reg [31:0] 	DDR_WSTRB_0,
output reg 			DDR_WLAST_0,
output reg 		   	DDR_WVALID_0,
input 			   	DDR_WREADY_0,

input   [7:0] 		DDR_RID_0,
input   [255:0] 	DDR_RDATA_0,
input				DDR_RLAST_0,
input				DDR_RVALID_0,
output  reg			DDR_RREADY_0,
input   [1:0] 		DDR_RRESP_0,

input   [7:0] 		DDR_BID_0,
input 				DDR_BVALID_0,
output  reg			DDR_BREADY_0,

//==========================================================

output  	[3:0]   o_states,
output              o_out_trig,

input 		[255:0]	i_pattern_len,
output reg 	[6:0]   o_pattern_cnt,
output         		o_pattern_done,
output reg 			o_Start,
output reg 	[63:0] 	o_time_counter,
output reg 	[7:0] 	o_write_cnt,
output reg 	[63:0] 	o_total_len,
output reg	[32:0]	o_loop_n,
input 		[127:0]	i_lfsr_seed,
output		[127:0]	r_lfsr_1P,
input				i_pause,

output	reg			o_compare_error

);

localparam 		ASIZE = 3'b101;

localparam      IDLE            = 4'b0000;
localparam  	WRITE_ADDR      = 4'b0001;

localparam  	PRE_WRITE       = 4'b0010;
localparam  	WRITE           = 4'b0011;
localparam  	POST_WRITE      = 4'b0100;

localparam  	READ_ADDR       = 4'b0101;
localparam  	PRE_READ        = 4'b0110;
localparam  	POST_READ       = 4'b1000;
localparam  	COMPARE_READ    = 4'b1001;

reg [3:0]   r_states;
reg         r_pattern_done;
reg 		r_trig_state0;
reg 		r_trig_state1;

reg		[127:0]r_lfsr_1P;
reg		[127:0]r_lfsr_2P;

reg 	r_state_WR_RD0;
reg 	r_state_WR_RD1;

reg		[255:0]r_rd_buff;

wire 	w_fb_0P;
wire	[255:0]w_compare_buff;

assign o_pattern_done  	= r_pattern_done;
assign o_states 		= r_states;
assign o_out_trig 		= ~r_trig_state1 & r_trig_state0;

assign w_fb_0P = r_lfsr_1P[98] ^~ r_lfsr_1P[100] ^~ r_lfsr_1P[125] ^~ r_lfsr_1P[127];	//XNOR LSFR 128 
assign w_compare_buff = {r_lfsr_2P,r_lfsr_2P};


always@(posedge axi_clk or negedge rstn)
begin
    if(~rstn)
    begin
        r_trig_state0 <=1'b0;
        r_trig_state1 <=1'b0;        
    end
    else
    begin
        if(TRIGGER)
            r_trig_state0 <=1'b1;            
        else                            
            r_trig_state0 <=1'b0;
            r_trig_state1 <=1'b0;
        
        r_trig_state1 <=r_trig_state0;
    end
end

always @ (posedge axi_clk or negedge rstn)
begin
    if(~rstn)
    begin
        o_time_counter         <=  64'b0;
    end
    else
    begin
        if(o_Start)
		begin
			if(i_pause)
				o_time_counter	<=  o_time_counter;
			else
				o_time_counter 	<=  o_time_counter +1'b1;
		end
            
    end
end


always @ (posedge axi_clk or negedge rstn)
begin
	if(~rstn)
	begin
		o_compare_error	<=1'b0;
	end
	else
	begin
		if(w_compare_buff!=r_rd_buff)
			o_compare_error	<=1'b1;
	end
end

always @ (posedge axi_clk or negedge rstn)
begin
    if(~rstn)
    begin
        DDR_AID_0       <=  8'b0;
        DDR_AADDR_0     <= 32'b0;
        DDR_ALEN_0      <=  8'b0;
        DDR_ASIZE_0     <=  3'b0;
        DDR_ABURST_0    <=  2'b0;
        DDR_ALOCK_0     <=  2'b0;
        DDR_AVALID_0    <=  1'b0;
        DDR_ATYPE_0     <=  1'b0;
        DDR_WID_0       <=  8'b0;
        DDR_WDATA_0     <=256'b0;
        DDR_WSTRB_0     <= 32'hFFFFFFFF;
		DDR_WLAST_0		<=	1'b0;
		DDR_WVALID_0	<=	1'b0;
        DDR_RREADY_0    <=  1'b0;
        DDR_BREADY_0    <=  1'b0;
        o_pattern_cnt   <=  5'b0;
        r_pattern_done  <=  1'b1;
		r_lfsr_1P		<= i_lfsr_seed;
		r_lfsr_2P		<= i_lfsr_seed;
		r_rd_buff		<= {i_lfsr_seed,i_lfsr_seed};
        r_states        <=  IDLE;
        o_Start         <=  1'b0;
		o_total_len		<=	16'b0;
		o_loop_n		<=	33'b0;
		r_state_WR_RD0	<=	1'b0;
		r_state_WR_RD1	<=	1'b0;
    end
    else
    begin		
		if(o_loop_n <= LOOP_N)
		begin

			if(o_pattern_cnt <= PATTERN_NUMBER)
			begin
				r_pattern_done<= 1'b0;                      				 

				if(r_states == IDLE)
				begin                                                                
					DDR_ALOCK_0     <=2'b0;                                
					DDR_WID_0       <=8'b0;
					DDR_WDATA_0     <=256'b0;
					DDR_WSTRB_0     <=32'hFFFFFFFF;
					DDR_WVALID_0    <=1'b0;
					DDR_RREADY_0    <=1'b0;
					DDR_BREADY_0    <=1'b0;					

					r_state_WR_RD0	<=IN_PATTERN[o_pattern_cnt];				

					if(r_state_WR_RD0 != IN_PATTERN[o_pattern_cnt])		r_lfsr_1P	<=i_lfsr_seed;
					else												r_lfsr_1P	<=r_lfsr_1P;

					if(o_out_trig || o_Start)
					begin
						o_Start           <=1'b1;
						
						if(i_pause)
						begin
							r_states        <=IDLE;
							DDR_AVALID_0    <=1'b0;
						end
						else
						begin
							r_states        <=WRITE_ADDR;
							DDR_AVALID_0    <=1'b1;
						end
						
						DDR_AADDR_0     <=INADDR[(o_pattern_cnt*32)+:32];                    
						DDR_ALEN_0      <=i_pattern_len[(o_pattern_cnt*8)+:8];
						DDR_ASIZE_0     <=ASIZE;
						DDR_ABURST_0    <=2'b01;						
						DDR_ATYPE_0     <=IN_PATTERN[o_pattern_cnt];						
						o_write_cnt     <=i_pattern_len[(o_pattern_cnt*8)+:8];					                    
					end
					else
					begin
						DDR_AADDR_0     <=32'b0;
						DDR_ALEN_0      <=8'b0;
						DDR_ASIZE_0     <=3'b0;                                        
						DDR_ABURST_0    <=2'b0;
						DDR_AVALID_0    <=1'b0;     
						DDR_ATYPE_0     <=1'b0;           
						r_states 		<= IDLE;
					end
				end
				else if(r_states == WRITE_ADDR)
				begin
					if(DDR_AREADY_0)    
					begin
						DDR_AVALID_0    <=1'b0;                               
										
						if(DDR_ATYPE_0 == 1'b1)
						begin						
							
							DDR_BREADY_0    <=1'b1;
							DDR_WVALID_0    <=1'b1;							
							
							r_lfsr_1P		<= {r_lfsr_1P[126:0], w_fb_0P};
							DDR_WDATA_0     <= {r_lfsr_1P,r_lfsr_1P};							

							if(DDR_WREADY_0)
							begin
								
								o_write_cnt	<=o_write_cnt-1'b1;

								if(o_write_cnt <= 1'b0)  
								begin                
									DDR_WLAST_0 <=1'b1;                        
									r_states    <=POST_WRITE;
								end
								else
								begin
									DDR_WLAST_0 <=1'b0;
									r_states    <=WRITE;
								end
							end
							else
							begin
								r_states      <=WRITE;
							end
						end
						else
						begin	

							DDR_WDATA_0     <=1'b0;
							DDR_RREADY_0    <=1'b1;

							if(DDR_RVALID_0)
							begin

								r_lfsr_1P		<= {r_lfsr_1P[126:0], w_fb_0P};
								r_lfsr_2P		<= r_lfsr_1P;

								r_rd_buff		<= DDR_RDATA_0;

							end

							r_states          <=PRE_READ;
						end
					end
					else
					begin
						DDR_AVALID_0    <=1'b1;
						DDR_WVALID_0    <=1'b0;
						r_states        <=WRITE_ADDR;
					end
				end
				else if(r_states == WRITE)
				begin
					if(DDR_WREADY_0)
					begin
					
						o_write_cnt		<=o_write_cnt -1'b1;
						r_lfsr_1P		<= {r_lfsr_1P[126:0], w_fb_0P};
						DDR_WDATA_0     <={r_lfsr_1P,r_lfsr_1P};

						if(o_write_cnt <= 1'b0)  
						begin
							DDR_WLAST_0		<=1'b1;					
							r_states      	<=POST_WRITE;
						end
						else
						begin						
							DDR_WLAST_0 	<=1'b0;                					
							r_states      	<=WRITE;
						end  
					end
					else
					begin
						o_write_cnt		<=o_write_cnt;
					end
				end
				else if(r_states == POST_WRITE)
				begin
					
					if(DDR_WREADY_0)    
					begin
						DDR_WVALID_0    <= 1'b0;
						DDR_WLAST_0     <= 1'b0;
					end
					else
					begin
						DDR_WVALID_0    <= DDR_WVALID_0;
						DDR_WLAST_0     <= DDR_WLAST_0;
					end
					
				
					if(DDR_BVALID_0)   
					begin			
						DDR_BREADY_0    <= 1'b0;
						o_pattern_cnt 	<= o_pattern_cnt +1'b1;
						r_states        <= IDLE;
						o_total_len		<= o_total_len+(DDR_ALEN_0+1'b1);
					end
					else
					begin			
						DDR_BREADY_0 	<= 1'b1;
						o_pattern_cnt 	<= o_pattern_cnt;
						r_states  		<= POST_WRITE;					
					end
				end
				else if(r_states == PRE_READ)
				begin				
					if(DDR_RVALID_0)       
					begin						
						
						r_lfsr_1P		<= {r_lfsr_1P[126:0], w_fb_0P};
						r_lfsr_2P		<= r_lfsr_1P;						

						r_rd_buff		<= DDR_RDATA_0;						

						if(DDR_RLAST_0)
						begin
							DDR_RREADY_0    <=1'b0;                    
							o_pattern_cnt 	<= o_pattern_cnt +1'b1;
							r_states        <= IDLE;
							o_total_len		<= o_total_len+(DDR_ALEN_0+1'b1);
						end
						else
						begin							
							DDR_RREADY_0    <=1'b1;
							r_states        <=PRE_READ;
						end
					end
				end				
			end
			else
			begin 
				o_pattern_cnt		<=7'b0;								
				o_loop_n			<=o_loop_n+1'b1;
			end
		end
		else
		begin
			r_lfsr_1P			<= i_lfsr_seed;
			r_pattern_done		<= 1'b1;
			o_Start				<= 1'b0;
		end
    end
end


endmodule // axi_ctrl