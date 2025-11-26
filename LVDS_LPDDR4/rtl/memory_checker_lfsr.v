
`timescale 1ps/1ps
module memory_checker_lfsr (

input axi_clk,
input rstn,
input start,
output [5:0] awid,
output reg [32:0] awaddr,
output reg [7:0] awlen,
output reg [2:0] awsize,
output reg [1:0] awburst,
output reg [3:0] awcache,
output reg awlock,
output reg awvalid,
output reg awcobuf,
output reg awapcmd,
output reg awallstrb,
output reg awqos,
input awready,
output reg atype,

output [5:0] arid,
output reg [32:0] araddr,
output reg [7:0] arlen,
output reg [2:0] arsize,
output reg [1:0] arburst,
output reg arlock,
output reg arvalid,
output reg arapcmd,
output reg arqos,
input arready,

output reg [511:0] wdata,
output  [63:0] wstrb,
output reg wlast,
output reg wvalid,
input wready,

input [5:0] rid,
input [511:0] rdata,
output reg [511:0] rdata_obs,
output reg [511:0] rdata_exp,
input rlast,
input rvalid,
output reg rready,
input [1:0] rresp,

input [5:0] bid,
input bvalid,
input [31:0] check_mask,
output reg bready,
input [127:0] i_lfsr_seed,
output reg fail,
output reg done,
input lfsr_en,
input x16_en,
output [31:0]dq_fail_expression,
input [31:0]test_size,
input [1:0] test_mode  // 0=write+read, 1=write-only, 2=read-only

);
wire [511:0] mask;

// Multiple outstanding AXI IDs for improved bandwidth
// Cycle through 8 IDs to allow multiple transactions in flight
reg [2:0] axi_id_counter;
assign awid = {3'b000, axi_id_counter};  // Write ID
assign arid = {3'b000, axi_id_counter};  // Read ID

assign wstrb = 64'hFFFFFFFF_FFFFFFFF;
assign mask = {16{check_mask}};
parameter ALEN = 255;  // 256 transfers per burst (optimized from 64)
parameter ASIZE = 6;
parameter START_ADDR = 33'h000000000;
//parameter STOP_ADDR = 33'h000001900;
parameter ADDR_OFFSET = (ALEN + 1)*64;

// Bank-interleaved addressing is disabled for now - just use normal sequential addressing
// The original simple increment works fine
// (Bank interleaving optimization can be added later if needed)

//Main states
localparam
	IDLE = 4'b0000, 
	WRITE_ADDR = 4'b0001,
	PRE_WRITE = 4'b0010,
	WRITE = 4'b0011,
	POST_WRITE = 4'b0100,
	READ_ADDR = 4'b0101,
	PRE_READ = 4'b0110,
	READ_COMPARE = 4'b0111,
	POST_READ = 4'b1000,
	DONE = 4'b1001;

reg [3:0] states, nstates;
reg bvalid_done;
reg [1:0] start_sync;
reg [8:0] write_cnt, nxt_cnt, read_cnt;
reg [511:0] rdata_store;
reg [127:0] r_lfsr_1P, r_lfsr_2P, r_lfsr_buffer;
reg wburst_done, rburst_done, write_done, read_done;
wire w_fb_0P;
assign  w_fb_0P                 = r_lfsr_1P[98] ^~ r_lfsr_1P[100] ^~ r_lfsr_1P[125] ^~ r_lfsr_1P[127];  //XNOR max length tap LSFR 128 
reg r_lfsr_en;

reg [31:0]r_dq_fail_expression;
assign dq_fail_expression = r_dq_fail_expression;
reg [31:0]r_test_size;

always @(posedge axi_clk or negedge rstn) begin
	if (!rstn) begin
		start_sync <= 2'b00;
	end else begin
		start_sync[0] <= start;
		start_sync[1] <= start_sync[0];
	end
end

always @(posedge axi_clk or negedge rstn) begin
 	if (!rstn) begin
	states <= IDLE;
	end else begin
	states <= nstates;
	end
end
reg awready_f;

// test_mode: 0=write+read, 1=write-only, 2=read-only
always @(states or start_sync[1] or write_cnt or rburst_done or write_done or read_done or bvalid_done or awready or arready or wready or test_mode) begin
	case(states)
	IDLE 	   : if (start_sync[1])
	               nstates = (test_mode == 2'b10) ? READ_ADDR : WRITE_ADDR;  // read-only skips write
	             else
	               nstates = IDLE;
	WRITE_ADDR : 								nstates = PRE_WRITE;
	PRE_WRITE  : if (wready)			        nstates = WRITE;
		     else								nstates = PRE_WRITE;
	WRITE	   : if ((awready_f | awready) & write_cnt == 9'd0)		nstates = POST_WRITE;
		     else		 			nstates = WRITE;
	POST_WRITE : if (write_done & bvalid_done)
	               nstates = (test_mode == 2'b01) ? DONE : READ_ADDR;  // write-only skips read
		     else if (bvalid_done)			nstates = WRITE_ADDR;
		     else					nstates = POST_WRITE;
	READ_ADDR  : if (arready) 				nstates = PRE_READ;
		     else					nstates = READ_ADDR;
	PRE_READ   :						nstates = READ_COMPARE;
	READ_COMPARE  : if (rburst_done) 			nstates = POST_READ;
			else					nstates = READ_COMPARE;
	POST_READ  :	if (read_done) 				nstates = DONE;
			else					nstates = READ_ADDR;
	DONE	   : 	if(~start_sync[1])					nstates = IDLE;
                    else            nstates = DONE;
	default							nstates = IDLE;
	endcase
end

reg wready_f;

always @(posedge axi_clk or negedge rstn) begin
	if (!rstn) begin
		nxt_cnt <= 9'd0;
	end else begin
		nxt_cnt <= write_cnt - 1'b1;
	end 

end

always @(posedge axi_clk or negedge rstn) begin
	if (!rstn) begin
		awaddr 		<= START_ADDR;
		awvalid 	<= 1'b0;
		atype 		<= 1'b0;
		awburst 	<= 2'b00;
		awsize 		<= 3'b000;
		awlen 		<= 8'd0;
		awlock 		<= 1'b0;
		awcobuf 	<= 1'b0;
		awapcmd 	<= 1'b0;
		awallstrb 	<= 1'b0;
		awcache 	<= 4'd0;
		awqos 		<= 1'b0;
		araddr 		<= 33'd0;
		arvalid 	<= 1'b0;
		arburst 	<= 2'b00;
		arsize 		<= 3'b000;
		arlen 		<= 8'd0;
		arlock 		<= 1'b0;
		arapcmd 	<= 1'b0;
		arqos 		<= 1'b0;
		wvalid 		<= 1'b0;
		write_cnt 	<= ALEN;
		write_done 	<= 1'b0;
		wdata 		<= 512'b0;
		wburst_done <= 1'b0;
		wlast 		<= 1'b0;
		bready 		<= 1'b0;
		fail 		<= 1'b0;
		done 		<= 1'b0;
		rready 		<= 1'b0;
		bvalid_done <=1'b0;
		wready_f 	<= 1'b0;
		awready_f 	<= 1'b0;
        rdata_obs 	<= 512'd0;
        rdata_store <= 512'd0;
		rdata_exp	<= 512'd0;
		r_lfsr_1P 	<= 128'b0;
		r_lfsr_en	<= 1'b0;
		r_dq_fail_expression	<=32'b0;
		r_test_size		<=32'b0;
		axi_id_counter	<= 3'b000;

	end else begin
		if (states == IDLE) begin
			awaddr <= START_ADDR;
			awvalid <= 1'b0;
			atype <= 1'b0;
			awburst <= 2'b00;
			awsize <= 3'b000;
			awlen <= 8'd0;
			awlock <= 1'b0;
			awcobuf <= 1'b0;
			awapcmd <= 1'b0;
			awallstrb <= 1'b0;
			awcache <= 4'd0;
			awqos <= 1'b0;
			arqos <= 1'b0;
			araddr <= START_ADDR;
			arvalid <= 1'b0;
			arburst <= 2'b00;
			arsize <= 3'b000;
			arlen <= 8'd0;
			arlock <= 1'b0;
			arapcmd <= 1'b0;
			wvalid <= 1'b0;
			write_cnt <= ALEN + 1;
			wdata <= 512'b0;
			wburst_done <= 1'b0;
			wlast <= 1'b0;
			bready <= 1'b0;
			rready <= 1'b0;
			bvalid_done <= 1'b0;
			fail <= 1'b0;
			done <= 1'b0;
		    awready_f <= 1'b0;
            r_lfsr_1P <= i_lfsr_seed;
			r_lfsr_en <=lfsr_en;
			r_dq_fail_expression	<=32'b0;
			r_test_size	<= test_size;
			axi_id_counter <= 3'b000;
		end
		if (states == WRITE_ADDR) begin
			awvalid <= 1'b1;
			atype <= 1'b1;
			awsize <= ASIZE;
			awlen <= ALEN;
			awburst <= 2'b01;
			awlock <= 2'b00;
			awqos <= 1'b1;
			wvalid <= 1'b0;
			write_cnt <= write_cnt - 1;
			wburst_done <= 1'b0;
			bvalid_done <= 1'b0;
			bready <= 1'b0;
			rready <= 1'b0;
			done <= 1'b0;
			fail <= 1'b0;
			r_lfsr_1P <= {r_lfsr_1P[126:0], w_fb_0P};
			if(r_lfsr_en)
				wdata <= {4{r_lfsr_1P}} & mask;
			else
				wdata <= {4{i_lfsr_seed}} & mask;

		end
		if (states == PRE_WRITE) begin
		      if (awready) begin
			awvalid <= 1'b0;
		        awready_f <= 1'b1;
		      end else begin
			awvalid <= awvalid;
			awready_f <= awready_f;
		      end		
			atype <= 1'b0;
			bready <= 1'b1;
		      if(wready & wvalid) begin
			wready_f <= 1'b1;
                        write_cnt <= write_cnt - 1;
			r_lfsr_1P <= {r_lfsr_1P[126:0], w_fb_0P};

			if(r_lfsr_en)
				wdata <= {4{r_lfsr_1P}} & mask;
			else
				wdata <= {4{i_lfsr_seed}} & mask;
			
			if (write_cnt == 9'd1) begin
				wlast <= 1'b1;
			end
		      end else begin
			wvalid <= 1'b1;
		      end
		end
		if (states == WRITE) begin
		      if (awready) begin
                        awvalid <= 1'b0;
                        awready_f <= 1'b1;
                      end else begin
                        awvalid <= awvalid;
                        awready_f <= awready_f;
                      end
			if (wready) begin
				if (write_cnt == 9'd0) begin
				wburst_done <= 1'b1;
				wlast <= 1'b0;
				wvalid <= 1'b0;
					//if (awaddr >= STOP_ADDR) begin
					if (awaddr >= r_test_size) begin
					write_done <= 1'b1;
					end else begin
					write_done <= 1'b0;
					end
				end else if (write_cnt == 9'd1) begin
					wlast <= 1'b1;
					write_cnt <= write_cnt - 1;
					r_lfsr_1P <= {r_lfsr_1P[126:0], w_fb_0P};

					if(r_lfsr_en)
						wdata <= {4{r_lfsr_1P}} & mask;
					else
						wdata <= {4{i_lfsr_seed}} & mask;

				end else begin
					write_cnt <= write_cnt - 1;
					r_lfsr_1P <= {r_lfsr_1P[126:0], w_fb_0P};
					
					if(r_lfsr_en)
						wdata <= {4{r_lfsr_1P}} & mask;
					else
						wdata <= {4{i_lfsr_seed}} & mask;

				end
			end
		end
		if (states == POST_WRITE) begin
			if (write_done) begin
				awaddr <= START_ADDR;
                                r_lfsr_1P <= i_lfsr_seed;
				axi_id_counter <= 3'b000;  // Reset AXI ID counter for read phase

			end else begin
				if (bvalid) begin
				// Simple sequential addressing
				awaddr <= awaddr + ADDR_OFFSET;
				// Cycle through 8 AXI IDs for multiple outstanding transactions
				axi_id_counter <= axi_id_counter + 3'b001;
				write_cnt <= ALEN + 1;
				end
			end
			if (wready == 1'b1) begin
				wlast <= 1'b0;	
				wvalid <= 1'b0;	
			end
			if (bvalid) begin
				bvalid_done <= 1'b1;
				bready <= 1'b0;
			end
			awready_f <= 1'b0;
		end
		if (states == READ_ADDR) begin
			arvalid <= 1'b1;
			read_cnt <= ALEN + 1;
			arsize <= ASIZE;
			arlen <= ALEN;
			arburst <= 2'b01; 
				
		end
		if (states == PRE_READ) begin
			arvalid <= 1'b0;
			rburst_done <= 1'b0;
			r_lfsr_1P 	<= {r_lfsr_1P[126:0], w_fb_0P};
			
			if(r_lfsr_en)
				rdata_store <= {4{r_lfsr_1P}} & mask;
			else
				rdata_store <= {4{i_lfsr_seed}} & mask;
			
			read_cnt 	<= read_cnt - 1'b1;
		end
		if (states == READ_COMPARE) begin
			rready <= 1'b1;
			if (read_cnt != 9'd0) begin
			if (rvalid == 1'b1) begin
            	
				if(r_lfsr_en)
					rdata_store <= {4{r_lfsr_1P}} & mask;
				else
					rdata_store <= {4{i_lfsr_seed}} & mask;

				r_lfsr_1P <= {r_lfsr_1P[126:0], w_fb_0P};
				read_cnt <= read_cnt - 1'b1;

				if ((rdata  & mask)  !== rdata_store) 
				begin
					rdata_obs <= rdata;
                    rdata_exp <= rdata ^ rdata_store;
					fail <= 1'b1;
					`ifdef EFX_SIM
					$display("ERROR!! Read mismatch : read = 0x%x, expected = 0x%x",rdata,rdata_store);
					`endif 
				end else 
				begin
					`ifdef EFX_SIM
					$display("Read match: read = 0x%x, expected = 0x%x",rdata,rdata_store);
					`endif
				end

				if(x16_en)
				begin
					r_dq_fail_expression[7:0]	<=   rdata_exp[(0*16)+:8] | rdata_exp[(1*16)+:8] | rdata_exp[(2*16)+:8] | rdata_exp[(3*16)+:8]
													|rdata_exp[(4*16)+:8] | rdata_exp[(5*16)+:8] | rdata_exp[(6*16)+:8] | rdata_exp[(7*16)+:8]
													|rdata_exp[(8*16)+:8] | rdata_exp[(9*16)+:8] | rdata_exp[(10*16)+:8] | rdata_exp[(11*16)+:8]
													|rdata_exp[(12*16)+:8] | rdata_exp[(13*16)+:8] | rdata_exp[(14*16)+:8] | rdata_exp[(15*16)+:8]
													|rdata_exp[(16*16)+:8] | rdata_exp[(17*16)+:8] | rdata_exp[(18*16)+:8] | rdata_exp[(19*16)+:8]
													|rdata_exp[(20*16)+:8] | rdata_exp[(21*16)+:8] | rdata_exp[(22*16)+:8] | rdata_exp[(23*16)+:8]
													|rdata_exp[(24*16)+:8] | rdata_exp[(25*16)+:8] | rdata_exp[(26*16)+:8] | rdata_exp[(27*16)+:8]
													|rdata_exp[(28*16)+:8] | rdata_exp[(29*16)+:8] | rdata_exp[(30*16)+:8] | rdata_exp[(31*16)+:8]
													|r_dq_fail_expression[7:0];

					r_dq_fail_expression[15:8]	<=   rdata_exp[(0*16+8)+:8] | rdata_exp[(1*16+8)+:8] | rdata_exp[(2*16+8)+:8] | rdata_exp[(3*16+8)+:8]
													|rdata_exp[(4*16+8)+:8] | rdata_exp[(5*16+8)+:8] | rdata_exp[(6*16+8)+:8] | rdata_exp[(7*16+8)+:8]
													|rdata_exp[(8*16+8)+:8] | rdata_exp[(9*16+8)+:8] | rdata_exp[(10*16+8)+:8] | rdata_exp[(11*16+8)+:8]
													|rdata_exp[(12*16+8)+:8] | rdata_exp[(13*16+8)+:8] | rdata_exp[(14*16+8)+:8] | rdata_exp[(15*16+8)+:8]
													|rdata_exp[(16*16+8)+:8] | rdata_exp[(17*16+8)+:8] | rdata_exp[(18*16+8)+:8] | rdata_exp[(19*16+8)+:8]
													|rdata_exp[(20*16+8)+:8] | rdata_exp[(21*16+8)+:8] | rdata_exp[(22*16+8)+:8] | rdata_exp[(23*16+8)+:8]
													|rdata_exp[(24*16+8)+:8] | rdata_exp[(25*16+8)+:8] | rdata_exp[(26*16+8)+:8] | rdata_exp[(27*16+8)+:8]
													|rdata_exp[(28*16+8)+:8] | rdata_exp[(29*16+8)+:8] | rdata_exp[(30*16+8)+:8] | rdata_exp[(31*16+8)+:8]
													|r_dq_fail_expression[15:8];
				end
				else
				begin
					r_dq_fail_expression[7:0]	<=   rdata_exp[(0*32)+:8] | rdata_exp[(1*32)+:8] | rdata_exp[(2*32)+:8] | rdata_exp[(3*32)+:8]
													|rdata_exp[(4*32)+:8] | rdata_exp[(5*32)+:8] | rdata_exp[(6*32)+:8] | rdata_exp[(7*32)+:8]
													|rdata_exp[(8*32)+:8] | rdata_exp[(9*32)+:8] | rdata_exp[(10*32)+:8] | rdata_exp[(11*32)+:8]
													|rdata_exp[(12*32)+:8] | rdata_exp[(13*32)+:8] | rdata_exp[(14*32)+:8] | rdata_exp[(15*32)+:8]
													|r_dq_fail_expression[7:0];

					r_dq_fail_expression[15:8]	<=   rdata_exp[(0*32+8)+:8] | rdata_exp[(1*32+8)+:8] | rdata_exp[(2*32+8)+:8] | rdata_exp[(3*32+8)+:8]
													|rdata_exp[(4*32+8)+:8] | rdata_exp[(5*32+8)+:8] | rdata_exp[(6*32+8)+:8] | rdata_exp[(7*32+8)+:8]
													|rdata_exp[(8*32+8)+:8] | rdata_exp[(9*32+8)+:8] | rdata_exp[(10*32+8)+:8] | rdata_exp[(11*32+8)+:8]
													|rdata_exp[(12*32+8)+:8] | rdata_exp[(13*32+8)+:8] | rdata_exp[(14*32+8)+:8] | rdata_exp[(15*32+8)+:8]
													|r_dq_fail_expression[15:8];

					r_dq_fail_expression[23:16]	<=   rdata_exp[(0*32+16)+:8] | rdata_exp[(1*32+16)+:8] | rdata_exp[(2*32+16)+:8] | rdata_exp[(3*32+16)+:8]
													|rdata_exp[(4*32+16)+:8] | rdata_exp[(5*32+16)+:8] | rdata_exp[(6*32+16)+:8] | rdata_exp[(7*32+16)+:8]
													|rdata_exp[(8*32+16)+:8] | rdata_exp[(9*32+16)+:8] | rdata_exp[(10*32+16)+:8] | rdata_exp[(11*32+16)+:8]
													|rdata_exp[(12*32+16)+:8] | rdata_exp[(13*32+16)+:8] | rdata_exp[(14*32+16)+:8] | rdata_exp[(15*32+16)+:8]
													|r_dq_fail_expression[23:16];

					r_dq_fail_expression[31:24]	<=   rdata_exp[(0*32+24)+:8] | rdata_exp[(1*32+24)+:8] | rdata_exp[(2*32+24)+:8] | rdata_exp[(3*32+24)+:8]
													|rdata_exp[(4*32+24)+:8] | rdata_exp[(5*32+24)+:8] | rdata_exp[(6*32+24)+:8] | rdata_exp[(7*32+24)+:8]
													|rdata_exp[(8*32+24)+:8] | rdata_exp[(9*32+24)+:8] | rdata_exp[(10*32+24)+:8] | rdata_exp[(11*32+24)+:8]
													|rdata_exp[(12*32+24)+:8] | rdata_exp[(13*32+24)+:8] | rdata_exp[(14*32+24)+:8] | rdata_exp[(15*32+24)+:8]
													|r_dq_fail_expression[31:24];
				end
			end
			end
			if (read_cnt == 9'd0) begin
	                        if ((rvalid == 1'b1) && (rlast == 1'b1)) begin
                                       if ((rdata & mask) !== rdata_store ) begin
                                                fail <= 1'b1;
                                                `ifdef EFX_SIM
                                                $display("ERROR!! Read mismatch : read = 0x%x, expected = 0x%x",rdata,rdata_store);
                                                `endif
                                        end else begin
                                                `ifdef EFX_SIM
                                                $display("Read match: read = 0x%x, expected = 0x%x",rdata,rdata_store);
                                                `endif
                                        end


					//if (araddr >= STOP_ADDR) begin
					if (araddr >= r_test_size) begin
						read_done <= 1'b1;
					end else begin
						read_done <= 1'b0;
					end
					rburst_done <= 1'b1;
				end
			end	
		end
		if (states == POST_READ) begin
			// Simple sequential addressing (must match write pattern)
			araddr <= araddr + ADDR_OFFSET;
			// Cycle through 8 AXI IDs for multiple outstanding transactions
			axi_id_counter <= axi_id_counter + 3'b001;
			rready <= 1'b1;
		end
		if (states == DONE) begin
			done <= 1'b1;
		end
	end

end




endmodule
