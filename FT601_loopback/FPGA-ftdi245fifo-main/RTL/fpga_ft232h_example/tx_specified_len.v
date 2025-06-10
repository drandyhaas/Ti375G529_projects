
//--------------------------------------------------------------------------------------------------------
// Module  : tx_specified_len
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: receive 4 bytes from AXI-stream slave,
//           then regard the 4 bytes as a length, send length of bytes on AXI-stream master
//           this module will called by fpga_top_ft600_tx_mass.v or fpga_top_ft232h_tx_mass.v
//--------------------------------------------------------------------------------------------------------

module tx_specified_len (
    input  wire        rstn,
    input  wire        clk,
	 input  wire        ahmed_clk,
    input  wire        ahmed_data,
	 output wire        debugout,
    // AXI-stream slave
    output wire        i_tready,
    input  wire        i_tvalid,
    input  wire [ 7:0] i_tdata,
    // AXI-stream master
    input  wire        o_tready,
    output reg         o_tvalid,
    output wire [31:0] o_tdata,
    output wire [ 3:0] o_tkeep,
    output wire        o_tlast
);


localparam [2:0] RX_BYTE0 = 3'd0,
                 RX_BYTE1 = 3'd1,
                 RX_BYTE2 = 3'd2,
                 RX_BYTE3 = 3'd3,
					  LENHACK  = 3'd4,
                 TX_DATA  = 3'd5;

reg [ 2:0]       state = RX_BYTE0;

reg [31:0]       length = 0;

always @ (posedge clk or negedge rstn)
    if (~rstn) begin
        state  <= RX_BYTE0;
        length <= 0;
    end else begin
        case (state)
            RX_BYTE0 : if (i_tvalid) begin
                length[ 7: 0] <= i_tdata;
                state <= RX_BYTE1;
            end
            
            RX_BYTE1 : if (i_tvalid) begin
                length[15: 8] <= i_tdata;
                state <= RX_BYTE2;
            end
            
            RX_BYTE2 : if (i_tvalid) begin
                length[23:16] <= i_tdata;
                state <= RX_BYTE3;
            end
            
            RX_BYTE3 : if (i_tvalid) begin
                length[31:24] <= i_tdata;
					 state <= LENHACK;
				end
				
				LENHACK : begin
					 length = length + 4;
                state <= TX_DATA;
            end
            
            default :  // TX_DATA :
                if (o_tready && (rdusedw_sig>4) ) begin // require a few bytes in the queue
                    if (length >= 4) begin
                        length <= length - 4;								
								rdreq_sig <= 1'b1;
								o_tvalid <= 1'b1;								
                    end else begin
                        length <= 0;								
								rdreq_sig <= 1'b0;
								o_tvalid <= 1'b0;								
								state <= RX_BYTE0;
                    end
                end
					 else begin
						  rdreq_sig <= 1'b0;
						  o_tvalid <= 1'b0;
					 end
        endcase
    end


assign i_tready = (state != TX_DATA);

//assign o_tdata  = {length[7:0],length[7:0],7'd0,ahmed_clk,7'd0,ahmed_data }; // for testing
assign o_tdata = q_sig; // for real operation, get data from queue

assign o_tkeep  = (length>=4) ? 4'b1111 :
                  (length==3) ? 4'b0111 :
                  (length==2) ? 4'b0011 :
                  (length==1) ? 4'b0001 :
                 /*length==0*/  4'b0000;

assign o_tlast  = (length>=4) ? 1'b0 : 1'b1;

assign wrreq_sig = 1'b1; // add to fifo every wrclk cycle (whether it's full or not)
//ahmed_data gets put into fifo each wrclk cycle

assign debugout = wrfull_sig;
//assign debugout = ahmed_clk;

//`define TESTING
`ifndef TESTING
wire wrclk_sig; assign wrclk_sig = ahmed_clk; // real operation
`else
reg wrclk_sig;
integer counter=0;
always @ (posedge clk) begin // for testing
	if (counter==0) wrclk_sig <= 1'b1;
	else wrclk_sig <= 1'b0;
	if (counter> 10 ) counter <= 0;
	else counter <= counter+1;
end
`endif

reg rdreq_sig = 1'b0;
wire wrreq_sig;
wire [31:0] q_sig;
wire rdempty_sig;
wire [8:0] rdusedw_sig;
wire wrfull_sig;


ahmed_fifo	ahmed_fifo_inst (

	//inputs
	.data ( ahmed_data ),
	.rdclk ( clk ),
	.rdreq ( rdreq_sig ),
	.wrclk ( wrclk_sig ),
	.wrreq ( wrreq_sig ),
	
	//outputs
	.q ( q_sig ),
	.rdempty ( rdempty_sig ),
	.wrfull ( wrfull_sig ),
	.rdusedw ( rdusedw_sig )
	);


endmodule
