module j2a_bridge #(
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_DATA_WIDTH = 32,
    parameter AXI_USER_WIDTH = 6,
    parameter AXI_ID_WIDTH   = 3
) (
    // ==================== JTAG signals ===============================
    input  wire         tck_i,
    input  wire         tdi_i,                  // Not used, parallel data (data_register_i) come from debug_hub
    output wire         module_tdo_o,
    // ==================== TAP states =================================
    input  wire         trstn_i,
    input  wire         capture_dr_i,
    input  wire         shift_dr_i,
    input  wire         update_dr_i,
    // ==================== Other From / To Debug Hub ==================
    input  wire [63:0]  data_register_i,
    input  wire         module_select_i,
    output reg          top_inhibit_o,
    // ==================== AXI4 MASTER ================================
    input  wire         axi_aclk,
    input  wire         axi_aresetn,
    output wire         axi_master_wr_op,       // Not Used
    // ==================== AXI4 MASTER ================================
    // WRITE ADDRESS CHANNEL
    output reg                    axi_master_aw_valid,
    output reg [AXI_ADDR_WIDTH-1:0] axi_master_aw_addr,
    output [2:0]                  axi_master_aw_prot,
    output [3:0]                  axi_master_aw_region,
    output [7:0]                  axi_master_aw_len,
    output [2:0]                  axi_master_aw_size,
    output [1:0]                  axi_master_aw_burst,
    output [2:0]                  axi_master_aw_lock,
    output [3:0]                  axi_master_aw_cache,
    output [3:0]                  axi_master_aw_qos,
    output [AXI_ID_WIDTH-1:0]     axi_master_aw_id,
    output [AXI_USER_WIDTH-1:0]   axi_master_aw_user,
    input                         axi_master_aw_ready,
    // READ ADDRESS CHANNEL
    output reg                    axi_master_ar_valid,
    output reg [AXI_ADDR_WIDTH-1:0] axi_master_ar_addr,
    output [2:0]                  axi_master_ar_prot,
    output [3:0]                  axi_master_ar_region,
    output [7:0]                  axi_master_ar_len,
    output [2:0]                  axi_master_ar_size,
    output [1:0]                  axi_master_ar_burst,
    output [2:0]                  axi_master_ar_lock,
    output [3:0]                  axi_master_ar_cache,
    output [3:0]                  axi_master_ar_qos,
    output [AXI_ID_WIDTH-1:0]     axi_master_ar_id,
    output [AXI_USER_WIDTH-1:0]   axi_master_ar_user,
    input                         axi_master_ar_ready,
    // WRITE DATA CHANNEL
    output reg                    axi_master_w_valid,
    output reg [AXI_DATA_WIDTH-1:0] axi_master_w_data,
    output [AXI_DATA_WIDTH/8-1:0] axi_master_w_strb,
    output [AXI_USER_WIDTH-1:0]   axi_master_w_user,
    output                        axi_master_w_last,
    input                         axi_master_w_ready,
    // READ DATA CHANNEL
    input                         axi_master_r_valid,
    input  [AXI_DATA_WIDTH-1:0]   axi_master_r_data,
    input  [1:0]                  axi_master_r_resp,
    input                         axi_master_r_last,
    input  [AXI_ID_WIDTH-1:0]     axi_master_r_id,
    input  [AXI_USER_WIDTH-1:0]   axi_master_r_user,
    output                        axi_master_r_ready,
    // WRITE RESPONSE CHANNEL
    input                         axi_master_b_valid,
    input  [1:0]                  axi_master_b_resp,
    input  [AXI_ID_WIDTH-1:0]     axi_master_b_id,
    input  [AXI_USER_WIDTH-1:0]   axi_master_b_user,
    output                        axi_master_b_ready
);

reg [3:0]  state;
reg [3:0]  next_state;
reg [3:0]  test_state;

reg        jtag_idle;
reg        axi_read;
reg        a2j_capture;
reg        shift_en;
reg        lock_wdata;
reg        axi_write;
reg        wait_bvalid;
reg [4:0]  shift_cnt;
reg [31:0] axi_rdata_int;
reg [31:0] jtag_rdata;
reg [31:0] jtag_wdata;
reg [31:0] jtag_axi_addr;
reg        axi_master_w_valid_int;
reg        axi_master_aw_valid_int;
reg        axi_master_ar_valid_int;
reg         jtag_wdata_valid;
reg [7:0]  jtag_wdata_valid_cnt;

wire        module_cmd;
wire        go_read;
wire        go_write;
wire        rd_done;
wire        wr_done;
wire        shf_done;
wire        axi_read_sync;
wire        axi_write_sync;
wire [3:0]  operation_in;
wire [31:0] address_data_in;
wire [31:0] wdata_in;
wire        axi_read_cmd;
wire        axi_write_cmd;

assign module_cmd      = ~(data_register_i[63]);
assign operation_in    =   data_register_i[62:59];
assign address_data_in =   data_register_i[58:27];
assign wdata_in        =   data_register_i[63:32];

assign axi_read_cmd    = (operation_in == 4'b0111);
assign axi_write_cmd   = (operation_in == 4'b0011);
assign go_read         = module_select_i & axi_read_cmd  & update_dr_i;
assign go_write        = module_select_i & axi_write_cmd & update_dr_i;
assign shf_done        = (shift_cnt == 'd31);

localparam IDLE             = 0,
           AXI_READ32_PRE   = 1,
           AXI_READ32       = 2,
           AXI_READ32_POST  = 3,
           A2J_CAPTURE      = 4,
           READ32_TDO_SHFT  = 5,
           AXI_WRITE32_PRE  = 6,
           AXI_WRITE32      = 7,
           AXI_WRITE32_POST = 8;

always @ (posedge tck_i or negedge trstn_i) begin
    if(~trstn_i)
        state <= IDLE;
    else
        state <= next_state;
end

always @ (posedge tck_i) begin
    test_state <= next_state;
end

always @ (*) begin
    case (state)
        IDLE             : begin if (go_read)      next_state = AXI_READ32_PRE;   else if (go_write) next_state = AXI_WRITE32_PRE; else next_state = IDLE; end
        AXI_READ32_PRE   : begin                   next_state = AXI_READ32;                                           end
        AXI_READ32       : begin                   next_state = AXI_READ32_POST;                                      end
        AXI_READ32_POST  : begin if (rd_done)      next_state = A2J_CAPTURE;      else next_state = AXI_READ32_POST;  end
        A2J_CAPTURE      : begin if (capture_dr_i) next_state = READ32_TDO_SHFT;  else next_state = A2J_CAPTURE;      end
        READ32_TDO_SHFT  : begin if (shf_done)     next_state = IDLE;             else next_state = READ32_TDO_SHFT;  end
        AXI_WRITE32_PRE  : begin if (update_dr_i)  next_state = AXI_WRITE32;      else next_state = AXI_WRITE32_PRE;  end
        AXI_WRITE32      : begin                   next_state = AXI_WRITE32_POST;                                     end
        AXI_WRITE32_POST : begin if (wr_done)      next_state = IDLE;             else next_state = AXI_WRITE32_POST; end
        default          : begin                   next_state = IDLE;                                                 end
    endcase
end

always @ (*) begin
    top_inhibit_o = 1'b0;
    jtag_idle     = 1'b0;
    axi_read      = 1'b0;
    a2j_capture   = 1'b0;
    shift_en      = 1'b0;
    lock_wdata    = 1'b0;
    axi_write     = 1'b0;
    wait_bvalid   = 1'b0;
    case (state)
        IDLE             : begin jtag_idle   = 1'b1; end
        AXI_READ32_PRE   : begin end
        AXI_READ32       : begin axi_read    = 1'b1; end
        AXI_READ32_POST  : begin end
        A2J_CAPTURE      : begin a2j_capture = 1'b1; end
        READ32_TDO_SHFT  : begin shift_en    = 1'b1; end
        AXI_WRITE32_PRE  : begin lock_wdata  = 1'b1; top_inhibit_o = 1'b1; end
        AXI_WRITE32      : begin axi_write   = 1'b1; top_inhibit_o = 1'b1; end
        AXI_WRITE32_POST : begin wait_bvalid = 1'b1; top_inhibit_o = 1'b1; end
        default          : begin end
    endcase
end

efx_cdc_pulse_gen rd_done_a2j (.src_clk (axi_aclk), .dest_clk (tck_i), .rst_n (axi_aresetn), .pulse_in (axi_master_r_valid), .pulse_out (rd_done));
efx_cdc_pulse_gen wr_done_a2j (.src_clk (axi_aclk), .dest_clk (tck_i), .rst_n (axi_aresetn), .pulse_in (axi_master_b_valid), .pulse_out (wr_done));

assign module_tdo_o = jtag_rdata[0];

always @ (posedge tck_i or negedge trstn_i) begin
    if(~trstn_i) begin
        jtag_rdata <= 'h0;
    end
    else if (shift_en && shift_dr_i) begin
        jtag_rdata <= {1'b0,jtag_rdata[31:1]};
    end
    else if (a2j_capture && capture_dr_i) begin
        jtag_rdata <= axi_rdata_int;
    end
end

always @ (posedge tck_i or negedge trstn_i) begin
    if(~trstn_i) begin
        shift_cnt <= 'h0;
    end
    else if (shf_done) begin
        shift_cnt <= 'h0;
    end
    else if (shift_en) begin
        shift_cnt <= shift_cnt + 1'b1;
    end
end

always @ (posedge tck_i or negedge trstn_i) begin
    if(~trstn_i) begin
        jtag_axi_addr <= 'h0;
    end
    else if (jtag_idle && update_dr_i) begin
        jtag_axi_addr <= address_data_in;
    end
end

always @ (posedge tck_i or negedge trstn_i) begin
    if(~trstn_i) begin
        jtag_wdata <= 'h0;
    end
    else if (lock_wdata && update_dr_i) begin
        jtag_wdata <= wdata_in;
    end
end

// ================================================
// =============== AXI Clock Domain ===============
// ================================================

efx_cdc_pulse_gen axi_read_j2a  (.src_clk (tck_i), .dest_clk (axi_aclk), .rst_n (axi_aresetn), .pulse_in (axi_read),  .pulse_out (axi_read_sync));

always @ (posedge axi_aclk or negedge axi_aresetn) begin
    if(~axi_aresetn) begin
        axi_master_ar_addr <= 'h0;
    end
    else if (axi_read_sync) begin
        axi_master_ar_addr <= jtag_axi_addr;
    end
end

always @ (posedge axi_aclk or negedge axi_aresetn) begin
    if(~axi_aresetn) begin
        axi_master_ar_valid_int <= 'h0;
    end
    else if (axi_master_ar_valid_int && axi_master_ar_ready) begin
        axi_master_ar_valid_int <= 1'b0;
    end
    else if (axi_read_sync) begin
        axi_master_ar_valid_int <= 1'b1;
    end
end

always @ (posedge axi_aclk or negedge axi_aresetn) begin
    if(~axi_aresetn) begin
        axi_rdata_int <= 'h0;
    end
    else if (axi_master_r_valid && axi_master_r_ready) begin
        axi_rdata_int <= axi_master_r_data;
    end
end

efx_cdc_pulse_gen axi_write_j2a (.src_clk (tck_i), .dest_clk (axi_aclk), .rst_n (axi_aresetn), .pulse_in (axi_write), .pulse_out (axi_write_sync));

always @ (posedge axi_aclk or negedge axi_aresetn) begin
    if(~axi_aresetn) begin
        axi_master_aw_addr <= 'h0;
    end
    else if (axi_write_sync) begin
        axi_master_aw_addr <= jtag_axi_addr;
    end
end

always @ (posedge axi_aclk or negedge axi_aresetn) begin
    if (~axi_aresetn) begin
        axi_master_aw_valid_int <= 'h0;
    end
    else if (axi_master_aw_valid_int && axi_master_aw_ready) begin
        axi_master_aw_valid_int <= 1'b0;
    end
    else if (axi_write_sync) begin
        axi_master_aw_valid_int <= 1'b1;
    end
end

always @ (posedge axi_aclk or negedge axi_aresetn) begin
    if (~axi_aresetn) begin
        axi_master_w_data <= 'h0;
    end
    else if (axi_write_sync) begin
        axi_master_w_data <= jtag_wdata;
    end
end

always @ (posedge axi_aclk or negedge axi_aresetn) begin
    if (~axi_aresetn) begin
        axi_master_w_valid_int <= 'h0;
    end
    else if (axi_master_w_valid_int && axi_master_w_ready) begin
        axi_master_w_valid_int <= 1'b0;
    end
    else if (axi_write_sync) begin
        axi_master_w_valid_int <= 1'b1;
    end
end

always @ (posedge axi_aclk or negedge axi_aresetn) begin
    if (~axi_aresetn) begin
        axi_master_w_valid  = 1'b0;
        axi_master_aw_valid = 1'b0;
        axi_master_ar_valid = 1'b0;
    end
    else begin
        axi_master_w_valid  = axi_master_w_valid_int;
        axi_master_aw_valid = axi_master_aw_valid_int;
        axi_master_ar_valid = axi_master_ar_valid_int;
    end
end

assign                        axi_master_aw_prot   = 'h0;
assign                        axi_master_aw_region = 'h0;
assign                        axi_master_aw_len    = 'h0;
assign                        axi_master_aw_size   = 'h0;
assign                        axi_master_aw_burst  = 'h0;
assign                        axi_master_aw_lock   = 'h0;
assign                        axi_master_aw_cache  = 'h0;
assign                        axi_master_aw_qos    = 'h0;
assign                        axi_master_aw_id     = 'h0;
assign                        axi_master_aw_user   = 'h0;

assign                        axi_master_ar_prot   = 'h0;
assign                        axi_master_ar_region = 'h0;
assign                        axi_master_ar_len    = 'h0;
assign                        axi_master_ar_size   = 'd2;
assign                        axi_master_ar_burst  = 'h0;
assign                        axi_master_ar_lock   = 'h0;
assign                        axi_master_ar_cache  = 'h0;
assign                        axi_master_ar_qos    = 'h0;
assign                        axi_master_ar_id     = 'h0;
assign                        axi_master_ar_user   = 'h0;
assign                        axi_master_w_strb    = {(AXI_DATA_WIDTH/8){1'b1}};
assign                        axi_master_w_user    = {AXI_USER_WIDTH{1'b1}};
assign                        axi_master_w_last    = 1'b1;
assign                        axi_master_r_ready   = 1'b1;
assign                        axi_master_b_ready   = 1'b1;

endmodule