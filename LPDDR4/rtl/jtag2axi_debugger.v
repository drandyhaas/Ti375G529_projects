`resetall
`timescale 1ns / 1ps

//`define DBG_ID_WIDTH 4
//`define CS_WIDTH (1<<`DBG_ID_WIDTH)-1
//`define DR_WIDTH  82

module jtag2axi_debugger #(
    parameter NUM_OF_AXI_PORT  = 1,
    parameter AXI_ADDR_WIDTH   = 32,
    parameter AXI_DATA_WIDTH   = 256,
    parameter AXI_DATA_WIDTH_B = 128,
    parameter AXI_USER_WIDTH   = 6,
    parameter AXI_ID_WIDTH     = 8,
    parameter AXI_DATA_STRB    = AXI_DATA_WIDTH / 8
) (
// ==================== JTAG Interface ============================
   input  wire                                      jtag_inst1_CAPTURE,
   input  wire                                      jtag_inst1_DRCK,
   input  wire                                      jtag_inst1_RESET,
   input  wire                                      jtag_inst1_RUNTEST,
   input  wire                                      jtag_inst1_SEL,
   input  wire                                      jtag_inst1_SHIFT,
   input  wire                                      jtag_inst1_TCK,
   input  wire                                      jtag_inst1_TDI,
   input  wire                                      jtag_inst1_TMS,
   input  wire                                      jtag_inst1_UPDATE,
   output wire                                      jtag_inst1_TDO,
// ==================== AXI Global Signal =========================
   input  wire                                      axi_aclk,
   input  wire                                      axi_aresetn,
// ==================== AXI Write Channel =========================
   input  wire [NUM_OF_AXI_PORT-1:0]                axi_master_aw_ready,
   output wire [NUM_OF_AXI_PORT*AXI_ID_WIDTH-1:0]   axi_master_aw_id,
   output wire [NUM_OF_AXI_PORT-1:0]                axi_master_aw_valid,
   output wire [NUM_OF_AXI_PORT*AXI_ADDR_WIDTH-1:0] axi_master_aw_addr,
   output wire [NUM_OF_AXI_PORT*8-1:0]              axi_master_aw_len,
   output wire [NUM_OF_AXI_PORT*4-1:0]              axi_master_aw_size,
   output wire [NUM_OF_AXI_PORT*2-1:0]              axi_master_aw_burst,
   output wire [NUM_OF_AXI_PORT*3-1:0]              axi_master_aw_lock,
   output wire [NUM_OF_AXI_PORT*3-1:0]              axi_master_aw_prot,
   output wire [NUM_OF_AXI_PORT*4-1:0]              axi_master_aw_region,
   output wire [NUM_OF_AXI_PORT*4-1:0]              axi_master_aw_cache,
   output wire [NUM_OF_AXI_PORT*4-1:0]              axi_master_aw_qos,
   output wire [NUM_OF_AXI_PORT*AXI_USER_WIDTH-1:0] axi_master_aw_user,
// ==================== AXI Read  Channel =========================
   input  wire [NUM_OF_AXI_PORT-1:0]                axi_master_ar_ready,
   output wire [NUM_OF_AXI_PORT-1:0]                axi_master_ar_valid,
   output wire [NUM_OF_AXI_PORT*AXI_ADDR_WIDTH-1:0] axi_master_ar_addr,
   output wire [NUM_OF_AXI_PORT*3-1:0]              axi_master_ar_prot,
   output wire [NUM_OF_AXI_PORT*4-1:0]              axi_master_ar_region,
   output wire [NUM_OF_AXI_PORT*8-1:0]              axi_master_ar_len,
   output wire [NUM_OF_AXI_PORT*3-1:0]              axi_master_ar_size,
   output wire [NUM_OF_AXI_PORT*2-1:0]              axi_master_ar_burst,
   output wire [NUM_OF_AXI_PORT*3-1:0]              axi_master_ar_lock,
   output wire [NUM_OF_AXI_PORT*4-1:0]              axi_master_ar_cache,
   output wire [NUM_OF_AXI_PORT*4-1:0]              axi_master_ar_qos,
   output wire [NUM_OF_AXI_PORT*AXI_ID_WIDTH-1:0]   axi_master_ar_id,
   output wire [NUM_OF_AXI_PORT*AXI_USER_WIDTH-1:0] axi_master_ar_user,
// ==================== AXI Write Data Channel ====================
   input  wire [NUM_OF_AXI_PORT-1:0]                axi_master_w_ready,
   output wire [NUM_OF_AXI_PORT-1:0]                axi_master_w_valid,
   output wire [NUM_OF_AXI_PORT*AXI_DATA_WIDTH-1:0] axi_master_w_data,
   output wire [NUM_OF_AXI_PORT*AXI_ID_WIDTH-1:0]   axi_master_w_id,
   output wire [NUM_OF_AXI_PORT*AXI_DATA_STRB-1:0]  axi_master_w_strb,
   output wire [NUM_OF_AXI_PORT-1:0]                axi_master_w_last,
   output wire [NUM_OF_AXI_PORT*AXI_USER_WIDTH-1:0] axi_master_w_user,
// ==================== AXI Write Response Channel ================
   input  wire [NUM_OF_AXI_PORT*AXI_ID_WIDTH-1:0]   axi_master_b_id,
   input  wire [NUM_OF_AXI_PORT-1:0]                axi_master_b_valid,
   input  wire [NUM_OF_AXI_PORT*2-1:0]              axi_master_b_resp,
   input  wire [NUM_OF_AXI_PORT*AXI_USER_WIDTH-1:0] axi_master_b_user,
   output wire [NUM_OF_AXI_PORT-1:0]                axi_master_b_ready,
// ==================== AXI Read Data Channel =====================
   input  wire [NUM_OF_AXI_PORT-1:0]                axi_master_r_valid,
   input  wire [NUM_OF_AXI_PORT*AXI_DATA_WIDTH-1:0] axi_master_r_data,
   input  wire [NUM_OF_AXI_PORT*2-1:0]              axi_master_r_resp,
   input  wire [NUM_OF_AXI_PORT-1:0]                axi_master_r_last,
   input  wire [NUM_OF_AXI_PORT*AXI_ID_WIDTH-1:0]   axi_master_r_id,
   input  wire [NUM_OF_AXI_PORT*AXI_USER_WIDTH-1:0] axi_master_r_user,
   output wire [NUM_OF_AXI_PORT-1:0]                axi_master_r_ready
);

   localparam NUM_DEV_SUPPORT = 3;

  //Internal Signals
   wire [`CS_WIDTH-1:0]           edb_module_selects;
   wire [`DR_WIDTH-1:0]           edb_user_dr;
   wire [`CS_WIDTH-1:0]           edb_module_inhibit;
   wire [`CS_WIDTH-1:0]           edb_module_tdo;

   assign edb_module_inhibit[`CS_WIDTH-1:NUM_OF_AXI_PORT] = 'h0;
   assign edb_module_tdo[`CS_WIDTH-1:NUM_OF_AXI_PORT]     = 'h0;

   debug_hub #(
      .ID_WIDTH(`DBG_ID_WIDTH)     //Set to 4 (For consistency purpose)
   ) u_debug_hub (
      .bscan_CAPTURE      (jtag_inst1_CAPTURE),
      .bscan_DRCK         (jtag_inst1_DRCK),
      .bscan_RESET        (jtag_inst1_RESET),
      .bscan_RUNTEST      (jtag_inst1_RUNTEST),
      .bscan_SEL          (jtag_inst1_SEL),
      .bscan_SHIFT        (jtag_inst1_SHIFT),
      .bscan_TCK          (jtag_inst1_TCK),
      .bscan_TDI          (jtag_inst1_TDI),
      .bscan_TMS          (jtag_inst1_TMS),
      .bscan_UPDATE       (jtag_inst1_UPDATE),
      .bscan_TDO          (jtag_inst1_TDO),
      .edb_module_inhibit (edb_module_inhibit),
      .edb_module_tdo     (edb_module_tdo),
      .edb_module_selects (edb_module_selects),
      .edb_user_dr        (edb_user_dr)
   );

genvar i;
generate
   for (i=0;i<NUM_OF_AXI_PORT;i=i+1) begin : AXI_MASTER
   j2a_bridge #(
      .AXI_ADDR_WIDTH       (AXI_ADDR_WIDTH),
      .AXI_DATA_WIDTH       (AXI_DATA_WIDTH),
      .AXI_USER_WIDTH       (AXI_USER_WIDTH),
      .AXI_ID_WIDTH         (AXI_ID_WIDTH)
   ) bridge0 (
      .tck_i                (jtag_inst1_TCK),
      .tdi_i                (jtag_inst1_TDI),
      .capture_dr_i         (jtag_inst1_CAPTURE),
      .shift_dr_i           (jtag_inst1_SHIFT),
      .update_dr_i          (jtag_inst1_UPDATE),
      .trstn_i              (~jtag_inst1_RESET),
      .module_tdo_o         (edb_module_tdo[i]),
      .data_register_i      (edb_user_dr[(`DR_WIDTH-1):(`DR_WIDTH-64)]),
      .module_select_i      (edb_module_selects[i]),
      .top_inhibit_o        (edb_module_inhibit[i]),
      .axi_aclk             (axi_aclk),
      .axi_aresetn          (axi_aresetn),

      .axi_master_aw_valid  (axi_master_aw_valid[i]),
      .axi_master_aw_addr   (axi_master_aw_addr[i*AXI_ADDR_WIDTH+:AXI_ADDR_WIDTH]),
      .axi_master_aw_prot   (axi_master_aw_prot[i*3+:3]),
      .axi_master_aw_region (axi_master_aw_region[i*4+:4]),
      .axi_master_aw_len    (axi_master_aw_len[i*8+:8]),
      .axi_master_aw_size   (axi_master_aw_size[i*3+:3]),
      .axi_master_aw_burst  (axi_master_aw_burst[i*2+:2]),
      .axi_master_aw_lock   (axi_master_aw_lock[i*3+:3]),
      .axi_master_aw_cache  (axi_master_aw_cache[i*4+:4]),
      .axi_master_aw_qos    (axi_master_aw_qos[i*4+:4]),
      .axi_master_aw_id     (axi_master_aw_id[i*AXI_ID_WIDTH+:AXI_ID_WIDTH]),
      .axi_master_aw_user   (axi_master_aw_user[i*AXI_USER_WIDTH+:AXI_USER_WIDTH]),
      .axi_master_aw_ready  (axi_master_aw_ready[i]),

      .axi_master_ar_valid  (axi_master_ar_valid[i]),
      .axi_master_ar_addr   (axi_master_ar_addr[i*AXI_ADDR_WIDTH+:AXI_ADDR_WIDTH]),
      .axi_master_ar_prot   (axi_master_ar_prot[i*3+:3]),
      .axi_master_ar_region (axi_master_ar_region[i*4+:4]),
      .axi_master_ar_len    (axi_master_ar_len[i*8+:8]),
      .axi_master_ar_size   (axi_master_ar_size[i*3+:3]),
      .axi_master_ar_burst  (axi_master_ar_burst[i*2+:2]),
      .axi_master_ar_lock   (axi_master_ar_lock[i*3+:3]),
      .axi_master_ar_cache  (axi_master_ar_cache[i*4+:4]),
      .axi_master_ar_qos    (axi_master_ar_qos[i*4+:4]),
      .axi_master_ar_id     (axi_master_ar_id[i*AXI_ID_WIDTH+:AXI_ID_WIDTH]),
      .axi_master_ar_user   (axi_master_ar_user[i*AXI_USER_WIDTH+:AXI_USER_WIDTH]),
      .axi_master_ar_ready  (axi_master_ar_ready[i]),

      .axi_master_w_valid   (axi_master_w_valid[i]),
      .axi_master_w_data    (axi_master_w_data[i*AXI_DATA_WIDTH+:AXI_DATA_WIDTH]),
      .axi_master_w_strb    (axi_master_w_strb[i*AXI_DATA_STRB+:AXI_DATA_STRB]),
      .axi_master_w_user    (axi_master_w_user[i*AXI_USER_WIDTH+:AXI_USER_WIDTH]),
      .axi_master_w_last    (axi_master_w_last[i]),
      .axi_master_w_ready   (axi_master_w_ready[i]),

      .axi_master_r_valid   (axi_master_r_valid[i]),
      .axi_master_r_data    (axi_master_r_data[i*AXI_DATA_WIDTH+:AXI_DATA_WIDTH]),
      .axi_master_r_resp    (axi_master_r_resp[i*2+:2]),
      .axi_master_r_last    (axi_master_r_last[i]),
      .axi_master_r_id      (axi_master_r_id[i*AXI_ID_WIDTH+:AXI_ID_WIDTH]),
      .axi_master_r_user    (axi_master_r_user[i*AXI_USER_WIDTH+:AXI_USER_WIDTH]),
      .axi_master_r_ready   (axi_master_r_ready[i]),

      .axi_master_b_valid   (axi_master_b_valid[i]),
      .axi_master_b_resp    (axi_master_b_resp[i*2+:2]),
      .axi_master_b_id      (axi_master_b_id[i*AXI_ID_WIDTH+:AXI_ID_WIDTH]),
      .axi_master_b_user    (axi_master_b_user[i*AXI_USER_WIDTH+:AXI_USER_WIDTH]),
      .axi_master_b_ready   (axi_master_b_ready[i])

   );
   end
endgenerate
endmodule
