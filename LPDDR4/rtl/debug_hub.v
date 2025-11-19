module debug_hub #(
    parameter ID_WIDTH = 4,
    parameter CS_WIDTH = (1<<ID_WIDTH)-1
)(
    // JTAG Interface
    input  bscan_CAPTURE,
    input  bscan_DRCK,
    input  bscan_RESET,
    input  bscan_RUNTEST,
    input  bscan_SEL,
    input  bscan_SHIFT,
    input  bscan_TCK,
    input  bscan_TDI,
    input  bscan_TMS,
    input  bscan_UPDATE,
    output bscan_TDO,

    output [CS_WIDTH-1:0]   edb_module_selects,
    input  [CS_WIDTH-1:0]   edb_module_inhibit,
    input  [CS_WIDTH-1:0]   edb_module_tdo,
    output [`DR_WIDTH-1:0]  edb_user_dr
);

// ==========================================

   reg [`DR_WIDTH-1:0] 	    shift_reg;
   wire 		            hub_select;
   wire [ID_WIDTH-1:0] 	    module_id_in;
   reg [ID_WIDTH-1:0] 	    module_id_reg;
   wire [ID_WIDTH-1:0] 	    module_id_sub1;
   wire 		             select_inhibit;
   reg [CS_WIDTH-1:0] 	    module_selects;
   wire [(1<<ID_WIDTH)-1:0] module_tdo_pwr2;
   integer 		            i;

    assign hub_select   = shift_reg[`DR_WIDTH-1];
    assign module_id_in = shift_reg[`DR_WIDTH-2 -: ID_WIDTH];
    assign edb_user_dr  = shift_reg;

    assign select_inhibit = | edb_module_inhibit;

    always @(posedge bscan_TCK or posedge bscan_RESET) begin
        if (bscan_RESET)
            shift_reg <= {`DR_WIDTH{1'b0}};
        else if (bscan_SEL && bscan_SHIFT)
            shift_reg <= {bscan_TDI, shift_reg[`DR_WIDTH-1:1]};
    end

    always @(posedge bscan_TCK or posedge bscan_RESET) begin
        if (bscan_RESET)
            module_id_reg <= {ID_WIDTH{1'b0}};
        else if (bscan_SEL && hub_select && bscan_UPDATE && !select_inhibit)
            module_id_reg <= module_id_in;
    end

    always @(*) begin
        for (i = 0; i < CS_WIDTH; i = i + 1) begin
            if (module_id_reg == i + 1)
                module_selects[i] <= 1'b1;
            else
                module_selects[i] <= 1'b0;
        end
    end

    assign edb_module_selects = module_selects;

    assign module_id_sub1 = module_id_reg - 1'b1;
    assign module_tdo_pwr2 = {1'b0, edb_module_tdo}; // 1'b0 for id 15
    assign bscan_TDO = module_tdo_pwr2[module_id_sub1];

endmodule

