module efx_datasync #(
    parameter STAGE = 2,
    parameter WIDTH = 1
) (
    input  wire             clk_i,
    input  wire             rst_n,
    input  wire [WIDTH-1:0] data_in,
    output wire [WIDTH-1:0] data_out
);

(* async_reg = "true" *) reg [WIDTH-1:0] pipe_reg [STAGE-1:0];
integer i;

assign data_out = pipe_reg[STAGE-1];

always @(posedge clk_i) begin
    for (i=STAGE-1; i>0; i = i - 1) begin
        pipe_reg[i] <= pipe_reg[i-1];
    end
    pipe_reg[0] <= data_in;
end

endmodule

