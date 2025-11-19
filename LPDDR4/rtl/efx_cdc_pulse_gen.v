module efx_cdc_pulse_gen (
    input  wire src_clk,
    input  wire dest_clk,
    input  wire rst_n,
    input  wire pulse_in,
    output wire pulse_out
);

reg pulse_stretch;
reg pulse_stretch_sync_r;

wire pulse_clear;
wire pulse_stretch_sync;

always @ (posedge src_clk or negedge rst_n) begin
    if (~rst_n) begin
         pulse_stretch <= 1'b0;
    end
    else if (pulse_clear) begin
        pulse_stretch <= 1'b0;
    end
    else if (pulse_in) begin
        pulse_stretch <= 1'b1;
    end
end

efx_datasync #(
    .STAGE (2),
    .WIDTH (1)
) xpulse_clear_sync (
    .clk_i    (src_clk),
    .rst_n    (rst_n),
    .data_in  (pulse_stretch_sync_r),
    .data_out (pulse_clear)
);

efx_datasync #(
    .STAGE (2),
    .WIDTH (1)
) xpulse_stretch_sync (
    .clk_i    (dest_clk),
    .rst_n    (rst_n),
    .data_in  (pulse_stretch),
    .data_out (pulse_stretch_sync)
);

always @ (posedge dest_clk or negedge rst_n) begin
    if (~rst_n) begin
        pulse_stretch_sync_r <= 1'b0;
    end
    else begin
        pulse_stretch_sync_r <= pulse_stretch_sync;
    end
end

assign pulse_out = ~pulse_stretch_sync_r & pulse_stretch_sync;

endmodule
