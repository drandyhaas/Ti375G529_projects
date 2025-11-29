// Dual-port RAM for storing LVDS samples
// Port A: Write port (from downsampler, controlled by triggerer)
// Port B: Read port (for command_processor)
//
// TODO: Add CDC (Clock Domain Crossing) protection
// TODO: Consider using FPGA block RAM primitives for better performance

module sample_ram (
   // Write port (Port A) - LVDS clock domain
   input wire        clk_wr,
   input wire        wr_en,
   input wire [9:0]  wr_addr,
   input wire [559:0] wr_data,

   // Read port (Port B) - Command processor clock domain
   input wire        clk_rd,
   input wire [9:0]  rd_addr,
   output reg [559:0] rd_data
);

// RAM storage: 1024 entries × 560 bits
reg [559:0] ram [1023:0];

// Write port
always @(posedge clk_wr) begin
   if (wr_en)
      ram[wr_addr] <= wr_data;
end

// Read port
always @(posedge clk_rd) begin
   rd_data <= ram[rd_addr];
end

endmodule
