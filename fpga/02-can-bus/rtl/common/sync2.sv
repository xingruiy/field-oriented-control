// ============================================================================
// sync2.sv
//
//  Generic 2-FF input synchronizer for asynchronous, slowly-changing inputs
//  (e.g. slide switches / buttons crossing into the clk domain). Parameterized
//  bus width; reset value is all-zeros.
// ============================================================================

module sync2 #(
  parameter int unsigned W = 1
)(
  input  logic         clk,
  input  logic         rst_n,
  input  logic [W-1:0] d,    // async input
  output logic [W-1:0] q     // synchronized output
);

  logic [W-1:0] meta;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      meta <= '0;
      q    <= '0;
    end else begin
      meta <= d;
      q    <= meta;
    end
  end

endmodule
