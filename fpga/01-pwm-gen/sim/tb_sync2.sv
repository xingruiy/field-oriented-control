// ============================================================================
// tb_sync2.sv
//
//  Self-checking testbench for the 2-FF synchronizer. Verifies that a change
//  on the input appears on the output exactly two clocks later.
//
//  Emits exactly one banner: TB_PASS / TB_FAIL.
// ============================================================================
`timescale 1ns / 1ps

module tb_sync2;
  localparam int unsigned W = 4;

  logic         clk = 0;
  logic         rst_n;
  logic [W-1:0] d;
  logic [W-1:0] q;

  int errors = 0;

  always #5 clk = ~clk;

  sync2 #(.W(W)) dut (.*);

  initial begin
    rst_n = 1'b0;
    d     = '0;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // drive a new value, expect it 2 clocks later
    d = 4'hA;
    @(posedge clk);              // captured into meta
    if (q === 4'hA) begin $display("  MISMATCH q updated too early"); errors++; end
    @(posedge clk);              // propagated to q
    #1;
    if (q !== 4'hA) begin $display("  MISMATCH q=%0h expected A", q); errors++; end

    d = 4'h5;
    repeat (2) @(posedge clk);
    #1;
    if (q !== 4'h5) begin $display("  MISMATCH q=%0h expected 5", q); errors++; end

    if (errors == 0) $display("TB_PASS: tb_sync2");
    else             $display("TB_FAIL: tb_sync2 (%0d errors)", errors);
    $finish;
  end

endmodule
