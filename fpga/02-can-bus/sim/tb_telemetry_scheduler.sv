`timescale 1ns/1ps

module tb_telemetry_scheduler;
  import can_pkg::*;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n = 1'b0;
  logic tick = 1'b0;
  logic signed [15:0] speed = 16'sd123;
  logic signed [15:0] current = -16'sd45;
  logic enable = 1'b1;
  logic mode = MODE_SPEED;
  logic tx_valid, tx_ready = 1'b1, tx_pulse;
  logic [31:0] tx_data;

  telemetry_scheduler #(.TELEM_TICKS(2), .STATUS_EVERY(2)) dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .tick     (tick),
    .speed    (speed),
    .current  (current),
    .enable   (enable),
    .mode     (mode),
    .tx_valid (tx_valid),
    .tx_ready (tx_ready),
    .tx_data  (tx_data),
    .tx_pulse (tx_pulse)
  );

  logic [31:0] got [0:5];
  int n = 0;

  always_ff @(posedge clk) begin
    if (tx_valid && tx_ready) begin
      got[n] <= tx_data;
      n <= n + 1;
    end
  end

  task automatic pulse_tick;
    @(negedge clk); tick = 1'b1;
    @(negedge clk); tick = 1'b0;
  endtask

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    pulse_tick();
    pulse_tick();
    repeat (3) @(negedge clk);
    pulse_tick();
    pulse_tick();
    repeat (8) @(negedge clk);

    assert(n >= 5) else $fatal(1, "expected at least 5 telemetry words, got %0d", n);
    assert(got[0][31:24] == MUX_SPEED) else $fatal(1, "first word is not speed");
    assert(got[1][31:24] == MUX_CURRENT) else $fatal(1, "second word is not current");
    assert(got[2][31:24] == MUX_STATUS) else $fatal(1, "third word is not status");
    assert(got[3][31:24] == MUX_SPEED) else $fatal(1, "fourth word is not speed");
    assert(got[4][31:24] == MUX_CURRENT) else $fatal(1, "fifth word is not current");

    $display("TB_PASS: tb_telemetry_scheduler");
    $finish;
  end
endmodule
