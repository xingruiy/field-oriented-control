`timescale 1ns/1ps

module tb_cmd_parser;
  import can_pkg::*;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n = 1'b0;
  logic rx_valid, rx_last, rx_ide;
  logic [7:0] rx_data;
  logic motor_enable, ctrl_mode, cmd_pulse;
  logic signed [15:0] speed_sp, current_sp;

  cmd_parser dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .rx_valid     (rx_valid),
    .rx_last      (rx_last),
    .rx_data      (rx_data),
    .rx_ide       (rx_ide),
    .motor_enable (motor_enable),
    .ctrl_mode    (ctrl_mode),
    .speed_sp     (speed_sp),
    .current_sp   (current_sp),
    .cmd_pulse    (cmd_pulse)
  );

  task automatic send4(input logic [7:0] b0, b1, b2, b3, input logic ide);
    @(negedge clk); rx_ide = ide; rx_valid = 1'b1; rx_last = 1'b0; rx_data = b0;
    @(negedge clk); rx_data = b1;
    @(negedge clk); rx_data = b2;
    @(negedge clk); rx_last = 1'b1; rx_data = b3;
    @(negedge clk); rx_valid = 1'b0; rx_last = 1'b0; rx_data = '0; rx_ide = 1'b0;
    repeat (2) @(negedge clk);
  endtask

  initial begin
    rx_valid = 1'b0;
    rx_last  = 1'b0;
    rx_ide   = 1'b0;
    rx_data  = '0;

    repeat (5) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    send4(OP_SET_ENABLE, 8'h01, 8'h00, 8'h00, 1'b0);
    assert(motor_enable) else $fatal(1, "enable command failed");

    send4(OP_SET_MODE, 8'h01, 8'h00, 8'h00, 1'b0);
    assert(ctrl_mode == MODE_CURRENT) else $fatal(1, "mode command failed");

    send4(OP_SET_SPEED, 8'h04, 8'hD2, 8'h00, 1'b0);
    assert(speed_sp == 16'sd1234) else $fatal(1, "speed command failed: %0d", speed_sp);

    send4(OP_SET_CURRENT, 8'hFF, 8'h38, 8'h00, 1'b0);
    assert(current_sp == -16'sd200) else $fatal(1, "current command failed: %0d", current_sp);

    send4(OP_SET_ENABLE, 8'h00, 8'h00, 8'h00, 1'b1);
    assert(motor_enable) else $fatal(1, "extended frame should be ignored");

    $display("TB_PASS: tb_cmd_parser");
    $finish;
  end
endmodule
