`timescale 1ns/1ps

module tb_motor_model;
  import can_pkg::*;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n = 1'b0;
  logic enable = 1'b0;
  logic mode = MODE_SPEED;
  logic signed [15:0] speed_sp = '0;
  logic signed [15:0] current_sp = '0;
  logic signed [15:0] speed, current;
  logic tick;

  motor_model #(.TICK_DIV(4)) dut (
    .clk        (clk),
    .rst_n      (rst_n),
    .enable     (enable),
    .mode       (mode),
    .speed_sp   (speed_sp),
    .current_sp (current_sp),
    .speed      (speed),
    .current    (current),
    .tick       (tick)
  );

  task automatic wait_ticks(input int n);
    int seen = 0;
    while (seen < n) begin
      @(posedge clk);
      if (tick) seen++;
    end
  endtask

  initial begin
    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    enable = 1'b1;
    mode = MODE_SPEED;
    speed_sp = 16'sd3000;
    wait_ticks(400);
    assert(speed > 16'sd2200) else $fatal(1, "speed loop did not rise: %0d", speed);
    assert(current > 16'sd0) else $fatal(1, "speed loop current did not rise: %0d", current);

    mode = MODE_CURRENT;
    current_sp = -16'sd500;
    wait_ticks(80);
    assert(current < -16'sd400) else $fatal(1, "current loop did not track negative current: %0d", current);

    enable = 1'b0;
    wait_ticks(250);
    assert((current > -16'sd5) && (current < 16'sd5)) else $fatal(1, "disable did not collapse current: %0d", current);

    $display("TB_PASS: tb_motor_model");
    $finish;
  end
endmodule
