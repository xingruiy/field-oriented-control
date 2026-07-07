// ============================================================================
// tb_pwm_core.sv
//
//  Self-checking testbench for pwm_core.
//
//  For each programmed duty it measures, over one full period:
//    - the number of cycles the output is high  (== duty)
//    - the period length between rising edges    (== PWM_TOP + 1)
//  and checks the 0 % corner (duty = 0 -> always low).
//
//  Emits exactly one banner: TB_PASS / TB_FAIL.
// ============================================================================
`timescale 1ns / 1ps

module tb_pwm_core
  import pwm_pkg::*;
;
  localparam int unsigned WIDTH  = PWM_W;
  localparam int unsigned PERIOD = PWM_TOP + 1; // PWM_PERIOD = 1250

  logic              clk = 0;
  logic              rst_n;
  logic [WIDTH-1:0]  duty;
  logic              pwm;
  logic [WIDTH-1:0]  cnt;

  int errors = 0;

  always #5 clk = ~clk; // 100 MHz

  pwm_core #(.WIDTH(WIDTH)) dut (.*);

  // -- helpers ---------------------------------------------------------------

  // Align to the start of a period (cnt wraps to 0), then count high cycles
  // over exactly PERIOD clocks and check against the programmed duty.
  task automatic measure_duty(input logic [WIDTH-1:0] d);
    int high;
    duty = d;
    // wait for a clean period boundary
    @(posedge clk);
    while (cnt != '0) @(posedge clk);
    high = 0;
    for (int i = 0; i < PERIOD; i++) begin
      if (pwm) high++;
      @(posedge clk);
    end
    if (high !== int'(d)) begin
      $display("  MISMATCH duty=%0d : measured high=%0d expected=%0d", d, high, d);
      errors++;
    end else begin
      $display("  ok duty=%0d -> high=%0d (%0d.%02d %%)",
               d, high, (high*100)/PERIOD, ((high*10000)/PERIOD)%100);
    end
  endtask

  // -- continuous invariant: period between rising edges == PERIOD -----------
  int last_rise = -1;
  int tick      = 0;
  logic pwm_d;
  always @(posedge clk) begin
    if (rst_n) begin
      tick <= tick + 1;
      if (pwm && !pwm_d) begin
        if (last_rise >= 0 && (tick - last_rise) != PERIOD) begin
          $display("  MISMATCH period=%0d expected=%0d", tick - last_rise, PERIOD);
          errors++;
        end
        last_rise <= tick;
      end
      pwm_d <= pwm;
    end
  end

  // -- stimulus --------------------------------------------------------------
  initial begin
    rst_n = 1'b0;
    duty  = '0;
    pwm_d = 1'b0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    measure_duty(WIDTH'(0));    // 0 %
    measure_duty(WIDTH'(78));   // ~6.25 %  (sw=0001)
    measure_duty(WIDTH'(625));  // 50 %     (sw=1000)
    measure_duty(WIDTH'(1171)); // ~93.7 %  (sw=1111)
    measure_duty(WIDTH'(1249)); // ~99.9 %  (PWM_TOP)

    if (errors == 0) $display("TB_PASS: tb_pwm_core");
    else             $display("TB_FAIL: tb_pwm_core (%0d errors)", errors);
    $finish;
  end

endmodule
