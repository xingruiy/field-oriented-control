// ============================================================================
// pwm_core.sv
//
//  Left-aligned (edge-aligned) PWM generator.
//
//  - Free-running counter cnt counts 0 .. TOP and wraps (period = TOP+1).
//  - Output is high while cnt < duty, so duty is the number of high cycles
//    per period:
//        duty = 0          -> always low  (0 %)
//        duty = PWM_PERIOD -> always high (100 %, needs duty wider than the count)
//        duty = N          -> N / PWM_PERIOD duty cycle
//  - The output is registered for clean, glitch-free edges.
//
//  Conventions: single clock `clk`, asynchronous active-low reset `rst_n`.
// ============================================================================

module pwm_core
  import pwm_pkg::*;
#(
  parameter int unsigned WIDTH = PWM_W
)(
  input  logic             clk,
  input  logic             rst_n,
  input  logic [WIDTH-1:0] duty,  // high-cycle count per period
  output logic             pwm,   // PWM output
  output logic [WIDTH-1:0] cnt    // free-running counter (exposed for sim/debug)
);

  localparam logic [WIDTH-1:0] TOP = WIDTH'(PWM_TOP);

  // free-running period counter
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) cnt <= '0;
    else        cnt <= (cnt == TOP) ? '0 : (cnt + 1'b1);
  end

  // registered compare: high for the first `duty` counts of each period
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) pwm <= 1'b0;
    else        pwm <= (duty != '0) && (cnt < duty);
  end

endmodule
