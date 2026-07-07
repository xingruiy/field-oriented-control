// ============================================================================
// motor_model.sv
//
//  Simulated first-order motor plant, so telemetry responds dynamically to
//  the commands without any motor hardware. States are signed Q15.16
//  (integer part = rpm / mA) and update once per `tick` (1 kHz on the board).
//  All gains are arithmetic shifts - no multipliers.
//
//    speed loop (mode = MODE_SPEED):
//      speed   += (speed_sp - speed) >>> 8            tau ~ 256 ticks (256 ms)
//      i_tgt    = (err >>> 2) + (speed >>> 4)         accel spike + viscous load
//      current += (i_tgt - current) >>> 3             electrical tau ~ 8 ticks
//
//    current loop (mode = MODE_CURRENT):
//      current += (current_sp - current) >>> 3        tau ~ 8 ticks
//      speed   += (current >>> 6) - (speed >>> 9)     torque vs drag;
//                                                     steady state 8 rpm/mA,
//                                                     mechanical tau ~ 512 ticks
//    disabled:
//      speed   -= speed >>> 7                         coast, tau ~ 128 ticks
//      current -= current >>> 2                       collapses quickly
//
//  Both states clamp at +/-SPEED_SAT / +/-CURRENT_SAT so the int16 outputs
//  never wrap. Per-tick math is independent of TICK_DIV, so testbenches can
//  shrink TICK_DIV and keep identical tick-domain behavior.
// ============================================================================

module motor_model
  import can_pkg::*;
#(
  parameter int unsigned TICK_DIV = TICK_DIV_DEFAULT  // clk cycles per tick
)(
  input  logic               clk,
  input  logic               rst_n,

  input  logic               enable,
  input  logic               mode,        // MODE_SPEED / MODE_CURRENT
  input  logic signed [15:0] speed_sp,    // rpm
  input  logic signed [15:0] current_sp,  // mA

  output logic signed [15:0] speed,       // rpm
  output logic signed [15:0] current,     // mA
  output logic               tick         // 1-clk strobe, F_CLK / TICK_DIV
);

  // ---- tick generator -------------------------------------------------------
  logic [$clog2(TICK_DIV)-1:0] tick_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tick_cnt <= '0;
      tick     <= 1'b0;
    end else begin
      tick <= 1'b0;
      if (tick_cnt == TICK_DIV - 1) begin
        tick_cnt <= '0;
        tick     <= 1'b1;
      end else begin
        tick_cnt <= tick_cnt + 1'b1;
      end
    end
  end

  // ---- plant states, Q15.16 --------------------------------------------------
  localparam int signed SPEED_MAX   = SPEED_SAT   <<< 16;
  localparam int signed CURRENT_MAX = CURRENT_SAT <<< 16;

  logic signed [31:0] speed_q, current_q;

  // 36-bit intermediates: sums of two Q15.16 values can exceed 32 bits
  logic signed [35:0] spd_next, cur_next, err, i_tgt;

  function automatic logic signed [31:0] clamp(input logic signed [35:0] v,
                                               input int signed        lim);
    if      (v >  36'(lim)) clamp =  lim;
    else if (v < -36'(lim)) clamp = -lim;
    else                    clamp = v[31:0];
  endfunction

  always_comb begin
    err   = 36'(signed'({speed_sp, 16'h0})) - 36'(speed_q);
    i_tgt = '0;
    if (!enable) begin
      spd_next = 36'(speed_q)   - (36'(speed_q)   >>> 7);
      cur_next = 36'(current_q) - (36'(current_q) >>> 2);
    end else if (mode == MODE_SPEED) begin
      i_tgt    = (err >>> 2) + (36'(speed_q) >>> 4);
      spd_next = 36'(speed_q)   + (err >>> 8);
      cur_next = 36'(current_q) + ((i_tgt - 36'(current_q)) >>> 3);
    end else begin // MODE_CURRENT
      cur_next = 36'(current_q) + ((36'(signed'({current_sp, 16'h0})) - 36'(current_q)) >>> 3);
      spd_next = 36'(speed_q)   + (36'(current_q) >>> 6) - (36'(speed_q) >>> 9);
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      speed_q   <= '0;
      current_q <= '0;
    end else if (tick) begin
      speed_q   <= clamp(spd_next, SPEED_MAX);
      current_q <= clamp(cur_next, CURRENT_MAX);
    end
  end

  assign speed   = speed_q[31:16];
  assign current = current_q[31:16];

endmodule
