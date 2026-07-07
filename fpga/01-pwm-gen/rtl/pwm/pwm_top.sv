// ============================================================================
// pwm_top.sv
//
//  Board top level for the Arty S7-50 PWM demo.
//
//    clk100 ----IBUF/BUFG---> clk
//    ck_rstn --reset sync---> rst_n
//    sw[3:0] ---2FF sync----> sw_s --x PWM_PERIOD/16--> duty[10:0] -> pwm_core -> pwm
//                                                                    |---> pwm_ja  (Pmod JA pin 1)
//                                                                    |---> pwm_led (LED0)
//
//  Duty mapping: duty = sw_s * PWM_PERIOD / 16, giving 16 steps of ~6.25 %:
//    sw=0000 -> 0   ->  0.00 %      sw=1000 -> 625  -> 50.00 %
//    sw=0001 -> 78  ->  6.24 %      sw=1111 -> 1171 -> 93.68 %
// ============================================================================

module pwm_top
  import pwm_pkg::*;
(
  input  logic            clk100,   // 100 MHz oscillator (pin R2)
  input  logic            ck_rstn,  // active-low reset button (pin C18)
  input  logic [SW_W-1:0] sw,       // slide switches (duty select)
  output logic            pwm_ja,   // PWM out to Pmod JA pin 1 (scope probe)
  output logic            pwm_led   // PWM out mirrored to LED0
);

  logic clk;
  logic rst_n;

  clk_rst_gen u_clk (
    .clk_in  (clk100),
    .rstn_in (ck_rstn),
    .clk     (clk),
    .rst_n   (rst_n)
  );

  // synchronize the asynchronous slide switches into the clk domain
  logic [SW_W-1:0] sw_s;
  sync2 #(.W(SW_W)) u_sw_sync (
    .clk   (clk),
    .rst_n (rst_n),
    .d     (sw),
    .q     (sw_s)
  );

  // map 4-bit switch value to a duty count: duty = sw_s * PWM_PERIOD / 16
  // (division by 16 == >> SW_W; the intermediate product fits in SW_W+PWM_W bits)
  logic [PWM_W-1:0] duty;
  assign duty = PWM_W'((sw_s * PWM_PERIOD) >> SW_W);

  logic pwm;
  pwm_core #(.WIDTH(PWM_W)) u_pwm (
    .clk   (clk),
    .rst_n (rst_n),
    .duty  (duty),
    .pwm   (pwm),
    .cnt   ()
  );

  assign pwm_ja  = pwm;
  assign pwm_led = pwm;

endmodule
