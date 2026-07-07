// ============================================================================
// can_bus_top.sv
//
//  Arty S7-50 board top for a small CAN motor-command demo.
//
//  Hardware:
//    Arty S7 Pmod JA1 -> HVD230 D/TXD  (FPGA drives CAN transmit bit)
//    Arty S7 Pmod JA2 <- HVD230 R/RXD  (FPGA samples CAN receive bit)
//    Arty S7 Pmod JA3 -> HVD230 RS     (driven low for high-speed mode)
//
//  The FPGA-CAN controller carries one 4-byte command frame from the PC and
//  returns 4-byte telemetry frames. The app layer is intentionally a motor
//  model, not a real inverter controller, so it is safe to run with only the
//  CAN transceiver connected.
// ============================================================================

module can_bus_top
  import can_pkg::*;
#(
  parameter int unsigned TICK_DIV     = TICK_DIV_DEFAULT,
  parameter int unsigned TELEM_TICKS  = TELEM_TICKS_DEFAULT,
  parameter int unsigned STATUS_EVERY = STATUS_EVERY_DEFAULT,
  parameter logic [15:0] CAN_TIMING_PTS  = CAN_PTS,
  parameter logic [15:0] CAN_TIMING_PBS1 = CAN_PBS1,
  parameter logic [15:0] CAN_TIMING_PBS2 = CAN_PBS2
)(
  input  logic       clk100,
  input  logic       ck_rstn,

  input  logic       can_rx_ja2,
  output logic       can_tx_ja1,
  output logic       can_rs_ja3,

  output logic [3:0] led
);

  logic clk;
  logic rst_n;

  clk_rst_gen u_clk_rst (
    .clk_in  (clk100),
    .rstn_in (ck_rstn),
    .clk     (clk),
    .rst_n   (rst_n)
  );

  // HVD230 RS low selects normal high-speed operation.
  assign can_rs_ja3 = 1'b0;

  logic can_rx_sync;
  sync2 #(.W(1)) u_can_rx_sync (
    .clk   (clk),
    .rst_n (rst_n),
    .d     (can_rx_ja2),
    .q     (can_rx_sync)
  );

  logic        rx_valid;
  logic        rx_last;
  logic [7:0]  rx_data;
  logic [28:0] rx_id;
  logic        rx_ide;

  logic        tx_valid;
  logic        tx_ready;
  logic [31:0] tx_data;

  can_top #(
    .LOCAL_ID           (ID_TELEM),
    .RX_ID_SHORT_FILTER (ID_CMD),
    .RX_ID_SHORT_MASK   (11'h7ff),
    .default_c_PTS      (CAN_TIMING_PTS),
    .default_c_PBS1     (CAN_TIMING_PBS1),
    .default_c_PBS2     (CAN_TIMING_PBS2)
  ) u_can (
    .rstn     (rst_n),
    .clk      (clk),
    .can_rx   (can_rx_sync),
    .can_tx   (can_tx_ja1),
    .tx_valid (tx_valid),
    .tx_ready (tx_ready),
    .tx_data  (tx_data),
    .rx_valid (rx_valid),
    .rx_last  (rx_last),
    .rx_data  (rx_data),
    .rx_id    (rx_id),
    .rx_ide   (rx_ide)
  );

  logic               motor_enable;
  logic               ctrl_mode;
  logic signed [15:0] speed_sp;
  logic signed [15:0] current_sp;
  logic               cmd_pulse;

  cmd_parser u_cmd_parser (
    .clk          (clk),
    .rst_n        (rst_n),
    .rx_valid     (rx_valid && (rx_id[10:0] == ID_CMD)),
    .rx_last      (rx_last),
    .rx_data      (rx_data),
    .rx_ide       (rx_ide),
    .motor_enable (motor_enable),
    .ctrl_mode    (ctrl_mode),
    .speed_sp     (speed_sp),
    .current_sp   (current_sp),
    .cmd_pulse    (cmd_pulse)
  );

  logic signed [15:0] speed;
  logic signed [15:0] current;
  logic               tick;

  motor_model #(
    .TICK_DIV (TICK_DIV)
  ) u_motor_model (
    .clk        (clk),
    .rst_n      (rst_n),
    .enable     (motor_enable),
    .mode       (ctrl_mode),
    .speed_sp   (speed_sp),
    .current_sp (current_sp),
    .speed      (speed),
    .current    (current),
    .tick       (tick)
  );

  logic tx_pulse;

  telemetry_scheduler #(
    .TELEM_TICKS  (TELEM_TICKS),
    .STATUS_EVERY (STATUS_EVERY)
  ) u_telemetry_scheduler (
    .clk       (clk),
    .rst_n     (rst_n),
    .tick      (tick),
    .speed     (speed),
    .current   (current),
    .enable    (motor_enable),
    .mode      (ctrl_mode),
    .tx_valid  (tx_valid),
    .tx_ready  (tx_ready),
    .tx_data   (tx_data),
    .tx_pulse  (tx_pulse)
  );

  logic [23:0] hb_cnt;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) hb_cnt <= '0;
    else        hb_cnt <= hb_cnt + 1'b1;
  end

  logic cmd_seen;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)      cmd_seen <= 1'b0;
    else if (cmd_pulse) cmd_seen <= ~cmd_seen;
  end

  assign led[0] = motor_enable;
  assign led[1] = (ctrl_mode == MODE_CURRENT);
  assign led[2] = cmd_seen;
  assign led[3] = hb_cnt[23] ^ tx_pulse;

endmodule
