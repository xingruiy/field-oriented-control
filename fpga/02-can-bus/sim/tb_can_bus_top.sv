`timescale 1ns/1ps

module tb_can_bus_top;
  import can_pkg::*;

  logic clk100 = 1'b0;
  always #5 clk100 = ~clk100;

  logic ck_rstn = 1'b0;
  logic fpga_tx;
  logic fpga_rx;
  logic fpga_rs;
  logic [3:0] led;

  // A second FPGA-CAN instance acts as the CANable/PC endpoint. CAN is wired
  // dominant-low: the shared bus is the logical AND of all TX pins.
  logic pc_tx;
  logic bus;
  assign bus = fpga_tx & pc_tx;
  assign fpga_rx = bus;

  can_bus_top #(
    .TICK_DIV        (8),
    .TELEM_TICKS     (4),
    .STATUS_EVERY    (2),
    .CAN_TIMING_PTS  (16'd4),
    .CAN_TIMING_PBS1 (16'd2),
    .CAN_TIMING_PBS2 (16'd3)
  ) dut (
    .clk100     (clk100),
    .ck_rstn    (ck_rstn),
    .can_rx_ja2 (fpga_rx),
    .can_tx_ja1 (fpga_tx),
    .can_rs_ja3 (fpga_rs),
    .led        (led)
  );

  logic        pc_tx_valid;
  logic        pc_tx_ready;
  logic [31:0] pc_tx_data;
  logic        pc_rx_valid;
  logic        pc_rx_last;
  logic [7:0]  pc_rx_data;
  logic [28:0] pc_rx_id;
  logic        pc_rx_ide;

  can_top #(
    .LOCAL_ID           (ID_CMD),
    .RX_ID_SHORT_FILTER (ID_TELEM),
    .RX_ID_SHORT_MASK   (11'h7ff),
    .default_c_PTS      (16'd4),
    .default_c_PBS1     (16'd2),
    .default_c_PBS2     (16'd3)
  ) pc (
    .rstn     (ck_rstn),
    .clk      (clk100),
    .can_rx   (bus),
    .can_tx   (pc_tx),
    .tx_valid (pc_tx_valid),
    .tx_ready (pc_tx_ready),
    .tx_data  (pc_tx_data),
    .rx_valid (pc_rx_valid),
    .rx_last  (pc_rx_last),
    .rx_data  (pc_rx_data),
    .rx_id    (pc_rx_id),
    .rx_ide   (pc_rx_ide)
  );

  task automatic send_cmd(input logic [31:0] word);
    @(negedge clk100);
    while (!pc_tx_ready) @(negedge clk100);
    pc_tx_data = word;
    pc_tx_valid = 1'b1;
    @(negedge clk100);
    pc_tx_valid = 1'b0;
  endtask

  logic seen_speed, seen_current, seen_status;
  logic first_byte;  // only byte 0 of a frame carries the mux code

  always_ff @(posedge clk100 or negedge ck_rstn) begin
    if (!ck_rstn) begin
      seen_speed   <= 1'b0;
      seen_current <= 1'b0;
      seen_status  <= 1'b0;
      first_byte   <= 1'b1;
    end else if (pc_rx_valid) begin
      first_byte <= pc_rx_last;
      if (first_byte && !pc_rx_ide && pc_rx_id[10:0] == ID_TELEM) begin
        if (pc_rx_data == MUX_SPEED)   seen_speed   <= 1'b1;
        if (pc_rx_data == MUX_CURRENT) seen_current <= 1'b1;
        if (pc_rx_data == MUX_STATUS)  seen_status  <= 1'b1;
      end
    end
  end

  initial begin
    pc_tx_valid = 1'b0;
    pc_tx_data  = '0;

    repeat (20) @(negedge clk100);
    ck_rstn = 1'b1;
    repeat (200) @(negedge clk100);

    send_cmd({OP_SET_ENABLE, 8'h01, 8'h00, 8'h00});
    repeat (20000) @(negedge clk100);
    assert(led[0]) else $fatal(1, "enable command did not reach the FPGA app");

    send_cmd({OP_SET_SPEED, 8'h05, 8'hDC, 8'h00}); // 1500 rpm
    repeat (120000) @(negedge clk100);

    assert(fpga_rs == 1'b0) else $fatal(1, "HVD230 RS should be driven low");
    assert(seen_speed) else $fatal(1, "PC endpoint did not see speed telemetry");
    assert(seen_current) else $fatal(1, "PC endpoint did not see current telemetry");
    assert(seen_status) else $fatal(1, "PC endpoint did not see status telemetry");

    $display("TB_PASS: tb_can_bus_top");
    $finish;
  end
endmodule
