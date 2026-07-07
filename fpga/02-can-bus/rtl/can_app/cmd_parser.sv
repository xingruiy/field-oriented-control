// ============================================================================
// cmd_parser.sv
//
//  Decodes motor command frames arriving on can_top's streaming RX interface
//  (one byte per clock, rx_last on the final byte, no backpressure - so this
//  is a pure per-clock register capture).
//
//  can_top's short-ID filter already restricts the stream to ID_CMD frames;
//  extended-ID frames are additionally rejected here via rx_ide.
//
//  Frame format (DLC = 4, big-endian): {opcode, arg_hi, arg_lo, reserved}.
//  Frames shorter than 4 bytes or with an unknown opcode are ignored.
//  Longer frames are tolerated (bytes past the 4th are ignored).
//  Accepted fields commit atomically one clock after rx_last.
// ============================================================================

module cmd_parser
  import can_pkg::*;
(
  input  logic               clk,
  input  logic               rst_n,

  // from can_top RX stream
  input  logic               rx_valid,
  input  logic               rx_last,
  input  logic [7:0]         rx_data,
  input  logic               rx_ide,        // 1 = extended frame -> ignore

  // registered command state
  output logic               motor_enable,
  output logic               ctrl_mode,     // MODE_SPEED / MODE_CURRENT
  output logic signed [15:0] speed_sp,      // rpm
  output logic signed [15:0] current_sp,    // mA
  output logic               cmd_pulse      // 1-clk strobe per accepted command
);

  logic [31:0] cap;       // first 4 payload bytes, big-endian
  logic [2:0]  idx;       // bytes captured so far (saturates at 5)
  logic        commit;    // frame ended last cycle -> decode now
  logic        frame_ok;  // >= 4 bytes and standard ID

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cap      <= '0;
      idx      <= '0;
      commit   <= 1'b0;
      frame_ok <= 1'b0;
    end else begin
      commit <= 1'b0;
      if (rx_valid) begin
        if (idx < 3'd4) cap <= {cap[23:0], rx_data};
        if (idx < 3'd5) idx <= idx + 3'd1;
        if (rx_last) begin
          commit   <= 1'b1;
          frame_ok <= !rx_ide && (idx >= 3'd3);  // this byte is the 4th or later
        end
      end
      if (commit) idx <= '0;                     // ready for the next frame
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      motor_enable <= 1'b0;
      ctrl_mode    <= MODE_SPEED;
      speed_sp     <= '0;
      current_sp   <= '0;
      cmd_pulse    <= 1'b0;
    end else begin
      cmd_pulse <= 1'b0;
      if (commit && frame_ok) begin
        case (cap[31:24])
          OP_SET_ENABLE:  begin motor_enable <= (cap[23:16] != 8'd0); cmd_pulse <= 1'b1; end
          OP_SET_MODE:    begin ctrl_mode    <= cap[16];              cmd_pulse <= 1'b1; end
          OP_SET_SPEED:   begin speed_sp     <= signed'(cap[23:8]);   cmd_pulse <= 1'b1; end
          OP_SET_CURRENT: begin current_sp   <= signed'(cap[23:8]);   cmd_pulse <= 1'b1; end
          default: ;                                                  // unknown -> ignore
        endcase
      end
    end
  end

endmodule
