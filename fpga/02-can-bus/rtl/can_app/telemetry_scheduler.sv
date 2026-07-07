// ============================================================================
// telemetry_scheduler.sv
//
//  Periodically pushes telemetry frames into can_top's TX FIFO (each 32-bit
//  word becomes one DLC-4 CAN frame with ID_TELEM). Every TELEM_TICKS ticks
//  a "slot" fires: SPEED and CURRENT frames are queued (values latched
//  together at slot start so the pair is coherent), plus a STATUS frame on
//  every STATUS_EVERY-th slot.
//
//  Words are big-endian, mux code in the top byte:
//    SPEED   {MUX_SPEED,   rpm[15:8], rpm[7:0], seq}
//    CURRENT {MUX_CURRENT, mA[15:8],  mA[7:0],  seq}
//    STATUS  {MUX_STATUS,  {6'b0, mode, enable}, heartbeat[15:8], heartbeat[7:0]}
//
//  seq increments per slot (shared by the SPEED/CURRENT pair); heartbeat
//  increments per STATUS frame. The tx_valid/tx_ready handshake is honored,
//  so a full FIFO stalls the FSM; a slot firing while a previous slot is
//  still stalled is dropped (visible to the PC as a seq gap).
// ============================================================================

module telemetry_scheduler
  import can_pkg::*;
#(
  parameter int unsigned TELEM_TICKS  = TELEM_TICKS_DEFAULT,   // ticks per slot
  parameter int unsigned STATUS_EVERY = STATUS_EVERY_DEFAULT   // slots per status
)(
  input  logic               clk,
  input  logic               rst_n,

  input  logic               tick,        // from motor_model
  input  logic signed [15:0] speed,       // rpm
  input  logic signed [15:0] current,     // mA
  input  logic               enable,
  input  logic               mode,

  // to can_top TX FIFO
  output logic               tx_valid,
  input  logic               tx_ready,
  output logic [31:0]        tx_data,
  output logic               tx_pulse     // 1-clk strobe per accepted word
);

  typedef enum logic [1:0] {IDLE, PUSH_SPEED, PUSH_CURRENT, PUSH_STATUS} state_e;
  state_e state;

  logic [$clog2(TELEM_TICKS)-1:0]  slot_cnt;
  logic [$clog2(STATUS_EVERY)-1:0] status_cnt;
  logic                            slot_fire;

  logic signed [15:0] speed_l, current_l;  // latched at slot start
  logic               status_due;
  logic [7:0]         seq;
  logic [15:0]        heartbeat;

  // ---- slot timer -------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      slot_cnt  <= '0;
      slot_fire <= 1'b0;
    end else begin
      slot_fire <= 1'b0;
      if (tick) begin
        if (slot_cnt == TELEM_TICKS - 1) begin
          slot_cnt  <= '0;
          slot_fire <= 1'b1;
        end else begin
          slot_cnt <= slot_cnt + 1'b1;
        end
      end
    end
  end

  // ---- push FSM -----------------------------------------------------------------
  wire accepted = tx_valid & tx_ready;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= IDLE;
      status_cnt <= '0;
      speed_l    <= '0;
      current_l  <= '0;
      status_due <= 1'b0;
      seq        <= '0;
      heartbeat  <= '0;
      tx_pulse   <= 1'b0;
    end else begin
      tx_pulse <= 1'b0;
      case (state)
        IDLE: begin
          if (slot_fire) begin
            speed_l    <= speed;
            current_l  <= current;
            status_due <= (status_cnt == '0);
            if (status_cnt == STATUS_EVERY - 1) status_cnt <= '0;
            else                                status_cnt <= status_cnt + 1'b1;
            state <= PUSH_SPEED;
          end
        end
        PUSH_SPEED: if (accepted) begin
          tx_pulse <= 1'b1;
          state    <= PUSH_CURRENT;
        end
        PUSH_CURRENT: if (accepted) begin
          tx_pulse <= 1'b1;
          seq      <= seq + 1'b1;
          state    <= status_due ? PUSH_STATUS : IDLE;
        end
        PUSH_STATUS: if (accepted) begin
          tx_pulse  <= 1'b1;
          heartbeat <= heartbeat + 1'b1;
          state     <= IDLE;
        end
        default: state <= IDLE;
      endcase
    end
  end

  always_comb begin
    tx_valid = 1'b1;
    unique case (state)
      PUSH_SPEED:   tx_data = {MUX_SPEED,   speed_l[15:8],   speed_l[7:0],   seq};
      PUSH_CURRENT: tx_data = {MUX_CURRENT, current_l[15:8], current_l[7:0], seq};
      PUSH_STATUS:  tx_data = {MUX_STATUS,  {6'b0, mode, enable}, heartbeat[15:8], heartbeat[7:0]};
      default: begin
        tx_valid = 1'b0;
        tx_data  = '0;
      end
    endcase
  end

endmodule
