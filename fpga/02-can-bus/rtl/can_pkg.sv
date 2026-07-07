// ============================================================================
// can_pkg.sv
//
//  Shared parameters for the 02-can-bus project. Compiled first by the
//  simulate/build flows, then imported where needed via `import can_pkg::*;`.
//
//  CAN bit timing (Arty S7-50, 100 MHz oscillator):
//    baud    = F_CLK / (1 + PTS + PBS1 + PBS2) = 100e6 / 200 = 500 kbit/s
//    sample  = (1 + PTS + 1) / 200             = ~70 % of the bit
//    (can_level_bit samples on the FIRST cycle of PBS1, so PBS1 length does
//     not move the sample point - it only extends the pre-transmit phase)
//
//  Message map (all standard 11-bit IDs, all frames DLC = 4, big-endian):
//    0x101 PC -> FPGA command  : {opcode, arg_hi, arg_lo, reserved}
//    0x201 FPGA -> PC telemetry: {mux,    val_hi, val_lo, seq}   (see MUX_*)
//
//  Scaling: speed 1 LSB = 1 rpm, current 1 LSB = 1 mA, both signed 16-bit.
//
//  Keep this package scalar-only: xsim silently elaborates packed-struct
//  ARRAY localparams to zeros, so none are allowed anywhere in the project.
// ============================================================================

package can_pkg;

  parameter int unsigned F_CLK_HZ = 100_000_000;   // board oscillator

  // ---- CAN bit timing: 500 kbit/s, 80 % sample point ----
  parameter logic [15:0] CAN_PTS  = 16'd139;
  parameter logic [15:0] CAN_PBS1 = 16'd20;
  parameter logic [15:0] CAN_PBS2 = 16'd40;

  // ---- message IDs (standard 11-bit) ----
  parameter logic [10:0] ID_CMD   = 11'h101;       // PC -> FPGA commands
  parameter logic [10:0] ID_TELEM = 11'h201;       // FPGA -> PC telemetry

  // ---- command opcodes (byte 0 of an ID_CMD frame) ----
  parameter logic [7:0] OP_SET_ENABLE  = 8'h01;    // byte1: 0 = off, nonzero = on
  parameter logic [7:0] OP_SET_MODE    = 8'h02;    // byte1 bit0: 0 = speed loop, 1 = current loop
  parameter logic [7:0] OP_SET_SPEED   = 8'h03;    // bytes1-2: int16 BE, rpm
  parameter logic [7:0] OP_SET_CURRENT = 8'h04;    // bytes1-2: int16 BE, mA

  // ---- telemetry mux codes (byte 0 of an ID_TELEM frame) ----
  parameter logic [7:0] MUX_SPEED   = 8'h10;       // int16 BE rpm  + seq
  parameter logic [7:0] MUX_CURRENT = 8'h11;       // int16 BE mA   + seq
  parameter logic [7:0] MUX_STATUS  = 8'h12;       // flags + uint16 BE heartbeat

  // ---- control modes ----
  parameter logic MODE_SPEED   = 1'b0;
  parameter logic MODE_CURRENT = 1'b1;

  // ---- board-default scheduling ----
  parameter int unsigned TICK_DIV_DEFAULT     = 100_000; // motor-model tick: 1 kHz
  parameter int unsigned TELEM_TICKS_DEFAULT  = 10;      // telemetry slot every 10 ticks = 100 Hz
  parameter int unsigned STATUS_EVERY_DEFAULT = 10;      // status frame every 10th slot  = 10 Hz

  // ---- model limits ----
  parameter int SPEED_SAT   = 30_000;              // |rpm| clamp (int16 headroom)
  parameter int CURRENT_SAT = 30_000;              // |mA|  clamp

endpackage : can_pkg
