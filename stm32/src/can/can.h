#pragma once

/* CAN control & telemetry over FDCAN1 (PD0=RX, PD1=TX → SN65HVD230 → USB-CAN).
 *
 * Classic CAN, 500 kbps, 11-bit standard IDs. The FDCAN peripheral (bit timing,
 * message-RAM/FIFO allocation, pins, clock) is configured in CubeMX; this module
 * only installs the RX acceptance filter, starts the peripheral, drains received
 * command frames, and broadcasts telemetry. See settings.h for the protocol IDs.
 *
 * Command frames (PC → MCU) reuse the same FOC/fault accessors as the CLI:
 *   0x100 b0=opcode  (0 dis, 1 en, 2 clear-fault, 3 speed-off, 4 cal, 5 hcal)
 *   0x101 int16 mA   -> Iq reference
 *   0x102 int16 RPM  -> speed reference
 *   0x103 int32 cdeg -> TMAG arm absolute position target
 *
 * Telemetry (MCU → PC) @100 Hz:
 *   0x200 flags, hall, speed_rpm, iq_ref_mA, iq_mA
 *   0x201 Ia_mA, Ib_mA, Ic_mA, theta_e_cdeg
 *   0x202 cal result (one-shot after opcode 4/5): type, ok, off_a/b/c
 *   0x204 arm state/flags, target/current deg*10, output rpm*10
 *
 * can_init() — configure filter + start FDCAN1. Call once after cli_init().
 * can_poll() — call from main loop; non-blocking RX dispatch + paced telemetry TX.
 */

void can_init(void);
void can_poll(void);

/* Disarm the CAN dead-man (CAN_CMD_TIMEOUT_MS): a local CLI iq/spd command
 * takes ownership of the motion setpoint away from the CAN host. */
void can_motion_release(void);
