# CAN Bus Design Notes

## Bit Timing

The Arty S7 oscillator is 100 MHz. The default timing in `rtl/can_pkg.sv` uses:

```text
bit time = 1 + PTS + PBS1 + PBS2 = 1 + 139 + 20 + 40 = 200 clocks
baud     = 100 MHz / 200 = 500 kbit/s
sample   = (1 + 139 + 1) / 200 = ~70 %
```

Note: `can_level_bit` samples RX on the *first* clock of PBS1, so the sample
point is right after PTS ends (~70 %), not at the PBS1/PBS2 boundary that the
upstream FPGA-CAN README formula suggests. CAN tolerates sample points of
roughly 60-90 %, so this is fine at 500 kbit/s.

## Frame Map

All frames use CAN 2.0A standard 11-bit identifiers and DLC 4.

### PC to FPGA: `0x101`

Payload format: `{opcode, arg_hi, arg_lo, reserved}`.

| Opcode | Meaning | Argument |
|--------|---------|----------|
| `0x01` | enable/disable | byte 1: `0` disables, nonzero enables |
| `0x02` | set mode | byte 1 bit 0: `0` speed loop, `1` current loop |
| `0x03` | set speed | bytes 1-2: signed int16 rpm, big-endian |
| `0x04` | set current | bytes 1-2: signed int16 mA, big-endian |

### FPGA to PC: `0x201`

Payload format: `{mux, val_hi, val_lo, seq_or_lsb}`.

| Mux | Meaning | Payload |
|-----|---------|---------|
| `0x10` | speed | signed int16 rpm in bytes 1-2, byte 3 sequence |
| `0x11` | current | signed int16 mA in bytes 1-2, byte 3 sequence |
| `0x12` | status | byte 1 flags `{mode, enable}`, bytes 2-3 heartbeat |

## Manual Checks

With CANable up as `can0`, install `can-utils` and use:

```sh
candump can0
cansend can0 101#01010000   # enable
cansend can0 101#02000000   # speed mode
cansend can0 101#0305DC00   # speed setpoint = 1500 rpm
cansend can0 101#02010000   # current mode
cansend can0 101#04032000   # current setpoint = 800 mA
cansend can0 101#01000000   # disable
```

Expected telemetry examples:

```text
can0  201   [4]  10 xx xx nn   speed
can0  201   [4]  11 xx xx nn   current
can0  201   [4]  12 00/01/02/03 hh hh   status
```

LEDs:

| LED | Meaning |
|-----|---------|
| LED0 | motor model enabled |
| LED1 | current-loop mode selected |
| LED2 | toggles on every accepted command |
| LED3 | heartbeat/telemetry activity |
