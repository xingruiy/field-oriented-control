# 02-can-bus

CAN motor-command demo for the **Arty S7-50** (`xc7s50csga324-1`) using an
HVD230/SN65HVD230-style transceiver on Pmod JA and a CANable on the PC.

The low-level CAN controller is adapted from `../FPGA-CAN` and kept as the
project's battle-tested bit/packet engine. The new RTL wraps it with a simple
motor command protocol, a first-order motor model, telemetry scheduling, xsim
testbenches, Arty S7 constraints, and a Python CANable visualization script.

## What It Does

- Runs CAN 2.0A standard IDs at **500 kbit/s** from the 100 MHz Arty clock.
- Accepts 4-byte PC command frames on standard ID `0x101`.
- Sends 4-byte FPGA telemetry frames on standard ID `0x201`.
- Commands include enable/disable, speed-loop mode, current-loop mode, speed
  setpoint, and current setpoint.
- Telemetry reports modeled speed, modeled current, enable/mode status, and a
  heartbeat. This demo does not drive real motor power hardware.

## Layout

```text
rtl/
  can_pkg.sv                  IDs, opcodes, timing, scaling
  common/clk_rst_gen.sv       Arty clock/reset
  common/sync2.sv             async input synchronizer
  can_core/*.sv               adapted FPGA-CAN controller
  can_app/can_bus_top.sv      Arty S7 top level
  can_app/cmd_parser.sv       command frame decoder
  can_app/motor_model.sv      safe demo motor plant
  can_app/telemetry_scheduler.sv
sim/
  tb_*.sv                     self-checking xsim benches
scripts/
  simulate.sh, regress.sh     xsim wrappers
  can_demo.py                 CANable telemetry plot/text demo
tcl/
  build.tcl, program.tcl      Vivado non-project flow
xdc/
  arty_s7.xdc                 Arty S7/HVD230 pin constraints
docs/
  can_bus.md                  protocol and verification notes
```

## Hardware Wiring

Use one HVD230 transceiver module between the Arty S7 and the two-wire CAN bus:

| Arty S7 | HVD230 | Direction |
|---------|--------|-----------|
| JA1     | D/TXD  | FPGA to transceiver |
| JA2     | R/RXD  | transceiver to FPGA |
| JA3     | RS     | FPGA drives low for high-speed mode |
| JA 3V3  | VCC    | power |
| JA GND  | GND    | ground |

Connect HVD230 `CANH`/`CANL` to CANable `CANH`/`CANL`. Use 120 ohm termination
across CANH/CANL if your CANable/transceiver boards do not already provide it.

## Build And Simulate

First put Vivado on PATH:

```sh
source ~/amd/2025.2/Vivado/settings64.sh
```

Run from this directory:

```sh
make              # show targets
make sim          # run tb_can_bus_top
make regress      # run all xsim testbenches
make gui TOP=tb_can_bus_top
make build        # synth + implement -> build/impl/can_bus_top.bit
make program      # program the Arty S7-50 over USB-JTAG
```

## PC Demo With CANable

On Linux with CANable in candleLight/SocketCAN firmware mode:

```sh
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
python3 -m pip install python-can matplotlib
python3 scripts/can_demo.py --channel can0
```

Use text mode when running over SSH:

```sh
python3 scripts/can_demo.py --channel can0 --text
```

The script cycles speed and current commands, then plots telemetry returned by
the FPGA. Stop it with Ctrl-C; it sends a disable command before exiting.

See [docs/can_bus.md](docs/can_bus.md) for the frame map and manual `cansend`
checks.
