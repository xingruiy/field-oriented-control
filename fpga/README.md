# FPGA Projects

This directory contains FPGA projects organized as numbered, self-contained
folders. Each project owns its RTL, simulation benches, constraints, Vivado TCL,
scripts, and project-specific documentation.

## Projects

- `00-full-foc/` - Complete field-oriented current-loop control design for the
  Arty S7-50, DRV8316REVM gate driver, and hall-sensor BLDC motor.
- `01-pwm-gen/` - Small Arty S7-50 PWM generator demo with switch-controlled
  duty cycle, xsim testbenches, Vivado build flow, and oscilloscope bring-up
  notes.
- `02-can-bus/` - CAN 2.0A motor-command demo for the Arty S7-50 using an
  HVD230/SN65HVD230-style transceiver and CANable host tooling.

## Convention

Use a two-digit numeric prefix for each top-level FPGA project so the folders
sort in learning or integration order:

```text
NN-short-name/
```

Run project commands from inside the individual project directory. For example:

```sh
cd 01-pwm-gen
make regress
make build
```

See each project's own `README.md` for hardware wiring, simulation, build, and
programming details.
