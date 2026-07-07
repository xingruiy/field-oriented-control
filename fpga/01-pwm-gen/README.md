# 01-pwm-gen

A small, self-contained PWM generator for the **Arty S7-50** (`xc7s50csga324-1`).
Built to be simulated in Vivado `xsim`, synthesized + flashed over USB-JTAG, and
verified on an **oscilloscope**. Structure and conventions mirror the sibling
`../fpga` project.

## What it does

- Edge-aligned PWM running from the 100 MHz oscillator:
  `F_PWM = 100 MHz / 1250 = 80.000 kHz` (period **12.5 µs**).
- Duty cycle selected live by the 4 slide switches `SW[3:0]` (16 steps,
  `duty = SW × PWM_PERIOD / 16`).
- The PWM signal is driven to **Pmod JA pin 1** (scope probe) and mirrored to
  **LED0** (visual brightness ≈ duty).

| `SW[3:0]` | duty count | duty cycle |
|-----------|-----------:|-----------:|
| `0000`    |    0       |   0.00 %   |
| `0001`    |   78       |   6.24 %   |
| `0100`    |  312       |  24.96 %   |
| `1000`    |  625       |  50.00 %   |
| `1100`    |  937       |  74.96 %   |
| `1111`    | 1171       |  93.68 %   |

## Layout

```
rtl/
  pwm_pkg.sv            shared parameters (F_CLK_HZ, F_PWM_HZ, PWM_PERIOD, PWM_W, SW_W)
  common/clk_rst_gen.sv IBUF->BUFG clock + async-assert/sync-deassert reset
  pwm/sync2.sv          2-FF input synchronizer (switch CDC)
  pwm/pwm_core.sv       free-running counter + compare (the PWM engine)
  pwm/pwm_top.sv        board top: clk/reset + switch map + outputs
sim/
  tb_pwm_core.sv        self-checking duty/period testbench
  tb_sync2.sv           self-checking synchronizer testbench
scripts/                simulate.sh (one TB), regress.sh (all TBs)
tcl/                    build.tcl (synth->bitstream), program.tcl (JTAG)
xdc/arty_s7.xdc         pin constraints
docs/pwm.md             design notes + scope expectations
```

## Usage

First put Vivado on PATH:

```sh
source ~/amd/2025.2/Vivado/settings64.sh
```

```sh
make            # show targets
make sim        # run tb_pwm_core (override TOP=tb_sync2, etc.)
make regress    # run all testbenches, one verdict line each
make gui TOP=tb_pwm_core   # open the xsim waveform GUI
make build      # synth + implement -> build/impl/pwm_top.bit
make program    # flash the Arty S7-50 over USB-JTAG
```

## Verifying on an oscilloscope

1. `make build && make program`.
2. Probe **Pmod JA pin 1** (top row, pin 1) with the ground clip on a **JA GND
   pin** (pin 5 or 11).
3. Expect a 3.3 V square wave at **80.0 kHz** (period **12.5 µs**).
4. Change `SW[3:0]` and watch the duty cycle track the table above
   (`1000` ≈ 50 %, `1111` ≈ 93.7 %). LED0 brightness tracks duty as a quick
   visual sanity check.
5. Press the reset button (`ck_rstn`, active-low) to confirm the output drops low
   while held.

See `docs/pwm.md` for the math and design notes.
