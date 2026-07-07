# PWM design notes

## Operating point

| quantity        | value                       | source |
|-----------------|-----------------------------|--------|
| clock           | 100 MHz (10 ns)             | `clk100` pin R2 |
| counter width   | 11 bit (`PWM_W`)            | `pwm_pkg.sv` |
| period          | `PWM_PERIOD = 1250` counts  | `pwm_pkg.sv` |
| PWM frequency   | 100 MHz / 1250 = 80.000 kHz | derived |
| period (time)   | 12.5 µs                     | derived |
| duty resolution | 1250 internal / 16 via switches | `pwm_core` / `pwm_top` |

## How the engine works (`pwm_core.sv`)

A free-running counter `cnt` runs `0 → TOP → 0` (period `TOP+1`). The output is
high while `cnt < duty`, so `duty` is literally the number of high clock cycles
per period:

```
duty = 0       -> output always low   (0 %)
duty = N       -> N / (TOP+1) high     (edge-aligned, rising edge at cnt==0)
duty = TOP+1   -> output always high   (100 %, needs WIDTH+1 bits to express)
```

The compare result is registered, so output edges are clean (no combinational
glitches) and align to clock edges.

## Switch → duty mapping (`pwm_top.sv`)

The 4 slide switches give 16 steps. We scale the switch value by the period
(division by 16 == shift right by `SW_W`):

```
duty = sw_s * PWM_PERIOD / 16 = (sw_s * 1250) >> 4
```

So `sw_s` 0..15 maps to duty 0, 78, 156, …, 1171 (0 % … 93.7 % in ~6.25 % steps;
integer division truncates each step). The slide switches are asynchronous to
`clk`, so they pass through a 2-FF synchronizer (`sync2`) before use.

The core itself is full `PWM_W`-bit (0..PWM_PERIOD); the switch mapping is only
the demo's input method. Driving `duty` from a counter (breathing) or UART would
give finer control without touching `pwm_core`.

## Clock / reset (`clk_rst_gen.sv`)

The 100 MHz oscillator enters through `IBUF → BUFG` (no MMCM/PLL). The board
reset button `ck_rstn` (active-low) feeds a reset synchronizer: asynchronous
assert, synchronous 2-FF deassert, producing the design-wide active-low `rst_n`.

## Simulation

`tb_pwm_core` programs several duty values and, over exactly one period, counts
the high cycles (must equal `duty`) and checks the rising-edge-to-rising-edge
period equals 1250. `tb_sync2` checks the 2-clock synchronizer latency. Each
prints a single `TB_PASS:` / `TB_FAIL:` banner consumed by `scripts/regress.sh`.

## Oscilloscope expectations

- Signal: 3.3 V LVCMOS square wave on **Pmod JA pin 1** (`L17`).
- Frequency: **80.0 kHz**, period **12.5 µs** — independent of switch setting.
- Duty: tracks `SW[3:0]` per the table in the README.
- Suggested scope setup: 1 V/div, 5 µs/div, rising-edge trigger ~1.6 V.
- Holding the reset button forces the output low.
