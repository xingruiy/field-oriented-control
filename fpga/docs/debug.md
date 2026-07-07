# Debug Notes & Subtle Findings

A running log of non-obvious behaviours uncovered while reading/reviewing
the RTL — the kind of thing that is correct in the code but easy to
misread from the comments, or invariants that hold only because of an
*upstream* guarantee. Companion documents: [`docs/pwm.md`](pwm.md) (gate
generation), [`docs/foc.md`](foc.md) (control math), [`docs/config.md`](config.md)
(operating point).

---

## 1. `pwm_gen` is single-buffered, not "double-buffered"

The header in [`rtl/pwm/pwm_gen.sv`](../rtl/pwm/pwm_gen.sv) used to call the
compare-value reload "double-buffered". That overstates the mechanism.

There is exactly **one** shadow register per phase, `ccr[i]`, reloaded at
the period boundary (`cnt == 0`, the trough):

```systemverilog
else if (!en || cnt == '0) begin
  ccr[0] <= ccr_of(duty_a); // etc.
```

A classic STM32-style double buffer has **two** registers — a *preload*
that software can write any time, and a *shadow/active* that copies from
preload at the update event. Here the "preload" is just the live
`duty_*` input; there is no internal register decoupling the input from the
sampling instant.

The glitch-free property still holds, but it now carries a **caller
obligation**: `duty_*` must be stable across the `cnt == 0` edge. That is
satisfied today because the duties are registered flops in
[`rtl/foc/foc_core.sv`](../rtl/foc/foc_core.sv) on the *same clock* (no CDC,
no combinational duty path), and the control pipeline lands them ~32 clk
after `cnt_peak` — hundreds of clocks before the next trough.

**Takeaway:** the comments now say "latched into a per-phase shadow
register"; do not reintroduce "double-buffered". If a future caller drives
`duty_*` combinationally or from another clock domain, the glitch-free
guarantee breaks.

---

## 2. The new `ccr` takes effect on the *up*-ramp, not at `update`

A natural misreading: "at `update` (trough) the high side is on — how does
it turn off with the new `ccr`?" The answer is that `update` is **not** the
turn-off moment. The high-side pulse `raw = cnt < ccr` is one continuous
pulse centred on the trough, with two edges:

- **turn-ON**  — on the *down*-ramp, when `cnt` falls below `ccr` (old value)
- **turn-OFF** — on the *up*-ramp, when `cnt` rises back to `ccr` (new value)

`ccr` reloads *between* those two edges, so the ON edge uses the old value
and the OFF edge uses the new one. The high side rides straight through the
trough and only switches off later, when the rising counter reaches the new
compare.

Trace (`ARR = 10`, `ccr_old = 4`, `ccr_new = 7`, dead-time ignored):

```
 cnt :  5  4  3  2  1  0 | 1  2  3  4  5  6  7  8
 ccr :  4  4  4  4  4  4 | 7  7  7  7  7  7  7  7   <- reloads at the cnt==0 edge
 raw :  0  0  1  1  1  1 | 1  1  1  1  1  1  0  0
                  ^on(old=4)              ^off(new=7)
```

Note `ccr` is a register: during the `cnt == 0` cycle it still holds the
old value; `ccr <= ccr_new` takes effect from `cnt == 1` onward.

**Consequence:** the single pulse straddling the boundary is **asymmetric
on that one changeover period** — descending half sized by `ccr_old`,
ascending half by `ccr_new`. Every subsequent period is symmetric with
`ccr_new`. This half-old/half-new blend is the normal, expected behaviour
of center-aligned PWM updated at underflow; it is still a single clean
pulse, no glitch.

---

## 3. Why reload `ccr` *only* at the trough

The PWM reacts to a new `ccr` either way — the latch is about reacting as
**one clean edge per period** instead of chopping the pulse mid-flight.

If `raw` compared against the live combinational duty, a mid-period change
could add or remove edges, because the high side is a level compare:

```
 cnt        :  2   3   4   5   6
 ccr (live) :  5   5   3   7   7      <- duty wiggling each cycle
 raw        :  1   1   0   1   1
                         ^off  ^ON AGAIN  -> two rising edges in one period
```

A second pulse in the same period means the bridge hard-switches off then
on again (re-arming dead time each time, the low side briefly conducting in
between), and the period's volt-seconds correspond to *no valid duty*.

Freezing `ccr` for the whole period guarantees `raw` is **monotonic** —
exactly one off→on→off, symmetric about the trough. That buys:

1. **Glitch-free** — at most one switching event per gate per period.
2. **Correct volt-second average** — the period average equals the
   commanded duty *only* if `ccr` is constant across the period.
3. **Deterministic latency** — exactly one update per PWM period, which is
   the "one PWM period of transport delay" the current-loop gains in
   `foc_core` are tuned around. Live updates would make the plant delay
   duty-dependent and jittery.
4. **Minimum switching** — every extra edge is extra loss and EMI.

The **trough** specifically (vs peak) is chosen because the high-side pulse
is centred there; reloading at underflow keeps the pulse symmetric and
matches the standard center-aligned convention.

---

## 4. `duty = 32767` would silently kill that phase's current sense

**Hazard (real at the `pwm_gen` level).** With `ARR = 625`:

```
ccr_of(32767) = (32767*625 + 16384) >> 15 = 625 = ARR
```

Then `raw = cnt < ccr` is low only at the single peak point `cnt == 625` —
a 1-cycle window, narrower than the dead time `DT = 20` cyc (200 ns). The
low-side gate logic needs `raw` low for `DT` cycles before `l[i]` asserts,
so **the low side never turns on near the peak**. But `cnt_peak` — the XADC
trigger for the low-side CSAs — fires exactly there. The CSA samples a phase
whose low FET is off (current freewheeling through a body diode), so that
phase reads garbage/offset, and **`pwm_gen` raises no flag.**

**Why it cannot actually happen.** [`rtl/foc/svpwm.sv`](../rtl/foc/svpwm.sv)
clamps each phase to `±REL_MAX = MAX_MOD_Q15/2 = 14254` about 0.5:

```
duty ∈ [16384 − 14254, 16384 + 14254] = [2130, 30638]  ≈ [0.065, 0.935]
```

The ceiling is **30638, never 32767**. At that max, `ccr_of(30638) ≈ 584`,
giving a low-side window of `2*(625−584)+1 = 83` cyc ≈ 830 ns — the
`(1−MAX_MOD)·Tsw/2 = 812 ns` budget that covers dead time + XADC aperture +
settling. The low FET is on and settled at the peak, so the sample is valid.

The only duty `foc_core` ever drives *without* going through `svpwm` is
`16384` (50/50/50) during reset / `cal_active` / `!en`. Open-loop
(`ol_mode`) still routes through `inv_park → svpwm`, so it is clamped too.
And whenever the clamp engages, `svpwm` asserts `sat`, which propagates to
`sat_any` telemetry — so it is not even silent at the system level.

**Takeaway:** the protection lives **upstream in `svpwm`**, not in
`pwm_gen`. `pwm_gen` itself has no guard — it trusts its caller. The only
way to hit the hazard is to bypass `svpwm` (a direct testbench drive, or a
future module wired straight to `pwm_gen.duty_*`). If defense-in-depth is
wanted, clamp `ccr` to `ARR − DT` (or assert `duty ≤ DUTY_MAX`) inside
`pwm_gen` to make the guarantee local instead of an upstream invariant.
Currently redundant, but it removes a sharp edge for future reuse.
