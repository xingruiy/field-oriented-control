# Motor & Operating-Point Configuration

Every tunable that ties the RTL to *this* motor and *this* driver lives in
one of two places: compile-time parameters in
[`rtl/foc/foc_pkg.sv`](../rtl/foc/foc_pkg.sv) (the locked operating point)
and run-time values pushed over UART (gains and references). This document
is the reference for both. Rationale and datasheet derivations are
in [`docs/hardware.md`](hardware.md); the control math is in
[`docs/foc.md`](foc.md).

Target machine: **Moons ECU16052H24-S002** — 3-phase BLDC, hall feedback,
per-phase R_s = 1.58 Ω, L_s = 127 µH (datasheet phase-to-phase 3.16 Ω /
0.253 mH ÷ 2), K_t = 14.85 mNm/A, **pole pairs = 1**, rated current
0.22 A, on a 24 V bus through a **DRV8316REVM**.

---

## 1. Compile-time parameters (`foc_pkg.sv`)

These are `parameter`s — change them in the package and re-synthesize. They
are deliberately *locked* for v1; the values below are the as-shipped
defaults.

### Clocking / PWM

| Parameter      | Value        | Meaning |
|----------------|--------------|---------|
| `F_CLK_HZ`     | 100 MHz      | Single BUFG clock, no PLL/MMCM. |
| `F_SW_HZ`      | 80 kHz       | PWM switching frequency. |
| `PWM_ARR`      | 625          | Center-aligned counter top = `F_CLK_HZ / (2·F_SW_HZ)`. Period = `2·PWM_ARR` clocks. Also sets the current-loop rate (T_s = 12.5 µs). |
| `DEADTIME_NS`  | 200 ns       | Complementary-gate dead time; must exceed the DRV8316 minimum. |
| `DEADTIME_CYC` | 20 cycles    | Derived from `DEADTIME_NS` at `F_CLK_HZ`. |

Changing `F_SW_HZ` or `F_CLK_HZ` re-derives `PWM_ARR` and the loop period,
which in turn shifts the effective `Ki` (T_s is folded into the integral
gain — see §2).

### Current / voltage scaling

| Parameter       | Value          | Meaning |
|-----------------|----------------|---------|
| `I_FULLSCALE_A` | 1.25 A         | XADC full scale. q15 current `1.0` ≡ 1.25 A. Set by CSA gain 1.2 V/A + front-end scaling. |
| `VBUS_NOM_V`    | 24.0 V         | Bus voltage. **24 V only** — there is no 12 V mode. |
| `MAX_MOD_Q15`   | 28508 (0.87)   | Modulation-index cap; keeps SVPWM out of overmodulation. |
| `OCP_TRIP_Q15`  | 23593 (0.72)   | RTL over-current trip = 0.9 A / 1.25 A FS. Firmware-level guard on top of the DRV8316's own driver-level OCP. |

The current full scale is the single most motor-dependent number: it is
chosen so the rated 0.22 A and worst-case ripple sit comfortably below the
0.9 A trip. Re-deriving it for a different motor/CSA gain means re-checking
`OCP_TRIP_Q15` and the telemetry scaling in §2.

### Driver (DRV8316) startup config

Written once at power-up by `drv8316_spi`, then read back and verified:

| Parameter        | Field            | Value |
|------------------|------------------|-------|
| `DRV_CTRL2_CFG`  | PWM mode / slew  | 6× PWM, SLEW = 50 V/µs |
| `DRV_CTRL4_CFG`  | OCP              | driver-level defaults (16 A class) |
| `DRV_CTRL5_CFG`  | CSA gain         | 1.2 V/A (`DRV_CSA_GAIN_1V2`) |

CSA gain here **must** match `I_FULLSCALE_A` above — they are two ends of
the same scaling chain. The register field encodings follow datasheet
rev B and should be re-verified against the exact silicon rev during
bring-up.

---

## 2. Run-time values (UART)

Pushed live over the host protocol (see the README command table). Lost on
reset — reload after every power cycle or fold into a host init script.

| Value     | Command          | Format            | Default | Notes |
|-----------|------------------|-------------------|---------|-------|
| `iq_ref`  | `iq <int16>`     | Q1.15 (1.0 = 1.25 A) | 0    | Torque-producing current setpoint. |
| `kp`      | `kp <uint16>`    | Q4.12             | 170 (≈0.0415) | PI proportional gain. |
| `ki`      | `ki <uint16>`    | Q4.12             | 26 (≈0.0063) | PI integral gain, T_s already folded in. |
| hall diag | `hall`           | no args            | read-only | Prints live observed Hall edge crossings. |

### PI gains

Defaults target **ω_c = 2π·1 kHz** against the per-phase RL plant
(R_s = 1.58 Ω, L_s = 127 µH, τ_e ≈ 80 µs), the same design point as the
working STM32 reference; derivation in `docs/foc.md` §5. Validated in
`tb_foc_top`. Because T_s is folded into `Ki`, retuning is required if
`F_SW_HZ` changes. Both axes (d and q) share the same gains. If the motor
runs warm (bench-measured R_s ≈ 2.83 Ω with leads), `ki 47` matches better.

### Hall calibration table

The FPGA uses the calibrated per-state Hall centers from the STM32 config as
the source of truth: `../stm32/src/common/settings.h`
`HALL_SECTOR_ANGLE_INIT`. Those radians are converted to 16-bit electrical
angle codes and baked into `foc_pkg.sv` as `HALL_CENTER`, ordered by the
FPGA sector mapping `001, 011, 010, 110, 100, 101`.

Current derived defaults:

| FPGA sector | Hall code | STM32 state | Center |
|-------------|-----------|-------------|--------|
| 0 | `001` | 1 | 54923 (301.7°) |
| 1 | `011` | 3 | 65481 (359.7°) |
| 2 | `010` | 2 | 10286 (56.5°) |
| 3 | `110` | 6 | 21554 (118.4°) |
| 4 | `100` | 4 | 33005 (181.3°) |
| 5 | `101` | 5 | 44109 (242.3°) |

Because **pole pairs = 1**, hall placement error maps 1:1 into electrical
angle. Re-run STM32 `hcal` after rewiring or swapping motors, paste its
results into `settings.h`, then regenerate these codes and rebuild the FPGA.
The FPGA `hall` UART command is a read-only sanity check; it does not update
the calibration table.

---

## 3. Porting checklist (different motor)

1. Update R_s, L_s, K_t in `docs/foc.md` (use **per-phase** values =
   phase-to-phase ÷ 2) and re-derive `kp`/`ki` per the §5 tuning math.
2. Recompute `I_FULLSCALE_A` from the new rated current and CSA gain; keep
   `DRV_CTRL5_CFG` (CSA gain) consistent.
3. Reset `OCP_TRIP_Q15` to a safe multiple of the new rated current.
4. If pole pairs ≠ 1, the hall→electrical-angle mapping in
   [`rtl/hall/hall_angle_est.sv`](../rtl/hall/hall_angle_est.sv) changes —
   the absolute-over-one-revolution assumption no longer holds.
5. Re-run STM32 `hcal`, update `settings.h`, and port the derived Hall
   center codes into `foc_pkg.sv`.
