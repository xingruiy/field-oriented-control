#pragma once

/* Moons ECU16052H24-S002 — verified electrical parameters */
#define MOTOR_POLE_PAIRS        1U
/* Effective per-phase Rs as measured by `ctune` on this unit (warm + lead R);
 * the datasheet/cold value is 1.58 Ω. This is the value the firmware drives
 * with — it sizes the hcal open-loop voltage and matches the baked PI gains —
 * so it must track reality, not the datasheet. */
#define MOTOR_RS_OHM            2.923f      /* measured per phase */
#define MOTOR_RS_DATASHEET_OHM  1.58f       /* cold datasheet ref, for ctune report */
#define MOTOR_LS_H              3.07e-4f    /* per phase, phase-to-phase / 2 */
#define MOTOR_KE_V_S_RAD        0.01485f    /* Ke = Kt = 14.85 mNm/A */
#define MOTOR_RATED_A           0.22f
#define MOTOR_CURRENT_LIMIT_A   0.3f        /* hard clamp in foc_set_iq_ref() */
/* Software overcurrent backstop checked every 40 kHz current-loop tick. Trips
 * (cuts MOE, latches a fault) before the DRV8316 hardware OCP so a bad angle or
 * runaway fails soft in firmware. Set above ctune's Ls-step transient (~1.1 A)
 * and the 1.0 A command clamp, below the ±2.75 A CSA full-scale. */
#define MOTOR_OC_TRIP_A         2.0f

/* DRV8316 gate drive (values from drv8316.h). Fastest slew rate minimises the
 * driver's internal dead time (0.5–0.75 µs vs 1.8–3.4 µs at 25 V/µs) — each µs
 * of dead time is a ~1 V voltage dead-band at 24 V/40 kHz, huge next to this
 * motor's sub-volt working voltages. Delay compensation (CTRL10) equalises the
 * switching-edge delay to DLY_TARGET; TI pairs 1.2 µs with 200 V/µs slew. */
#define DRV_SLEW_SETTING        DRV_SLEW_200VUS
#define DRV_DLY_TARGET_SETTING  DRV_DLY_1p2US

/* DRV8316 integrated CSA — no external shunt
 * V_SOx = I_phase * CSA_GAIN + Vref/2
 * CSA_GAIN register 0b10 = 0.60 V/A → full-scale ±2.75 A at 3.3 V ref */
#define CSA_GAIN_V_PER_A        0.60f
#define ADC_VREF_V              3.3f
/* STM32H755: ADC in 16-bit mode with 8x oversampling + right-shift 3, so the
 * averaged result keeps the full 16-bit scale (0..65535, mid 32768). */
#define ADC_RESOLUTION          65536U
/* A/LSB = Vref / (res * gain) = 3.3 / (65536 * 0.6) ≈ 8.39e-5 */
#define ADC_CURRENT_SCALE       (ADC_VREF_V / ((float)ADC_RESOLUTION * CSA_GAIN_V_PER_A))

/* Hall — angle offset folded into the measured per-state table below (see
 * `hcal`), so the scalar offset is 0. Override live with `hoff <rad>` if needed. */
#define HALL_ANGLE_OFFSET_RAD   0.0f

/* Per-state electrical-angle centres (rad), indexed by the 3-bit Hall code
 * [0..7] (0/7 invalid). These are the reboot defaults; `hcal` overwrites the
 * live RAM copy in hall.c and prints the new values — paste them here to make a
 * calibration permanent. Re-run hcal after any rewiring or motor swap.
 * Values below: measured centres on this unit (hcal fwd+rev). */
#define HALL_SECTOR_ANGLE_INIT { \
    0.0f,       /* 0b000 invalid                */ \
    5.26566f,   /* 0b001 state 1 = 301.7 deg    */ \
    0.98611f,   /* 0b010 state 2 =  56.5 deg    */ \
    6.27795f,   /* 0b011 state 3 = 359.7 deg    */ \
    3.16428f,   /* 0b100 state 4 = 181.3 deg    */ \
    4.22893f,   /* 0b101 state 5 = 242.3 deg    */ \
    2.06647f,   /* 0b110 state 6 = 118.4 deg    */ \
    0.0f,       /* 0b111 invalid                */ \
}

/* Hall angle/speed observer. Per Hall edge, e = wrap_pi(boundary − θ̂):
 *   θ̂ += KP·e                        (soft phase correction → continuous angle)
 *   ω̂ += KI·(edge_speed − ω̂)         (low-pass of the measured edge speed)
 * KP = fraction of phase error corrected per edge (~0.3, converges in a few
 * edges). KI = speed-filter coefficient in (0,1]: →1 tracks like the old
 * hard-snap (fast, steppy), lower = smoother but laggier (too low starves the
 * speed loop of true speed → runaway). Live-tune with `hpll <kp> <ki>`. */
#define HALL_PLL_KP             0.30f
#define HALL_PLL_KI             0.50f

/* Invalid Hall code (0b000/0b111) supervision: while a Hall-consuming drive
 * mode is running (FOC, block, rotor-voltage), an invalid code persisting for
 * this many consecutive 40 kHz ticks kills the bridge and latches a fault.
 * The persistence filter exists because sensor skew during a normal edge can
 * pass through 0/7 for a few µs — a real disconnect (pull-ups → 0b111) lasts
 * forever. 8 ticks = 200 µs. */
#define HALL_INVALID_TRIP_TICKS 8U

/* Sign mapping electrical-angle direction → physical rotation, so reported
 * direction/speed match reality. On this unit the rotor turns CW as the
 * electrical angle θe increases, so the relationship is inverted (−1). This
 * affects only hall_get_dir()/hall_get_omega_e() readouts, never the θe
 * interpolation used by the control loop. */
#define HALL_PHYS_DIR_SIGN      (-1)

/* Current PI — ωc = 2π*1000 rad/s bandwidth, Kp = Ls*ωc, Ki = Rs*ωc. */
#define PID_D_KP                0.798f
#define PID_D_KI                18200.6f
#define PID_Q_KP                0.798f
#define PID_Q_KI                18200.6f

/* dq voltage-vector limit. The largest undistorted (linear SVM) phase-voltage
 * amplitude is Vbus/√3; beyond it foc_svm() clamps per-phase duty and bends
 * the vector. Limit |v| to that circle with a small margin, d-axis first:
 * vd gets the full budget, vq gets what remains (√(V²−vd²)) — flux control
 * keeps priority and the PI anti-windup stays truthful in saturation. */
#define VDQ_LIMIT_FRAC          0.95f
#define VDQ_MAX_V               (VBUS_V * VDQ_LIMIT_FRAC / M_SQRT3_F)  /* ≈13.2 V */

/* Dead-time compensation. Each switching edge loses ~t_dt of commanded volt-
 * seconds in the direction opposing the phase current: ΔV = Vbus·t_dt·f_pwm·
 * sign(i). At 24 V / 40 kHz this is ~0.5 V — the same order as this motor's
 * whole I·R working voltage — so foc_svm() adds it back per phase. t_dt is the
 * effective bridge dead time: TIM1 150 ns + DRV8316 ~0.35 µs at 200 V/µs slew
 * (delay compensation on). Deliberately a mild underestimate — overcompensation
 * causes zero-crossing oscillation; undercompensation just leaves residual
 * distortion. Set to 0 to disable. Below DT_COMP_I_TH_A the correction tapers
 * linearly through zero so measurement noise cannot chatter the sign. */
#define DT_COMP_T_S             0.5e-6f
#define DT_COMP_V               (VBUS_V * DT_COMP_T_S * 40000.0f)      /* ≈0.48 V */
#define DT_COMP_I_TH_A          0.05f

/* TIM1 parameters — must match CubeMX configuration.
 * STM32H755 @ 400 MHz sysclk (VOS1): 
 * timer clock = 200 MHz,
 * center-aligned, ARR=2500 → 40 kHz. */
#define PWM_ARR                 2500U
#define PWM_DT_S                (1.0f / 40000.0f)   /* 25 µs */

/* Speed PI — outer cascaded loop, runs at PWM rate / SPEED_LOOP_DIV.
 * Output is an Iq current command (A), clamped to ±MOTOR_CURRENT_LIMIT_A so the
 * pid_update() integral clamp also bounds the commanded current.
 * Tyreus–Luyben tuning rule. Re-run `stune` if load/inertia changes. */
#define SPEED_KP                0.00054f     /* A per (elec rad/s)        */
#define SPEED_KI                0.00151f     /* A per (elec rad/s) per s  */
#define SPEED_LOOP_DIV          40U         /* 40 kHz / 40 = 1 kHz loop  */
#define SPEED_DT_S              (PWM_DT_S * (float)SPEED_LOOP_DIV)   /* 1 ms */

/* Reference slew-rate limits. Setpoint commands (`iq`, `spd`, CAN) land in a
 * target; the active reference ramps toward it so a step command cannot slam
 * the loops. iq_ref ramps at 40 kHz (torque mode only — in speed mode iq_ref
 * is already rate-shaped by the omega ramp); omega_ref ramps at the 1 kHz
 * speed loop and starts from the measured speed on speed-mode entry. */
#define IQ_REF_SLEW_A_PER_S     5.0f        /* 0 → 0.3 A limit in 60 ms        */
#define SPEED_REF_SLEW_RAD_S2   400.0f      /* elec rad/s²; 0 → 500 RPM ≈ 0.13 s */

/* RPM <-> electrical rad/s (θe = θmech for 1 pole pair) */
#define RPM_TO_OMEGA_E(rpm)     ((rpm) * (M_TWOPI_F / 60.0f) * (float)MOTOR_POLE_PAIRS)
#define OMEGA_E_TO_RPM(w)       ((w)   * (60.0f / M_TWOPI_F) / (float)MOTOR_POLE_PAIRS)

/* Nominal DC bus voltage — no Vbus ADC channel */
#define VBUS_V                  24.0f

/* ------------------------------------------------------------------ */
/* 6-step (block commutation)                                          */
/* ------------------------------------------------------------------ */
#define BLOCK_DUTY_MAX          0.95f       /* clamp duty away from 100%       */
#define BLOCK_RAMP_STEP         0.002f      /* duty increment per 25 µs tick   */
/* Commutation table: {high_phase, low_phase} per Hall state [0..7].
 * Phase indices: 0=A, 1=B, 2=C. States 0 and 7 are invalid (0,0 filler).
 * Derived from hcal sector centers + 90 electrical-degree lead.
 * Swapping hi/lo in this table reverses rotation direction. */
#define BLOCK_TABLE_HI { 0, 2, 0, 2, 1, 1, 0, 0 }
#define BLOCK_TABLE_LO { 0, 0, 1, 1, 2, 0, 2, 0 }

/* ------------------------------------------------------------------ */
/* Autotune (`ctune` / `stune`)                                        */
/* ------------------------------------------------------------------ */

/* Current-loop design bandwidth: Kp = Ls*wc, Ki = Rs*wc */
#define TUNE_WC_RAD_S           (M_TWOPI_F * 1000.0f)

/* Rs: two d-axis voltage levels; Rs = (Vhi-Vlo)/(Ihi-Ilo) cancels the
 * (roughly constant) dead-time voltage error. Brief over-current is OK. */
#define TUNE_ALIGN_V            1.0f        /* hold to pull rotor to d-axis */
#define TUNE_ALIGN_MS           500u
#define TUNE_RS_V_LO            1.0f
#define TUNE_RS_V_HI            2.0f
#define TUNE_RS_SETTLE_MS       150u

/* Ls: apply a d-axis voltage step from 0, capture the current at the PWM
 * rate, fit the L/R rise (τ = Ls/Rs) using the measured settled current as
 * the asymptote, so the result is independent of voltage/dead-time scaling. */
#define TUNE_LS_VSTEP_V         3.0f
#define TUNE_LS_CAP_N           16u         /* PWM-rate samples per capture  */
#define TUNE_LS_DECAY_MS        5u          /* let current decay before step */
/* Transport delay of the step, in PWM ticks. Sample k is taken (counter peak)
 * BEFORE the CCR write at tick 0, and CCR is preloaded, so the step reaches
 * the bridge only at the next update event: sample k has seen between
 * (k−1)·Ts and (k−0.5)·Ts of drive, not k·Ts. With τ ≈ 4 ticks, ignoring this
 * overestimates Ls by ~30%. 0.75 is the midpoint of the update-phase bound. */
#define TUNE_LS_DELAY_TICKS     0.75f

/* Speed-loop relay (Åström) autotune → Tyreus–Luyben PI rule. */
#define TUNE_SPEED_RPM          500.0f      /* default relay setpoint        */
#define TUNE_SPEED_H            0.30f       /* relay Iq amplitude (A)        */
#define TUNE_SPEED_HYST_FRAC    0.05f       /* hysteresis vs setpoint        */
#define TUNE_SPEED_CYCLES       8u          /* limit cycles to average       */
#define TUNE_SPEED_SPINUP_MS    3000u
#define TUNE_SPEED_TIMEOUT_MS   12000u

/* ------------------------------------------------------------------ */
/* CAN (FDCAN1 @ 500 kbps, classic, 11-bit IDs) — bit timing & message */
/* RAM are configured in CubeMX; this is the application protocol only. */
/* ------------------------------------------------------------------ */

/* Control frames (PC → MCU). One RX mask filter accepts CAN_CTRL_BASE..+0xF. */
#define CAN_CTRL_BASE           0x100U      /* filter base ID                  */
#define CAN_CTRL_MASK           0x7F0U      /* accepts 0x100..0x10F            */
#define CAN_ID_CMD              0x100U      /* b0 opcode (see CAN_OP_*)        */
#define CAN_ID_SET_IQ           0x101U      /* int16 LE, mA                    */
#define CAN_ID_SET_SPEED        0x102U      /* int16 LE, RPM                   */
#define CAN_ID_SET_ARM_POS      0x103U      /* int32 LE, arm target cdeg       */
#define CAN_OP_DISABLE          0U
#define CAN_OP_ENABLE           1U
#define CAN_OP_CLEAR_FAULT      2U
#define CAN_OP_SPEED_OFF        3U          /* manual mode, iq=0               */
#define CAN_OP_CAL_ADC          4U          /* `cal`  ADC offset calibration   */
#define CAN_OP_HCAL             5U          /* `hcal` Hall angle calibration   */
#define CAN_OP_ARM_OFF          6U          /* disable TMAG arm position PID   */

/* Telemetry frames (MCU → PC), broadcast every CAN_TLM_PERIOD_MS. */
#define CAN_ID_STATUS           0x200U      /* flags, hall, speed, iq_ref, iq  */
#define CAN_ID_CURRENTS         0x201U      /* Ia, Ib, Ic (mA), theta_e (cdeg) */
#define CAN_ID_CAL_RESULT       0x202U      /* type, ok, off_a/b/c (one-shot)  */
#define CAN_ID_ENCODER          0x203U      /* angle, speed, turns, ok, mag    */
#define CAN_ID_ARM_STATUS       0x204U      /* state, target/current/error/out */
#define CAN_CAL_TYPE_ADC        1U
#define CAN_CAL_TYPE_HALL       2U
#define CAN_TLM_PERIOD_MS       10U         /* 100 Hz                          */

/* CAN dead-man: if the last motion setpoint (iq/speed) came over CAN and no
 * control frame arrives for this long while FOC is enabled, disable the bridge
 * (the host is presumed dead — don't keep spinning at its last command).
 * 0 disables the timeout. Default 0 because the bundled can_gui sends one-shot
 * setpoints; set to e.g. 1000 once the host re-sends its setpoint periodically.
 * A local CLI iq/spd command takes ownership back and disarms the timeout;
 * the arm-position supervisor is autonomous and never subject to it. */
#define CAN_CMD_TIMEOUT_MS      0U

/* ------------------------------------------------------------------ */
/* Debugging / bring-up features                                      */
/* ------------------------------------------------------------------ */

/* Production builds leave test drives, tuning routines, register dumps, live
 * Hall PLL/offset tuning, Hall check, and black-box capture compiled out. */
#ifndef FOC_DEBUG_ENABLE
#define FOC_DEBUG_ENABLE        0
#endif

/* hchk pass limit: per-state |circular mean| of θ̂ − θ_forced (degrees). */
#define HCHK_PASS_ERR_DEG       5.0f

/* Independent watchdog (IWDG1 on LSI ≈ 32 kHz), refreshed from fault_poll()
 * in the main superloop. Catches a hung main loop — the 40 kHz ISR would
 * otherwise keep driving the motor with no fault supervision, CLI, or CAN.
 * The timeout must outlast the longest blocking CLI routine (stune ~15 s,
 * hcal ~8 s, neither refreshes the watchdog), hence prescaler 256 / reload
 * 4095 → ~32.8 s. Frozen while the core is halted by a debugger (DBGMCU). */
#define FOC_IWDG_ENABLE         1

/* Black box: disabled by default so it cannot add work to the 40 kHz loop.
 * Set FOC_DEBUG_ENABLE and FOC_BBOX_ENABLE to 1 for a diagnostic build. */
#ifndef FOC_BBOX_ENABLE
#define FOC_BBOX_ENABLE         0
#endif

/* Black box: 8 int16 channels sampled every 40 kHz tick into RAM_D1.
 * 8 ch × 2 B × BBOX_LEN = 256 KB ≈ 0.41 s of lossless history. */
#define BBOX_LEN                16384U
#define BBOX_POST_TRIG          (BBOX_LEN / 4U)  /* run-on after trigger      */
/* Auto-freeze when |ω̂| exceeds this (rad/s elec). Vbus-feasible max is
 * ~13.8 V / Ke ≈ 930 rad/s; normal running speeds are <100 rad/s, so a trip
 * just above the physical ceiling freezes the capture at the instant of
 * overspeed — keeping the standstill→runaway onset in the pre-trigger window. */
#define BBOX_TRIP_OMEGA         950.0f

/* Status flag bits (byte 0 of CAN_ID_STATUS). */
#define CAN_FLAG_ENABLED        (1U << 0)
#define CAN_FLAG_SPEED_MODE     (1U << 1)
#define CAN_FLAG_BLOCK_MODE     (1U << 2)
#define CAN_FLAG_FAULT          (1U << 3)
#define CAN_FLAG_ARM_POS        (1U << 4)

/* Payload scaling (little-endian). */
#define CAN_CURRENT_SCALE       1000.0f                 /* A → mA (int16)      */
#define CAN_ANGLE_CDEG_SCALE    (36000.0f / M_TWOPI_F)  /* rad → centidegrees  */
#define CAN_ENC_ANGLE_SCALE     100.0f      /* deg → centideg (uint16 0..35999) */
#define CAN_ENC_SPEED_SCALE     10.0f       /* deg/s → 0.1 deg/s (int16)        */

/* ------------------------------------------------------------------ */
/* TMAG arm position loop — low-rate outer PID in the main loop.       */
/* Output is motor RPM into the existing speed loop.                   */
/* ------------------------------------------------------------------ */
#define ARM_POS_MIN_DEG         -360.0f     /* absolute encoder total degrees  */
#define ARM_POS_MAX_DEG          360.0f
#define ARM_POS_TOL_DEG          1.0f
#define ARM_POS_MAX_RPM          60.0f
#define ARM_POS_I_LIMIT_RPM      20.0f
#define ARM_POS_KP               1.0f       /* RPM per deg                     */
#define ARM_POS_KI               0.0f       /* RPM per deg*s                   */
#define ARM_POS_KD               0.02f      /* RPM per deg/s                   */
#define ARM_POS_DIR_SIGN         1          /* flip if positive error moves away */
#define ARM_POS_CMD_SCALE        100.0f     /* deg → centideg                  */
#define ARM_POS_TLM_SCALE        10.0f      /* deg/RPM → 0.1 units             */

/* ------------------------------------------------------------------ */
/* External angle encoder — TMAG5273 on I2C1 (PB8=SCL, PB9=SDA, 400k). */
/* Monitors an external rotating object; independent of the FOC loops. */
/* Polled (blocking I2C) from the main loop only — never from an ISR.  */
/* Sensor is one half of the snap-apart TMAG5273EVM (both halves have  */
/* 4.7k I2C pull-ups and the same 0x35 address — one on the bus at a   */
/* time). The A1/A2 variant is auto-detected from DEVICE_ID at init    */
/* and selects the XY range below; TI's Rotate&Push magnet is >40 mT,  */
/* so A1 needs the 80 mT range (EVM guide SLYU058 sec 4.2).            */
/* ------------------------------------------------------------------ */
#define ENC_I2C_ADDR            0x35U       /* 7-bit default address           */
#define ENC_CONV_AVG            0x4U        /* CONV_AVG: 16x averaging         */
#define ENC_MAG_CH_EN           0x3U        /* MAG_CH_EN: X,Y channels         */
#define ENC_ANGLE_EN            0x1U        /* ANGLE_EN: 1h=XY 2h=YZ 3h=XZ     */
#define ENC_XY_RANGE_A1         0x1U        /* A1: 1 = ±80 mT (0 = ±40 mT)     */
#define ENC_XY_RANGE_A2         0x0U        /* A2: 0 = ±133 mT (1 = ±266 mT)   */
#define ENC_POLL_PERIOD_MS      5U          /* 200 Hz angle read               */
#define ENC_I2C_TIMEOUT_MS      2U          /* per HAL transaction             */
#define ENC_SPEED_LPF_ALPHA     0.15f       /* speed 1st-order LPF, per sample */
#define ENC_ERR_LIMIT           8U          /* consecutive errors → not-ok     */
#define ENC_REINIT_MS           1000U       /* re-probe cadence when not-ok    */
