#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "common/settings.h"

/* -----------------------------------------------------------------------
 * DRV8316 SPI register map
 * Frame (16-bit, MSB first, SPI Mode 1):
 *   Bit 15   : R/W# (0=write, 1=read)
 *   Bits 14:9: Address [5:0]
 *   Bit  8   : Even parity (set so total 1-count across all 16 bits is even)
 *   Bits  7:0: Data [7:0]
 * ----------------------------------------------------------------------- */

/* Register addresses */
#define DRV_REG_IC_STATUS   0x00u   /* RO: general fault flags            */
#define DRV_REG_STATUS1     0x01u   /* RO: per-phase OCP flags            */
#define DRV_REG_STATUS2     0x02u   /* RO: SPI/supply fault flags         */
#define DRV_REG_CTRL1       0x03u   /* REG_LOCK                           */
#define DRV_REG_CTRL2       0x04u   /* PWM_MODE, CLR_FLT, SLEW            */
#define DRV_REG_CTRL3       0x05u   /* OVP, OTW reporting                 */
#define DRV_REG_CTRL4       0x06u   /* OCP settings                       */
#define DRV_REG_CTRL5       0x07u   /* CSA_GAIN                           */
#define DRV_REG_CTRL6       0x08u   /* Buck regulator                     */
#define DRV_REG_CTRL10      0x0Cu   /* Gate drive delay compensation      */

/* CTRL1 — register lock/unlock */
#define DRV_REGLOCK_UNLOCK  0x03u
#define DRV_REGLOCK_LOCK    0x06u

/* CTRL2 bitfield positions */
#define DRV_CLR_FLT_BIT     (1u << 0)
#define DRV_PWM_MODE_SHIFT  1u
#define DRV_PWM_MODE_6X     (0x00u << DRV_PWM_MODE_SHIFT)  /* 6x independent PWM */
/* SLEW (bits 4:3): OUTx slew rate. Faster slew = shorter internal dead time
 * (tDEAD @24 V: 25 V/µs → 1.8–3.4 µs, 200 V/µs → 0.5–0.75 µs), which sets the
 * inverter's voltage dead-band. Selection lives in settings.h (DRV_SLEW_SETTING). */
#define DRV_SLEW_SHIFT      3u
#define DRV_SLEW_25VUS      (0x00u << DRV_SLEW_SHIFT)
#define DRV_SLEW_50VUS      (0x01u << DRV_SLEW_SHIFT)
#define DRV_SLEW_125VUS     (0x02u << DRV_SLEW_SHIFT)
#define DRV_SLEW_200VUS     (0x03u << DRV_SLEW_SHIFT)
/* SDO output mode (bit 5): reset default is push-pull. MUST be kept set, because
 * MISO/PG9 has no MCU pull-up — clearing it (open-drain SDO) leaves register reads
 * floating and corrupted. Include this bit in every CTRL2 write. */
#define DRV_SDO_MODE_SHIFT  5u
#define DRV_SDO_MODE_PUSHPULL (0x01u << DRV_SDO_MODE_SHIFT)
/* CTRL3 is never written by firmware; it resets to this value. Used as a SPI
 * read-path sanity check in drv8316_init(). */
#define DRV_CTRL3_RESET     0x46u

/* CTRL4 — OCP */
#define DRV_OCP_DEG_SHIFT   4u
#define DRV_OCP_DEG_0p6US   (0x01u << DRV_OCP_DEG_SHIFT)  /* ~0.6 µs deglitch */
#define DRV_OCP_CBC_BIT     (1u << 6)                       /* cycle-by-cycle   */

/* CTRL5 — CSA gain: V/A (integrated sensing, no external shunt) */
#define DRV_CSA_GAIN_0p60   0x02u   /* ← use this: full-scale ±2.75 A   */

/* CTRL10 — driver delay compensation (SPI variant only). Equalises the
 * propagation delay of every switching edge to DLY_TARGET, removing the
 * current-direction-dependent duty distortion of tPD/tDEAD (datasheet §8.3.9.1,
 * DLY_TARGET pairing per Table 8-6). Selection lives in settings.h. */
#define DRV_DLYCMP_EN       (1u << 4)
#define DRV_DLY_0p4US       0x01u
#define DRV_DLY_1p2US       0x05u   /* recommended for 200 V/µs slew */
#define DRV_DLY_1p8US       0x08u   /* recommended for 125 V/µs slew */
#define DRV_DLY_2p4US       0x0Bu   /* recommended for  50 V/µs slew */
#define DRV_DLY_3p2US       0x0Fu   /* recommended for  25 V/µs slew */

/* IC_STATUS (0x00) — summary flags */
#define DRV_IC_FAULT        (1u << 0)   /* OR of OT/OVP/OCP/SPI_FLT/BK_FLT */
#define DRV_IC_OT           (1u << 1)   /* overtemperature                  */
#define DRV_IC_OVP          (1u << 2)   /* supply overvoltage               */
#define DRV_IC_OCP          (1u << 4)   /* overcurrent                      */
#define DRV_IC_SPI_FLT      (1u << 5)   /* SPI communication fault          */
#define DRV_IC_BK_FLT       (1u << 6)   /* buck regulator fault             */

/* Use only the FAULT summary bit for fault detection; NPOR is a status bit
 * (1 = supply OK above POR threshold, NOT a fault condition). */
#define DRV_FAULT_MASK      DRV_IC_FAULT

/* STATUS1 (0x01) — per-FET OCP + thermal */
#define DRV_S1_OCP_LA       (1u << 0)
#define DRV_S1_OCP_HA       (1u << 1)
#define DRV_S1_OCP_LB       (1u << 2)
#define DRV_S1_OCP_HB       (1u << 3)
#define DRV_S1_OCP_LC       (1u << 4)
#define DRV_S1_OCP_HC       (1u << 5)
#define DRV_S1_OTW          (1u << 6)   /* overtemperature WARNING (soft)   */
#define DRV_S1_OTS          (1u << 7)   /* overtemperature SHUTDOWN (latched)*/

/* STATUS2 (0x02) — supply / charge-pump / SPI detail */
#define DRV_S2_OTP_ERR      (1u << 0)   /* OTP register load error          */
#define DRV_S2_BUCK_OCP     (1u << 1)
#define DRV_S2_BUCK_UV      (1u << 2)
#define DRV_S2_VCP_UV       (1u << 3)   /* charge-pump undervoltage         */
#define DRV_S2_SPI_PARITY   (1u << 4)
#define DRV_S2_SPI_SCLK_FLT (1u << 5)
#define DRV_S2_SPI_ADDR_FLT (1u << 6)
/* bit 7 of STATUS2 is a normal-operation status bit (mirrors NPOR meaning);
 * it is NOT a fault indicator and should not be included in any fault mask. */

/* ---------------------------------------------------------------------- */
void     drv8316_init(void);
uint8_t  drv8316_read(uint8_t addr);
void     drv8316_write(uint8_t addr, uint8_t data);
void     drv8316_clear_faults(void);
/* Read the three status registers in one call (any may be NULL). */
void     drv8316_read_status(uint8_t *ic, uint8_t *s1, uint8_t *s2);
/* Count of SPI transfers that returned !HAL_OK or raised OVR/MODF. */
uint32_t drv8316_spi_errors(void);
/* Count of config writes that failed read-back verification during the last
 * drv8316_init() (includes the CTRL3 read-path sanity check). 0 = healthy link. */
uint32_t drv8316_cfg_errors(void);
/* Fill buf with human-readable register dump; returns bytes written */
#if FOC_DEBUG_ENABLE
int      drv8316_dump_regs(char *buf, int len);
#endif
