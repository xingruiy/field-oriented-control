#include "drv8316/drv8316.h"
#include "common/settings.h"
#include "main.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

/* PC8 = SPI1 chip select, active-low (nSCS_Pin / nSCS_GPIO_Port from main.h) */
#define CS_LOW()   HAL_GPIO_WritePin(nSCS_GPIO_Port, nSCS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(nSCS_GPIO_Port, nSCS_Pin, GPIO_PIN_SET)

/* Running count of SPI transfers that failed (HAL error or OVR/MODF). */
static volatile uint32_t s_spi_err;

/* Config writes that failed read-back verification during the last init. */
static volatile uint32_t s_cfg_err;

/* >= 400 ns nSCS high between SPI words per DRV8316 datasheet */
static void drv_cs_idle(void)
{
    for (volatile uint32_t i = 0; i < 600u; i++) { /* spin */ }
}

static uint8_t drv_parity(uint16_t frame)
{
    frame ^= frame >> 8;
    frame ^= frame >> 4;
    frame ^= frame >> 2;
    frame ^= frame >> 1;
    return (uint8_t)(frame & 1u);
}

/* SPI1 is configured for 16-bit transfers; pass a single uint16_t word.
 * Every transfer is checked: a non-OK HAL return, or an OVR/MODF flag, bumps
 * s_spi_err so the fault layer can surface a degraded SPI link. */
static uint16_t drv_transfer(uint16_t tx)
{
    uint16_t rx = 0;
    CS_LOW();
    /* HAL expects uint8_t* but handles 16-bit words when DataSize = 16-bit */
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1,
                                                   (uint8_t *)&tx,
                                                   (uint8_t *)&rx, 1, 10);
    CS_HIGH();
    drv_cs_idle();   /* guarantee ≥400 ns nSCS-high before the next word */

    if (st != HAL_OK) {
        s_spi_err++;
    }
    /* OVR: clear flag, bump error count */
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_OVR)) {
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
        s_spi_err++;
    }
    /* MODF: clear, re-init master, bump error count */
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_MODF)) {
        __HAL_SPI_CLEAR_MODFFLAG(&hspi1);
        SET_BIT(hspi1.Instance->CFG2, SPI_CFG2_MASTER);
        __HAL_SPI_ENABLE(&hspi1);
        s_spi_err++;
    }
    return rx;
}

uint32_t drv8316_spi_errors(void)
{
    return s_spi_err;
}

uint32_t drv8316_cfg_errors(void)
{
    return s_cfg_err;
}

/* Write a config register, then read it back to confirm the value latched. Retries
 * a few times before giving up (and bumping s_cfg_err). Only valid for plain R/W
 * config registers — NOT for self-clearing bits like CLR_FLT. */
static bool drv_write_verify(uint8_t addr, uint8_t data)
{
    for (uint32_t attempt = 0; attempt < 3u; attempt++) {
        drv8316_write(addr, data);
        if (drv8316_read(addr) == data) {
            return true;
        }
    }
    s_cfg_err++;
    return false;
}

void drv8316_read_status(uint8_t *ic, uint8_t *s1, uint8_t *s2)
{
    if (ic) *ic = drv8316_read(DRV_REG_IC_STATUS);
    if (s1) *s1 = drv8316_read(DRV_REG_STATUS1);
    if (s2) *s2 = drv8316_read(DRV_REG_STATUS2);
}

static uint16_t drv_build_write(uint8_t addr, uint8_t data)
{
    /* [15]=0(write) [14:9]=addr [8]=parity [7:0]=data */
    uint16_t f = ((uint16_t)(addr & 0x3Fu) << 9) | (uint16_t)data;
    f |= (uint16_t)(drv_parity(f) << 8);
    return f;
}

static uint16_t drv_build_read(uint8_t addr)
{
    uint16_t f = (1u << 15) | ((uint16_t)(addr & 0x3Fu) << 9);
    f |= (uint16_t)(drv_parity(f) << 8);
    return f;
}

uint8_t drv8316_read(uint8_t addr)
{
    uint16_t rx = drv_transfer(drv_build_read(addr));
    return (uint8_t)(rx & 0xFFu);
}

void drv8316_write(uint8_t addr, uint8_t data)
{
    drv_transfer(drv_build_write(addr, data));
}

void drv8316_clear_faults(void)
{
    drv8316_write(DRV_REG_CTRL1, DRV_REGLOCK_UNLOCK);
    /* Keep SDO_MODE push-pull — a CTRL2 write without it would flip SDO to
     * open-drain and corrupt all subsequent register reads (no MISO pull-up). */
    drv8316_write(DRV_REG_CTRL2, DRV_SDO_MODE_PUSHPULL | DRV_PWM_MODE_6X |
                                 DRV_SLEW_SETTING | DRV_CLR_FLT_BIT);
    drv8316_write(DRV_REG_CTRL1, DRV_REGLOCK_LOCK);
}

void drv8316_init(void)
{
    s_cfg_err = 0;

    /* CubeMX initialised CS low; de-assert immediately */
    CS_HIGH();
    HAL_Delay(2);  /* power-on settle */

    /* 1. Unlock registers (REG_LOCK resets unlocked, but be explicit) */
    drv8316_write(DRV_REG_CTRL1, DRV_REGLOCK_UNLOCK);

    /* 2. PWM mode: 6x independent (dead-time supplied by TIM1 hardware).
     * SDO_MODE push-pull MUST stay set (MISO/PG9 has no pull-up). */
    drv_write_verify(DRV_REG_CTRL2,
                     DRV_SDO_MODE_PUSHPULL | DRV_PWM_MODE_6X | DRV_SLEW_SETTING);

    /* 3. Current sense gain: 0.60 V/A → full-scale ±2.75 A */
    drv_write_verify(DRV_REG_CTRL5, DRV_CSA_GAIN_0p60);

    /* 4. OCP: ~0.6 µs deglitch, cycle-by-cycle mode */
    drv_write_verify(DRV_REG_CTRL4, DRV_OCP_DEG_0p6US | DRV_OCP_CBC_BIT);

    /* 5. Delay compensation: fixed switching-edge delay = DLY_TARGET, removes
     * the current-direction-dependent duty distortion of tPD/tDEAD. */
    drv_write_verify(DRV_REG_CTRL10, DRV_DLYCMP_EN | DRV_DLY_TARGET_SETTING);

    /* 6. Clear any latched faults (CLR_FLT is self-clearing — pulse, don't verify) */
    drv8316_write(DRV_REG_CTRL2, DRV_SDO_MODE_PUSHPULL | DRV_PWM_MODE_6X |
                                 DRV_SLEW_SETTING | DRV_CLR_FLT_BIT);

    /* 7. Re-lock */
    drv8316_write(DRV_REG_CTRL1, DRV_REGLOCK_LOCK);

    /* 8. SPI read-path sanity check: CTRL3 is never written and resets to 0x46.
     * A mismatch means reads are not returning real register data (open-drain SDO,
     * framing/timing, or wiring) — flagged via drv8316_cfg_errors() / 'fault'. */
    if (drv8316_read(DRV_REG_CTRL3) != DRV_CTRL3_RESET) {
        s_cfg_err++;
    }

    /* Init done; cfg-error count and any fault are visible via CLI 'fault'. */
}

#if FOC_DEBUG_ENABLE
int drv8316_dump_regs(char *buf, int len)
{
    static const struct { uint8_t addr; const char *name; } regs[] = {
        { DRV_REG_IC_STATUS, "IC_STATUS" },
        { DRV_REG_STATUS1,   "STATUS1  " },
        { DRV_REG_STATUS2,   "STATUS2  " },
        { DRV_REG_CTRL1,     "CTRL1    " },
        { DRV_REG_CTRL2,     "CTRL2    " },
        { DRV_REG_CTRL3,     "CTRL3    " },
        { DRV_REG_CTRL4,     "CTRL4    " },
        { DRV_REG_CTRL5,     "CTRL5    " },
        { DRV_REG_CTRL6,     "CTRL6    " },
        { DRV_REG_CTRL10,    "CTRL10   " },
    };
    int n = 0;
    for (int i = 0; i < (int)(sizeof regs / sizeof regs[0]); i++) {
        uint8_t v = drv8316_read(regs[i].addr);
        n += snprintf(buf + n, (size_t)(len - n),
                      "  %s [0x%02X] = 0x%02X\r\n",
                      regs[i].name, regs[i].addr, v);
        if (n >= len - 1) break;
    }
    return n;
}
#endif
