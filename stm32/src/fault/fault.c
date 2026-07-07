#include "fault/fault.h"
#include "drv8316/drv8316.h"
#include "control/foc.h"
#include "cli/cli.h"
#include "main.h"
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

static volatile uint8_t s_nfault_pending;  /* set by EXTI, cleared by poll */
static bool     s_fault_active;            /* latched until fault_clear()   */
static uint8_t  s_ic, s_s1, s_s2;          /* last decoded status snapshot  */
static uint32_t s_last_poll_ms;
static bool     s_otw_warned;              /* one-shot OTW warning latch     */

/* nFAULT is open-drain active-low: asserted when the pin reads LOW. */
static bool nfault_asserted(void)
{
    return HAL_GPIO_ReadPin(nFault_GPIO_Port, nFault_Pin) == GPIO_PIN_RESET;
}

/* ------------------------------------------------------------------ */
/* nFAULT EXTI — overrides HAL weak callback                          */
/* Invoked from EXTI9_5_IRQHandler -> HAL_GPIO_EXTI_IRQHandler.        */
/* ------------------------------------------------------------------ */

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == nFault_Pin) {
        foc_emergency_stop();   /* MOE off immediately — single register write */
        s_nfault_pending = 1;   /* main loop decodes the cause over SPI         */
    }
}

/* ------------------------------------------------------------------ */
/* Decode tables                                                       */
/* ------------------------------------------------------------------ */

struct bitname { uint8_t mask; const char *name; };

static const struct bitname ic_bits[] = {
    { DRV_IC_FAULT,  "FAULT"  }, { DRV_IC_OT,     "OT"     },
    { DRV_IC_OVP,    "OVP"    }, { DRV_IC_OCP,    "OCP"    },
    { DRV_IC_SPI_FLT,"SPI_FLT"}, { DRV_IC_BK_FLT, "BK_FLT" },
    /* DRV_IC_NPOR (bit 3) = 1 means supply OK, not a fault — omitted */
};
static const struct bitname s1_bits[] = {
    { DRV_S1_OCP_LA, "OCP_LA" }, { DRV_S1_OCP_HA, "OCP_HA" },
    { DRV_S1_OCP_LB, "OCP_LB" }, { DRV_S1_OCP_HB, "OCP_HB" },
    { DRV_S1_OCP_LC, "OCP_LC" }, { DRV_S1_OCP_HC, "OCP_HC" },
    { DRV_S1_OTW,    "OTW"    }, { DRV_S1_OTS,    "OTS"    },
};
static const struct bitname s2_bits[] = {
    { DRV_S2_OTP_ERR,  "OTP_ERR"  }, { DRV_S2_BUCK_OCP, "BUCK_OCP" },
    { DRV_S2_BUCK_UV,  "BUCK_UV"  }, { DRV_S2_VCP_UV,   "VCP_UV"   },
    { DRV_S2_SPI_PARITY,"SPI_PAR" }, { DRV_S2_SPI_SCLK_FLT,"SPI_SCLK"},
    { DRV_S2_SPI_ADDR_FLT,"SPI_ADDR"},
    /* STATUS2 bit 7 is a normal-operation status bit — omitted */
};

static int append_bits(char *buf, int len, int n, uint8_t val,
                       const struct bitname *tbl, int cnt)
{
    for (int i = 0; i < cnt; i++) {
        if ((val & tbl[i].mask) && n < len - 1) {
            n += snprintf(buf + n, (size_t)(len - n), " %s", tbl[i].name);
        }
    }
    return n;
}

int fault_describe(char *buf, int len)
{
    if (len <= 0) return 0;
    int n = 0;
    n += snprintf(buf + n, (size_t)(len - n),
                  "  IC_STATUS=0x%02X  STATUS1=0x%02X  STATUS2=0x%02X\r\n"
                  "  active=%s  spi_err=%lu  cfg_err=%lu\r\n  flags:",
                  s_ic, s_s1, s_s2,
                  s_fault_active ? "yes" : "no",
                  (unsigned long)drv8316_spi_errors(),
                  (unsigned long)drv8316_cfg_errors());
    n = append_bits(buf, len, n, s_ic, ic_bits, (int)(sizeof ic_bits / sizeof ic_bits[0]));
    n = append_bits(buf, len, n, s_s1, s1_bits, (int)(sizeof s1_bits / sizeof s1_bits[0]));
    n = append_bits(buf, len, n, s_s2, s2_bits, (int)(sizeof s2_bits / sizeof s2_bits[0]));
    if (n < len - 1) n += snprintf(buf + n, (size_t)(len - n), "\r\n");
    return n;
}

/* ------------------------------------------------------------------ */
/* Service / poll                                                      */
/* ------------------------------------------------------------------ */

static void service_fault(void)
{
    foc_disable();                          /* full disable in thread context */
    drv8316_read_status(&s_ic, &s_s1, &s_s2);
    s_fault_active = true;

    char buf[256];
    /* All-0xFF = SPI not driven: chip has lost power or entered hard reset.
     * Do not decode the register bits — they are not real fault flags. */
    if (s_ic == 0xFF && s_s1 == 0xFF && s_s2 == 0xFF) {
        cli_print("\r\n!! DRV8316 NOT RESPONDING (SPI all-0xFF)\r\n"
                  "  Chip may have lost VVM/VCC. Check supply at board.\r\n");
    } else {
        cli_print("\r\n!! DRV8316 FAULT\r\n");
        fault_describe(buf, sizeof buf);
        cli_print(buf);
    }
    cli_print("> ");
}

void fault_poll(void)
{
    /* Software overcurrent backstop: the 40 kHz loop already cut MOE in-ISR;
     * latch it as a fault here (thread ctx) so `en` is blocked until `clrf`. */
    if (foc_oc_tripped()) {
        foc_clear_oc_trip();
        if (!s_fault_active) {
            foc_disable();
            s_fault_active = true;
            cli_print("\r\n!! SOFTWARE OVERCURRENT TRIP — bridge disabled\r\n"
                      "  phase current exceeded MOTOR_OC_TRIP_A. Check Hall angle /\r\n"
                      "  wiring; clear with 'clrf'.\r\n> ");
        }
    }

    /* Fast path: nFAULT EXTI flagged a hard fault — decode and latch. */
    if (s_nfault_pending) {
        s_nfault_pending = 0;
        if (!s_fault_active) {
            service_fault();
        }
    }

    /* Soft path: throttled status poll for warnings / missed edges. */
    uint32_t now = HAL_GetTick();
    if (now - s_last_poll_ms >= 100u) {
        s_last_poll_ms = now;
        if (!s_fault_active) {
            uint8_t ic = drv8316_read(DRV_REG_IC_STATUS);
            if (ic & DRV_FAULT_MASK) {
                /* nFAULT asserted but EXTI not (yet) serviced — handle here. */
                service_fault();
            } else {
                uint8_t s1 = drv8316_read(DRV_REG_STATUS1);
                if (s1 & DRV_S1_OTW) {
                    if (!s_otw_warned) {
                        s_otw_warned = true;
                        cli_print("\r\n[warn] DRV8316 overtemperature warning (OTW)\r\n> ");
                    }
                } else {
                    s_otw_warned = false;
                }
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void fault_init(void)
{
    s_nfault_pending = 0;
    s_fault_active   = false;
    s_otw_warned     = false;
    s_last_poll_ms   = HAL_GetTick();
    if (nfault_asserted()) {
        s_nfault_pending = 1;   /* fault already asserted at boot */
    }
}

bool fault_is_active(void)
{
    return s_fault_active;
}

void fault_clear(void)
{
    drv8316_clear_faults();
    foc_clear_oc_trip();
    s_fault_active = false;
    s_otw_warned   = false;
    s_nfault_pending = 0;
    drv8316_read_status(&s_ic, &s_s1, &s_s2);
    /* If the line is still held low, let the next poll re-latch the fault. */
    if (nfault_asserted()) {
        s_nfault_pending = 1;
    }
}
