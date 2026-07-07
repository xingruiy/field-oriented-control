#pragma once

/* SWV/ITM trace — optional fallback. The primary high-rate capture path is the
 * black-box recorder (common/bbox.h): lossless 40 kHz RAM logging dumped with
 * tools/bbox/bbox_dump.py, no SWO pin or CubeMX change needed.
 *
 * When FOC_TRACE is 0 (default) every trace_u16() compiles to nothing, so the
 * 40 kHz current loop is untouched. Set FOC_TRACE to 1 and enable SWV in the
 * debug config (SYS Debug = Trace Asynchronous Sw in CubeMX — project change,
 * confirm PB3 is free; core clock 400 MHz, ITM ports enabled) to stream
 * int16 counts to the SWV ITM console instead. */
#define FOC_TRACE   0

/* Suggested stimulus-port assignment (must be enabled in the debug config). */
#define TRACE_PORT_THETA   0
#define TRACE_PORT_OMEGA   1
#define TRACE_PORT_IQ      2
#define TRACE_PORT_ID      3
#define TRACE_PORT_IQREF   4
#define TRACE_PORT_HALL    5

/* Decimation: emit one sample every TRACE_DECIM current-loop ticks so the SWO
 * link and CubeIDE trace buffer are not saturated. 40 kHz / 40 = 1 kHz per
 * channel before ITM packet/timestamp overhead. */
#define TRACE_DECIM        40u

#if FOC_TRACE
#include "main.h"   /* pulls in the CMSIS core header (ITM registers) */

/* Blocking-free ITM write: skip if trace is disabled at runtime or the port
 * FIFO is full, so a stalled/absent SWV host never stretches the ISR. */
static inline void trace_u16(uint32_t port, int16_t val)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port))) {
        if (ITM->PORT[port].u32 != 0UL) {
            ITM->PORT[port].u16 = (uint16_t)val;
        }
    }
}
#else
static inline void trace_u16(uint32_t port, int16_t val) { (void)port; (void)val; }
#endif
