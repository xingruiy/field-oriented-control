#include "can/can.h"
#include "control/foc.h"
#include "control/foc_math.h"
#include "hall/hall.h"
#include "fault/fault.h"
#include "encoder/encoder.h"
#include "arm/arm_pos.h"
#include "common/settings.h"
#include "main.h"
#include <math.h>

extern FDCAN_HandleTypeDef hfdcan1;

/* ------------------------------------------------------------------ */
/* Pack / transmit helpers                                             */
/* ------------------------------------------------------------------ */

static int16_t sat16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lrintf(v);
}

static void put16(uint8_t *p, uint16_t v)   /* little-endian */
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static int32_t get32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static void can_abort_pending_tx(void)
{
    HAL_FDCAN_AbortTxRequest(&hfdcan1, 0xFFu);  /* Tx FIFO uses buffers 0..7. */
}

static bool can_tx(uint32_t id, const uint8_t *data)
{
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        can_abort_pending_tx();
        return false;
    }

    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)data) == HAL_OK;
}

/* ------------------------------------------------------------------ */
/* Command dispatch (PC → MCU) — reuses the CLI's FOC/fault accessors   */
/* ------------------------------------------------------------------ */

/* One-shot result frame after a calibration (offsets valid for ADC cal). */
static void can_send_cal_result(uint8_t type, bool ok)
{
    uint8_t b[8] = {0};
    b[0] = type;
    b[1] = ok ? 1 : 0;
    put16(&b[2], (uint16_t)lrintf(foc_get_offset_a()));
    put16(&b[4], (uint16_t)lrintf(foc_get_offset_b()));
    put16(&b[6], (uint16_t)lrintf(foc_get_offset_c()));
    can_tx(CAN_ID_CAL_RESULT, b);
}

/* Blocking calibrations (same thread context + guards as the CLI). These
 * stall the main loop while running (cal ~ms, hcal ~5 s open-loop drive). */
static void can_run_cal_adc(void)
{
    if (arm_pos_is_active()) return;
    if (foc_is_enabled()) return;
    can_send_cal_result(CAN_CAL_TYPE_ADC, foc_recalibrate());
}

static void can_run_hcal(void)
{
    if (arm_pos_is_active()) return;
    if (foc_is_enabled() || fault_is_active()) return;
    char report[640];
    can_send_cal_result(CAN_CAL_TYPE_HALL, hall_calibrate(report, sizeof report));
}

static void can_dispatch(uint32_t id, const uint8_t *d)
{
    int16_t v = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));

    switch (id) {
    case CAN_ID_CMD:
        switch (d[0]) {
        case CAN_OP_DISABLE:     arm_pos_stop(); foc_disable();     break;
        case CAN_OP_ENABLE:      if (!fault_is_active()) foc_enable(); break;
        case CAN_OP_CLEAR_FAULT: fault_clear();                     break;
        case CAN_OP_SPEED_OFF:   arm_pos_stop(); foc_speed_disable(); foc_set_iq_ref(0.0f); break;
        case CAN_OP_CAL_ADC:     can_run_cal_adc();                 break;
        case CAN_OP_HCAL:        can_run_hcal();                    break;
        case CAN_OP_ARM_OFF:     arm_pos_stop();                    break;
        }
        break;
    case CAN_ID_SET_IQ:
        arm_pos_stop();
        foc_set_iq_ref((float)v / CAN_CURRENT_SCALE);
        break;
    case CAN_ID_SET_SPEED:
        if (!fault_is_active()) {
            arm_pos_stop();
            foc_set_speed_ref(RPM_TO_OMEGA_E((float)v));
        }
        break;
    case CAN_ID_SET_ARM_POS:
        arm_pos_set_target_deg((float)get32(d) / ARM_POS_CMD_SCALE);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Telemetry (MCU → PC)                                                 */
/* ------------------------------------------------------------------ */

static void can_send_status(void)
{
    uint8_t b[8] = {0};
    uint8_t flags = 0;
    if (foc_is_enabled())    flags |= CAN_FLAG_ENABLED;
    if (foc_is_speed_mode()) flags |= CAN_FLAG_SPEED_MODE;
    if (foc_is_block_mode()) flags |= CAN_FLAG_BLOCK_MODE;
    if (fault_is_active())   flags |= CAN_FLAG_FAULT;
    if (arm_pos_is_active()) flags |= CAN_FLAG_ARM_POS;

    b[0] = flags;
    b[1] = (uint8_t)((hall_get_state() & 0x07) | ((hall_get_dir() & 0x03) << 4));
    put16(&b[2], (uint16_t)sat16(OMEGA_E_TO_RPM(hall_get_omega_e())));
    put16(&b[4], (uint16_t)sat16(foc_get_iq_ref() * CAN_CURRENT_SCALE));
    put16(&b[6], (uint16_t)sat16(foc_get_iq()     * CAN_CURRENT_SCALE));
    can_tx(CAN_ID_STATUS, b);
}

static void can_send_currents(void)
{
    uint8_t b[8] = {0};
    put16(&b[0], (uint16_t)sat16(foc_get_ia() * CAN_CURRENT_SCALE));
    put16(&b[2], (uint16_t)sat16(foc_get_ib() * CAN_CURRENT_SCALE));
    put16(&b[4], (uint16_t)sat16(foc_get_ic() * CAN_CURRENT_SCALE));
    put16(&b[6], (uint16_t)(hall_get_theta_e() * CAN_ANGLE_CDEG_SCALE));
    can_tx(CAN_ID_CURRENTS, b);
}

/* TMAG5273 external encoder: angle uint16 cdeg, speed int16 0.1 deg/s,
 * turns int16 saturated (full int32 via CLI `enc`), status byte
 * (bit0 = ok, bits2:1 = variant 1=A1/2=A2), magnitude. */
static void can_send_encoder(void)
{
    uint8_t b[8] = {0};
    put16(&b[0], (uint16_t)lrintf(encoder_get_angle_deg() * CAN_ENC_ANGLE_SCALE));
    put16(&b[2], (uint16_t)sat16(encoder_get_speed_dps() * CAN_ENC_SPEED_SCALE));
    put16(&b[4], (uint16_t)sat16((float)encoder_get_turns()));
    b[6] = (uint8_t)((encoder_is_ok() ? 1u : 0u)
                   | ((encoder_get_variant() & 0x3u) << 1));
    b[7] = encoder_get_magnitude();
    can_tx(CAN_ID_ENCODER, b);
}

static void can_send_arm_status(void)
{
    uint8_t b[8] = {0};
    b[0] = (uint8_t)arm_pos_get_status();
    b[1] = (uint8_t)((arm_pos_is_active() ? 1u : 0u) |
                     (encoder_is_ok() ? 2u : 0u));
    put16(&b[2], (uint16_t)sat16(arm_pos_get_target_deg() * ARM_POS_TLM_SCALE));
    put16(&b[4], (uint16_t)sat16(encoder_get_total_deg() * ARM_POS_TLM_SCALE));
    put16(&b[6], (uint16_t)sat16(arm_pos_get_output_rpm() * ARM_POS_TLM_SCALE));
    can_tx(CAN_ID_ARM_STATUS, b);
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void can_init(void)
{
    /* Accept the control ID block (0x100..0x10F) into RX FIFO0, reject rest. */
    FDCAN_FilterTypeDef f = {0};
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = CAN_CTRL_BASE;
    f.FilterID2    = CAN_CTRL_MASK;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_Start(&hfdcan1);
}

void can_poll(void)
{
    /* Drain any received command frames. */
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)) {
        FDCAN_RxHeaderTypeDef rx;
        uint8_t d[8] = {0};
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, d) != HAL_OK)
            break;
        can_dispatch(rx.Identifier, d);
    }

    /* Broadcast telemetry at CAN_TLM_PERIOD_MS. */
    static uint32_t last;
    uint32_t now = HAL_GetTick();
    if (now - last >= CAN_TLM_PERIOD_MS) {
        last = now;
        can_send_status();
        can_send_currents();
        can_send_encoder();
        can_send_arm_status();
    }
}
