#include "encoder/encoder.h"
#include "common/settings.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

/* TMAG5273 register map (datasheets/tmag5273.pdf, table 8-1) */
#define TMAG_DEVICE_CONFIG_1    0x00    /* CRC[7] TEMPCO[6:5] CONV_AVG[4:2] I2C_RD[1:0] */
#define TMAG_DEVICE_CONFIG_2    0x01    /* OPERATING_MODE[1:0]: 2h = continuous */
#define TMAG_SENSOR_CONFIG_1    0x02    /* MAG_CH_EN[7:4] SLEEPTIME[3:0]        */
#define TMAG_SENSOR_CONFIG_2    0x03    /* ANGLE_EN[3:2] X_Y_RANGE[1] Z_RANGE[0]*/
#define TMAG_DEVICE_ID          0x0D    /* VER[1:0]: 1h=A1 (40/80mT), 2h=A2     */
#define TMAG_MANUFACTURER_LSB   0x0E    /* 0x49, then 0x54 at 0x0F ("IT")       */
#define TMAG_ANGLE_MSB          0x19    /* deg*16 in [12:0]; 0x1A LSB, 0x1B mag */
#define TMAG_OP_CONTINUOUS      0x2

static bool     s_ok;
static float    s_angle_deg, s_speed_dps;
static int32_t  s_turns;
static uint8_t  s_variant;              /* DEVICE_ID VER: 1=A1, 2=A2, 0=unknown */
static uint8_t  s_magnitude;
static uint32_t s_err_count, s_consec_err;
static uint32_t s_last_ms, s_reinit_ms;

static bool wr(uint8_t reg, uint8_t v)
{
    return HAL_I2C_Mem_Write(&hi2c1, ENC_I2C_ADDR << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, &v, 1,
                             ENC_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool rd(uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(&hi2c1, ENC_I2C_ADDR << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, n,
                            ENC_I2C_TIMEOUT_MS) == HAL_OK;
}

static float read_angle_deg(bool *ok)
{
    uint8_t b[3];                       /* ANGLE_MSB, ANGLE_LSB, MAGNITUDE */
    *ok = rd(TMAG_ANGLE_MSB, b, 3);
    if (!*ok) return s_angle_deg;
    s_magnitude = b[2];
    return (float)((((uint16_t)b[0] << 8) | b[1]) & 0x1FFF) / 16.0f;
}

void encoder_init(void)
{
    s_ok = false;
    uint8_t id[2], ver;
    if (!rd(TMAG_MANUFACTURER_LSB, id, 2) || id[0] != 0x49 || id[1] != 0x54)
        return;                         /* absent/wrong chip: report not-ok */
    if (!rd(TMAG_DEVICE_ID, &ver, 1))
        return;
    s_variant = ver & 0x3;

    /* Per-variant XY range: the EVM magnet exceeds the A1 40 mT range. */
    uint8_t xy_range = (s_variant == 2) ? ENC_XY_RANGE_A2 : ENC_XY_RANGE_A1;
    bool ok = wr(TMAG_DEVICE_CONFIG_1, ENC_CONV_AVG << 2)
           && wr(TMAG_SENSOR_CONFIG_1, ENC_MAG_CH_EN << 4)
           && wr(TMAG_SENSOR_CONFIG_2, (ENC_ANGLE_EN << 2) | (xy_range << 1))
           && wr(TMAG_DEVICE_CONFIG_2, TMAG_OP_CONTINUOUS);  /* mode last */
    if (!ok) return;

    HAL_Delay(5);                       /* first conversions complete */
    s_angle_deg = read_angle_deg(&ok);  /* seed: no phantom first turn/speed */
    if (!ok) return;

    s_speed_dps  = 0.0f;
    s_consec_err = 0;
    s_last_ms    = HAL_GetTick();
    s_ok         = true;
}

void encoder_poll(void)
{
    uint32_t now = HAL_GetTick();
    if (!s_ok) {                        /* periodic re-probe (hot-plug)     */
        if (now - s_reinit_ms >= ENC_REINIT_MS) {
            s_reinit_ms = now;
            encoder_init();
        }
        return;
    }
    if (now - s_last_ms < ENC_POLL_PERIOD_MS) return;
    float dt = (float)(now - s_last_ms) * 1e-3f;
    s_last_ms = now;

    bool  ok;
    float ang = read_angle_deg(&ok);
    if (!ok) {
        s_err_count++;
        if (++s_consec_err >= ENC_ERR_LIMIT) s_ok = false;
        return;
    }
    s_consec_err = 0;

    /* Unwrap: shortest path across the 0/360 seam counts a turn. Valid while
     * the object moves < 180 deg per poll (36000 deg/s at 5 ms — ample). */
    float d = ang - s_angle_deg;
    if      (d >  180.0f) { d -= 360.0f; s_turns--; }
    else if (d < -180.0f) { d += 360.0f; s_turns++; }
    s_angle_deg = ang;

    s_speed_dps += ENC_SPEED_LPF_ALPHA * (d / dt - s_speed_dps);
}

bool     encoder_is_ok(void)         { return s_ok; }
float    encoder_get_angle_deg(void) { return s_angle_deg; }
float    encoder_get_total_deg(void) { return (float)s_turns * 360.0f + s_angle_deg; }
int32_t  encoder_get_turns(void)     { return s_turns; }
float    encoder_get_speed_dps(void) { return s_speed_dps; }
uint8_t  encoder_get_magnitude(void) { return s_magnitude; }
uint8_t  encoder_get_variant(void)   { return s_variant; }
uint32_t encoder_get_err_count(void) { return s_err_count; }
void     encoder_reset_turns(void)   { s_turns = 0; }
