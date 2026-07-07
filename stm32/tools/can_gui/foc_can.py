"""FOC CAN protocol codec (classic CAN, 500 kbps, 11-bit IDs).

Mirror of src/common/settings.h — keep in lockstep with the firmware.
Pure stdlib (struct only); no Qt / python-can dependency so it stays testable.
"""
import struct

# --- Control frames (PC -> MCU) ---
ID_CMD        = 0x100   # b0 = opcode below
ID_SET_IQ     = 0x101   # int16 LE, mA
ID_SET_SPEED  = 0x102   # int16 LE, RPM
OP_DISABLE     = 0
OP_ENABLE      = 1
OP_CLEAR_FAULT = 2
OP_SPEED_OFF   = 3
OP_CAL_ADC     = 4   # ADC offset calibration (`cal`)
OP_HCAL        = 5   # Hall angle calibration (`hcal`)

# --- Telemetry frames (MCU -> PC) ---
ID_STATUS     = 0x200
ID_CURRENTS   = 0x201
ID_CAL_RESULT = 0x202
ID_ENCODER    = 0x203   # TMAG5273 external angle encoder
CAL_TYPE_ADC  = 1
CAL_TYPE_HALL = 2

# --- Status flag bits (byte 0 of ID_STATUS) ---
FLAG_ENABLED    = 1 << 0
FLAG_SPEED_MODE = 1 << 1
FLAG_BLOCK_MODE = 1 << 2
FLAG_FAULT      = 1 << 3

CURRENT_SCALE   = 1000.0   # A  -> mA (int16)
ANGLE_SCALE     = 100.0    # deg -> centideg (uint16)
ENC_SPEED_SCALE = 10.0     # deg/s -> 0.1 deg/s (int16)
ENC_VARIANTS    = {1: "A1", 2: "A2"}   # DEVICE_ID VER bits


def _sat16(v):
    return max(-32768, min(32767, int(round(v))))


# ---------------- encoders: return (can_id, bytes) ----------------

def enc_cmd(op):
    return ID_CMD, bytes([op & 0xFF])


def enc_iq(amps):
    return ID_SET_IQ, struct.pack("<h", _sat16(amps * CURRENT_SCALE))


def enc_speed(rpm):
    return ID_SET_SPEED, struct.pack("<h", _sat16(rpm))


# ---------------- decoders: bytes -> dict ----------------

def dec_status(data):
    flags, b1, speed, iq_ref, iq = struct.unpack("<BBhhh", bytes(data[:8]))
    raw_dir = (b1 >> 4) & 0x03
    return {
        "enabled":    bool(flags & FLAG_ENABLED),
        "speed_mode": bool(flags & FLAG_SPEED_MODE),
        "block_mode": bool(flags & FLAG_BLOCK_MODE),
        "fault":      bool(flags & FLAG_FAULT),
        "hall":       b1 & 0x07,
        "dir":        -1 if raw_dir == 3 else raw_dir,
        "speed_rpm":  speed,
        "iq_ref_a":   iq_ref / CURRENT_SCALE,
        "iq_a":       iq / CURRENT_SCALE,
    }


def dec_currents(data):
    ia, ib, ic, cdeg = struct.unpack("<hhhH", bytes(data[:8]))
    return {
        "ia_a":      ia / CURRENT_SCALE,
        "ib_a":      ib / CURRENT_SCALE,
        "ic_a":      ic / CURRENT_SCALE,
        "theta_deg": cdeg / ANGLE_SCALE,
    }


def dec_cal_result(data):
    typ, ok, oa, ob, oc = struct.unpack("<BBHHH", bytes(data[:8]))
    return {"type": typ, "ok": bool(ok), "off_a": oa, "off_b": ob, "off_c": oc}


def dec_encoder(data):
    cdeg, speed, turns, status, mag = struct.unpack("<HhhBB", bytes(data[:8]))
    return {
        "ok":        bool(status & 0x01),
        "variant":   ENC_VARIANTS.get((status >> 1) & 0x03, "?"),
        "angle_deg": cdeg / ANGLE_SCALE,
        "speed_dps": speed / ENC_SPEED_SCALE,
        "turns":     turns,          # int16-saturated; full int32 on the CLI
        "magnitude": mag,
    }


if __name__ == "__main__":
    # round-trip / packing self-test
    assert enc_cmd(OP_ENABLE) == (0x100, b"\x01")
    assert enc_iq(0.2) == (0x101, b"\xc8\x00")        # 200 mA
    assert enc_speed(300) == (0x102, b"\x2c\x01")     # 0x012C
    assert enc_iq(-0.2) == (0x101, b"\x38\xff")       # -200 mA

    st = struct.pack("<BBhhh", FLAG_ENABLED | FLAG_SPEED_MODE,
                     (0x03 << 4) | 0x05, 300, 250, 240)
    d = dec_status(st)
    assert d["enabled"] and d["speed_mode"] and not d["fault"]
    assert d["hall"] == 5 and d["dir"] == -1 and d["speed_rpm"] == 300
    assert abs(d["iq_ref_a"] - 0.25) < 1e-9 and abs(d["iq_a"] - 0.24) < 1e-9

    cur = struct.pack("<hhhH", 100, -50, -50, 9000)
    c = dec_currents(cur)
    assert abs(c["ia_a"] - 0.1) < 1e-9 and abs(c["theta_deg"] - 90.0) < 1e-9

    cr = dec_cal_result(struct.pack("<BBHHH", CAL_TYPE_ADC, 1, 32770, 32760, 32750))
    assert cr["type"] == CAL_TYPE_ADC and cr["ok"] and cr["off_a"] == 32770

    en = dec_encoder(struct.pack("<HhhBB", 12345, -3600, -3, (2 << 1) | 1, 87))
    assert en["ok"] and en["variant"] == "A2" and en["turns"] == -3
    assert abs(en["angle_deg"] - 123.45) < 1e-9 and abs(en["speed_dps"] + 360.0) < 1e-9
    assert en["magnitude"] == 87

    print("foc_can self-test OK")
