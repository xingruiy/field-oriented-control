# FOC CAN Console (GUI)

Desktop control & telemetry front end for the STM32H755 BLDC board over CAN.
PyQt5 + matplotlib GUI, python-can backend.

## Protocol
`foc_can.py` mirrors `src/common/settings.h` — keep the two in lockstep.

| Dir | ID | Payload |
|-----|-----|---------|
| PC→MCU | `0x100` | b0 opcode: 0 disable, 1 enable, 2 clear-fault, 3 speed-off, 4 cal, 5 hcal |
| PC→MCU | `0x101` | int16 LE, Iq in **mA** |
| PC→MCU | `0x102` | int16 LE, speed in **RPM** |
| MCU→PC | `0x200` | flags, hall\|dir, speed_rpm, iq_ref_mA, iq_mA (100 Hz) |
| MCU→PC | `0x201` | Ia_mA, Ib_mA, Ic_mA, theta_e_centideg |
| MCU→PC | `0x202` | cal result (one-shot): type, ok, off_a/b/c |
| MCU→PC | `0x203` | TMAG5273 encoder: angle_centideg u16, speed_0.1dps i16, turns i16, status (b0 ok, b2:1 variant), magnitude |

## Install
```bash
python3 -m pip install -r requirements.txt   # python-can (PyQt5/matplotlib already present)
```

## Run
```bash
python3 foc_can_gui.py                        # defaults: slcan /dev/ttyACM1 @ 500000
python3 foc_can_gui.py --demo                 # synthetic telemetry, no hardware
python3 foc_can_gui.py --backend socketcan --channel can0
```

### CANable in slcan mode without slcand (default — `/dev/ttyACM1`)
python-can's `slcan` backend can open the serial device directly — no `slcand`/`ip link`,
no can-utils. Ensure port access (`sudo usermod -aG dialout $USER`, re-login), then pick
backend **slcan**, channel **/dev/ttyACM1**, bitrate **500000**.

### Native SocketCAN (`can0`)
Bring the interface up once, then select **socketcan** in the GUI:
```bash
sudo ip link set can0 up type can bitrate 500000
```
Pick backend **socketcan**, channel **can0**, bitrate **500000**. If the CANable is in
slcan mode it first needs `slcand` to create `can0`:
```bash
sudo slcand -o -s6 -t hw /dev/ttyACM1 can0 && sudo ip link set can0 up   # -s6 = 500 kbps
```

## Using it
**Control tab**
- **DISABLE (E-STOP)** / **ENABLE** / **Clear fault** — motor on/off + fault reset.
- **Iq** spinbox + *Set Iq* — torque (current) command in A.
- **Spd** spinbox + *Set spd* / *Spd off* — closed-loop speed in RPM (engages speed mode).

**Calibration tab**
- **Calibrate ADC offsets (cal)** and **Calibrate Hall angles (hcal)** — trigger the same
  routines as the CLI `cal`/`hcal`. FOC must be disabled and the rotor free; hcal drives
  open-loop for ~5 s (telemetry pauses). Result (ok + ADC offsets) shows in the tab; the
  full report prints on the USART3 CLI.

Live plots: measured vs commanded speed, Iq ref vs measured, phase currents Ia/Ib/Ic,
and the TMAG5273 external encoder telemetry (angle + speed; OK/variant/turns/magnitude
in the telemetry panel). TMAG telemetry is never used as a motor speed reference.
The link auto-reconnects if the adapter drops. Cross-check readings against the USART3
CLI (`status`, `idq`, `hall`).
