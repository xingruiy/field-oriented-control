# Black-box dump tool

PC side of the firmware black-box recorder (`src/common/bbox.c`): pulls the
frozen 40 kHz capture out of target RAM over ST-LINK and renders it.

## Flow

1. CLI: `bb arm`
2. Reproduce. Freeze happens on fault/OC (`foc_emergency_stop`), on
   `|omega_hat| > BBOX_TRIP_OMEGA`, or manually with `bb trig`.
   `bb` shows the state (`frozen` = ready).
3. `python3 tools/bbox/bbox_dump.py` → writes
   `debugging/captures/bbox_<ts>.csv` + `.html`.

Open the HTML in a browser: wheel/box zoom down to single 25 µs ticks, hover
readouts, red dashed line = trigger (t = 0). The CSV loads directly into
PlotJuggler if preferred.

## Options

- `--shared` — share the ST-LINK with a running CubeIDE debug session.
- `--map <file>` — .map to resolve the `g_bbox` address
  (default `CM7/Debug/bldc_CM7.map`; must match the flashed firmware).
- `--cli <path>` — STM32_Programmer_CLI (default: auto-glob under
  `/opt/st/stm32cubeide_*`).

## Channels

| col | signal | scaling on wire |
|---|---|---|
| theta_deg | Hall observer θ̂ | u16 = rad·65536/2π |
| omega_rad_s | ω̂ (θe-frame, raw) | int16 rad/s |
| innov_deg | per-edge innovation | int16 centideg |
| iq_A / id_A / iqref_A / ia_A | currents | int16 mA |
| sector | Hall sector | int16 |

Requires: python3 with numpy, pandas, plotly (all already installed);
STM32CubeIDE's bundled STM32_Programmer_CLI.
