#!/usr/bin/env python3
"""Dump the firmware black-box recorder (src/common/bbox.c) over ST-LINK.

Reads the g_bbox capture buffer from target RAM with STM32_Programmer_CLI
(address taken from the build .map), decodes the 8 int16 channels, and writes
  debugging/captures/bbox_<timestamp>.csv   (raw, PlotJuggler-friendly)
  debugging/captures/bbox_<timestamp>.html  (interactive Plotly viewer)

Typical use: `bb arm` on the CLI, reproduce (fault/OC/omega trip auto-freezes,
or `bb trig`), then:  python3 tools/bbox/bbox_dump.py [--shared]
Use --shared while a CubeIDE debug session holds the ST-LINK.
"""

import argparse
import glob
import re
import struct
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

ROOT = Path(__file__).resolve().parents[2]

BBOX_MAGIC = 0x31584242  # "BBX1"
STATE_NAMES = {0: "idle", 1: "armed", 2: "triggered", 3: "frozen"}
HDR_FMT = "<8I"  # magic, state, widx, nsamp, trig_idx, len, nch, tick_hz
HDR_SIZE = struct.calcsize(HDR_FMT)
CLI_GLOB = ("/opt/st/stm32cubeide_*/plugins/"
            "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer*/tools/bin/"
            "STM32_Programmer_CLI")


def find_programmer(explicit):
    if explicit:
        return explicit
    hits = sorted(glob.glob(CLI_GLOB))
    if not hits:
        sys.exit(f"STM32_Programmer_CLI not found (searched {CLI_GLOB}); pass --cli")
    return hits[-1]


def find_symbol(map_path, name="g_bbox"):
    """Return the address of .bss.<name>/.data.<name> from a GNU ld map file.
    Handles the two-line form used when the section name is long."""
    text = Path(map_path).read_text(errors="replace")
    pat = re.compile(r"\.(?:bss|data)\." + re.escape(name) +
                     r"\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)", re.S)
    m = pat.search(text)
    if not m:
        sys.exit(f"symbol {name} not found in {map_path} — rebuild the CM7 firmware?")
    return int(m.group(1), 16), int(m.group(2), 16)


def read_target(cli, addr, size, shared):
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        out = f.name
    connect = ["-c", "port=SWD", "mode=HOTPLUG"] + (["shared"] if shared else [])
    cmd = [cli] + connect + ["-u", f"0x{addr:08X}", str(size), out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    data = Path(out).read_bytes() if Path(out).exists() else b""
    Path(out).unlink(missing_ok=True)
    if r.returncode != 0 or len(data) < size:
        sys.exit(f"target read failed:\n{r.stdout}\n{r.stderr}")
    return data


def decode(raw):
    magic, state, widx, nsamp, trig_idx, length, nch, tick_hz = \
        struct.unpack_from(HDR_FMT, raw)
    if magic != BBOX_MAGIC:
        sys.exit(f"bad magic 0x{magic:08X} — wrong address or stale .map")
    samp = np.frombuffer(raw, dtype="<i2", offset=HDR_SIZE,
                         count=length * nch).reshape(length, nch)
    if nsamp < length:                      # never wrapped: chronological already
        ordered, trig_pos = samp[:nsamp], trig_idx
    else:                                   # unroll the ring
        ordered = np.concatenate([samp[widx:], samp[:widx]])
        trig_pos = (trig_idx - widx) % length
    triggered = state in (2, 3)
    t_ms = (np.arange(len(ordered)) - (trig_pos if triggered else 0)) \
        * 1000.0 / tick_hz
    df = pd.DataFrame({
        "t_ms":        t_ms,
        "theta_deg":   ordered[:, 0].astype(np.uint16) * (360.0 / 65536.0),
        "omega_rad_s": ordered[:, 1].astype(float),
        "innov_deg":   ordered[:, 2] / 100.0,
        "iq_A":        ordered[:, 3] / 1000.0,
        "id_A":        ordered[:, 4] / 1000.0,
        "iqref_A":     ordered[:, 5] / 1000.0,
        "sector":      ordered[:, 6],
        "vq_V":        ordered[:, 7] / 1000.0,
    })
    return df, state, triggered


def render(df, triggered, html_path):
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True,
                        vertical_spacing=0.04,
                        specs=[[{}], [{"secondary_y": True}], [{}]],
                        subplot_titles=("theta_hat + hall sector",
                                        "omega_hat / per-edge innovation",
                                        "currents"))
    x = df["t_ms"]

    def gl(y, name, **kw):
        return go.Scattergl(x=x, y=y, name=name, line=dict(width=1), **kw)

    fig.add_trace(gl(df["theta_deg"], "theta_hat [deg]"), row=1, col=1)
    fig.add_trace(gl(df["sector"] * 60, "sector (x60 deg)", line_shape="hv"),
                  row=1, col=1)
    fig.add_trace(gl(df["omega_rad_s"], "omega_hat [rad/s]"), row=2, col=1)
    fig.add_trace(gl(df["innov_deg"], "innovation [deg]"), row=2, col=1,
                  secondary_y=True)
    for c, n in [("iq_A", "Iq [A]"), ("id_A", "Id [A]"),
                 ("iqref_A", "Iq_ref [A]"), ("vq_V", "Vq [V]")]:
        fig.add_trace(gl(df[c], n), row=3, col=1)
    if triggered:
        fig.add_vline(x=0, line_dash="dash", line_color="red",
                      annotation_text="trigger")
    fig.update_xaxes(title_text="t [ms] (0 = trigger)", row=3, col=1)
    fig.update_layout(height=900, hovermode="x", dragmode="zoom",
                      title=html_path.stem)
    fig.write_html(html_path)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", default=ROOT / "CM7/Debug/bldc_CM7.map")
    ap.add_argument("--cli", help="path to STM32_Programmer_CLI")
    ap.add_argument("--shared", action="store_true",
                    help="share the ST-LINK with a live CubeIDE debug session")
    ap.add_argument("--out", default=ROOT / "debugging/captures")
    args = ap.parse_args()

    cli = find_programmer(args.cli)
    addr, size = find_symbol(args.map)
    print(f"g_bbox @ 0x{addr:08X}  ({size} bytes)")

    raw = read_target(cli, addr, size, args.shared)
    df, state, triggered = decode(raw)
    print(f"state={STATE_NAMES.get(state, state)}  samples={len(df)}"
          + ("" if state == 3 else "  (warning: not frozen — data may be mid-write)"))

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    stem = outdir / f"bbox_{datetime.now():%Y%m%d_%H%M%S}"
    df.to_csv(stem.with_suffix(".csv"), index=False)
    render(df, triggered, stem.with_suffix(".html"))
    print(f"wrote {stem}.csv and {stem}.html")


if __name__ == "__main__":
    main()
