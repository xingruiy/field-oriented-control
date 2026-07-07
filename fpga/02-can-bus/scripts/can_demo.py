#!/usr/bin/env python3
"""CANable demo client for the 02-can-bus FPGA design.

The script sends a small motor-command sequence and plots telemetry returned by
the FPGA. It uses python-can over SocketCAN, so bring the CANable up as can0
before running this script.
"""

from __future__ import annotations

import argparse
import collections
import math
import signal
import struct
import sys
import time
from dataclasses import dataclass

try:
    import can
except ImportError as exc:  # pragma: no cover - user environment dependency
    raise SystemExit("python-can is required: python3 -m pip install python-can") from exc


ID_CMD = 0x101
ID_TELEM = 0x201

OP_SET_ENABLE = 0x01
OP_SET_MODE = 0x02
OP_SET_SPEED = 0x03
OP_SET_CURRENT = 0x04

MODE_SPEED = 0
MODE_CURRENT = 1

MUX_SPEED = 0x10
MUX_CURRENT = 0x11
MUX_STATUS = 0x12


@dataclass
class Telemetry:
    t: float
    speed_rpm: int | None = None
    current_ma: int | None = None
    enable: bool | None = None
    mode: int | None = None
    heartbeat: int | None = None


class Demo:
    def __init__(self, channel: str, bustype: str) -> None:
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)
        self.t0 = time.monotonic()
        self.latest = Telemetry(t=0.0)
        self.samples: collections.deque[Telemetry] = collections.deque(maxlen=1000)

    def close(self) -> None:
        self.bus.shutdown()

    def send_cmd(self, opcode: int, arg: int = 0) -> None:
        if not -32768 <= arg <= 32767:
            raise ValueError(f"argument out of int16 range: {arg}")
        data = bytes([opcode]) + struct.pack(">h", arg) + b"\x00"
        self.bus.send(can.Message(arbitration_id=ID_CMD, data=data, is_extended_id=False))

    def enable(self, on: bool) -> None:
        self.bus.send(
            can.Message(
                arbitration_id=ID_CMD,
                data=bytes([OP_SET_ENABLE, 1 if on else 0, 0, 0]),
                is_extended_id=False,
            )
        )

    def mode(self, mode: int) -> None:
        self.bus.send(
            can.Message(
                arbitration_id=ID_CMD,
                data=bytes([OP_SET_MODE, mode & 1, 0, 0]),
                is_extended_id=False,
            )
        )

    def read_once(self, timeout: float = 0.05) -> Telemetry | None:
        msg = self.bus.recv(timeout)
        if msg is None or msg.arbitration_id != ID_TELEM or msg.is_extended_id or len(msg.data) < 4:
            return None

        mux = msg.data[0]
        value = struct.unpack(">h", bytes(msg.data[1:3]))[0]
        now = time.monotonic() - self.t0

        if mux == MUX_SPEED:
            self.latest.speed_rpm = value
        elif mux == MUX_CURRENT:
            self.latest.current_ma = value
        elif mux == MUX_STATUS:
            flags = msg.data[1]
            self.latest.mode = (flags >> 1) & 1
            self.latest.enable = bool(flags & 1)
            self.latest.heartbeat = (msg.data[2] << 8) | msg.data[3]
        else:
            return None

        snap = Telemetry(
            t=now,
            speed_rpm=self.latest.speed_rpm,
            current_ma=self.latest.current_ma,
            enable=self.latest.enable,
            mode=self.latest.mode,
            heartbeat=self.latest.heartbeat,
        )
        self.samples.append(snap)
        return snap

    def command_profile(self, elapsed: float) -> None:
        phase = int(elapsed // 4.0) % 4
        if phase == 0:
            self.mode(MODE_SPEED)
            self.send_cmd(OP_SET_SPEED, 1500)
        elif phase == 1:
            self.mode(MODE_SPEED)
            self.send_cmd(OP_SET_SPEED, -1000)
        elif phase == 2:
            self.mode(MODE_CURRENT)
            self.send_cmd(OP_SET_CURRENT, 800)
        else:
            self.mode(MODE_CURRENT)
            self.send_cmd(OP_SET_CURRENT, -600)


def run_text(demo: Demo, duration: float) -> None:
    demo.enable(True)
    next_cmd = 0.0
    end = time.monotonic() + duration if duration > 0 else math.inf
    while time.monotonic() < end:
        elapsed = time.monotonic() - demo.t0
        if elapsed >= next_cmd:
            demo.command_profile(elapsed)
            next_cmd = elapsed + 1.0
        snap = demo.read_once(0.1)
        if snap is not None:
            mode = "current" if snap.mode == MODE_CURRENT else "speed"
            print(
                f"t={snap.t:6.2f}s speed={snap.speed_rpm!s:>6} rpm "
                f"current={snap.current_ma!s:>6} mA enable={snap.enable} "
                f"mode={mode:7s} hb={snap.heartbeat}",
                end="\r",
                flush=True,
            )
    print()
    demo.enable(False)


def run_plot(demo: Demo, duration: float) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; falling back to text view", file=sys.stderr)
        run_text(demo, duration)
        return

    demo.enable(True)
    plt.ion()
    fig, (ax_speed, ax_current) = plt.subplots(2, 1, sharex=True)
    (speed_line,) = ax_speed.plot([], [], label="speed rpm")
    (current_line,) = ax_current.plot([], [], label="current mA", color="tab:red")
    ax_speed.set_ylabel("rpm")
    ax_current.set_ylabel("mA")
    ax_current.set_xlabel("seconds")
    ax_speed.grid(True)
    ax_current.grid(True)
    ax_speed.legend(loc="upper right")
    ax_current.legend(loc="upper right")

    next_cmd = 0.0
    next_draw = 0.0
    end = time.monotonic() + duration if duration > 0 else math.inf
    while time.monotonic() < end and plt.fignum_exists(fig.number):
        elapsed = time.monotonic() - demo.t0
        if elapsed >= next_cmd:
            demo.command_profile(elapsed)
            next_cmd = elapsed + 1.0

        demo.read_once(0.02)
        if elapsed >= next_draw and demo.samples:
            xs = [s.t for s in demo.samples]
            ys_speed = [s.speed_rpm if s.speed_rpm is not None else math.nan for s in demo.samples]
            ys_current = [s.current_ma if s.current_ma is not None else math.nan for s in demo.samples]
            speed_line.set_data(xs, ys_speed)
            current_line.set_data(xs, ys_current)
            for ax in (ax_speed, ax_current):
                ax.relim()
                ax.autoscale_view()
            fig.canvas.draw_idle()
            plt.pause(0.001)
            next_draw = elapsed + 0.1

    demo.enable(False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0", help="SocketCAN channel, default can0")
    parser.add_argument("--bustype", default="socketcan", help="python-can bus type")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds; <=0 runs until interrupted")
    parser.add_argument("--text", action="store_true", help="print telemetry instead of plotting")
    args = parser.parse_args()

    demo = Demo(args.channel, args.bustype)

    def stop(_signum, _frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop)
    try:
        if args.text:
            run_text(demo, args.duration)
        else:
            run_plot(demo, args.duration)
    except KeyboardInterrupt:
        demo.enable(False)
    finally:
        demo.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
