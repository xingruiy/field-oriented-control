#!/usr/bin/env python3
"""FOC CAN control & telemetry GUI (PyQt5 + matplotlib, python-can backend).

Talks to the STM32H755 BLDC board over CAN (see foc_can.py / firmware settings.h).
Run:  python3 foc_can_gui.py [--backend slcan] [--channel /dev/ttyACM1] [--demo]
"""
import argparse
import math
import sys
import time
from collections import deque

from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

import foc_can as proto

WINDOW_S = 10.0          # rolling plot window
BUF = int(WINDOW_S * 120) + 200
UI_SAMPLE_S = 1.0 / 50.0
PLOT_SMOOTH_ALPHA = 0.12
YLIM_PAD = 0.12
YLIM_HYST = 0.25


# --------------------------------------------------------------------------
# Bus thread: owns the python-can bus, auto-reconnects, decodes telemetry.
# In --demo/'virtual' mode it synthesizes telemetry with no hardware.
# --------------------------------------------------------------------------
class RxThread(QtCore.QThread):
    link     = QtCore.pyqtSignal(bool, str)
    status   = QtCore.pyqtSignal(dict)
    currents = QtCore.pyqtSignal(dict)
    cal      = QtCore.pyqtSignal(dict)
    encoder  = QtCore.pyqtSignal(dict)

    def __init__(self, backend, channel, bitrate, reconnect=True):
        super().__init__()
        self.backend, self.channel, self.bitrate = backend, channel, int(bitrate)
        self.reconnect = reconnect
        self._running = True
        self._bus = None
        self._lock = QtCore.QMutex()
        self._latest = {"status": None, "currents": None, "encoder": None, "ref": None}
        self._last_emit = 0.0

    def run(self):
        if self.backend == "demo":
            self._run_demo()
            return
        import can
        while self._running:
            try:
                self._bus = can.Bus(interface=self.backend, channel=self.channel,
                                    bitrate=self.bitrate)
                self.link.emit(True, f"{self.backend}:{self.channel} @ {self.bitrate}")
                while self._running:
                    msg = self._bus.recv(timeout=0.2)
                    if msg is not None:
                        self._dispatch(msg.arbitration_id, msg.data)
                    self._flush_latest()
            except Exception as e:                       # noqa: BLE001
                self.link.emit(False, str(e))
            finally:
                self._close_bus()
            if not (self._running and self.reconnect):
                break
            time.sleep(1.0)
        self.link.emit(False, "disconnected")

    def _dispatch(self, can_id, data):
        if can_id == proto.ID_STATUS and len(data) >= 8:
            self._latest["status"] = proto.dec_status(data)
        elif can_id == proto.ID_CURRENTS and len(data) >= 8:
            self._latest["currents"] = proto.dec_currents(data)
        elif can_id == proto.ID_CAL_RESULT and len(data) >= 8:
            self.cal.emit(proto.dec_cal_result(data))
        elif can_id == proto.ID_ENCODER and len(data) >= 8:
            self._latest["encoder"] = proto.dec_encoder(data)

    def _flush_latest(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_emit < UI_SAMPLE_S:
            return
        self._last_emit = now
        for name, signal in (("status", self.status),
                             ("currents", self.currents),
                             ("encoder", self.encoder)):
            value = self._latest[name]
            if value is not None:
                signal.emit(value)
                self._latest[name] = None

    def _run_demo(self):
        self.link.emit(True, "demo (synthetic telemetry)")
        t0 = time.monotonic()
        while self._running:
            t = time.monotonic() - t0
            spd = 300 * math.sin(t * 0.5)
            iq = 0.3 * math.sin(t * 0.5 + 0.3)
            self.status.emit({"enabled": True, "speed_mode": True, "block_mode": False,
                              "fault": False, "hall": int(t * 6) % 6 + 1,
                              "dir": 1 if spd >= 0 else -1, "speed_rpm": int(spd),
                              "iq_ref_a": 0.3, "iq_a": iq})
            ph = t * 4.0
            self.currents.emit({"ia_a": iq * math.cos(ph),
                                "ib_a": iq * math.cos(ph - 2.094),
                                "ic_a": iq * math.cos(ph + 2.094),
                                "theta_deg": (math.degrees(ph)) % 360})
            enc_deg = 90.0 * t
            self.encoder.emit({"ok": True, "variant": "A1",
                               "angle_deg": enc_deg % 360.0,
                               "speed_dps": 90.0 + 10.0 * math.sin(t),
                               "turns": int(enc_deg // 360.0),
                               "magnitude": 87})
            time.sleep(UI_SAMPLE_S)
        self.link.emit(False, "disconnected")

    def send(self, can_id, data):
        if self.backend == "demo":
            print(f"[demo] would send id=0x{can_id:03X} data={data.hex()}")
            return
        with QtCore.QMutexLocker(self._lock):
            if self._bus is None:
                return
            try:
                import can
                self._bus.send(can.Message(arbitration_id=can_id, data=data,
                                           is_extended_id=False))
            except Exception as e:                       # noqa: BLE001
                print("send failed:", e)

    def _close_bus(self):
        with QtCore.QMutexLocker(self._lock):
            if self._bus is not None:
                try:
                    self._bus.shutdown()
                except Exception:                        # noqa: BLE001
                    pass
                self._bus = None

    def stop(self):
        self._running = False
        self.wait(2000)


# --------------------------------------------------------------------------
# Rolling matplotlib panel (speed, Iq, phase currents, external encoder).
# --------------------------------------------------------------------------
class Plots(FigureCanvasQTAgg):
    def __init__(self):
        fig = Figure(figsize=(5, 6), tight_layout=True)
        super().__init__(fig)
        self.ax_s, self.ax_i, self.ax_p, self.ax_e = fig.subplots(4, 1, sharex=True)
        self.t0 = time.monotonic()
        self.buf = {k: deque(maxlen=BUF) for k in
                    ("t", "spd", "spd_set", "iqr", "iq", "ia", "ib", "ic", "ea", "es")}
        self.smooth = {}
        self.l_spd,  = self.ax_s.plot([], [], "b-", label="meas")
        self.l_sset, = self.ax_s.plot([], [], "r--", label="set")
        self.l_iqr,  = self.ax_i.plot([], [], "r--", label="Iq ref")
        self.l_iq,   = self.ax_i.plot([], [], "b-", label="Iq")
        self.l_ia,   = self.ax_p.plot([], [], "r-", label="Ia")
        self.l_ib,   = self.ax_p.plot([], [], "g-", label="Ib")
        self.l_ic,   = self.ax_p.plot([], [], "b-", label="Ic")
        self.l_ea,   = self.ax_e.plot([], [], "g-", label="angle (deg)")
        self.l_es,   = self.ax_e.plot([], [], "b-", label="speed (deg/s)")
        self.ax_s.set_ylabel("RPM");  self.ax_i.set_ylabel("Iq (A)")
        self.ax_p.set_ylabel("Iph (A)")
        self.ax_e.set_ylabel("encoder"); self.ax_e.set_xlabel("t (s)")
        for a in (self.ax_s, self.ax_i, self.ax_p, self.ax_e):
            a.grid(True, alpha=0.3); a.legend(loc="upper right", fontsize=8)

    def add_status(self, d, spd_set):
        t = time.monotonic() - self.t0
        self.buf["t"].append(t)
        self.buf["spd"].append(self._smooth("spd", d["speed_rpm"]))
        self.buf["spd_set"].append(spd_set)
        self.buf["iqr"].append(d["iq_ref_a"])
        self.buf["iq"].append(self._smooth("iq", d["iq_a"]))
        # keep the other frames' buffers time-aligned even if they lag a tick
        for k in ("ia", "ib", "ic", "ea", "es"):
            self.buf[k].append(self.buf[k][-1] if self.buf[k] else 0.0)

    def add_currents(self, d):
        if not self.buf["ia"]:
            return
        self.buf["ia"][-1] = self._smooth("ia", d["ia_a"])
        self.buf["ib"][-1] = self._smooth("ib", d["ib_a"])
        self.buf["ic"][-1] = self._smooth("ic", d["ic_a"])

    def add_encoder(self, d):
        if not self.buf["ea"]:
            return
        self.buf["ea"][-1] = d["angle_deg"]
        self.buf["es"][-1] = self._smooth("es", d["speed_dps"])

    def _smooth(self, key, value):
        prev = self.smooth.get(key)
        if prev is None:
            self.smooth[key] = value
        else:
            self.smooth[key] = prev + PLOT_SMOOTH_ALPHA * (value - prev)
        return self.smooth[key]

    def _set_stable_ylim(self, ax, *series):
        vals = [v for s in series for v in s]
        if not vals:
            return
        lo = min(vals); hi = max(vals)
        span = max(hi - lo, 1.0)
        target = (lo - span * YLIM_PAD, hi + span * YLIM_PAD)
        cur = ax.get_ylim()
        cur_span = max(cur[1] - cur[0], 1e-9)
        inside = target[0] >= cur[0] and target[1] <= cur[1]
        close = abs(target[0] - cur[0]) < cur_span * YLIM_HYST and abs(target[1] - cur[1]) < cur_span * YLIM_HYST
        if not (inside and close):
            ax.set_ylim(*target)

    def redraw(self):
        t = self.buf["t"]
        if not t:
            return
        self.l_spd.set_data(t, self.buf["spd"]);  self.l_sset.set_data(t, self.buf["spd_set"])
        self.l_iqr.set_data(t, self.buf["iqr"]);  self.l_iq.set_data(t, self.buf["iq"])
        self.l_ia.set_data(t, self.buf["ia"]);    self.l_ib.set_data(t, self.buf["ib"])
        self.l_ic.set_data(t, self.buf["ic"])
        self.l_ea.set_data(t, self.buf["ea"]);    self.l_es.set_data(t, self.buf["es"])
        now = t[-1]
        for a in (self.ax_s, self.ax_i, self.ax_p, self.ax_e):
            a.set_xlim(max(0, now - WINDOW_S), max(WINDOW_S, now))
        self._set_stable_ylim(self.ax_s, self.buf["spd"], self.buf["spd_set"])
        self._set_stable_ylim(self.ax_i, self.buf["iqr"], self.buf["iq"])
        self._set_stable_ylim(self.ax_p, self.buf["ia"], self.buf["ib"], self.buf["ic"])
        self._set_stable_ylim(self.ax_e, self.buf["ea"], self.buf["es"])
        self.draw_idle()


# --------------------------------------------------------------------------
# Main window
# --------------------------------------------------------------------------
class MainWindow(QtWidgets.QWidget):
    def __init__(self, backend, channel, bitrate):
        super().__init__()
        self.setWindowTitle("FOC CAN Console")
        self.rx = None
        self.spd_set = 0.0
        root = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QVBoxLayout()
        left.addWidget(self._conn_group(backend, channel, bitrate))
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._control_group(), "Control")
        tabs.addTab(self._calibration_group(), "Calibration")
        left.addWidget(tabs)
        left.addWidget(self._telemetry_group())
        left.addStretch(1)
        root.addLayout(left, 0)
        self.plots = Plots()
        root.addWidget(self.plots, 1)

        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.plots.redraw)
        self.timer.start(25)   # ~40 Hz
        self._set_controls_enabled(False)

    # ---- UI groups -------------------------------------------------------
    def _conn_group(self, backend, channel, bitrate):
        g = QtWidgets.QGroupBox("Connection"); f = QtWidgets.QFormLayout(g)
        self.cmb = QtWidgets.QComboBox(); self.cmb.addItems(["slcan", "socketcan", "demo"])
        self.cmb.setCurrentText(backend)
        self.ed_ch = QtWidgets.QLineEdit(channel)
        self.ed_br = QtWidgets.QLineEdit(str(bitrate))
        self.btn_conn = QtWidgets.QPushButton("Connect"); self.btn_conn.clicked.connect(self._toggle_conn)
        self.lbl_link = QtWidgets.QLabel("disconnected"); self.lbl_link.setStyleSheet("color:#a00;")
        f.addRow("backend", self.cmb); f.addRow("channel", self.ed_ch)
        f.addRow("bitrate", self.ed_br); f.addRow(self.btn_conn); f.addRow("status", self.lbl_link)
        return g

    def _control_group(self):
        g = QtWidgets.QWidget(); v = QtWidgets.QVBoxLayout(g)
        self.btn_dis = QtWidgets.QPushButton("DISABLE  (E-STOP)")
        self.btn_dis.setStyleSheet("background:#c0392b;color:white;font-weight:bold;padding:8px;")
        self.btn_dis.clicked.connect(lambda: self._send(proto.enc_cmd(proto.OP_DISABLE)))
        self.btn_en = QtWidgets.QPushButton("ENABLE")
        self.btn_en.clicked.connect(lambda: self._send(proto.enc_cmd(proto.OP_ENABLE)))
        self.btn_clr = QtWidgets.QPushButton("Clear fault")
        self.btn_clr.clicked.connect(lambda: self._send(proto.enc_cmd(proto.OP_CLEAR_FAULT)))
        v.addWidget(self.btn_dis); row = QtWidgets.QHBoxLayout()
        row.addWidget(self.btn_en); row.addWidget(self.btn_clr); v.addLayout(row)

        self.sp_iq = QtWidgets.QDoubleSpinBox(); self.sp_iq.setRange(-1.0, 1.0)
        self.sp_iq.setSingleStep(0.05); self.sp_iq.setDecimals(3); self.sp_iq.setSuffix(" A")
        b_iq = QtWidgets.QPushButton("Set Iq"); b_iq.clicked.connect(self._set_iq)
        r1 = QtWidgets.QHBoxLayout(); r1.addWidget(QtWidgets.QLabel("Iq")); r1.addWidget(self.sp_iq); r1.addWidget(b_iq)
        v.addLayout(r1)

        self.sp_spd = QtWidgets.QDoubleSpinBox(); self.sp_spd.setRange(-10000, 10000)
        self.sp_spd.setSingleStep(50); self.sp_spd.setSuffix(" RPM")
        b_spd = QtWidgets.QPushButton("Set spd"); b_spd.clicked.connect(self._set_spd)
        b_off = QtWidgets.QPushButton("Spd off"); b_off.clicked.connect(self._spd_off)
        r2 = QtWidgets.QHBoxLayout(); r2.addWidget(QtWidgets.QLabel("Spd")); r2.addWidget(self.sp_spd)
        r2.addWidget(b_spd); r2.addWidget(b_off); v.addLayout(r2)
        return g

    def _calibration_group(self):
        g = QtWidgets.QWidget(); v = QtWidgets.QVBoxLayout(g)
        note = QtWidgets.QLabel(
            "FOC must be DISABLED and the rotor free to move.\n"
            "Hall cal drives the motor open-loop for ~5 s; telemetry pauses\n"
            "while it runs. The full report prints on the USART3 CLI.")
        note.setWordWrap(True); note.setStyleSheet("color:#555;")
        v.addWidget(note)
        self.btn_cal = QtWidgets.QPushButton("Calibrate ADC offsets  (cal)")
        self.btn_cal.clicked.connect(lambda: self._send(proto.enc_cmd(proto.OP_CAL_ADC)))
        self.btn_hcal = QtWidgets.QPushButton("Calibrate Hall angles  (hcal)")
        self.btn_hcal.clicked.connect(lambda: self._send(proto.enc_cmd(proto.OP_HCAL)))
        v.addWidget(self.btn_cal); v.addWidget(self.btn_hcal)
        self.lbl_cal = QtWidgets.QLabel("no calibration run yet")
        self.lbl_cal.setWordWrap(True)
        v.addWidget(self.lbl_cal); v.addStretch(1)
        return g

    def _telemetry_group(self):
        g = QtWidgets.QGroupBox("Telemetry"); self.tel = QtWidgets.QFormLayout(g)
        self.t_flags = QtWidgets.QLabel("-"); self.t_spd = QtWidgets.QLabel("-")
        self.t_iq = QtWidgets.QLabel("-"); self.t_hall = QtWidgets.QLabel("-")
        self.t_iph = QtWidgets.QLabel("-"); self.t_th = QtWidgets.QLabel("-")
        self.t_enc = QtWidgets.QLabel("-"); self.t_enc_pos = QtWidgets.QLabel("-")
        self.tel.addRow("flags", self.t_flags); self.tel.addRow("speed", self.t_spd)
        self.tel.addRow("Iq ref/meas", self.t_iq); self.tel.addRow("hall/dir", self.t_hall)
        self.tel.addRow("Ia/Ib/Ic", self.t_iph); self.tel.addRow("theta", self.t_th)
        self.tel.addRow("encoder", self.t_enc); self.tel.addRow("enc pos", self.t_enc_pos)
        return g

    # ---- connection ------------------------------------------------------
    def _toggle_conn(self):
        if self.rx is not None:
            self.rx.stop(); self.rx = None; self.btn_conn.setText("Connect")
            self._set_controls_enabled(False); return
        self.rx = RxThread(self.cmb.currentText(), self.ed_ch.text(), self.ed_br.text())
        self.rx.link.connect(self._on_link)
        self.rx.status.connect(self._on_status)
        self.rx.currents.connect(self._on_currents)
        self.rx.cal.connect(self._on_cal)
        self.rx.encoder.connect(self._on_encoder)
        self.rx.start(); self.btn_conn.setText("Disconnect")

    def _on_link(self, up, msg):
        self.lbl_link.setText(("● " if up else "○ ") + msg)
        self.lbl_link.setStyleSheet("color:#080;" if up else "color:#a00;")
        self._set_controls_enabled(up)

    def _set_controls_enabled(self, on):
        for w in (self.btn_dis, self.btn_en, self.btn_clr, self.sp_iq, self.sp_spd,
                  self.btn_cal, self.btn_hcal):
            w.setEnabled(on)

    # ---- telemetry sinks -------------------------------------------------
    def _on_status(self, d):
        fl = [n for n, b in (("EN", d["enabled"]), ("SPD", d["speed_mode"]),
                             ("BLK", d["block_mode"]), ("FAULT", d["fault"])) if b]
        self.t_flags.setText(" ".join(fl) or "—")
        self.t_flags.setStyleSheet("color:#c00;font-weight:bold;" if d["fault"] else "")
        self.t_spd.setText(f"{d['speed_rpm']} RPM")
        self.t_iq.setText(f"{d['iq_ref_a']:+.3f} / {d['iq_a']:+.3f} A")
        self.t_hall.setText(f"{d['hall']} / {d['dir']:+d}")
        self.plots.add_status(d, self.spd_set)

    def _on_currents(self, d):
        self.t_iph.setText(f"{d['ia_a']:+.3f} {d['ib_a']:+.3f} {d['ic_a']:+.3f} A")
        self.t_th.setText(f"{d['theta_deg']:.1f}°")
        self.plots.add_currents(d)

    def _on_encoder(self, d):
        ok = d["ok"]
        self.t_enc.setText(f"{'OK' if ok else 'FAIL'} ({d['variant']})  mag {d['magnitude']}")
        self.t_enc.setStyleSheet("" if ok else "color:#c00;font-weight:bold;")
        self.t_enc_pos.setText(f"{d['angle_deg']:.2f}°  turns {d['turns']:+d}  "
                               f"{d['speed_dps']:+.1f}°/s")
        self.plots.add_encoder(d)

    def _on_cal(self, d):
        name = "ADC" if d["type"] == proto.CAL_TYPE_ADC else "Hall"
        res = "OK" if d["ok"] else "FAILED"
        extra = (f"  offsets a/b/c = {d['off_a']}/{d['off_b']}/{d['off_c']}"
                 if d["type"] == proto.CAL_TYPE_ADC else "")
        self.lbl_cal.setText(f"{name} calibration {res}{extra}")
        self.lbl_cal.setStyleSheet("color:#080;" if d["ok"] else "color:#a00;")

    # ---- control senders -------------------------------------------------
    def _send(self, id_data):
        if self.rx is not None:
            self.rx.send(*id_data)

    def _set_iq(self):
        self._send(proto.enc_iq(self.sp_iq.value()))

    def _set_spd(self):
        self.spd_set = self.sp_spd.value()
        self._send(proto.enc_speed(self.spd_set))

    def _spd_off(self):
        self.spd_set = 0.0
        self._send(proto.enc_cmd(proto.OP_SPEED_OFF))

    def closeEvent(self, ev):
        if self.rx is not None:
            self.rx.stop()
        ev.accept()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="slcan")
    ap.add_argument("--channel", default="/dev/ttyACM1")
    ap.add_argument("--bitrate", default="500000")
    ap.add_argument("--demo", action="store_true", help="synthetic telemetry, no hardware")
    a = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow("demo" if a.demo else a.backend, a.channel, a.bitrate)
    w.resize(1000, 640); w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
