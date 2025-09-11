#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# JUAMPI HMI – rápido, selectivo y con filtros anti-picos (firmware v1.2 @ 500000)

import sys, csv, pathlib, threading
from datetime import datetime
from collections import deque
import numpy as np
import serial, serial.tools.list_ports
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

# -------- Calibración fija para corriente --------
I_OFFSET_V       = 2.50     # V (bias ~2.5 V)
I_GAIN_A_PER_V   = -0.10     # A/V  (usa -0.10 si pendiente invertida)

# ================== Filtros ==================
def median_abs_deviation(x):
    # MAD robusto (escala para gaussiano)
    med = np.median(x)
    return med, 1.4826 * np.median(np.abs(x - med))

class HampelFilter:
    """Filtro anti-picos por ventana deslizante (mediana ± k*MAD)."""
    def __init__(self, win=9, k=3.0):
        self.win = max(3, int(win) | 1)  # impar
        self.k = float(k)
        self.buf = deque(maxlen=self.win)
    def set_params(self, win=None, k=None):
        if win is not None:
            old = list(self.buf)
            self.win = max(3, int(win) | 1)
            self.buf = deque(old[-self.win:], maxlen=self.win)
        if k is not None: self.k = float(k)
    def apply(self, x):
        self.buf.append(float(x))
        arr = np.fromiter(self.buf, dtype=float)
        if arr.size < 3:
            return x
        med, mad = median_abs_deviation(arr)
        if mad <= 1e-12:
            return med  # si todo igual, usar mediana
        if abs(x - med) > self.k * mad:
            return med
        return x

class SMAFilter:
    """Media móvil simple."""
    def __init__(self, win=7):
        self.win = max(1, int(win))
        self.buf = deque(maxlen=self.win)
        self.sum = 0.0
    def set_window(self, win):
        win = max(1, int(win))
        old = list(self.buf)
        self.win = win
        self.buf = deque(old[-win:], maxlen=win)
        self.sum = float(np.sum(self.buf)) if self.buf else 0.0
    def apply(self, x):
        if len(self.buf) == self.win:
            self.sum -= self.buf[0]
        self.buf.append(float(x))
        self.sum += self.buf[-1]
        return self.sum / len(self.buf)

# ================== Plot widget ==================
class Plot3(FigureCanvasQTAgg):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(8,6), constrained_layout=True)
        super().__init__(self.fig)
        gs = self.fig.add_gridspec(3, 1, height_ratios=[1,1,1])

        self.axV  = self.fig.add_subplot(gs[0, 0])
        self.axI  = self.fig.add_subplot(gs[1, 0], sharex=self.axV)
        self.axIV = self.fig.add_subplot(gs[2, 0])

        self.axV.set_ylabel("V (V)")
        self.axI.set_ylabel("I (A)")
        self.axI.set_xlabel("t (s)")
        self.axIV.set_xlabel("V (V)")
        self.axIV.set_ylabel("I (A)")

        (self.lnV,)  = self.axV.plot([], [], lw=1.2)
        (self.lnI,)  = self.axI.plot([], [], lw=1.2)
        (self.lnIV,) = self.axIV.plot([], [], lw=1.2)

    def refresh_plot(self, xs, ysV, ysI, draw_Vt, draw_It, ivV=None, ivI=None, draw_IV=False):
        # V(t)
        if draw_Vt and xs and ysV:
            self.lnV.set_data(xs, ysV)
            x0, x1 = xs[0], max(xs[-1], xs[0]+1e-3)
            self.axV.set_xlim(x0, x1)
            m, M = float(np.min(ysV)), float(np.max(ysV))
            if m == M: M = m + 1e-9
            pad = 0.08*(M-m)
            self.axV.set_ylim(m-pad, M+pad)
        else:
            self.lnV.set_data([], [])

        # I(t)
        if draw_It and xs and ysI:
            self.lnI.set_data(xs, ysI)
            x0, x1 = xs[0], max(xs[-1], xs[0]+1e-3)
            self.axI.set_xlim(x0, x1)
            m, M = float(np.min(ysI)), float(np.max(ysI))
            if m == M: M = m + 1e-9
            pad = 0.08*(M-m)
            self.axI.set_ylim(m-pad, M+pad)
        else:
            self.lnI.set_data([], [])

        # I–V
        if draw_IV and ivV and ivI:
            self.lnIV.set_data(ivV, ivI)
            vmin, vmax = float(np.min(ivV)), float(np.max(ivV))
            imin, imax = float(np.min(ivI)), float(np.max(ivI))
            if vmin == vmax: vmax = vmin + 1e-9
            if imin == imax: imax = imin + 1e-9
            vpad = 0.08*(vmax-vmin); ipad = 0.08*(imax-imin)
            self.axIV.set_xlim(vmin-vpad, vmax+vpad)
            self.axIV.set_ylim(imin-ipad, imax+ipad)
        else:
            self.lnIV.set_data([], [])

        self.draw_idle()

# ================== Serie worker ==================
class SerialWorker(QtCore.QObject):
    line_received = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()
    def __init__(self, port, baud=500000):
        super().__init__()
        self._port = port; self._baud = baud
        self._stop = threading.Event()
        self.ser = None
    def stop(self):
        self._stop.set()
        try:
            if self.ser and self.ser.is_open: self.ser.close()
        except Exception: pass
    @QtCore.pyqtSlot()
    def run(self):
        try:
            self.ser = serial.Serial(self._port, self._baud, timeout=0.1)
            self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
            while not self._stop.is_set():
                s = self.ser.readline().decode("ascii", errors="ignore").strip()
                if s: self.line_received.emit(s)
        except Exception as e:
            self.line_received.emit(f"#ERROR {e}")
        finally:
            self.finished.emit()

# ================== Ventana principal ==================
class Main(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("JUAMI – HMI (filtros anti-picos)")

        # Conexión
        self.cbPort = QtWidgets.QComboBox()
        self.btnRefresh = QtWidgets.QPushButton("Refrescar")
        self.cbBaud = QtWidgets.QComboBox()
        self.cbBaud.addItems(["1000000","500000","230400","115200","57600"])
        self.cbBaud.setCurrentText("500000")
        self.btnConnect = QtWidgets.QPushButton("Conectar")
        self.btnDisconnect = QtWidgets.QPushButton("Desconectar"); self.btnDisconnect.setEnabled(False)
        hb = QtWidgets.QHBoxLayout()
        hb.addWidget(QtWidgets.QLabel("Puerto:")); hb.addWidget(self.cbPort); hb.addWidget(self.btnRefresh)
        hb.addStretch(1)
        hb.addWidget(QtWidgets.QLabel("Baud:")); hb.addWidget(self.cbBaud)
        hb.addWidget(self.btnConnect); hb.addWidget(self.btnDisconnect)

        # Tabs (CA, LSV, CV)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._mk_tab_ca(),  "CA")
        self.tabs.addTab(self._mk_tab_lsv(), "LSV")
        self.tabs.addTab(self._mk_tab_cv(),  "CV")

        # Visualización
        self.chkEnableVis = QtWidgets.QCheckBox("Habilitar visualización"); self.chkEnableVis.setChecked(True)
        self.chkVt = QtWidgets.QCheckBox("V(t)")
        self.chkIt = QtWidgets.QCheckBox("I(t)")
        self.chkIV = QtWidgets.QCheckBox("I–V")

        # Filtros (aplican a V e I)
        self.chkAntiSpike = QtWidgets.QCheckBox("Anti-picos (Hampel)"); self.chkAntiSpike.setChecked(True)
        self.spnHampelN = QtWidgets.QSpinBox(); self.spnHampelN.setRange(3, 101); self.spnHampelN.setSingleStep(2); self.spnHampelN.setValue(9)
        self.spnHampelK = QtWidgets.QDoubleSpinBox(); self.spnHampelK.setDecimals(1); self.spnHampelK.setRange(0.5, 10.0); self.spnHampelK.setValue(3.0)

        self.chkSMA = QtWidgets.QCheckBox("Suavizado (SMA)"); self.chkSMA.setChecked(True)
        self.spnSMA = QtWidgets.QSpinBox(); self.spnSMA.setRange(1, 101); self.spnSMA.setValue(7)

        visLay = QtWidgets.QHBoxLayout()
        visLay.addWidget(QtWidgets.QLabel("Visualización:"))
        visLay.addWidget(self.chkEnableVis); visLay.addStretch(1)
        visLay.addWidget(self.chkVt); visLay.addWidget(self.chkIt); visLay.addWidget(self.chkIV)

        filtLay = QtWidgets.QHBoxLayout()
        filtLay.addWidget(self.chkAntiSpike)
        filtLay.addWidget(QtWidgets.QLabel("N=")); filtLay.addWidget(self.spnHampelN)
        filtLay.addWidget(QtWidgets.QLabel("k=")); filtLay.addWidget(self.spnHampelK)
        filtLay.addSpacing(20)
        filtLay.addWidget(self.chkSMA)
        filtLay.addWidget(QtWidgets.QLabel("N=")); filtLay.addWidget(self.spnSMA)
        filtLay.addStretch(1)

        # Plot + STOP + Guardar PNG
        self.plot = Plot3()
        self.btnStop = QtWidgets.QPushButton("STOP")
        self.btnSavePNG = QtWidgets.QPushButton("Guardar PNG")

        # Layout
        v = QtWidgets.QVBoxLayout(self)
        v.addLayout(hb); v.addWidget(self.tabs)
        v.addLayout(visLay); v.addLayout(filtLay)
        v.addWidget(self.plot)
        hbuttons = QtWidgets.QHBoxLayout()
        hbuttons.addWidget(self.btnStop); hbuttons.addStretch(1); hbuttons.addWidget(self.btnSavePNG)
        v.addLayout(hbuttons)

        # Datos y estado
        self.xs = deque(); self.yV = deque(); self.yI = deque()
        self.ivV = []; self.ivI = []
        self._iv_decim = 0
        self.csv_file = None; self.csv_writer = None
        self._current_mode = None

        # Filtros (uno por señal)
        self.hampelV = HampelFilter(win=self.spnHampelN.value(), k=self.spnHampelK.value())
        self.hampelI = HampelFilter(win=self.spnHampelN.value(), k=self.spnHampelK.value())
        self.smaV = SMAFilter(win=self.spnSMA.value())
        self.smaI = SMAFilter(win=self.spnSMA.value())
        # reconfigurar en vivo
        self.spnHampelN.valueChanged.connect(lambda n: (self.hampelV.set_params(win=n), self.hampelI.set_params(win=n)))
        self.spnHampelK.valueChanged.connect(lambda k: (self.hampelV.set_params(k=k), self.hampelI.set_params(k=k)))
        self.spnSMA.valueChanged.connect(lambda n: (self.smaV.set_window(n), self.smaI.set_window(n)))

        # Refresco con QTimer (~30 FPS)
        self._needs_refresh = False
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start()

        # Señales
        self.btnRefresh.clicked.connect(self.refresh_ports)
        self.btnConnect.clicked.connect(self.connect_port)
        self.btnDisconnect.clicked.connect(self.disconnect_port)
        self.btnStop.clicked.connect(lambda: self._send("STOP"))
        self.btnSavePNG.clicked.connect(self.save_png)

        self.refresh_ports()
        self.worker = None; self.thread = None
        self.plot.refresh_plot([], [], [], False, False, [], [], False)

    # ---------- Pestañas ----------
    def _mk_tab_ca(self):
        w = QtWidgets.QWidget(); f = QtWidgets.QFormLayout(w)
        self.ca_v   = QtWidgets.QDoubleSpinBox(); self.ca_v.setRange(-5,5); self.ca_v.setValue(0.2); self.ca_v.setDecimals(4)
        self.ca_sec = QtWidgets.QDoubleSpinBox(); self.ca_sec.setRange(0.001, 4.0e6); self.ca_sec.setValue(5.0); self.ca_sec.setDecimals(3)
        self.btnCA  = QtWidgets.QPushButton("Iniciar CA")
        f.addRow("Vset (V):", self.ca_v); f.addRow("Tiempo (s):", self.ca_sec); f.addRow(self.btnCA)
        self.btnCA.clicked.connect(self._start_ca)
        return w

    def _mk_tab_lsv(self):
        w = QtWidgets.QWidget(); f = QtWidgets.QFormLayout(w)
        self.lsv_vneg = QtWidgets.QDoubleSpinBox(); self.lsv_vneg.setRange(-5,5); self.lsv_vneg.setValue(0.0); self.lsv_vneg.setDecimals(4)
        self.lsv_vpos = QtWidgets.QDoubleSpinBox(); self.lsv_vpos.setRange(-5,5); self.lsv_vpos.setValue(0.8); self.lsv_vpos.setDecimals(4)
        self.lsv_rate = QtWidgets.QDoubleSpinBox(); self.lsv_rate.setRange(0.001, 1000.0); self.lsv_rate.setValue(0.050)  # V/s
        self.lsv_cycles = QtWidgets.QSpinBox(); self.lsv_cycles.setRange(1, 10000); self.lsv_cycles.setValue(1)
        self.btnLSV = QtWidgets.QPushButton("Iniciar LSV")
        f.addRow("Vneg (V):", self.lsv_vneg); f.addRow("Vpos (V):", self.lsv_vpos)
        f.addRow("r (V/s):", self.lsv_rate); f.addRow("Ciclos:", self.lsv_cycles); f.addRow(self.btnLSV)
        self.btnLSV.clicked.connect(self._start_lsv)
        return w

    def _mk_tab_cv(self):
        w = QtWidgets.QWidget(); f = QtWidgets.QFormLayout(w)
        self.cv_vneg = QtWidgets.QDoubleSpinBox(); self.cv_vneg.setRange(-5,5); self.cv_vneg.setValue(-0.5); self.cv_vneg.setDecimals(4)
        self.cv_vpos = QtWidgets.QDoubleSpinBox(); self.cv_vpos.setRange(-5,5); self.cv_vpos.setValue(0.5);  self.cv_vpos.setDecimals(4)
        self.cv_rate = QtWidgets.QDoubleSpinBox(); self.cv_rate.setRange(0.001, 1000.0); self.cv_rate.setValue(0.100)  # V/s
        self.cv_cycles = QtWidgets.QSpinBox(); self.cv_cycles.setRange(1, 10000); self.cv_cycles.setValue(2)
        self.btnCV = QtWidgets.QPushButton("Iniciar CV")
        f.addRow("Vneg (V):", self.cv_vneg); f.addRow("Vpos (V):", self.cv_vpos)
        f.addRow("r (V/s):", self.cv_rate); f.addRow("Ciclos:", self.cv_cycles); f.addRow(self.btnCV)
        self.btnCV.clicked.connect(self._start_cv)
        return w

    # ---------- Conexión ----------
    def refresh_ports(self):
        self.cbPort.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cbPort.addItems(ports)

    def connect_port(self):
        port = self.cbPort.currentText()
        if not port:
            QtWidgets.QMessageBox.warning(self,"Aviso","Selecciona un puerto.")
            return
        self._open_csv()
        self._clear_all_buffers()

        self.worker = SerialWorker(port, int(self.cbBaud.currentText()))
        self.thread = QtCore.QThread(self)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.line_received.connect(self.on_line)
        self.worker.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.thread.deleteLater)
        self.btnConnect.setEnabled(False); self.btnDisconnect.setEnabled(True)
        self.thread.start()
        self._send("IDN?")

    def disconnect_port(self):
        if self.worker: self.worker.stop()
        self.btnConnect.setEnabled(True); self.btnDisconnect.setEnabled(False)
        self._close_csv()

    def _send(self, text):
        try:
            if self.worker and self.worker.ser and self.worker.ser.is_open:
                self.worker.ser.write((text.strip()+"\n").encode("ascii"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(self,"Serie",str(e))

    # ---------- Inicios de modo (ajustan vis por defecto) ----------
    def _start_ca(self):
        self._current_mode = "CA"
        self.chkEnableVis.setChecked(True); self.chkVt.setChecked(False)
        self.chkIt.setChecked(True); self.chkIV.setChecked(False)
        self._send(f"CAS {self.ca_v.value():.6f} {float(self.ca_sec.value()):.6f}")

    def _start_lsv(self):
        self._current_mode = "LSV"
        self.chkEnableVis.setChecked(True); self.chkVt.setChecked(True)
        self.chkIt.setChecked(False); self.chkIV.setChecked(False)
        v0 = self.lsv_vneg.value(); v1 = self.lsv_vpos.value()
        rate_mVs = self.lsv_rate.value()*1000.0; cyc = self.lsv_cycles.value()
        for _ in range(cyc):
            self._send(f"LSV {v0:.6f} {v1:.6f} {rate_mVs:.6f}")

    def _start_cv(self):
        self._current_mode = "CV"
        self.chkEnableVis.setChecked(True); self.chkVt.setChecked(True)
        self.chkIt.setChecked(False); self.chkIV.setChecked(False)
        vl = self.cv_vneg.value(); vh = self.cv_vpos.value()
        rate_mVs = self.cv_rate.value()*1000.0; cyc = self.cv_cycles.value()
        self._send(f"CV {vl:.6f} {vh:.6f} {rate_mVs:.6f} {int(cyc)}")

    # ---------- CSV ----------
    def _open_csv(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = pathlib.Path(f"juami_{ts}.csv")
        self.csv_file = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["t_ms","set_V","VMON_V","IMON_V","ADC0","ADC1","PWMCODE","I_A"])
        self._csv_path = path

    def _close_csv(self):
        try:
            if self.csv_file:
                self.csv_file.close()
                QtWidgets.QMessageBox.information(self,"CSV",f"Guardado: {self._csv_path}")
        except Exception: pass
        self.csv_file = None; self.csv_writer = None

    # ---------- Buffers ----------
    def _clear_all_buffers(self):
        self.xs.clear(); self.yV.clear(); self.yI.clear()
        self.ivV = []; self.ivI = []
        self._iv_decim = 0
        self._needs_refresh = True
        # reset filtros
        self.hampelV = HampelFilter(win=self.spnHampelN.value(), k=self.spnHampelK.value())
        self.hampelI = HampelFilter(win=self.spnHampelN.value(), k=self.spnHampelK.value())
        self.smaV = SMAFilter(win=self.spnSMA.value())
        self.smaI = SMAFilter(win=self.spnSMA.value())

    # ---------- Guardar PNG ----------
    def save_png(self):
        if not hasattr(self, "_csv_path"):
            QtWidgets.QMessageBox.information(self, "PNG", "Aún no hay corrida/CSV.")
            return
        png_path = pathlib.Path(self._csv_path).with_suffix(".png")
        try:
            self.plot.fig.savefig(png_path, dpi=150)
            QtWidgets.QMessageBox.information(self, "PNG", f"Guardado: {png_path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "PNG", str(e))

    # ---------- Timer de refresco ----------
    def _on_timer(self):
        if not self.chkEnableVis.isChecked():
            return
        if self._needs_refresh:
            self.plot.refresh_plot(
                self.xs, self.yV, self.yI,
                self.chkVt.isChecked(),
                self.chkIt.isChecked(),
                self.ivV if self.chkIV.isChecked() else None,
                self.ivI if self.chkIV.isChecked() else None,
                self.chkIV.isChecked()
            )
            self._needs_refresh = False

    # ---------- Procesamiento de líneas ----------
    @QtCore.pyqtSlot(str)
    def on_line(self, s):
        if s.startswith("#ERROR"):
            QtWidgets.QMessageBox.critical(self,"Serie",s); return

        if s == "START":
            self._clear_all_buffers(); return
        if s in ("READY","JUAMI-UNO","OK"): return
        if s == "ERR":
            QtWidgets.QMessageBox.warning(self,"Arduino","ERR recibido. Verifica parámetros."); return
        if s == "END":
            self._needs_refresh = True; return

        parts = s.split(',')
        if len(parts) != 7: return
        try:
            t_ms = float(parts[0]); setV = float(parts[1])
            Vmon = float(parts[2]); ImonV = float(parts[3])
            adc0 = int(parts[4]);   adc1 = int(parts[5]); code = int(parts[6])
        except Exception:
            return

        # ---- Filtros en Vmon / ImonV ----
        if self.chkAntiSpike.isChecked():
            Vmon = self.hampelV.apply(Vmon)
            ImonV = self.hampelI.apply(ImonV)

        if self.chkSMA.isChecked():
            Vmon = self.smaV.apply(Vmon)
            ImonV = self.smaI.apply(ImonV)
        else:
            # si el suavizado está apagado, reinicia acumuladores para no arrastrar historia
            self.smaV.set_window(self.spnSMA.value())
            self.smaI.set_window(self.spnSMA.value())

        # Limitar a rangos físicos (protege ejes ante salidas raras)
        Vmon = float(np.clip(Vmon, -0.1, 5.1))
        ImonV = float(np.clip(ImonV, -0.1, 5.1))

        # Corriente (según calibración fija)
        I = (ImonV - I_OFFSET_V) * I_GAIN_A_PER_V

        # CSV
        if self.csv_writer:
            self.csv_writer.writerow([t_ms, setV, Vmon, ImonV, adc0, adc1, code, I])

        # ¿Qué graficar?
        need_Vt = self.chkEnableVis.isChecked() and self.chkVt.isChecked()
        need_It = self.chkEnableVis.isChecked() and self.chkIt.isChecked()
        need_IV = self.chkEnableVis.isChecked() and self.chkIV.isChecked()

        # Buffers tiempo real
        t_s = t_ms/1000.0
        if need_Vt or need_It:
            self.xs.append(t_s)
            if need_Vt: self.yV.append(Vmon)
            else:
                if len(self.yV) < len(self.xs): self.yV.append(self.yV[-1] if self.yV else Vmon)
            if need_It: self.yI.append(I)
            else:
                if len(self.yI) < len(self.xs): self.yI.append(self.yI[-1] if self.yI else 0.0)
            while self.xs and (self.xs[-1]-self.xs[0] > 30.0):
                self.xs.popleft(); self.yV.popleft(); self.yI.popleft()

        # I–V (decimación 1/3)
        if need_IV:
            self._iv_decim = (self._iv_decim + 1) % 3
            if self._iv_decim == 0:
                self.ivV.append(Vmon); self.ivI.append(I)

        if self.chkEnableVis.isChecked():
            self._needs_refresh = True

def main():
    app = QtWidgets.QApplication(sys.argv)
    m = Main(); m.resize(1060, 760); m.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
