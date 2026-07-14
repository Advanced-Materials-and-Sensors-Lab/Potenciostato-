# ============================================================
# Potenciostato ESP32 - HMI Bluetooth v2
# CA, LSV y CV con graficas, filtros y exportacion
#
# Compatible con firmware nuevo:
# ID_Prueba,Modo,Concentracion_Cu_ppb,Tiempo_ms,Vset,Vce_calc,Vpwm,Duty,
# ADC_A01,ADC_A02,Voltaje_A01,Voltaje_A02,Corriente_uA,Corriente_nA
#
# Requisitos:
#   pip install pyserial matplotlib pandas openpyxl
# ============================================================

import time
import threading
import queue
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import serial
import serial.tools.list_ports
import pandas as pd

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

APP_NAME = "Potenciostato ESP32"
BAUDRATE = 115200

DATA_COLUMNS = [
    "ID_Prueba", "Modo", "Concentracion_Cu_ppb", "Ciclo", "Tiempo_ms",
    "Vset", "Vce_calc", "Vpwm", "Duty",
    "Voltaje_A01", "Voltaje_A02",
    "Corriente_uA", "Corriente_nA",
]


def moving_average(values, window):
    if window <= 1 or len(values) < 2:
        return list(values)
    window = int(window)
    result = []
    half = window // 2
    for i in range(len(values)):
        ini = max(0, i - half)
        fin = min(len(values), i + half + 1)
        result.append(sum(values[ini:fin]) / (fin - ini))
    return result


def median_filter(values, window):
    if window <= 1 or len(values) < 2:
        return list(values)
    window = int(window)
    result = []
    half = window // 2
    for i in range(len(values)):
        ini = max(0, i - half)
        fin = min(len(values), i + half + 1)
        chunk = sorted(values[ini:fin])
        n = len(chunk)
        if n % 2 == 1:
            result.append(chunk[n // 2])
        else:
            result.append((chunk[n // 2 - 1] + chunk[n // 2]) / 2)
    return result


def exponential_filter(values, alpha):
    if not values:
        return []
    try:
        alpha = float(alpha)
    except Exception:
        alpha = 0.15
    alpha = max(0.01, min(1.0, alpha))
    result = [values[0]]
    for v in values[1:]:
        result.append(alpha * v + (1 - alpha) * result[-1])
    return result


def baseline_correct(values, n_points):
    if not values:
        return []
    try:
        n_points = int(float(n_points))
    except Exception:
        n_points = 10
    n_points = max(1, min(n_points, len(values)))
    base = sum(values[:n_points]) / n_points
    return [v - base for v in values]


class PotenciostatoHMI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_NAME)
        self.geometry("1360x800")
        self.minsize(1180, 720)

        self.ser = None
        self.reader_thread = None
        self.running_reader = False
        self.rx_queue = queue.Queue()

        self.test_running = False
        self.current_mode = "LSV"
        self.data = []
        self.full_history = []

        self.save_folder = Path.cwd() / "datos_potenciostato"
        self.save_folder.mkdir(exist_ok=True)

        self.current_unit = tk.StringVar(value="nA")
        self.x_axis = tk.StringVar(value="Vce_calc")
        self.show_raw = tk.BooleanVar(value=True)
        self.show_filtered = tk.BooleanVar(value=True)
        self.filter_type = tk.StringVar(value="Promedio móvil")
        self.filter_window = tk.StringVar(value="7")
        self.exp_alpha = tk.StringVar(value="0.15")
        self.baseline_enabled = tk.BooleanVar(value=False)
        self.baseline_points = tk.StringVar(value="10")

        self._build_style()
        self._build_ui()
        self.after(80, self.process_rx_queue)
        self.after(1000, self.update_clock)

    def _build_style(self):
        self.configure(bg="#eaf1f8")
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#eaf1f8")
        style.configure("Card.TFrame", background="white", relief="solid", borderwidth=1)
        style.configure("Sidebar.TFrame", background="#13263b")
        style.configure("TLabel", background="#eaf1f8", foreground="#1b2a41", font=("Segoe UI", 10))
        style.configure("Card.TLabel", background="white", foreground="#1b2a41", font=("Segoe UI", 10))
        style.configure("Title.TLabel", background="#eaf1f8", foreground="#1b2a41", font=("Segoe UI", 21, "bold"))
        style.configure("Sidebar.TLabel", background="#13263b", foreground="white", font=("Segoe UI", 10))
        style.configure("SidebarTitle.TLabel", background="#13263b", foreground="#4cc9f0", font=("Segoe UI", 22, "bold"))
        style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=6)
        style.configure("Green.TButton", background="#22a447", foreground="white")
        style.map("Green.TButton", background=[("active", "#1b8c3c")])
        style.configure("Red.TButton", background="#d93636", foreground="white")
        style.map("Red.TButton", background=[("active", "#b92e2e")])
        style.configure("TNotebook", background="#eaf1f8", borderwidth=0)
        style.configure("TNotebook.Tab", padding=[18, 8], font=("Segoe UI", 10, "bold"))
        style.configure("Treeview", rowheight=24, font=("Segoe UI", 9))
        style.configure("Treeview.Heading", font=("Segoe UI", 9, "bold"))

    def _build_ui(self):
        root = ttk.Frame(self)
        root.pack(fill="both", expand=True)
        self.sidebar = ttk.Frame(root, style="Sidebar.TFrame", width=255)
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)
        main = ttk.Frame(root)
        main.pack(side="right", fill="both", expand=True)
        self._build_sidebar()
        self._build_header(main)
        self._build_tabs(main)
        self._build_statusbar(main)

    def _build_sidebar(self):
        ttk.Label(self.sidebar, text="∿", style="SidebarTitle.TLabel").pack(pady=(24, 0))
        ttk.Label(self.sidebar, text="POTENCIOSTATO", style="Sidebar.TLabel", font=("Segoe UI", 13, "bold")).pack()
        ttk.Label(self.sidebar, text="ESP32", style="SidebarTitle.TLabel").pack()
        ttk.Label(self.sidebar, text="Electroquímica Inteligente", style="Sidebar.TLabel").pack(pady=(0, 22))
        box = tk.Frame(self.sidebar, bg="#1d334d", bd=0)
        box.pack(fill="x", padx=12, pady=10)
        tk.Label(box, text="CONEXIÓN BLUETOOTH", bg="#1d334d", fg="white", font=("Segoe UI", 10, "bold")).pack(anchor="w", padx=12, pady=(12, 6))
        tk.Label(box, text="Puerto COM", bg="#1d334d", fg="white").pack(anchor="w", padx=12)
        port_frame = tk.Frame(box, bg="#1d334d")
        port_frame.pack(fill="x", padx=12, pady=(2, 8))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, state="readonly", width=18)
        self.port_combo.pack(side="left", fill="x", expand=True)
        ttk.Button(port_frame, text="↻", command=self.refresh_ports, width=3).pack(side="left", padx=(6, 0))
        self.conn_status = tk.StringVar(value="DESCONECTADO")
        self.conn_label = tk.Label(box, textvariable=self.conn_status, bg="#1d334d", fg="#ff5c5c", font=("Segoe UI", 10, "bold"))
        self.conn_label.pack(anchor="w", padx=12, pady=4)
        self.btn_connect = ttk.Button(box, text="CONECTAR", command=self.toggle_connection)
        self.btn_connect.pack(fill="x", padx=12, pady=(8, 12))
        ttk.Separator(self.sidebar).pack(fill="x", padx=15, pady=20)
        ttk.Label(self.sidebar, text="Pruebas disponibles", style="Sidebar.TLabel", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=22, pady=(0, 8))
        ttk.Label(self.sidebar, text="• LSV", style="Sidebar.TLabel").pack(anchor="w", padx=30, pady=4)
        ttk.Label(self.sidebar, text="• CV", style="Sidebar.TLabel").pack(anchor="w", padx=30, pady=4)
        ttk.Label(self.sidebar, text="• CA", style="Sidebar.TLabel").pack(anchor="w", padx=30, pady=4)
        ttk.Label(self.sidebar, text="Proyecto de Tesis\n2026", style="Sidebar.TLabel", justify="center").pack(side="bottom", pady=25)
        self.refresh_ports()

    def _build_header(self, parent):
        header = ttk.Frame(parent)
        header.pack(fill="x", padx=20, pady=(14, 6))
        left = ttk.Frame(header)
        left.pack(side="left", fill="x", expand=True)
        ttk.Label(left, text="Inicio", style="Title.TLabel").pack(anchor="w")
        ttk.Label(left, text="Control, adquisición, filtrado y exportación de datos").pack(anchor="w")
        cards = ttk.Frame(header)
        cards.pack(side="right")
        self.clock_label = ttk.Label(cards, text="", style="Card.TLabel", padding=11)
        self.clock_label.pack(side="left", padx=5)
        self.id_label = ttk.Label(cards, text="ID Actual: --", style="Card.TLabel", padding=11)
        self.id_label.pack(side="left", padx=5)
        self.mode_label = ttk.Label(cards, text="Modo: --", style="Card.TLabel", padding=11)
        self.mode_label.pack(side="left", padx=5)

    def _build_tabs(self, parent):
        self.nb = ttk.Notebook(parent)
        self.nb.pack(fill="both", expand=True, padx=20, pady=8)
        self.tab_lsv = ttk.Frame(self.nb)
        self.tab_cv = ttk.Frame(self.nb)
        self.tab_ca = ttk.Frame(self.nb)
        self.tab_data = ttk.Frame(self.nb)
        self.nb.add(self.tab_lsv, text="LSV")
        self.nb.add(self.tab_cv, text="CV")
        self.nb.add(self.tab_ca, text="CA")
        self.nb.add(self.tab_data, text="Datos")
        self.info_vars = {}
        self.plots = {}
        self._build_test_tab(self.tab_lsv, "LSV")
        self._build_test_tab(self.tab_cv, "CV")
        self._build_test_tab(self.tab_ca, "CA")
        self._build_data_tab(self.tab_data)

    def _build_test_tab(self, tab, mode):
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(0, weight=1)
        panel = ttk.Frame(tab, style="Card.TFrame")
        panel.grid(row=0, column=0, sticky="ns", padx=(0, 12), pady=5)
        panel.configure(padding=12)
        ttk.Label(panel, text=f"Parámetros {mode}", style="Card.TLabel", font=("Segoe UI", 12, "bold")).pack(anchor="w", pady=(0, 10))
        entries = {}
        def add_entry(label, default):
            ttk.Label(panel, text=label, style="Card.TLabel").pack(anchor="w", pady=(5, 0))
            var = tk.StringVar(value=default)
            ttk.Entry(panel, textvariable=var, width=23).pack(fill="x", pady=(2, 4))
            entries[label] = var
            return var
        if mode in ("LSV", "CV"):
            add_entry("Voltaje inicial (V)", "-1.00")
            add_entry("Voltaje final (V)", "1.00")
            add_entry("Velocidad (mV/s)", "100")
            if mode == "CV":
                add_entry("Ciclos", "1")
            add_entry("Concentración Cu (ppb)", "0")
        else:
            add_entry("Voltaje aplicado (V)", "-0.40")
            add_entry("Tiempo total (s)", "60")
            add_entry("Concentración Cu (ppb)", "0")
        btns = ttk.Frame(panel, style="Card.TFrame")
        btns.pack(fill="x", pady=12)
        ttk.Button(btns, text="▶ INICIAR", style="Green.TButton", command=lambda m=mode, e=entries: self.start_test(m, e)).pack(side="left", expand=True, fill="x", padx=(0, 5))
        ttk.Button(btns, text="■ DETENER", style="Red.TButton", command=self.stop_test).pack(side="left", expand=True, fill="x", padx=(5, 0))
        ttk.Separator(panel).pack(fill="x", pady=9)
        ttk.Label(panel, text="Filtros de visualización", style="Card.TLabel", font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(5, 5))
        ttk.Checkbutton(panel, text="Mostrar señal original", variable=self.show_raw, command=self.update_plots).pack(anchor="w")
        ttk.Checkbutton(panel, text="Mostrar señal filtrada", variable=self.show_filtered, command=self.update_plots).pack(anchor="w")
        ttk.Label(panel, text="Tipo de filtro", style="Card.TLabel").pack(anchor="w", pady=(5, 0))
        cb = ttk.Combobox(panel, textvariable=self.filter_type, values=["Ninguno", "Promedio móvil", "Mediana", "Exponencial"], state="readonly", width=20)
        cb.pack(fill="x")
        cb.bind("<<ComboboxSelected>>", lambda e: self.update_plots())
        ttk.Label(panel, text="Ventana", style="Card.TLabel").pack(anchor="w", pady=(5, 0))
        ttk.Entry(panel, textvariable=self.filter_window).pack(fill="x")
        ttk.Label(panel, text="Alpha exponencial", style="Card.TLabel").pack(anchor="w", pady=(5, 0))
        ttk.Entry(panel, textvariable=self.exp_alpha).pack(fill="x")
        ttk.Checkbutton(panel, text="Corregir línea base", variable=self.baseline_enabled, command=self.update_plots).pack(anchor="w", pady=(6, 0))
        ttk.Label(panel, text="Puntos de línea base", style="Card.TLabel").pack(anchor="w")
        ttk.Entry(panel, textvariable=self.baseline_points).pack(fill="x")
        ttk.Button(panel, text="Actualizar filtro", command=self.update_plots).pack(fill="x", pady=8)
        ttk.Separator(panel).pack(fill="x", pady=9)
        info = {"ID": tk.StringVar(value="--"), "Modo": tk.StringVar(value="--"), "Concentración": tk.StringVar(value="--"), "Puntos": tk.StringVar(value="0")}
        self.info_vars[mode] = info
        ttk.Label(panel, text="Información", style="Card.TLabel", font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(5, 6))
        for k, v in info.items():
            row = ttk.Frame(panel, style="Card.TFrame")
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=k, style="Card.TLabel").pack(side="left")
            ttk.Label(row, textvariable=v, style="Card.TLabel", font=("Segoe UI", 10, "bold")).pack(side="right")
        right = ttk.Frame(tab)
        right.grid(row=0, column=1, sticky="nsew", pady=5)
        right.rowconfigure(1, weight=3)
        right.rowconfigure(2, weight=2)
        right.columnconfigure(0, weight=1)
        right.columnconfigure(1, weight=1)
        controls = ttk.Frame(right)
        controls.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        ttk.Label(controls, text="Eje X:").pack(side="left", padx=(0, 4))
        ttk.Combobox(controls, textvariable=self.x_axis, values=["Vce_calc", "Vset", "Tiempo_ms"], width=12, state="readonly").pack(side="left")
        ttk.Label(controls, text="Unidad corriente:").pack(side="left", padx=(18, 4))
        ttk.Combobox(controls, textvariable=self.current_unit, values=["nA", "µA"], width=8, state="readonly").pack(side="left")
        ttk.Button(controls, text="Guardar CSV", command=self.save_csv).pack(side="right", padx=4)
        ttk.Button(controls, text="Guardar Excel", command=self.save_excel).pack(side="right", padx=4)
        ttk.Button(controls, text="Guardar gráfica", command=self.save_current_plot).pack(side="right", padx=4)
        ttk.Button(controls, text="Limpiar", command=self.clear_current_data).pack(side="right", padx=4)
        fig = Figure(figsize=(7.4, 4.1), dpi=100)
        ax = fig.add_subplot(111)
        canvas = FigureCanvasTkAgg(fig, master=right)
        canvas.get_tk_widget().grid(row=1, column=0, columnspan=2, sticky="nsew")
        fig_v = Figure(figsize=(4, 2.4), dpi=100)
        ax_v = fig_v.add_subplot(111)
        canvas_v = FigureCanvasTkAgg(fig_v, master=right)
        canvas_v.get_tk_widget().grid(row=2, column=0, sticky="nsew", padx=(0, 6), pady=(8, 0))
        fig_i = Figure(figsize=(4, 2.4), dpi=100)
        ax_i = fig_i.add_subplot(111)
        canvas_i = FigureCanvasTkAgg(fig_i, master=right)
        canvas_i.get_tk_widget().grid(row=2, column=1, sticky="nsew", padx=(6, 0), pady=(8, 0))
        self.plots[mode] = {"fig": fig, "ax": ax, "canvas": canvas, "fig_v": fig_v, "ax_v": ax_v, "canvas_v": canvas_v, "fig_i": fig_i, "ax_i": ax_i, "canvas_i": canvas_i}
        self.current_mode = mode
        self.update_plots()

    def _build_data_tab(self, tab):
        tab.rowconfigure(0, weight=1)
        tab.columnconfigure(0, weight=1)
        frame = ttk.Frame(tab)
        frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        self.tree = ttk.Treeview(frame, columns=DATA_COLUMNS, show="headings")
        for col in DATA_COLUMNS:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=112, anchor="center")
        yscroll = ttk.Scrollbar(frame, orient="vertical", command=self.tree.yview)
        xscroll = ttk.Scrollbar(frame, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        buttons = ttk.Frame(tab)
        buttons.grid(row=1, column=0, sticky="ew", padx=5, pady=8)
        ttk.Button(buttons, text="Guardar CSV", command=self.save_csv).pack(side="left", padx=4)
        ttk.Button(buttons, text="Guardar Excel", command=self.save_excel).pack(side="left", padx=4)
        ttk.Button(buttons, text="Guardar gráfica PNG", command=self.save_current_plot).pack(side="left", padx=4)
        ttk.Button(buttons, text="Limpiar datos", command=self.clear_current_data).pack(side="left", padx=4)

    def _build_statusbar(self, parent):
        bar = tk.Frame(parent, bg="#13263b", height=32)
        bar.pack(side="bottom", fill="x")
        self.status_text = tk.StringVar(value="Listo")
        tk.Label(bar, textvariable=self.status_text, bg="#13263b", fg="white", font=("Segoe UI", 9)).pack(side="left", padx=12)
        tk.Label(bar, text="Datos crudos + filtros solo para visualización", bg="#13263b", fg="white", font=("Segoe UI", 9)).pack(side="right", padx=12)

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        values = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo["values"] = values
        if values and not self.port_var.get():
            self.port_var.set(values[0])

    def selected_port(self):
        text = self.port_var.get()
        return text.split(" - ")[0].strip() if text else None

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.selected_port()
        if not port:
            messagebox.showwarning("Puerto", "Selecciona un puerto COM.")
            return
        try:
            self.ser = serial.Serial(port, BAUDRATE, timeout=0.2)
            time.sleep(1.0)
            self.running_reader = True
            self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.reader_thread.start()
            self.conn_status.set("CONECTADO")
            self.conn_label.configure(fg="#28d45a")
            self.btn_connect.configure(text="DESCONECTAR")
            self.status_text.set(f"Conectado a {port}")
        except Exception as e:
            messagebox.showerror("Error de conexión", str(e))

    def disconnect(self):
        self.running_reader = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.conn_status.set("DESCONECTADO")
        self.conn_label.configure(fg="#ff5c5c")
        self.btn_connect.configure(text="CONECTAR")
        self.status_text.set("Desconectado")

    def read_serial_loop(self):
        while self.running_reader:
            try:
                if self.ser and self.ser.is_open:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self.rx_queue.put(line)
                else:
                    time.sleep(0.1)
            except Exception as e:
                self.rx_queue.put(f"ERROR_PC,{e}")
                time.sleep(0.3)

    def send_command(self, cmd):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Bluetooth", "Primero conecta el Bluetooth/COM del ESP32.")
            return False
        try:
            self.ser.write((cmd + "\n").encode("utf-8"))
            self.status_text.set(f"Enviado: {cmd}")
            return True
        except Exception as e:
            messagebox.showerror("Error enviando comando", str(e))
            return False

    def start_test(self, mode, entries):
        if self.test_running:
            messagebox.showwarning("Prueba activa", "Primero detén la prueba actual.")
            return
        try:
            if mode == "LSV":
                cmd = f"START,LSV,{float(entries['Voltaje inicial (V)'].get())},{float(entries['Voltaje final (V)'].get())},{float(entries['Velocidad (mV/s)'].get())},{float(entries['Concentración Cu (ppb)'].get())}"
            elif mode == "CV":
                cmd = f"START,CV,{float(entries['Voltaje inicial (V)'].get())},{float(entries['Voltaje final (V)'].get())},{float(entries['Velocidad (mV/s)'].get())},{int(float(entries['Ciclos'].get()))},{float(entries['Concentración Cu (ppb)'].get())}"
            elif mode == "CA":
                cmd = f"START,CA,{float(entries['Voltaje aplicado (V)'].get())},{float(entries['Tiempo total (s)'].get())},{float(entries['Concentración Cu (ppb)'].get())}"
            else:
                return
        except ValueError:
            messagebox.showerror("Parámetros", "Revisa que todos los parámetros sean numéricos.")
            return
        self.current_mode = mode
        self.nb.select({"LSV": self.tab_lsv, "CV": self.tab_cv, "CA": self.tab_ca}[mode])
        self.data.clear()
        self.update_table()
        self.update_plots()
        if self.send_command(cmd):
            self.test_running = True
            self.mode_label.configure(text=f"Modo: {mode}")
            self.status_text.set(f"Prueba {mode} iniciada")

    def stop_test(self):
        self.send_command("STOP")
        self.test_running = False
        self.status_text.set("STOP enviado")

    def process_rx_queue(self):
        while not self.rx_queue.empty():
            self.handle_line(self.rx_queue.get())
        self.after(80, self.process_rx_queue)

    def handle_line(self, line):
        if line.startswith("START"):
            self.test_running = True
            self.status_text.set(line)
            return
        if line == "END":
            self.test_running = False
            self.status_text.set("Prueba finalizada")
            self.update_table()
            self.update_plots()
            self.auto_save_current()
            return
        if line.startswith("ERROR"):
            self.status_text.set(line)
            return
        if "ID_Prueba" in line:
            return
        parts = line.split(",")
        try:
            # Firmware definitivo:
            # ID_Prueba,Modo,Concentracion_Cu_ppb,Ciclo,Tiempo_ms,Vset_V,A01_V,Vreal_V,A02_V,I_uA,Vpwm_V,Duty
            if len(parts) == 12:
                iua = float(parts[9])
                row = {
                    "ID_Prueba": int(float(parts[0])),
                    "Modo": parts[1].strip(),
                    "Concentracion_Cu_ppb": float(parts[2]),
                    "Ciclo": int(float(parts[3])),
                    "Tiempo_ms": int(float(parts[4])),
                    "Vset": float(parts[5]),
                    "Voltaje_A01": float(parts[6]),
                    "Vce_calc": float(parts[7]),
                    "Voltaje_A02": float(parts[8]),
                    "Corriente_uA": iua,
                    "Corriente_nA": iua * 1000.0,
                    "Vpwm": float(parts[10]),
                    "Duty": int(float(parts[11])),
                }
            # Compatibilidad con firmware anterior de 14 columnas
            elif len(parts) == 14:
                iua = float(parts[12])
                row = {
                    "ID_Prueba": int(float(parts[0])),
                    "Modo": parts[1].strip(),
                    "Concentracion_Cu_ppb": float(parts[2]),
                    "Ciclo": 1,
                    "Tiempo_ms": int(float(parts[3])),
                    "Vset": float(parts[4]),
                    "Vce_calc": float(parts[5]),
                    "Vpwm": float(parts[6]),
                    "Duty": int(float(parts[7])),
                    "Voltaje_A01": float(parts[10]),
                    "Voltaje_A02": float(parts[11]),
                    "Corriente_uA": iua,
                    "Corriente_nA": iua * 1000.0,
                }
            else:
                return
        except Exception:
            return
        self.current_mode = row["Modo"]
        self.data.append(row)
        self.full_history.append(row)
        self.id_label.configure(text=f"ID Actual: {row['ID_Prueba']}")
        self.mode_label.configure(text=f"Modo: {row['Modo']}")
        info = self.info_vars.get(row["Modo"])
        if info:
            info["ID"].set(str(row["ID_Prueba"]))
            info["Modo"].set(row["Modo"])
            info["Concentración"].set(f"{row['Concentracion_Cu_ppb']:.4g} ppb")
            info["Puntos"].set(str(len(self.data)))
        if len(self.data) % 3 == 0:
            self.update_table(max_rows=350)
            self.update_plots()

    def get_current_values(self, rows):
        if self.current_unit.get() == "µA":
            return [r["Corriente_uA"] for r in rows], "Corriente (µA)"
        return [r["Corriente_nA"] for r in rows], "Corriente (nA)"

    def apply_filter(self, values):
        y = list(values)
        if self.baseline_enabled.get():
            y = baseline_correct(y, self.baseline_points.get())
        try:
            w = int(float(self.filter_window.get()))
        except Exception:
            w = 7
        w = max(1, w)
        if w % 2 == 0:
            w += 1
        f = self.filter_type.get()
        if f == "Ninguno":
            return y
        if f == "Promedio móvil":
            return moving_average(y, w)
        if f == "Mediana":
            return median_filter(y, w)
        if f == "Exponencial":
            return exponential_filter(y, self.exp_alpha.get())
        return y

    def update_plots(self):
        mode = self.current_mode
        if mode not in self.plots:
            return
        p = self.plots[mode]
        ax, ax_v, ax_i = p["ax"], p["ax_v"], p["ax_i"]
        ax.clear(); ax_v.clear(); ax_i.clear()
        ax.grid(True, alpha=0.30); ax_v.grid(True, alpha=0.30); ax_i.grid(True, alpha=0.30)
        rows = [r for r in self.data if r["Modo"] == mode]
        if rows:
            xkey = self.x_axis.get()
            if xkey == "Tiempo_ms":
                x = [r["Tiempo_ms"] / 1000.0 for r in rows]
                xlabel = "Tiempo (s)"
            else:
                x = [r[xkey] for r in rows]
                xlabel = f"{xkey} (V)"
            y_raw, ylabel = self.get_current_values(rows)
            y_filt = self.apply_filter(y_raw)
            t = [r["Tiempo_ms"] / 1000.0 for r in rows]
            vce = [r["Vce_calc"] for r in rows]

            # Omite únicamente el punto previo al inicio real del barrido.
            # Esto evita la línea recta causada por el salto inicial de voltaje,
            # sin modificar los datos recibidos, guardados ni filtrados.
            plot_start = 0
            limite_inicio = min(len(vce), 10)
            for i in range(1, limite_inicio):
                if abs(vce[i] - vce[i - 1]) > 0.20:
                    plot_start = i
                    break

            x_plot = x[plot_start:]
            y_raw_plot = y_raw[plot_start:]
            y_filt_plot = y_filt[plot_start:]
            t_plot = t[plot_start:]
            vce_plot = vce[plot_start:]

            if self.show_raw.get():
                ax.plot(x_plot, y_raw_plot, linewidth=1.0, alpha=0.55, label="Original")
                ax_i.plot(t, y_raw, linewidth=1.0, alpha=0.55, label="Original")
            if self.show_filtered.get():
                ax.plot(x_plot, y_filt_plot, linewidth=1.8, label="Filtrada")
                ax_i.plot(t, y_filt, linewidth=1.8, label="Filtrada")
            ax.legend(loc="best"); ax_i.legend(loc="best")
            ax.set_title("Gráfica principal")
            ax.set_xlabel(xlabel); ax.set_ylabel(ylabel)
            ax_v.plot(t_plot, vce_plot, linewidth=1.5)
            ax_v.set_title("Voltaje aplicado")
            ax_v.set_xlabel("Tiempo (s)"); ax_v.set_ylabel("Vce (V)")
            ax_i.set_title("Corriente vs tiempo")
            ax_i.set_xlabel("Tiempo (s)"); ax_i.set_ylabel(ylabel)
        else:
            ax.set_title("Gráfica principal"); ax.set_xlabel("Vce_calc (V)"); ax.set_ylabel(f"Corriente ({self.current_unit.get()})")
            ax_v.set_title("Voltaje aplicado"); ax_v.set_xlabel("Tiempo (s)"); ax_v.set_ylabel("Vce (V)")
            ax_i.set_title("Corriente vs tiempo"); ax_i.set_xlabel("Tiempo (s)"); ax_i.set_ylabel(f"Corriente ({self.current_unit.get()})")
        p["canvas"].draw_idle(); p["canvas_v"].draw_idle(); p["canvas_i"].draw_idle()

    def update_table(self, max_rows=350):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for r in self.data[-max_rows:]:
            values = [r["ID_Prueba"], r["Modo"], f'{r["Concentracion_Cu_ppb"]:.5f}', r.get("Ciclo", 1), r["Tiempo_ms"], f'{r["Vset"]:.5f}', f'{r["Vce_calc"]:.5f}', f'{r["Vpwm"]:.5f}', r["Duty"], f'{r["Voltaje_A01"]:.5f}', f'{r["Voltaje_A02"]:.5f}', f'{r["Corriente_uA"]:.9f}', f'{r["Corriente_nA"]:.6f}']
            self.tree.insert("", "end", values=values)

    def clear_current_data(self):
        self.data.clear()
        self.update_table(); self.update_plots()
        self.status_text.set("Datos limpiados")

    def dataframe_current(self):
        if not self.data:
            return pd.DataFrame(columns=DATA_COLUMNS + ["Corriente_filtrada_nA", "Corriente_filtrada_uA"])
        df = pd.DataFrame(self.data)
        y_raw, _ = self.get_current_values(self.data)
        y_filt = self.apply_filter(y_raw)
        if self.current_unit.get() == "µA":
            df["Corriente_filtrada_uA"] = y_filt
            df["Corriente_filtrada_nA"] = [v * 1000.0 for v in y_filt]
        else:
            df["Corriente_filtrada_nA"] = y_filt
            df["Corriente_filtrada_uA"] = [v / 1000.0 for v in y_filt]
        df["Filtro"] = self.filter_type.get()
        df["Ventana_filtro"] = self.filter_window.get()
        df["Linea_base_activa"] = self.baseline_enabled.get()
        return df

    def save_csv(self):
        df = self.dataframe_current()
        if df.empty:
            messagebox.showinfo("Guardar", "No hay datos para guardar."); return
        default = f"potenciostato_{self.current_mode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile=default, filetypes=[("CSV", "*.csv")])
        if not path: return
        df.to_csv(path, index=False, encoding="utf-8-sig")
        self.status_text.set(f"CSV guardado: {path}")

    def save_excel(self):
        df = self.dataframe_current()
        if df.empty:
            messagebox.showinfo("Guardar", "No hay datos para guardar."); return
        default = f"potenciostato_{self.current_mode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
        path = filedialog.asksaveasfilename(defaultextension=".xlsx", initialfile=default, filetypes=[("Excel", "*.xlsx")])
        if not path: return
        df.to_excel(path, index=False)
        self.status_text.set(f"Excel guardado: {path}")

    def save_current_plot(self):
        mode = self.current_mode
        if mode not in self.plots: return
        default = f"grafica_{mode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        path = filedialog.asksaveasfilename(defaultextension=".png", initialfile=default, filetypes=[("PNG", "*.png")])
        if not path: return
        self.plots[mode]["fig"].savefig(path, dpi=200, bbox_inches="tight")
        self.status_text.set(f"Gráfica guardada: {path}")

    def auto_save_current(self):
        df = self.dataframe_current()
        if df.empty: return
        try:
            mode = df["Modo"].iloc[-1]
            test_id = df["ID_Prueba"].iloc[-1]
            conc = df["Concentracion_Cu_ppb"].iloc[-1]
            filename = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{mode}_ID{test_id}_Cu{conc:g}ppb.csv"
            path = self.save_folder / filename
            df.to_csv(path, index=False, encoding="utf-8-sig")
            self.status_text.set(f"Prueba finalizada. Auto CSV: {path}")
        except Exception as e:
            self.status_text.set(f"No se pudo guardar automático: {e}")

    def update_clock(self):
        self.clock_label.configure(text=datetime.now().strftime("Fecha / Hora\n%d/%m/%Y\n%H:%M:%S"))
        self.after(1000, self.update_clock)

    def on_close(self):
        try:
            if self.test_running:
                self.send_command("STOP")
        except Exception:
            pass
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = PotenciostatoHMI()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
