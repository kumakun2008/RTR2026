import sys
import struct
import os
import time
import serial
import serial.tools.list_ports
import threading
import re
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
matplotlib.rcParams['font.family'] = 'MS Gothic'
matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

class PitotCalibrationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Team ЯTR - ピトー管 風洞校正支援システム (DT-8920 リアルタイム比較対応)")
        self.root.geometry("1350x885")
        self.root.configure(bg="#0F172A") # slate-900

        # Serial configuration (Pitot tube board)
        self.ser = None
        self.serial_connected = False
        self.read_thread = None
        
        # Serial configuration (DT-8920 reference meter)
        self.dt_ser = None
        self.dt_connected = False
        self.dt_read_thread = None
        
        # Real-time data storage (Pitot tube board)
        self.current_data = {
            "airspeed": 0.0,
            "aoa_pa": 0.0,
            "aos_pa": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "yaw": 0.0,
            "temp": 0.0,
            "humid": 0.0
        }
        
        # Real-time data storage (DT-8920 reference meter)
        self.dt_data = {
            "airspeed": 0.0,
            "pressure": 0.0
        }
        self.dt_raw_monitor_str = "未接続"
        self.dt_last_bytes = b""
        
        # Zero-point aerodynamic offsets (calibrated at 0 degrees under wind)
        self.aoa_zero_offset = 0.0
        self.aos_zero_offset = 0.0
        
        # Sweep data points (discrete calibration steps, using BNO055 angles directly)
        self.sweep_points = []
        
        # For trend plots
        self.plot_history_len = 100
        self.history_time = []
        self.history_airspeed = []
        self.history_aoa = []
        self.history_aos = []
        self.history_pitch = []
        self.history_yaw = []
        # DT-8920 history
        self.history_dt_speed = []
        self.history_dt_press = []
        
        self.start_app_time = time.time()
        
        # Status
        self.calibrating_zero = False
        self.recording_point = False
        
        # Configure styles
        self.setup_styles()
        
        # Build UI layout
        self.build_ui()
        
        # Start periodic GUI updates
        self.update_gui_loop()

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        
        # Custom colors
        style.configure(".", background="#0F172A", foreground="#FFFFFF")
        style.configure("TLabel", background="#0F172A", foreground="#E2E8F0", font=("Segoe UI", 10))
        style.configure("Header.TLabel", background="#0F172A", foreground="#06B6D4", font=("Segoe UI", 12, "bold"))
        style.configure("Card.TFrame", background="#1E293B", relief="flat")
        style.configure("CardLabel.TLabel", background="#1E293B", foreground="#F8FAFC", font=("Segoe UI", 10, "bold"))
        
        # Buttons
        style.configure("Action.TButton", font=("Segoe UI", 9, "bold"), foreground="#FFFFFF", background="#3B82F6")
        style.map("Action.TButton", background=[("active", "#2563EB")])
        
        style.configure("Start.TButton", font=("Segoe UI", 10, "bold"), foreground="#FFFFFF", background="#10B981")
        style.map("Start.TButton", background=[("active", "#059669")])
        
        style.configure("Stop.TButton", font=("Segoe UI", 10, "bold"), foreground="#FFFFFF", background="#EF4444")
        style.map("Stop.TButton", background=[("active", "#DC2626")])

        # Notebook
        style.configure("TNotebook", background="#0F172A", borderwidth=0)
        style.configure("TNotebook.Tab", background="#1E293B", foreground="#94A3B8", font=("Segoe UI", 10))
        style.map("TNotebook.Tab", background=[("selected", "#0F172A")], foreground=[("selected", "#06B6D4")])

        # Treeview (points grid)
        style.configure("Treeview", background="#1E293B", fieldbackground="#1E293B", foreground="#F8FAFC")
        style.configure("Treeview.Heading", background="#334155", foreground="#06B6D4", font=("Segoe UI", 9, "bold"))

        # Combobox style customization
        style.configure("TCombobox", fieldbackground="#1E293B", foreground="#FFFFFF", background="#334155", arrowcolor="#FFFFFF")
        style.map("TCombobox", fieldbackground=[("readonly", "#1E293B")], foreground=[("readonly", "#FFFFFF")])
        
        # Entry style customization
        style.configure("TEntry", fieldbackground="#1E293B", foreground="#FFFFFF", insertcolor="#FFFFFF")
        
        # Option database overrides for the dropdown listboxes (highly critical for Windows Clam theme)
        self.root.option_add("*TCombobox*Listbox.background", "#1E293B")
        self.root.option_add("*TCombobox*Listbox.foreground", "#FFFFFF")
        self.root.option_add("*TCombobox*Listbox.selectBackground", "#3B82F6")
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#FFFFFF")

    def build_ui(self):
        # Main horizontal split
        main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        
        # Left Panel (Controls and readouts)
        left_container = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(left_container, weight=1)
        
        # 1. Dual Serial Connection Card
        conn_frame = ttk.LabelFrame(left_container, text="1. 機器接続設定", padding=10)
        conn_frame.pack(fill=tk.X, pady=4, padx=5)
        
        # Sub-grid for Dual Serial
        conn_grid = ttk.Frame(conn_frame)
        conn_grid.pack(fill=tk.X)
        
        # Column 0: Pitot Board
        pitot_conn_frame = ttk.LabelFrame(conn_grid, text="ピトー管基板", padding=5)
        pitot_conn_frame.grid(row=0, column=0, padx=4, sticky=tk.NSEW)
        
        ttk.Label(pitot_conn_frame, text="COMポート:").pack(anchor=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(pitot_conn_frame, textvariable=self.port_var, values=self.get_serial_ports(), width=10)
        self.port_combo.pack(fill=tk.X, pady=2)
        
        ttk.Label(pitot_conn_frame, text="ボーレート:").pack(anchor=tk.W)
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(pitot_conn_frame, textvariable=self.baud_var, values=["9600", "38400", "57600", "115200"], width=10)
        self.baud_combo.pack(fill=tk.X, pady=2)
        
        self.btn_connect = ttk.Button(pitot_conn_frame, text="接続する", command=self.toggle_connection, style="Action.TButton")
        self.btn_connect.pack(fill=tk.X, pady=4)
        
        # Column 1: DT-8920 Reference Meter
        dt_conn_frame = ttk.LabelFrame(conn_grid, text="基準計 (DT-8920)", padding=5)
        dt_conn_frame.grid(row=0, column=1, padx=4, sticky=tk.NSEW)
        
        ttk.Label(dt_conn_frame, text="COMポート:").pack(anchor=tk.W)
        self.dt_port_var = tk.StringVar()
        self.dt_port_combo = ttk.Combobox(dt_conn_frame, textvariable=self.dt_port_var, values=self.get_serial_ports(), width=10)
        self.dt_port_combo.pack(fill=tk.X, pady=2)
        
        ttk.Label(dt_conn_frame, text="ボーレート:").pack(anchor=tk.W)
        self.dt_baud_var = tk.StringVar(value="9600")
        self.dt_baud_combo = ttk.Combobox(dt_conn_frame, textvariable=self.dt_baud_var, values=["2400", "9600", "19200", "115200"], width=10)
        self.dt_baud_combo.pack(fill=tk.X, pady=2)
        
        self.btn_dt_connect = ttk.Button(dt_conn_frame, text="接続する", command=self.toggle_dt_connection, style="Action.TButton")
        self.btn_dt_connect.pack(fill=tk.X, pady=4)
        
        # Global COM Port Refresh
        btn_refresh = ttk.Button(conn_frame, text="ポート再検出", command=self.refresh_ports, style="Action.TButton")
        btn_refresh.pack(fill=tk.X, pady=4)
        
        # 2. DT-8920 Parsing Parameter Settings Card
        dt_param_frame = ttk.LabelFrame(left_container, text="2. 基準計 (DT-8920) 設定＆モニター", padding=10)
        dt_param_frame.pack(fill=tk.X, pady=4, padx=5)
        
        ttk.Label(dt_param_frame, text="測定値の対象:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.dt_mode_var = tk.StringVar(value="speed")
        r_dt_speed = ttk.Radiobutton(dt_param_frame, text="対気速度 (m/s)", variable=self.dt_mode_var, value="speed")
        r_dt_speed.grid(row=0, column=1, sticky=tk.W, padx=5)
        r_dt_press = ttk.Radiobutton(dt_param_frame, text="差圧 (Pa)", variable=self.dt_mode_var, value="pressure")
        r_dt_press.grid(row=0, column=2, sticky=tk.W, padx=5)
        
        ttk.Label(dt_param_frame, text="解析デコーダ:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.dt_decoder_var = tk.StringVar(value="ascii")
        self.dt_decoder_combo = ttk.Combobox(dt_param_frame, textvariable=self.dt_decoder_var, values=["ascii", "cem_be", "cem_le"], width=12)
        self.dt_decoder_combo.grid(row=1, column=1, columnspan=2, sticky=tk.W, padx=5)
        self.dt_decoder_combo.bind("<<ComboboxSelected>>", self.on_dt_decoder_change)
        
        ttk.Label(dt_param_frame, text="生受信バイト:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.lbl_dt_monitor = ttk.Label(dt_param_frame, text="-- -- -- -- -- -- -- --", font=("Consolas", 9), foreground="#E2E8F0")
        self.lbl_dt_monitor.grid(row=2, column=1, columnspan=2, sticky=tk.W, padx=5)
        
        btn_diag = ttk.Button(dt_param_frame, text="通信診断ツールを開く", command=self.open_diagnostic_window, style="Action.TButton")
        btn_diag.grid(row=3, column=0, columnspan=3, pady=6, sticky=tk.EW)
        
        # 3. Environment settings
        env_frame = ttk.LabelFrame(left_container, text="3. 風洞環境パラメータ (空気密度用)", padding=10)
        env_frame.pack(fill=tk.X, pady=4, padx=5)
        
        ttk.Label(env_frame, text="現地大気圧 (hPa):").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.press_entry = ttk.Entry(env_frame, width=10)
        self.press_entry.insert(0, "1013.25")
        self.press_entry.grid(row=0, column=1, pady=2, padx=5, sticky=tk.W)
        
        # Temp & Humid overrides
        self.temp_override_var = tk.BooleanVar(value=True)
        self.chk_temp_override = ttk.Checkbutton(env_frame, text="温度を手動入力 (°C)", variable=self.temp_override_var)
        self.chk_temp_override.grid(row=1, column=0, sticky=tk.W, pady=2)
        self.temp_entry = ttk.Entry(env_frame, width=10)
        self.temp_entry.insert(0, "20.0")
        self.temp_entry.grid(row=1, column=1, pady=2, padx=5, sticky=tk.W)
        
        self.humid_override_var = tk.BooleanVar(value=True)
        self.chk_humid_override = ttk.Checkbutton(env_frame, text="湿度を手動入力 (%)", variable=self.humid_override_var)
        self.chk_humid_override.grid(row=2, column=0, sticky=tk.W, pady=2)
        self.humid_entry = ttk.Entry(env_frame, width=10)
        self.humid_entry.insert(0, "50.0")
        self.humid_entry.grid(row=2, column=1, pady=2, padx=5, sticky=tk.W)

        # 4. Wind Tunnel Calibration Run Card
        calib_frame = ttk.LabelFrame(left_container, text="4. 風洞キャリブレーション実行", padding=10)
        calib_frame.pack(fill=tk.X, pady=4, padx=5)
        
        # STEP A: Aerodynamic Zero Calibration
        ttk.Label(calib_frame, text="ステップA: 風中零点校正 (0°アライメント)", font=("Segoe UI", 9, "bold"), foreground="#06B6D4").pack(anchor=tk.W, pady=2)
        self.btn_zero_calib = ttk.Button(calib_frame, text="0°基準オフセット計測 (5秒平均)", command=self.run_zero_calibration, style="Action.TButton")
        self.btn_zero_calib.pack(fill=tk.X, pady=3)
        
        offset_labels_frame = ttk.Frame(calib_frame)
        offset_labels_frame.pack(fill=tk.X, pady=2)
        self.lbl_aoa_offset = ttk.Label(offset_labels_frame, text="AoAオフセット: 0.00 Pa", font=("Segoe UI", 9), foreground="#10B981")
        self.lbl_aoa_offset.pack(side=tk.LEFT, padx=5)
        self.lbl_aos_offset = ttk.Label(offset_labels_frame, text="AoSオフセット: 0.00 Pa", font=("Segoe UI", 9), foreground="#EC4899")
        self.lbl_aos_offset.pack(side=tk.RIGHT, padx=5)
        
        ttk.Separator(calib_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        
        # STEP B: Angle Sweep Data Points
        ttk.Label(calib_frame, text="ステップB: 計測ポイントの追加 (BNO姿勢角と同期)", font=("Segoe UI", 9, "bold"), foreground="#06B6D4").pack(anchor=tk.W, pady=2)
        
        self.btn_record_point = ttk.Button(calib_frame, text="現在のポジションを記録 (3秒平均)", command=self.record_sweep_point, style="Start.TButton")
        self.btn_record_point.pack(fill=tk.X, pady=4)
        
        self.lbl_status = ttk.Label(calib_frame, text="ステータス: 待機中", font=("Segoe UI", 10, "bold"), foreground="#94A3B8")
        self.lbl_status.pack(pady=4)
        
        # 5. Real-time Value Readouts Card (Pitot & DT-8920 side-by-side comparison)
        readout_frame = ttk.LabelFrame(left_container, text="5. リアルタイム比較・数値表示", padding=10)
        readout_frame.pack(fill=tk.BOTH, expand=True, pady=4, padx=5)
        
        self.readout_labels = {}
        vars_to_show = [
            ("ピトー管 対気速度", "airspeed", " m/s", "#3B82F6"),
            ("基準計 (DT-8920) 速度", "dt_speed", " m/s", "#06B6D4"),
            ("速度測定誤差 (ピトー - 基準)", "speed_error", " m/s", "#F8FAFC"),
            ("ピトー管 AoA差圧 (生値)", "aoa_pa", " Pa", "#10B981"),
            ("ピトー管 AoS差圧 (生値)", "aos_pa", " Pa", "#EC4899"),
            ("基準計 (DT-8920) 差圧", "dt_press", " Pa", "#06B6D4"),
            ("差圧測定誤差 (ピトー - 基準)", "press_error", " Pa", "#F8FAFC"),
            ("BNO055 ピッチ角 (AoA基準)", "pitch", "°", "#F59E0B"),
            ("BNO055 ヨー角 (AoS基準)", "yaw", "°", "#F59E0B"),
        ]
        
        for idx, (label, key, unit, color) in enumerate(vars_to_show):
            ttk.Label(readout_frame, text=label+":", font=("Segoe UI", 9)).grid(row=idx, column=0, sticky=tk.W, pady=1)
            lbl_val = ttk.Label(readout_frame, text="0.00"+unit, font=("Segoe UI", 10, "bold"), foreground=color)
            lbl_val.grid(row=idx, column=1, sticky=tk.E, pady=1, padx=15)
            self.readout_labels[key] = (lbl_val, unit)
            
        # Right Panel
        right_container = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(right_container, weight=3)
        
        self.notebook = ttk.Notebook(right_container)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Tab 1: Real-time Trends
        tab_trends = ttk.Frame(self.notebook, style="TFrame")
        self.notebook.add(tab_trends, text="対比リアルタイムグラフ")
        self.setup_trends_plot(tab_trends)
        
        # Tab 2: Calibration Sweep & Regression
        tab_calib = ttk.Frame(self.notebook, style="TFrame")
        self.notebook.add(tab_calib, text="ゲッチンゲン校正モード")
        self.setup_calibration_tab(tab_calib)

    def setup_trends_plot(self, parent):
        self.fig_trends = Figure(figsize=(8, 6), dpi=100, facecolor="#0F172A")
        
        # Subplot 1: Speed and Pressures comparison
        self.ax_press = self.fig_trends.add_subplot(211)
        self.ax_press.set_facecolor("#1E293B")
        self.ax_press.set_title("対気速度・差圧のリアルタイム対比 (実線: ピトー / 点線: DT-8920)", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_press.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_press.grid(True, color="#334155", linestyle=":")
        
        self.line_speed, = self.ax_press.plot([], [], label="ピトー管速度 (m/s)", color="#3B82F6", lw=2)
        self.line_dt_speed, = self.ax_press.plot([], [], label="基準計DT-8920速度 (m/s)", color="#06B6D4", lw=1.5, linestyle="--")
        
        self.line_aoa_p, = self.ax_press.plot([], [], label="ピトー管AoA差圧 (Pa)", color="#10B981", lw=1)
        self.line_dt_press, = self.ax_press.plot([], [], label="基準計DT-8920差圧 (Pa)", color="#EC4899", lw=1, linestyle=":")
        
        self.ax_press.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8)
        
        # Subplot 2: IMU
        self.ax_att = self.fig_trends.add_subplot(212)
        self.ax_att.set_facecolor("#1E293B")
        self.ax_att.set_title("BNO055 姿勢角度推移", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_att.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_att.grid(True, color="#334155", linestyle=":")
        
        self.line_pitch, = self.ax_att.plot([], [], label="ピッチ角 (Pitch) (°)", color="#F59E0B", lw=2)
        self.line_yaw, = self.ax_att.plot([], [], label="ヨー角 (Yaw) (°)", color="#10B981", lw=2)
        self.ax_att.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8)
        
        self.fig_trends.tight_layout()
        
        self.canvas_trends = FigureCanvasTkAgg(self.fig_trends, master=parent)
        self.canvas_trends.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_calibration_tab(self, parent):
        # Container
        split_frame = ttk.Frame(parent, style="TFrame")
        split_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left Side: Sweep points table and controls
        left_side = ttk.Frame(split_frame, style="TFrame")
        left_side.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        # Treeview (points grid)
        table_frame = ttk.LabelFrame(left_side, text="記録された測定ポイント一覧 (3秒時間平均化)", padding=5)
        table_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Calibration reference option
        self.use_ref_speed_var = tk.BooleanVar(value=True)
        chk_use_ref = ttk.Checkbutton(table_frame, text="校正時の動圧(q)基準として DT-8920 の対気速度を使用する (推奨)", variable=self.use_ref_speed_var)
        chk_use_ref.pack(anchor=tk.W, pady=2)
        
        columns = ("id", "speed", "dt_speed", "aoa_p", "aos_p", "pitch", "yaw")
        self.tree_points = ttk.Treeview(table_frame, columns=columns, show="headings", height=8)
        
        self.tree_points.heading("id", text="ID")
        self.tree_points.heading("speed", text="ピトー速度 (m/s)")
        self.tree_points.heading("dt_speed", text="基準速度 (m/s)")
        self.tree_points.heading("aoa_p", text="補正後 AoA差圧 (Pa)")
        self.tree_points.heading("aos_p", text="補正後 AoS差圧 (Pa)")
        self.tree_points.heading("pitch", text="BNOピッチ角 (°)")
        self.tree_points.heading("yaw", text="BNOヨー角 (°)")
        
        self.tree_points.column("id", width=35, anchor=tk.CENTER)
        self.tree_points.column("speed", width=95, anchor=tk.CENTER)
        self.tree_points.column("dt_speed", width=95, anchor=tk.CENTER)
        self.tree_points.column("aoa_p", width=105, anchor=tk.CENTER)
        self.tree_points.column("aos_p", width=105, anchor=tk.CENTER)
        self.tree_points.column("pitch", width=95, anchor=tk.CENTER)
        self.tree_points.column("yaw", width=95, anchor=tk.CENTER)
        
        self.tree_points.pack(fill=tk.BOTH, expand=True)
        
        # Sweep table buttons
        tbl_btn_box = ttk.Frame(left_side, style="TFrame")
        tbl_btn_box.pack(fill=tk.X, pady=5)
        
        self.btn_clear_points = ttk.Button(tbl_btn_box, text="選択した行を削除", command=self.clear_selected_point, style="Stop.TButton")
        self.btn_clear_points.pack(side=tk.LEFT, padx=5)
        
        self.btn_clear_all = ttk.Button(tbl_btn_box, text="全データをクリア", command=self.clear_all_points, style="Stop.TButton")
        self.btn_clear_all.pack(side=tk.LEFT, padx=5)
        
        self.btn_calc_fit = ttk.Button(tbl_btn_box, text="校正係数 (K) の算出実行", command=self.calculate_regression, style="Action.TButton")
        self.btn_calc_fit.pack(side=tk.RIGHT, padx=5)
        
        # Text Report Frame
        report_frame = ttk.LabelFrame(left_side, text="校正計算レポート ＆ 換算式", padding=10)
        report_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.txt_report = tk.Text(report_frame, bg="#1E293B", fg="#F8FAFC", font=("Consolas", 9), wrap=tk.WORD, height=10, borderwidth=0)
        self.txt_report.pack(fill=tk.BOTH, expand=True)
        self.txt_report.insert(tk.END, "測定ポイントを3点以上記録したのち、上記の『校正係数 (K) の算出実行』をクリックしてください。\n")
        
        btn_box = ttk.Frame(report_frame, style="TFrame")
        btn_box.pack(fill=tk.X, pady=4)
        
        self.btn_export_csv = ttk.Button(btn_box, text="CSVデータとして書き出し", command=self.export_csv, style="Action.TButton")
        self.btn_export_csv.pack(side=tk.LEFT, padx=5)
        self.btn_export_csv.state(["disabled"])
        
        self.btn_save_report = ttk.Button(btn_box, text="校正レポートを保存", command=self.save_report_file, style="Action.TButton")
        self.btn_save_report.pack(side=tk.LEFT, padx=5)
        self.btn_save_report.state(["disabled"])
        
        # Right Side: Scatter plot for calibration (Pitch vs AoA ratio, Yaw vs AoS ratio)
        self.fig_calib = Figure(figsize=(5, 6), dpi=100, facecolor="#0F172A")
        
        self.ax_calib_aoa = self.fig_calib.add_subplot(211)
        self.ax_calib_aoa.set_facecolor("#1E293B")
        self.ax_calib_aoa.set_title("ピッチ角 vs. AoA差圧比", color="#06B6D4", fontsize=9, fontweight="bold")
        self.ax_calib_aoa.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_calib_aoa.grid(True, color="#334155", linestyle=":")
        
        self.ax_calib_aos = self.fig_calib.add_subplot(212)
        self.ax_calib_aos.set_facecolor("#1E293B")
        self.ax_calib_aos.set_title("ヨー角 vs. AoS差圧比", color="#06B6D4", fontsize=9, fontweight="bold")
        self.ax_calib_aos.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_calib_aos.grid(True, color="#334155", linestyle=":")
        
        self.fig_calib.tight_layout()
        
        self.canvas_calib = FigureCanvasTkAgg(self.fig_calib, master=split_frame)
        self.canvas_calib.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def refresh_ports(self):
        ports = self.get_serial_ports()
        self.port_combo["values"] = ports
        self.dt_port_combo["values"] = ports
        if ports:
            # Set default values if not already selected
            if not self.port_var.get():
                self.port_combo.current(0)
            if len(ports) > 1 and not self.dt_port_var.get():
                self.dt_port_combo.current(1)
            elif not self.dt_port_var.get():
                self.dt_port_combo.current(0)

    def toggle_connection(self):
        if self.serial_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_var.get()
        baud = self.baud_var.get()
        
        if not port:
            messagebox.showerror("接続エラー", "ピトー管基板のCOMポートを選択してください。")
            return
            
        try:
            self.ser = serial.Serial(port, int(baud), timeout=1)
            self.ser.dtr = True
            self.ser.rts = True
            self.serial_connected = True
            self.btn_connect.configure(text="接続を切断", style="Stop.TButton")
            self.lbl_status.configure(text="ピトー管: 接続完了", foreground="#10B981")
            
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()
        except Exception as e:
            messagebox.showerror("接続エラー", f"ピトー管ポート {port} を開けませんでした:\n{str(e)}")

    def disconnect_serial(self):
        self.serial_connected = False
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
            
        self.btn_connect.configure(text="接続する", style="Action.TButton")
        self.lbl_status.configure(text="ピトー管: 切断", foreground="#EF4444")
        self.calibrating_zero = False
        self.recording_point = False

    def toggle_dt_connection(self):
        if self.dt_connected:
            self.disconnect_dt_serial()
        else:
            self.connect_dt_serial()

    def connect_dt_serial(self):
        port = self.dt_port_var.get()
        baud = self.dt_baud_var.get()
        
        if not port:
            messagebox.showerror("接続エラー", "基準計 DT-8920 のCOMポートを選択してください。")
            return
            
        try:
            # 8N1 configuration is standard
            self.dt_ser = serial.Serial(
                port=port,
                baudrate=int(baud),
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5
            )
            self.dt_ser.dtr = True
            self.dt_ser.rts = True
            self.dt_connected = True
            self.btn_dt_connect.configure(text="接続を切断", style="Stop.TButton")
            self.dt_raw_monitor_str = "接続完了 (受信待機中)"
            
            self.dt_read_thread = threading.Thread(target=self.dt8920_read_loop, daemon=True)
            self.dt_read_thread.start()
        except Exception as e:
            messagebox.showerror("接続エラー", f"基準計ポート {port} を開けませんでした:\n{str(e)}")

    def disconnect_dt_serial(self):
        self.dt_connected = False
        if self.dt_ser:
            try:
                self.dt_ser.close()
            except:
                pass
            self.dt_ser = None
            
        self.btn_dt_connect.configure(text="接続する", style="Action.TButton")
        self.dt_raw_monitor_str = "未接続"
        self.dt_data = {"airspeed": 0.0, "pressure": 0.0}

    def serial_read_loop(self):
        buffer = ""
        while self.serial_connected:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    chars = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += chars
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith(">"):
                            self.parse_teleplot_line(line)
            except Exception as e:
                self.serial_connected = False
                self.root.after(0, self.on_serial_loss)
                break
            time.sleep(0.005)

    def on_serial_loss(self):
        self.disconnect_serial()
        messagebox.showwarning("接続切断", "ピトー管基板とのシリアル通信が切断されました。")

    def parse_teleplot_line(self, line):
        try:
            parts = line[1:].split(":")
            if len(parts) == 2:
                key, val_str = parts[0], parts[1]
                val = float(val_str)
                
                if key == "pitot_airspeed":
                    self.current_data["airspeed"] = val
                elif key == "pitot_aoa":
                    self.current_data["aoa_pa"] = val
                elif key == "pitot_aos":
                    self.current_data["aos_pa"] = val
                elif key == "pitot_pitch":
                    self.current_data["pitch"] = val
                elif key == "pitot_roll":
                    self.current_data["roll"] = val
                elif key == "pitot_yaw":
                    self.current_data["yaw"] = val
                elif key == "pitot_temp":
                    self.current_data["temp"] = val
                elif key == "pitot_humid":
                    self.current_data["humid"] = val
        except:
            pass

    # Background thread reader for DT-8920 reference meter
    def dt8920_read_loop(self):
        buffer = b""
        while self.dt_connected:
            try:
                if self.dt_ser and self.dt_ser.is_open:
                    if self.dt_ser.in_waiting > 0:
                        raw = self.dt_ser.read(self.dt_ser.in_waiting)
                        buffer += raw
                        self.dt_last_bytes = raw
                        
                        # Process buffer
                        while True:
                            # Find header
                            header_idx = buffer.find(b'\xAA\xBB')
                            if header_idx == -1:
                                # No header found, clear buffer except possibly the last byte if it's \xAA
                                if len(buffer) > 0 and buffer[-1] == 0xAA:
                                    buffer = b'\xAA'
                                else:
                                    buffer = b""
                                break
                            
                            # Discard bytes before header
                            if header_idx > 0:
                                buffer = buffer[header_idx:]
                                
                            # Check if we have a full packet (46 bytes)
                            if len(buffer) < 46:
                                break
                                
                            # We have at least 46 bytes!
                            packet = buffer[:46]
                            buffer = buffer[46:]
                            
                            # Decode packet
                            try:
                                # Header: packet[0:4] = \xAA\xBB\x03\x01
                                temp = struct.unpack('<f', packet[4:8])[0] * 10.0
                                velocity = struct.unpack('<f', packet[8:12])[0]
                                flow = struct.unpack('<f', packet[12:16])[0]
                                pressure_mbar = struct.unpack('<f', packet[16:20])[0]
                                pressure_pa = pressure_mbar * 100.0
                                
                                # Update values
                                if self.dt_mode_var.get() == "speed":
                                    self.update_dt_value(velocity)
                                else:
                                    self.update_dt_value(pressure_pa)
                                    
                                # Display in Raw Monitor
                                self.dt_raw_monitor_str = f"風速:{velocity:.2f}m/s 差圧:{pressure_pa:.1f}Pa 気温:{temp:.1f}°C"
                            except Exception as parse_ex:
                                self.dt_raw_monitor_str = "パケット解析エラー"
            except Exception as e:
                self.dt_connected = False
                self.root.after(0, self.on_dt_serial_loss)
                break
            time.sleep(0.01)

    def update_dt_value(self, val):
        if self.dt_mode_var.get() == "speed":
            self.dt_data["airspeed"] = val
            # If in speed mode, set pressure to 0
            self.dt_data["pressure"] = 0.0
        else:
            self.dt_data["pressure"] = val
            # Calculate corresponding airspeed from reference pressure (dynamic pressure q)
            temp = self.get_temperature()
            humid = self.get_humidity()
            p_hpa = self.get_pressure()
            rho = self.calculate_density(temp, humid, p_hpa)
            if val > 0:
                self.dt_data["airspeed"] = np.sqrt(2 * val / rho)
            else:
                self.dt_data["airspeed"] = 0.0

    def on_dt_decoder_change(self, event):
        # Reset buffer on decoder change
        pass

    def on_dt_serial_loss(self):
        self.disconnect_dt_serial()
        messagebox.showwarning("接続切断", "基準計 DT-8920 とのシリアル通信が切断されました。")

    def get_temperature(self):
        if self.temp_override_var.get():
            try:
                return float(self.temp_entry.get())
            except:
                return 20.0
        return self.current_data["temp"]

    def get_humidity(self):
        if self.humid_override_var.get():
            try:
                return float(self.humid_entry.get())
            except:
                return 50.0
        return self.current_data["humid"]

    def get_pressure(self):
        try:
            return float(self.press_entry.get())
        except:
            return 1013.25

    def calculate_density(self, temp, humid, p_hpa):
        tk = temp + 273.15
        p_pa = p_hpa * 100.0
        p_sat = 610.78 * (10 ** (7.5 * temp / (temp + 237.3)))
        p_v = (humid / 100.0) * p_sat
        p_d = p_pa - p_v
        
        Rd = 287.058
        Rv = 461.495
        
        rho = (p_d / (Rd * tk)) + (p_v / (Rv * tk))
        return rho

    # Step A: Aerodynamic Zero Calibration
    def run_zero_calibration(self):
        if not self.serial_connected:
            messagebox.showwarning("接続エラー", "先にピトー管基板へ接続してください。")
            return
            
        self.calibrating_zero = True
        self.btn_zero_calib.state(["disabled"])
        self.btn_record_point.state(["disabled"])
        self.lbl_status.configure(text="0°での風中基準オフセットを収集中 (5秒間平均)...", foreground="#3B82F6")
        
        threading.Thread(target=self.collect_zero_samples, daemon=True).start()

    def collect_zero_samples(self):
        aoa_samples = []
        aos_samples = []
        start_time = time.time()
        
        while time.time() - start_time < 5.0:
            if self.serial_connected:
                aoa_samples.append(self.current_data["aoa_pa"])
                aos_samples.append(self.current_data["aos_pa"])
            time.sleep(0.1)
            
        if aoa_samples and self.serial_connected:
            self.aoa_zero_offset = np.mean(aoa_samples)
            self.aos_zero_offset = np.mean(aos_samples)
            self.root.after(0, self.update_zero_labels)
            
        self.root.after(0, self.on_zero_calibration_complete)

    def update_zero_labels(self):
        self.lbl_aoa_offset.configure(text=f"AoAオフセット: {self.aoa_zero_offset:.2f} Pa")
        self.lbl_aos_offset.configure(text=f"AoSオフセット: {self.aos_zero_offset:.2f} Pa")

    def on_zero_calibration_complete(self):
        self.calibrating_zero = False
        self.btn_zero_calib.state(["!disabled"])
        self.btn_record_point.state(["!disabled"])
        self.lbl_status.configure(text="零点校正完了", foreground="#10B981")
        messagebox.showinfo("零点校正完了", f"風中零点オフセットを登録しました：\nAoAオフセット: {self.aoa_zero_offset:.2f} Pa\nAoSオフセット: {self.aos_zero_offset:.2f} Pa")

    # Step B: Record Sweep Point (Filters vibrations by averaging)
    def record_sweep_point(self):
        if not self.serial_connected:
            messagebox.showwarning("接続エラー", "先にピトー管基板へ接続してください。")
            return
            
        self.recording_point = True
        self.btn_zero_calib.state(["disabled"])
        self.btn_record_point.state(["disabled"])
        self.lbl_status.configure(text="風洞の揺らぎとBNO角度、基準計を平均化処理中 (3秒)...", foreground="#3B82F6")
        
        threading.Thread(target=self.collect_sweep_samples, daemon=True).start()

    def collect_sweep_samples(self):
        samples = []
        start_time = time.time()
        
        while time.time() - start_time < 3.0:
            if self.serial_connected:
                # Correct raw pressure values with aerodynamic 0° offset
                aoa_corr = self.current_data["aoa_pa"] - self.aoa_zero_offset
                aos_corr = self.current_data["aos_pa"] - self.aos_zero_offset
                
                samples.append({
                    "speed": self.current_data["airspeed"],
                    "dt_speed": self.dt_data["airspeed"],
                    "aoa_corr": aoa_corr,
                    "aos_corr": aos_corr,
                    "pitch": self.current_data["pitch"],
                    "yaw": self.current_data["yaw"]
                })
            time.sleep(0.1) # 10Hz
            
        if samples and self.serial_connected:
            # Average variables over the 3-second block to cancel vibrations
            avg_speed = np.mean([s["speed"] for s in samples])
            avg_dt_speed = np.mean([s["dt_speed"] for s in samples])
            avg_aoa_corr = np.mean([s["aoa_corr"] for s in samples])
            avg_aos_corr = np.mean([s["aos_corr"] for s in samples])
            avg_pitch = np.mean([s["pitch"] for s in samples])
            avg_yaw = np.mean([s["yaw"] for s in samples])
            
            point = {
                "avg_speed": avg_speed,
                "avg_dt_speed": avg_dt_speed,
                "avg_aoa_corr": avg_aoa_corr,
                "avg_aos_corr": avg_aos_corr,
                "avg_pitch": avg_pitch,
                "avg_yaw": avg_yaw
            }
            self.sweep_points.append(point)
            
            # Insert into Treeview table
            self.root.after(0, lambda: self.add_point_to_treeview(point))
            
        self.root.after(0, self.on_sweep_recording_complete)

    def add_point_to_treeview(self, pt):
        idx = len(self.sweep_points)
        self.tree_points.insert("", "end", values=(
            idx,
            f"{pt['avg_speed']:.2f}",
            f"{pt['avg_dt_speed']:.2f}",
            f"{pt['avg_aoa_corr']:.2f}",
            f"{pt['avg_aos_corr']:.2f}",
            f"{pt['avg_pitch']:.2f}°",
            f"{pt['avg_yaw']:.2f}°"
        ))

    def on_sweep_recording_complete(self):
        self.recording_point = False
        self.btn_zero_calib.state(["!disabled"])
        self.btn_record_point.state(["!disabled"])
        self.lbl_status.configure(text="計測ポイントを追加しました", foreground="#10B981")

    def clear_selected_point(self):
        selected_items = self.tree_points.selection()
        if not selected_items:
            return
        for item in selected_items:
            vals = self.tree_points.item(item, "values")
            idx = int(vals[0]) - 1
            if 0 <= idx < len(self.sweep_points):
                self.sweep_points.pop(idx)
            self.tree_points.delete(item)
            
        # Re-index table
        self.rebuild_treeview()

    def clear_all_points(self):
        if messagebox.askyesno("全データクリア", "記録されたすべての校正用ポイントを削除しますか？"):
            self.sweep_points = []
            for item in self.tree_points.get_children():
                self.tree_points.delete(item)

    def rebuild_treeview(self):
        for item in self.tree_points.get_children():
            self.tree_points.delete(item)
        for idx, pt in enumerate(self.sweep_points):
            self.tree_points.insert("", "end", values=(
                idx + 1,
                f"{pt['avg_speed']:.2f}",
                f"{pt['avg_dt_speed']:.2f}",
                f"{pt['avg_aoa_corr']:.2f}",
                f"{pt['avg_aos_corr']:.2f}",
                f"{pt['avg_pitch']:.2f}°",
                f"{pt['avg_yaw']:.2f}°"
            ))

    # Regression and report generation
    def calculate_regression(self):
        if len(self.sweep_points) < 3:
            messagebox.showwarning("データ不足", "回帰計算を行うには最低3点の計測データが必要です。")
            return
            
        # Extract environment
        temp = self.get_temperature()
        humid = self.get_humidity()
        p_hpa = self.get_pressure()
        rho = self.calculate_density(temp, humid, p_hpa)
        
        # Extract variables
        pitches = [p["avg_pitch"] for p in self.sweep_points]
        yaws = [p["avg_yaw"] for p in self.sweep_points]
        aoas = [p["avg_aoa_corr"] for p in self.sweep_points]
        aoss = [p["avg_aos_corr"] for p in self.sweep_points]
        
        # Decide reference speed source
        use_dt = self.use_ref_speed_var.get() and self.dt_connected
        
        # Calculate dynamic pressure q for each sweep point
        qs = []
        for p in self.sweep_points:
            ref_speed = p["avg_dt_speed"] if use_dt else p["avg_speed"]
            q = 0.5 * rho * (ref_speed ** 2)
            qs.append(q)
            
        # Prepare regression arrays: Ratio (DP_corrected / q) vs Real Angle from BNO055
        x_aoa = []
        y_pitch = []
        x_aos = []
        y_yaw = []
        
        for idx in range(len(self.sweep_points)):
            if qs[idx] > 5.0: # filter out low speeds
                x_aoa.append(aoas[idx] / qs[idx])
                y_pitch.append(pitches[idx])
                x_aos.append(aoss[idx] / qs[idx])
                y_yaw.append(yaws[idx])
                
        aoa_valid = len(x_aoa) >= 3
        aos_valid = len(x_aos) >= 3
        
        k_aoa = 0.0
        offset_aoa = 0.0
        r2_aoa = 0.0
        
        if aoa_valid:
            slope, intercept = np.polyfit(x_aoa, y_pitch, 1)
            k_aoa = slope
            offset_aoa = intercept
            y_pred = [slope * x + intercept for x in x_aoa]
            ss_tot = np.sum((y_pitch - np.mean(y_pitch))**2)
            ss_res = np.sum((y_pitch - y_pred)**2)
            r2_aoa = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
            
        k_aos = 0.0
        offset_aos = 0.0
        r2_aos = 0.0
        
        if aos_valid:
            slope, intercept = np.polyfit(x_aos, y_yaw, 1)
            k_aos = slope
            offset_aos = intercept
            y_pred = [slope * x + intercept for x in x_aos]
            ss_tot = np.sum((y_yaw - np.mean(y_yaw))**2)
            ss_res = np.sum((y_yaw - y_pred)**2)
            r2_aos = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

        # Construct Report in Japanese
        report = []
        report.append("=========================================")
        report.append("  TEAM ЯTR - ピトー管風洞キャリブレーション報告書")
        report.append("  (BNO055姿勢角 ＆ 基準計 DT-8920 対比方式)")
        report.append("=========================================")
        report.append(f"レポート生成日時: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"測定ポイント数: {len(self.sweep_points)}")
        report.append(f"基準対気速度源: {'DT-8920 (基準外部計)' if use_dt else '自車ピトー管測定値'}")
        report.append("-----------------------------------------")
        report.append("風洞環境および零点オフセット設定:")
        report.append(f"  設定大気圧:     {p_hpa:.2f} hPa")
        report.append(f"  手動基準温度:   {temp:.1f} °C")
        report.append(f"  手動基準湿度:   {humid:.1f} %")
        report.append(f"  算出空気密度 (ρ): {rho:.4f} kg/m³")
        report.append(f"  迎角(AoA)風中零点オフセット差圧: {self.aoa_zero_offset:.2f} Pa")
        report.append(f"  横滑り角(AoS)風中零点オフセット差圧: {self.aos_zero_offset:.2f} Pa")
        report.append("-----------------------------------------")
        report.append("記録された測定ポイント（風中の脈動・振動を平均化済み）:")
        for i, pt in enumerate(self.sweep_points):
            ref_v = pt['avg_dt_speed'] if use_dt else pt['avg_speed']
            report.append(f"  点 {i+1:2d} | 基準速度: {ref_v:5.2f} m/s | ピトー速度: {pt['avg_speed']:5.2f} m/s | DP_AoA: {pt['avg_aoa_corr']:6.2f} Pa | DP_AoS: {pt['avg_aos_corr']:6.2f} Pa | BNO Pitch: {pt['avg_pitch']:5.2f}° | BNO Yaw: {pt['avg_yaw']:5.2f}°")
        report.append("-----------------------------------------")
        report.append("校正数式モデル（ 角度 ＝ K × (補正後差圧 / 動圧q) ＋ オフセット ）:")
        
        if aoa_valid:
            report.append(f"  【迎角 (AoA) 測定特性】")
            report.append(f"    迎角感度係数 K_AoA (傾き):  {k_aoa:.5f} °/(Pa/Pa)")
            report.append(f"    アライメントオフセット角:    {offset_aoa:.3f} °")
            report.append(f"    決定係数 (適合度) R²:        {r2_aoa:.4f}")
        else:
            report.append("  AoA 特性: 動圧不足 (q > 5 Paを満たすポイントがありません)")
            
        if aos_valid:
            report.append(f"  【横滑り角 (AoS) 測定特性】")
            report.append(f"    横滑り角感度係数 K_AoS (傾き): {k_aos:.5f} °/(Pa/Pa)")
            report.append(f"    アライメントオフセット角:      {offset_aos:.3f} °")
            report.append(f"    決定係数 (適合度) R²:          {r2_aos:.4f}")
        else:
            report.append("  AoS 特性: 動圧不足 (q > 5 Paを満たすポイントがありません)")
            
        report.append("=========================================")
        
        self.txt_report.delete(1.0, tk.END)
        self.txt_report.insert(tk.END, "\n".join(report))
        
        self.report_text = "\n".join(report)
        self.btn_export_csv.state(["!disabled"])
        self.btn_save_report.state(["!disabled"])
        
        # Draw regression lines
        self.draw_calibration_plots(x_aoa, y_pitch, k_aoa, offset_aoa, x_aos, y_yaw, k_aos, offset_aos)

    def draw_calibration_plots(self, x_aoa, y_pitch, k_aoa, offset_aoa, x_aos, y_yaw, k_aos, offset_aos):
        self.ax_calib_aoa.clear()
        self.ax_calib_aoa.set_facecolor("#1E293B")
        self.ax_calib_aoa.set_title("ピッチ角 vs. AoA差圧比", color="#06B6D4", fontsize=9, fontweight="bold")
        self.ax_calib_aoa.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_calib_aoa.grid(True, color="#334155", linestyle=":")
        
        if x_aoa:
            self.ax_calib_aoa.scatter(x_aoa, y_pitch, color="#10B981", s=35, marker="o", label="測定平均データ")
            x_line = np.linspace(min(x_aoa), max(x_aoa), 100)
            y_line = k_aoa * x_line + offset_aoa
            self.ax_calib_aoa.plot(x_line, y_line, color="#EF4444", lw=1.5, label=f"回帰直線 (K={k_aoa:.3f})")
            self.ax_calib_aoa.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=7)
            self.ax_calib_aoa.set_xlabel("差圧比: 補正後AoA差圧 / 動圧 (q)", color="#94A3B8", fontsize=7)
            self.ax_calib_aoa.set_ylabel("ピッチ角 BNO Pitch (°)", color="#94A3B8", fontsize=7)
            
        self.ax_calib_aos.clear()
        self.ax_calib_aos.set_facecolor("#1E293B")
        self.ax_calib_aos.set_title("ヨー角 vs. AoS差圧比", color="#06B6D4", fontsize=9, fontweight="bold")
        self.ax_calib_aos.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_calib_aos.grid(True, color="#334155", linestyle=":")
        
        if x_aos:
            self.ax_calib_aos.scatter(x_aos, y_yaw, color="#EC4899", s=35, marker="o", label="測定平均データ")
            x_line = np.linspace(min(x_aos), max(x_aos), 100)
            y_line = k_aos * x_line + offset_aos
            self.ax_calib_aos.plot(x_line, y_line, color="#3B82F6", lw=1.5, label=f"回帰直線 (K={k_aos:.3f})")
            self.ax_calib_aos.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=7)
            self.ax_calib_aos.set_xlabel("差圧比: 補正後AoS差圧 / 動圧 (q)", color="#94A3B8", fontsize=7)
            self.ax_calib_aos.set_ylabel("ヨー角 BNO Yaw (°)", color="#94A3B8", fontsize=7)
            
        self.fig_calib.tight_layout()
        self.canvas_calib.draw()

    def export_csv(self):
        if not self.sweep_points:
            return
            
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSVファイル", "*.csv"), ("すべてのファイル", "*.*")],
            title="測定データをエクスポートする"
        )
        
        if not filepath:
            return
            
        try:
            import csv
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Point ID", "Averaged Pitot Airspeed (m/s)", "Averaged DT-8920 Reference Airspeed (m/s)",
                    "Averaged Corrected DP_AoA (Pa)", "Averaged Corrected DP_AoS (Pa)",
                    "Averaged BNO055 Pitch Angle (°)", "Averaged BNO055 Yaw Angle (°)"
                ])
                for idx, pt in enumerate(self.sweep_points):
                    writer.writerow([
                        idx + 1, f"{pt['avg_speed']:.3f}", f"{pt['avg_dt_speed']:.3f}",
                        f"{pt['avg_aoa_corr']:.3f}", f"{pt['avg_aos_corr']:.3f}",
                        f"{pt['avg_pitch']:.3f}", f"{pt['avg_yaw']:.3f}"
                    ])
            messagebox.showinfo("保存完了", f"CSVファイルを正常にエクスポートしました：\n{filepath}")
        except Exception as e:
            messagebox.showerror("保存エラー", f"CSVファイルを保存できませんでした:\n{str(e)}")

    def save_report_file(self):
        if not hasattr(self, 'report_text') or not self.report_text:
            return
            
        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("テキストファイル", "*.txt"), ("すべてのファイル", "*.*")],
            title="校正レポートを保存する"
        )
        
        if not filepath:
            return
            
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(self.report_text)
            messagebox.showinfo("保存完了", f"校正レポートを保存しました：\n{filepath}")
        except Exception as e:
            messagebox.showerror("保存エラー", f"レポートファイルを保存できませんでした:\n{str(e)}")

    def update_gui_loop(self):
        temp = self.get_temperature()
        humid = self.get_humidity()
        p_hpa = self.get_pressure()
        rho = self.calculate_density(temp, humid, p_hpa)
        
        # Calculate real-time comparison differences
        speed_diff = self.current_data["airspeed"] - self.dt_data["airspeed"]
        
        # Raw pressures offset subtraction
        pitot_aoa_corr = self.current_data["aoa_pa"] - self.aoa_zero_offset
        pitot_aos_corr = self.current_data["aos_pa"] - self.aos_zero_offset
        press_diff = (pitot_aoa_corr + pitot_aos_corr)/2.0 - self.dt_data["pressure"]
        
        self.readout_labels["airspeed"][0].configure(text=f"{self.current_data['airspeed']:.2f}{self.readout_labels['airspeed'][1]}")
        self.readout_labels["dt_speed"][0].configure(text=f"{self.dt_data['airspeed']:.2f}{self.readout_labels['dt_speed'][1]}")
        self.readout_labels["speed_error"][0].configure(text=f"{speed_diff:+.2f}{self.readout_labels['speed_error'][1]}")
        
        self.readout_labels["aoa_pa"][0].configure(text=f"{self.current_data['aoa_pa']:.2f}{self.readout_labels['aoa_pa'][1]}")
        self.readout_labels["aos_pa"][0].configure(text=f"{self.current_data['aos_pa']:.2f}{self.readout_labels['aos_pa'][1]}")
        self.readout_labels["dt_press"][0].configure(text=f"{self.dt_data['pressure']:.2f}{self.readout_labels['dt_press'][1]}")
        self.readout_labels["press_error"][0].configure(text=f"{press_diff:+.2f}{self.readout_labels['press_error'][1]}")
        
        self.readout_labels["pitch"][0].configure(text=f"{self.current_data['pitch']:.2f}{self.readout_labels['pitch'][1]}")
        self.readout_labels["yaw"][0].configure(text=f"{self.current_data['yaw']:.2f}{self.readout_labels['yaw'][1]}")
        
        # Update raw DT-8920 bytes monitor
        self.lbl_dt_monitor.configure(text=self.dt_raw_monitor_str)
        
        # History queue
        curr_t = time.time() - self.start_app_time
        self.history_time.append(curr_t)
        self.history_airspeed.append(self.current_data["airspeed"])
        self.history_aoa.append(self.current_data["aoa_pa"])
        self.history_aos.append(self.current_data["aos_pa"])
        self.history_pitch.append(self.current_data["pitch"])
        self.history_yaw.append(self.current_data["yaw"])
        
        # DT-8920 history
        self.history_dt_speed.append(self.dt_data["airspeed"])
        self.history_dt_press.append(self.dt_data["pressure"])
        
        if len(self.history_time) > self.plot_history_len:
            self.history_time.pop(0)
            self.history_airspeed.pop(0)
            self.history_aoa.pop(0)
            self.history_aos.pop(0)
            self.history_pitch.pop(0)
            self.history_yaw.pop(0)
            self.history_dt_speed.pop(0)
            self.history_dt_press.pop(0)
            
        # Draw trends graph
        if self.notebook.index(self.notebook.select()) == 0:
            self.redraw_trends_plot()
            
        self.root.after(100, self.update_gui_loop)

    def redraw_trends_plot(self):
        if not self.history_time:
            return
            
        self.line_speed.set_data(self.history_time, self.history_airspeed)
        self.line_dt_speed.set_data(self.history_time, self.history_dt_speed)
        
        self.line_aoa_p.set_data(self.history_time, self.history_aoa)
        self.line_dt_press.set_data(self.history_time, self.history_dt_press)
        
        self.ax_press.relim()
        self.ax_press.autoscale_view()
        
        self.line_pitch.set_data(self.history_time, self.history_pitch)
        self.line_yaw.set_data(self.history_time, self.history_yaw)
        self.ax_att.relim()
        self.ax_att.autoscale_view()
        
        self.canvas_trends.draw()

    def open_diagnostic_window(self):
        diag_win = tk.Toplevel(self.root)
        diag_win.title("DT-8920 通信診断・パケットキャプチャツール")
        diag_win.geometry("700x550")
        diag_win.configure(bg="#0F172A")
        
        # Style setup for diagnostic window
        ctrl_frame = ttk.LabelFrame(diag_win, text="診断コントロール", padding=10)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Grid layout for controls
        ttk.Label(ctrl_frame, text="COMポート:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        port_var = tk.StringVar(value=self.dt_port_var.get())
        port_combo = ttk.Combobox(ctrl_frame, textvariable=port_var, values=self.get_serial_ports(), width=10)
        port_combo.grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(ctrl_frame, text="ボーレート:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        baud_var = tk.StringVar(value="9600")
        baud_combo = ttk.Combobox(ctrl_frame, textvariable=baud_var, values=["9600", "19200", "38400", "57600", "115200"], width=10)
        baud_combo.grid(row=0, column=3, sticky=tk.W, padx=5, pady=2)
        
        dtr_var = tk.BooleanVar(value=True)
        dtr_check = ttk.Checkbutton(ctrl_frame, text="DTR", variable=dtr_var)
        dtr_check.grid(row=0, column=4, padx=5)
        
        rts_var = tk.BooleanVar(value=True)
        rts_check = ttk.Checkbutton(ctrl_frame, text="RTS", variable=rts_var)
        rts_check.grid(row=0, column=5, padx=5)
        
        # Connection status & control
        is_running = False
        diag_ser = None
        
        # Text log area
        log_frame = ttk.LabelFrame(diag_win, text="通信ログ (Hex / ASCII)", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        log_text = tk.Text(log_frame, bg="#1E293B", fg="#F8FAFC", insertbackground="#FFFFFF", font=("Consolas", 10))
        log_text.pack(fill=tk.BOTH, expand=True)
        
        def log_message(msg):
            log_text.insert(tk.END, msg + "\n")
            log_text.see(tk.END)
            
        def toggle_scan():
            nonlocal is_running, diag_ser
            if is_running:
                is_running = False
                if diag_ser:
                    try: diag_ser.close()
                    except: pass
                btn_start.configure(text="診断開始", style="Start.TButton")
                log_message("--- 診断停止 ---")
            else:
                p = port_var.get()
                b = baud_var.get()
                if not p:
                    messagebox.showerror("エラー", "COMポートを選択してください")
                    return
                try:
                    diag_ser = serial.Serial(p, int(b), timeout=0.1)
                    diag_ser.dtr = dtr_var.get()
                    diag_ser.rts = rts_var.get()
                    is_running = True
                    btn_start.configure(text="診断停止", style="Stop.TButton")
                    log_message(f"--- 接続開始 ({p} @ {b} bps, DTR={diag_ser.dtr}, RTS={diag_ser.rts}) ---")
                    
                    # Start thread to read
                    threading.Thread(target=read_loop, daemon=True).start()
                except Exception as e:
                    messagebox.showerror("接続エラー", str(e))
                    
        def read_loop():
            nonlocal is_running, diag_ser
            while is_running and diag_ser and diag_ser.is_open:
                try:
                    if diag_ser.in_waiting > 0:
                        raw = diag_ser.read(diag_ser.in_waiting)
                        hex_str = " ".join([f"{x:02X}" for x in raw])
                        ascii_str = raw.decode('ascii', errors='replace').replace('\r', '\\r').replace('\n', '\\n')
                        diag_win.after(0, lambda h=hex_str, a=ascii_str: log_message(f"受信 [Hex]: {h}\n受信 [Asc]: {a}"))
                except Exception as e:
                    diag_win.after(0, lambda ex=str(e): log_message(f"エラー: {ex}"))
                    break
                time.sleep(0.05)
                
        def send_byte(b_data):
            nonlocal diag_ser
            if diag_ser and diag_ser.is_open:
                try:
                    diag_ser.dtr = dtr_var.get()
                    diag_ser.rts = rts_var.get()
                    diag_ser.write(b_data)
                    log_message(f"送信: {b_data} (Hex: " + " ".join([f"{x:02X}" for x in b_data]) + ")")
                except Exception as e:
                    log_message(f"送信エラー: {str(e)}")
            else:
                log_message("ポートが開いていません。")

        # Action panel for sending triggers
        send_frame = ttk.Frame(diag_win)
        send_frame.pack(fill=tk.X, padx=10, pady=5)
        
        btn_start = ttk.Button(send_frame, text="診断開始", command=toggle_scan, style="Start.TButton")
        btn_start.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(send_frame, text="トリガー送信:").pack(side=tk.LEFT, padx=5)
        
        triggers = [
            ("b'C'", b'C'),
            ("b'D'", b'D'),
            ("b'A'", b'A'),
            ("b'S'", b'S'),
            ("b'M'", b'M'),
            ("b'\\r\\n'", b'\r\n'),
            ("0x02", b'\x02'),
            ("0x03", b'\x03'),
            ("0x55", b'\x55'),
            ("0xAA", b'\xAA')
        ]
        for name, b_data in triggers:
            btn = ttk.Button(send_frame, text=name, width=6, command=lambda bd=b_data: send_byte(bd))
            btn.pack(side=tk.LEFT, padx=2)

if __name__ == "__main__":
    root = tk.Tk()
    app = PitotCalibrationApp(root)
    root.mainloop()
