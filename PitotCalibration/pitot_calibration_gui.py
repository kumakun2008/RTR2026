import sys
import os
import time
import serial
import serial.tools.list_ports
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

class PitotCalibrationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Team ЯTR - Pitot Tube Automatic Calibration Utility")
        self.root.geometry("1300x850")
        self.root.configure(bg="#0F172A") # slate-900

        # Serial configuration
        self.ser = None
        self.serial_connected = False
        self.read_thread = None
        
        # Real-time data storage
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
        style.configure("Action.TButton", font=("Segoe UI", 10, "bold"), foreground="#FFFFFF", background="#3B82F6")
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

    def build_ui(self):
        # Main horizontal split
        main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        
        # Left Panel (Controls and readouts)
        left_container = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(left_container, weight=1)
        
        # Serial Connection Card
        conn_frame = ttk.LabelFrame(left_container, text="1. CONNECTION SETTINGS", padding=10)
        conn_frame.pack(fill=tk.X, pady=4, padx=5)
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, values=self.get_serial_ports(), width=15)
        self.port_combo.grid(row=0, column=1, pady=2, padx=5)
        
        btn_refresh = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports, width=8)
        btn_refresh.grid(row=0, column=2, pady=2, padx=2)
        
        ttk.Label(conn_frame, text="Baud Rate:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, values=["9600", "38400", "57600", "115200"], width=15)
        self.baud_combo.grid(row=1, column=1, pady=2, padx=5, columnspan=2, sticky=tk.W)
        
        self.btn_connect = ttk.Button(conn_frame, text="CONNECT", command=self.toggle_connection, style="Action.TButton")
        self.btn_connect.grid(row=2, column=0, columnspan=3, pady=6, sticky=tk.EW)
        
        # Environment settings
        env_frame = ttk.LabelFrame(left_container, text="2. WIND TUNNEL ENVIRONMENT", padding=10)
        env_frame.pack(fill=tk.X, pady=4, padx=5)
        
        ttk.Label(env_frame, text="Baro Pressure (hPa):").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.press_entry = ttk.Entry(env_frame, width=10)
        self.press_entry.insert(0, "1013.25")
        self.press_entry.grid(row=0, column=1, pady=2, padx=5, sticky=tk.W)
        
        # Temp & Humid overrides
        self.temp_override_var = tk.BooleanVar(value=True)
        self.chk_temp_override = ttk.Checkbutton(env_frame, text="Manual Temp (°C)", variable=self.temp_override_var)
        self.chk_temp_override.grid(row=1, column=0, sticky=tk.W, pady=2)
        self.temp_entry = ttk.Entry(env_frame, width=10)
        self.temp_entry.insert(0, "20.0")
        self.temp_entry.grid(row=1, column=1, pady=2, padx=5, sticky=tk.W)
        
        self.humid_override_var = tk.BooleanVar(value=True)
        self.chk_humid_override = ttk.Checkbutton(env_frame, text="Manual Humidity (%)", variable=self.humid_override_var)
        self.chk_humid_override.grid(row=2, column=0, sticky=tk.W, pady=2)
        self.humid_entry = ttk.Entry(env_frame, width=10)
        self.humid_entry.insert(0, "50.0")
        self.humid_entry.grid(row=2, column=1, pady=2, padx=5, sticky=tk.W)

        # Wind Tunnel Calibration Run Card
        calib_frame = ttk.LabelFrame(left_container, text="3. WIND TUNNEL CALIBRATION MODE", padding=10)
        calib_frame.pack(fill=tk.X, pady=4, padx=5)
        
        # STEP A: Aerodynamic Zero Calibration
        ttk.Label(calib_frame, text="Step A: Aerodynamic Zeroing (0° Alignment)", font=("Segoe UI", 9, "bold"), foreground="#06B6D4").pack(anchor=tk.W, pady=2)
        self.btn_zero_calib = ttk.Button(calib_frame, text="Measure 0° Offset (5s Avg)", command=self.run_zero_calibration, style="Action.TButton")
        self.btn_zero_calib.pack(fill=tk.X, pady=3)
        
        offset_labels_frame = ttk.Frame(calib_frame)
        offset_labels_frame.pack(fill=tk.X, pady=2)
        self.lbl_aoa_offset = ttk.Label(offset_labels_frame, text="AoA Offset: 0.00 Pa", font=("Segoe UI", 9), foreground="#10B981")
        self.lbl_aoa_offset.pack(side=tk.LEFT, padx=5)
        self.lbl_aos_offset = ttk.Label(offset_labels_frame, text="AoS Offset: 0.00 Pa", font=("Segoe UI", 9), foreground="#EC4899")
        self.lbl_aos_offset.pack(side=tk.RIGHT, padx=5)
        
        ttk.Separator(calib_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        
        # STEP B: Angle Sweep Data Points
        ttk.Label(calib_frame, text="Step B: Sweep Point Recording (Using BNO055)", font=("Segoe UI", 9, "bold"), foreground="#06B6D4").pack(anchor=tk.W, pady=2)
        
        self.btn_record_point = ttk.Button(calib_frame, text="Record Current Position (3s Avg)", command=self.record_sweep_point, style="Start.TButton")
        self.btn_record_point.pack(fill=tk.X, pady=4)
        
        self.lbl_status = ttk.Label(calib_frame, text="Status: IDLE", font=("Segoe UI", 10, "bold"), foreground="#94A3B8")
        self.lbl_status.pack(pady=4)
        
        # Real-time Value Readouts Card
        readout_frame = ttk.LabelFrame(left_container, text="4. REAL-TIME DATA READOUT", padding=10)
        readout_frame.pack(fill=tk.BOTH, expand=True, pady=4, padx=5)
        
        self.readout_labels = {}
        vars_to_show = [
            ("Airspeed", "airspeed", " m/s", "#3B82F6"),
            ("AoA Raw Pressure (P1)", "aoa_pa", " Pa", "#10B981"),
            ("AoS Raw Pressure (P2)", "aos_pa", " Pa", "#EC4899"),
            ("BNO055 Pitch (AoA)", "pitch", "°", "#F59E0B"),
            ("BNO055 Yaw (AoS)", "yaw", "°", "#F59E0B"),
            ("Calculated Air Density", "density", " kg/m³", "#06B6D4")
        ]
        
        for idx, (label, key, unit, color) in enumerate(vars_to_show):
            ttk.Label(readout_frame, text=label+":", font=("Segoe UI", 9)).grid(row=idx, column=0, sticky=tk.W, pady=2)
            lbl_val = ttk.Label(readout_frame, text="0.00"+unit, font=("Segoe UI", 10, "bold"), foreground=color)
            lbl_val.grid(row=idx, column=1, sticky=tk.E, pady=2, padx=15)
            self.readout_labels[key] = (lbl_val, unit)
            
        # Right Panel
        right_container = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(right_container, weight=3)
        
        self.notebook = ttk.Notebook(right_container)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Tab 1: Real-time Trends
        tab_trends = ttk.Frame(self.notebook, style="TFrame")
        self.notebook.add(tab_trends, text="Real-time Trends")
        self.setup_trends_plot(tab_trends)
        
        # Tab 2: Calibration Sweep & Regression
        tab_calib = ttk.Frame(self.notebook, style="TFrame")
        self.notebook.add(tab_calib, text="Göttingen Calibration Mode")
        self.setup_calibration_tab(tab_calib)

    def setup_trends_plot(self, parent):
        self.fig_trends = Figure(figsize=(8, 6), dpi=100, facecolor="#0F172A")
        
        # Subplot 1: Speed and Pressures
        self.ax_press = self.fig_trends.add_subplot(211)
        self.ax_press.set_facecolor("#1E293B")
        self.ax_press.set_title("Sensors Data", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_press.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_press.grid(True, color="#334155", linestyle=":")
        
        self.line_speed, = self.ax_press.plot([], [], label="Airspeed (m/s)", color="#3B82F6", lw=2)
        self.line_aoa_p, = self.ax_press.plot([], [], label="AoA DP1 (Pa)", color="#10B981", lw=1.5)
        self.line_aos_p, = self.ax_press.plot([], [], label="AoS DP2 (Pa)", color="#EC4899", lw=1.5)
        self.ax_press.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8)
        
        # Subplot 2: IMU
        self.ax_att = self.fig_trends.add_subplot(212)
        self.ax_att.set_facecolor("#1E293B")
        self.ax_att.set_title("BNO055 Angles", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_att.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_att.grid(True, color="#334155", linestyle=":")
        
        self.line_pitch, = self.ax_att.plot([], [], label="Pitch Angle (°)", color="#F59E0B", lw=2)
        self.line_yaw, = self.ax_att.plot([], [], label="Yaw Angle (°)", color="#10B981", lw=2)
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
        table_frame = ttk.LabelFrame(left_side, text="RECORDED CALIBRATION POINTS (3S TIME-AVERAGED)", padding=5)
        table_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        columns = ("id", "speed", "aoa_p", "aos_p", "pitch", "yaw")
        self.tree_points = ttk.Treeview(table_frame, columns=columns, show="headings", height=8)
        
        self.tree_points.heading("id", text="ID")
        self.tree_points.heading("speed", text="Airspeed (m/s)")
        self.tree_points.heading("aoa_p", text="DP_AoA Corr (Pa)")
        self.tree_points.heading("aos_p", text="DP_AoS Corr (Pa)")
        self.tree_points.heading("pitch", text="BNO Pitch (AoA) (°)")
        self.tree_points.heading("yaw", text="BNO Yaw (AoS) (°)")
        
        self.tree_points.column("id", width=40, anchor=tk.CENTER)
        self.tree_points.column("speed", width=100, anchor=tk.CENTER)
        self.tree_points.column("aoa_p", width=110, anchor=tk.CENTER)
        self.tree_points.column("aos_p", width=110, anchor=tk.CENTER)
        self.tree_points.column("pitch", width=110, anchor=tk.CENTER)
        self.tree_points.column("yaw", width=110, anchor=tk.CENTER)
        
        self.tree_points.pack(fill=tk.BOTH, expand=True)
        
        # Sweep table buttons
        tbl_btn_box = ttk.Frame(left_side, style="TFrame")
        tbl_btn_box.pack(fill=tk.X, pady=5)
        
        self.btn_clear_points = ttk.Button(tbl_btn_box, text="Clear Selected", command=self.clear_selected_point, style="Stop.TButton")
        self.btn_clear_points.pack(side=tk.LEFT, padx=5)
        
        self.btn_clear_all = ttk.Button(tbl_btn_box, text="Clear All Points", command=self.clear_all_points, style="Stop.TButton")
        self.btn_clear_all.pack(side=tk.LEFT, padx=5)
        
        self.btn_calc_fit = ttk.Button(tbl_btn_box, text="COMPUTE CALIBRATION COEFFICIENTS (K)", command=self.calculate_regression, style="Action.TButton")
        self.btn_calc_fit.pack(side=tk.RIGHT, padx=5)
        
        # Text Report Frame
        report_frame = ttk.LabelFrame(left_side, text="CALIBRATION MATHEMATICS & FORMULAS", padding=10)
        report_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.txt_report = tk.Text(report_frame, bg="#1E293B", fg="#F8FAFC", font=("Consolas", 9), wrap=tk.WORD, height=10, borderwidth=0)
        self.txt_report.pack(fill=tk.BOTH, expand=True)
        self.txt_report.insert(tk.END, "Calibration calculations will be rendered here once sweep points are recorded and processed.\n")
        
        btn_box = ttk.Frame(report_frame, style="TFrame")
        btn_box.pack(fill=tk.X, pady=4)
        
        self.btn_export_csv = ttk.Button(btn_box, text="Export CSV Data", command=self.export_csv, style="Action.TButton")
        self.btn_export_csv.pack(side=tk.LEFT, padx=5)
        self.btn_export_csv.state(["disabled"])
        
        self.btn_save_report = ttk.Button(btn_box, text="Save Calibration Report", command=self.save_report_file, style="Action.TButton")
        self.btn_save_report.pack(side=tk.LEFT, padx=5)
        self.btn_save_report.state(["disabled"])
        
        # Right Side: Scatter plot for calibration (Pitch vs AoA ratio, Yaw vs AoS ratio)
        self.fig_calib = Figure(figsize=(5, 6), dpi=100, facecolor="#0F172A")
        
        self.ax_calib_aoa = self.fig_calib.add_subplot(211)
        self.ax_calib_aoa.set_facecolor("#1E293B")
        self.ax_calib_aoa.set_title("Pitch vs. AoA Differential Ratio", color="#06B6D4", fontsize=9, fontweight="bold")
        self.ax_calib_aoa.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_calib_aoa.grid(True, color="#334155", linestyle=":")
        
        self.ax_calib_aos = self.fig_calib.add_subplot(212)
        self.ax_calib_aos.set_facecolor("#1E293B")
        self.ax_calib_aos.set_title("Yaw vs. AoS Differential Ratio", color="#06B6D4", fontsize=9, fontweight="bold")
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
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        if self.serial_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_var.get()
        baud = self.baud_var.get()
        
        if not port:
            messagebox.showerror("Connection Error", "Please select a COM port.")
            return
            
        try:
            self.ser = serial.Serial(port, int(baud), timeout=1)
            self.serial_connected = True
            self.btn_connect.configure(text="DISCONNECT", style="Stop.TButton")
            self.lbl_status.configure(text="Status: CONNECTED", foreground="#10B981")
            
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()
        except Exception as e:
            messagebox.showerror("Connection Error", f"Could not open port {port}:\n{str(e)}")

    def disconnect_serial(self):
        self.serial_connected = False
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
            
        self.btn_connect.configure(text="CONNECT", style="Action.TButton")
        self.lbl_status.configure(text="Status: DISCONNECTED", foreground="#EF4444")
        self.calibrating_zero = False
        self.recording_point = False

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
        messagebox.showwarning("Connection Lost", "The serial connection was lost.")

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
            messagebox.showwarning("Connection Error", "Please connect to a COM port first.")
            return
            
        self.calibrating_zero = True
        self.btn_zero_calib.state(["disabled"])
        self.btn_record_point.state(["disabled"])
        self.lbl_status.configure(text="Recording Aerodynamic Zero at 0° (5s Avg)...", foreground="#3B82F6")
        
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
        self.lbl_aoa_offset.configure(text=f"AoA Offset: {self.aoa_zero_offset:.2f} Pa")
        self.lbl_aos_offset.configure(text=f"AoS Offset: {self.aos_zero_offset:.2f} Pa")

    def on_zero_calibration_complete(self):
        self.calibrating_zero = False
        self.btn_zero_calib.state(["!disabled"])
        self.btn_record_point.state(["!disabled"])
        self.lbl_status.configure(text="Aerodynamic Zero Calibrated", foreground="#10B981")
        messagebox.showinfo("Calibration complete", f"Aerodynamic zero values recorded successfully:\nAoA Offset: {self.aoa_zero_offset:.2f} Pa\nAoS Offset: {self.aos_zero_offset:.2f} Pa")

    # Step B: Record Sweep Point (Filters vibrations by averaging)
    def record_sweep_point(self):
        if not self.serial_connected:
            messagebox.showwarning("Connection Error", "Please connect to a COM port first.")
            return
            
        self.recording_point = True
        self.btn_zero_calib.state(["disabled"])
        self.btn_record_point.state(["disabled"])
        self.lbl_status.configure(text="Averaging wind tunnel vibration & BNO angles (3s)...", foreground="#3B82F6")
        
        threading.Thread(target=self.collect_sweep_samples, daemon=True).start()

    def collect_sweep_samples(self):
        samples = []
        start_time = time.time()
        
        while time.time() - start_time < 3.0:
            if self.serial_connected:
                temp = self.get_temperature()
                humid = self.get_humidity()
                p_hpa = self.get_pressure()
                rho = self.calculate_density(temp, humid, p_hpa)
                
                # Correct raw pressure values with aerodynamic 0° offset
                aoa_corr = self.current_data["aoa_pa"] - self.aoa_zero_offset
                aos_corr = self.current_data["aos_pa"] - self.aos_zero_offset
                
                q = 0.5 * rho * (self.current_data["airspeed"] ** 2)
                
                samples.append({
                    "speed": self.current_data["airspeed"],
                    "aoa_corr": aoa_corr,
                    "aos_corr": aos_corr,
                    "pitch": self.current_data["pitch"],
                    "yaw": self.current_data["yaw"],
                    "q": q
                })
            time.sleep(0.1) # 10Hz
            
        if samples and self.serial_connected:
            # Average variables over the 3-second block to cancel Göttingen tunnel vibrations & trust BNO055!
            avg_speed = np.mean([s["speed"] for s in samples])
            avg_aoa_corr = np.mean([s["aoa_corr"] for s in samples])
            avg_aos_corr = np.mean([s["aos_corr"] for s in samples])
            avg_pitch = np.mean([s["pitch"] for s in samples])
            avg_yaw = np.mean([s["yaw"] for s in samples])
            avg_q = np.mean([s["q"] for s in samples])
            
            point = {
                "avg_speed": avg_speed,
                "avg_aoa_corr": avg_aoa_corr,
                "avg_aos_corr": avg_aos_corr,
                "avg_pitch": avg_pitch,
                "avg_yaw": avg_yaw,
                "avg_q": avg_q
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
            f"{pt['avg_aoa_corr']:.2f}",
            f"{pt['avg_aos_corr']:.2f}",
            f"{pt['avg_pitch']:.2f}°",
            f"{pt['avg_yaw']:.2f}°"
        ))

    def on_sweep_recording_complete(self):
        self.recording_point = False
        self.btn_zero_calib.state(["!disabled"])
        self.btn_record_point.state(["!disabled"])
        self.lbl_status.configure(text="Sweep Point Recorded", foreground="#10B981")

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
        if messagebox.askyesno("Clear all", "Are you sure you want to clear all recorded points?"):
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
                f"{pt['avg_aoa_corr']:.2f}",
                f"{pt['avg_aos_corr']:.2f}",
                f"{pt['avg_pitch']:.2f}°",
                f"{pt['avg_yaw']:.2f}°"
            ))

    # Regression and report generation
    def calculate_regression(self):
        if len(self.sweep_points) < 3:
            messagebox.showwarning("Data Error", "Please record at least 3 sweep points to run regression.")
            return
            
        # Extract sweep point variables
        speeds = [p["avg_speed"] for p in self.sweep_points]
        aoas = [p["avg_aoa_corr"] for p in self.sweep_points]
        aoss = [p["avg_aos_corr"] for p in self.sweep_points]
        pitches = [p["avg_pitch"] for p in self.sweep_points]
        yaws = [p["avg_yaw"] for p in self.sweep_points]
        qs = [p["avg_q"] for p in self.sweep_points]
        
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

        # Construct Report
        report = []
        report.append("=========================================")
        report.append("  TEAM ЯTR - WIND TUNNEL CALIBRATION REPORT")
        report.append("  (Averaged BNO055 IMU Reference Method)")
        report.append("=========================================")
        report.append(f"Report Created: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"Total Sweep Points: {len(self.sweep_points)}")
        report.append("-----------------------------------------")
        report.append("ENVIRONMENT & ZERO CALIBRATION:")
        report.append(f"  Baro Pressure:   {self.get_pressure():.2f} hPa")
        report.append(f"  Reference Temp:  {self.get_temperature():.1f} °C")
        report.append(f"  Reference Humid: {self.get_humidity():.1f} %")
        report.append(f"  Aerodynamic AoA Zero Offset: {self.aoa_zero_offset:.2f} Pa")
        report.append(f"  Aerodynamic AoS Zero Offset: {self.aos_zero_offset:.2f} Pa")
        report.append("-----------------------------------------")
        report.append("RECORDED SWEEP POINTS (Filtered Göttingen Fluctuations):")
        for i, pt in enumerate(self.sweep_points):
            report.append(f"  Pt {i+1:2d} | Speed: {pt['avg_speed']:5.2f} m/s | DP_AoA: {pt['avg_aoa_corr']:6.2f} Pa | DP_AoS: {pt['avg_aos_corr']:6.2f} Pa | BNO Pitch: {pt['avg_pitch']:5.2f}° | BNO Yaw: {pt['avg_yaw']:5.2f}°")
        report.append("-----------------------------------------")
        report.append("CALIBRATION COEFFICIENTS (Angle = K * (DP_corrected / q) + Offset):")
        
        if aoa_valid:
            report.append(f"  Angle of Attack (AoA / Pitch):")
            report.append(f"    Sensitivity K_AoA (slope):  {k_aoa:.5f} °/(Pa/Pa)")
            report.append(f"    Offset Angle (intercept):   {offset_aoa:.3f} °")
            report.append(f"    Fit Quality R²:             {r2_aoa:.4f}")
        else:
            report.append("  AoA Calibration: Insufficient speed (q > 5 Pa) in sweep points.")
            
        if aos_valid:
            report.append(f"  Angle of Sideslip (AoS / Yaw):")
            report.append(f"    Sensitivity K_AoS (slope):  {k_aos:.5f} °/(Pa/Pa)")
            report.append(f"    Offset Angle (intercept):   {offset_aos:.3f} °")
            report.append(f"    Fit Quality R²:             {r2_aos:.4f}")
        else:
            report.append("  AoS Calibration: Insufficient speed (q > 5 Pa) in sweep points.")
            
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
        self.ax_calib_aoa.set_title("Pitch vs. AoA Ratio", color="#06B6D4", fontsize=9, fontweight="bold")
        self.ax_calib_aoa.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_calib_aoa.grid(True, color="#334155", linestyle=":")
        
        if x_aoa:
            self.ax_calib_aoa.scatter(x_aoa, y_pitch, color="#10B981", s=35, marker="o", label="Averaged points")
            x_line = np.linspace(min(x_aoa), max(x_aoa), 100)
            y_line = k_aoa * x_line + offset_aoa
            self.ax_calib_aoa.plot(x_line, y_line, color="#EF4444", lw=1.5, label=f"Fit (K={k_aoa:.3f})")
            self.ax_calib_aoa.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=7)
            self.ax_calib_aoa.set_xlabel("Ratio: DP_AoA_corrected / q", color="#94A3B8", fontsize=7)
            self.ax_calib_aoa.set_ylabel("Pitch Angle (°)", color="#94A3B8", fontsize=7)
            
        self.ax_calib_aos.clear()
        self.ax_calib_aos.set_facecolor("#1E293B")
        self.ax_calib_aos.set_title("Yaw vs. AoS Ratio", color="#06B6D4", fontsize=9, fontweight="bold")
        self.ax_calib_aos.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_calib_aos.grid(True, color="#334155", linestyle=":")
        
        if x_aos:
            self.ax_calib_aos.scatter(x_aos, y_yaw, color="#EC4899", s=35, marker="o", label="Averaged points")
            x_line = np.linspace(min(x_aos), max(x_aos), 100)
            y_line = k_aos * x_line + offset_aos
            self.ax_calib_aos.plot(x_line, y_line, color="#3B82F6", lw=1.5, label=f"Fit (K={k_aos:.3f})")
            self.ax_calib_aos.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=7)
            self.ax_calib_aos.set_xlabel("Ratio: DP_AoS_corrected / q", color="#94A3B8", fontsize=7)
            self.ax_calib_aos.set_ylabel("Yaw Angle (°)", color="#94A3B8", fontsize=7)
            
        self.fig_calib.tight_layout()
        self.canvas_calib.draw()

    def export_csv(self):
        if not self.sweep_points:
            return
            
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
            title="Export Sweep Calibration Points"
        )
        
        if not filepath:
            return
            
        try:
            import csv
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Point ID", "Averaged Airspeed (m/s)", 
                    "Averaged Corrected DP_AoA (Pa)", "Averaged Corrected DP_AoS (Pa)",
                    "Averaged BNO055 Pitch Angle (°)", "Averaged BNO055 Yaw Angle (°)", "Averaged Dynamic Pressure q (Pa)"
                ])
                for idx, pt in enumerate(self.sweep_points):
                    writer.writerow([
                        idx + 1, f"{pt['avg_speed']:.3f}", 
                        f"{pt['avg_aoa_corr']:.3f}", f"{pt['avg_aos_corr']:.3f}",
                        f"{pt['avg_pitch']:.3f}", f"{pt['avg_yaw']:.3f}", f"{pt['avg_q']:.3f}"
                    ])
            messagebox.showinfo("Export Successful", f"Sweep points exported to:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Could not write CSV file:\n{str(e)}")

    def save_report_file(self):
        if not hasattr(self, 'report_text') or not self.report_text:
            return
            
        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")],
            title="Save Calibration Report"
        )
        
        if not filepath:
            return
            
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(self.report_text)
            messagebox.showinfo("Save Successful", f"Report saved successfully to:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Could not write report file:\n{str(e)}")

    def update_gui_loop(self):
        temp = self.get_temperature()
        humid = self.get_humidity()
        p_hpa = self.get_pressure()
        rho = self.calculate_density(temp, humid, p_hpa)
        
        self.readout_labels["airspeed"][0].configure(text=f"{self.current_data['airspeed']:.2f}{self.readout_labels['airspeed'][1]}")
        self.readout_labels["aoa_pa"][0].configure(text=f"{self.current_data['aoa_pa']:.2f}{self.readout_labels['aoa_pa'][1]}")
        self.readout_labels["aos_pa"][0].configure(text=f"{self.current_data['aos_pa']:.2f}{self.readout_labels['aos_pa'][1]}")
        self.readout_labels["pitch"][0].configure(text=f"{self.current_data['pitch']:.2f}{self.readout_labels['pitch'][1]}")
        self.readout_labels["yaw"][0].configure(text=f"{self.current_data['yaw']:.2f}{self.readout_labels['yaw'][1]}")
        self.readout_labels["density"][0].configure(text=f"{rho:.4f}{self.readout_labels['density'][1]}")
        
        # History queue
        curr_t = time.time() - self.start_app_time
        self.history_time.append(curr_t)
        self.history_airspeed.append(self.current_data["airspeed"])
        self.history_aoa.append(self.current_data["aoa_pa"])
        self.history_aos.append(self.current_data["aos_pa"])
        self.history_pitch.append(self.current_data["pitch"])
        self.history_yaw.append(self.current_data["yaw"])
        
        if len(self.history_time) > self.plot_history_len:
            self.history_time.pop(0)
            self.history_airspeed.pop(0)
            self.history_aoa.pop(0)
            self.history_aos.pop(0)
            self.history_pitch.pop(0)
            self.history_yaw.pop(0)
            
        # Draw trends graph
        if self.notebook.index(self.notebook.select()) == 0:
            self.redraw_trends_plot()
            
        self.root.after(100, self.update_gui_loop)

    def redraw_trends_plot(self):
        if not self.history_time:
            return
            
        self.line_speed.set_data(self.history_time, self.history_airspeed)
        self.line_aoa_p.set_data(self.history_time, self.history_aoa)
        self.line_aos_p.set_data(self.history_time, self.history_aos)
        self.ax_press.relim()
        self.ax_press.autoscale_view()
        
        self.line_pitch.set_data(self.history_time, self.history_pitch)
        self.line_yaw.set_data(self.history_time, self.history_yaw)
        self.ax_att.relim()
        self.ax_att.autoscale_view()
        
        self.canvas_trends.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = PitotCalibrationApp(root)
    root.mainloop()
