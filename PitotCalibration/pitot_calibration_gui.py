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
        self.root.title("Team ЯTR - Pitot Tube Wind Tunnel Calibration Utility")
        self.root.geometry("1200x800")
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
        
        # For trend plots
        self.plot_history_len = 100
        self.history_time = []
        self.history_airspeed = []
        self.history_aoa = []
        self.history_aos = []
        self.history_pitch = []
        self.history_yaw = []
        self.start_app_time = time.time()
        
        # Recording status
        self.recording = False
        self.recorded_samples = []
        self.record_start_time = 0.0
        
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
        style.configure("Header.TLabel", background="#0F172A", foreground="#06B6D4", font=("Segoe UI", 14, "bold"))
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

    def build_ui(self):
        # Main horizontal paned split
        main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        
        # Left Panel (Controls and readouts) - Scrollable
        left_container = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(left_container, weight=1)
        
        # Serial Connection Card
        conn_frame = ttk.LabelFrame(left_container, text="1. CONNECTION SETTINGS", padding=10)
        conn_frame.pack(fill=tk.X, pady=5, padx=5)
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, values=self.get_serial_ports(), width=15)
        self.port_combo.grid(row=0, column=1, pady=2, padx=5)
        
        # Refresh ports button
        btn_refresh = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports, width=8)
        btn_refresh.grid(row=0, column=2, pady=2, padx=2)
        
        ttk.Label(conn_frame, text="Baud Rate:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, values=["9600", "38400", "57600", "115200"], width=15)
        self.baud_combo.grid(row=1, column=1, pady=2, padx=5, columnspan=2, sticky=tk.W)
        
        self.btn_connect = ttk.Button(conn_frame, text="CONNECT", command=self.toggle_connection, style="Action.TButton")
        self.btn_connect.grid(row=2, column=0, columnspan=3, pady=8, sticky=tk.EW)
        
        # Air Density Calibration Settings Card
        env_frame = ttk.LabelFrame(left_container, text="2. AIR ENVIRONMENT & CORRECTIONS", padding=10)
        env_frame.pack(fill=tk.X, pady=5, padx=5)
        
        ttk.Label(env_frame, text="Baro Pressure (hPa):").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.press_entry = ttk.Entry(env_frame, width=10)
        self.press_entry.insert(0, "1013.25")
        self.press_entry.grid(row=0, column=1, pady=2, padx=5, sticky=tk.W)
        
        # Temperature override
        self.temp_override_var = tk.BooleanVar(value=True)
        self.chk_temp_override = ttk.Checkbutton(env_frame, text="Manual Temp Override (°C)", variable=self.temp_override_var)
        self.chk_temp_override.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=2)
        
        self.temp_entry = ttk.Entry(env_frame, width=10)
        self.temp_entry.insert(0, "20.0")
        self.temp_entry.grid(row=2, column=0, padx=20, pady=2, sticky=tk.W)
        
        # Humidity override
        self.humid_override_var = tk.BooleanVar(value=True)
        self.chk_humid_override = ttk.Checkbutton(env_frame, text="Manual Humidity Override (%)", variable=self.humid_override_var)
        self.chk_humid_override.grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=2)
        
        self.humid_entry = ttk.Entry(env_frame, width=10)
        self.humid_entry.insert(0, "50.0")
        self.humid_entry.grid(row=4, column=0, padx=20, pady=2, sticky=tk.W)
        
        # Wind Tunnel Calibration Run Card
        calib_frame = ttk.LabelFrame(left_container, text="3. WIND TUNNEL CALIBRATION RUN", padding=10)
        calib_frame.pack(fill=tk.X, pady=5, padx=5)
        
        self.btn_start = ttk.Button(calib_frame, text="START RECORDING", command=self.start_recording, style="Start.TButton")
        self.btn_start.pack(fill=tk.X, pady=3)
        
        self.btn_stop = ttk.Button(calib_frame, text="STOP & CALCULATE AVERAGES", command=self.stop_recording, style="Stop.TButton")
        self.btn_stop.pack(fill=tk.X, pady=3)
        self.btn_stop.state(["disabled"])
        
        self.lbl_status = ttk.Label(calib_frame, text="Status: IDLE", font=("Segoe UI", 10, "bold"), foreground="#94A3B8")
        self.lbl_status.pack(pady=4)
        
        # Real-time Value Readouts Card
        readout_frame = ttk.LabelFrame(left_container, text="4. REAL-TIME DATA READOUT", padding=10)
        readout_frame.pack(fill=tk.BOTH, expand=True, pady=5, padx=5)
        
        self.readout_labels = {}
        vars_to_show = [
            ("Airspeed", "airspeed", " m/s", "#3B82F6"),
            ("AoA Pressure (DP1)", "aoa_pa", " Pa", "#10B981"),
            ("AoS Pressure (DP2)", "aos_pa", " Pa", "#EC4899"),
            ("Pitch (IMU)", "pitch", "°", "#F59E0B"),
            ("Yaw (IMU)", "yaw", "°", "#F59E0B"),
            ("Board Temp", "temp", " °C (Refer)", "#94A3B8"),
            ("Humidity", "humid", " %", "#94A3B8"),
            ("Calc Air Density", "density", " kg/m³", "#06B6D4")
        ]
        
        for idx, (label, key, unit, color) in enumerate(vars_to_show):
            ttk.Label(readout_frame, text=label+":", font=("Segoe UI", 9)).grid(row=idx, column=0, sticky=tk.W, pady=2)
            lbl_val = ttk.Label(readout_frame, text="0.00"+unit, font=("Segoe UI", 10, "bold"), foreground=color)
            lbl_val.grid(row=idx, column=1, sticky=tk.E, pady=2, padx=15)
            self.readout_labels[key] = (lbl_val, unit)
        
        # Right Panel (Tabbed Notebook for Plots & Calibration Results)
        right_container = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(right_container, weight=3)
        
        self.notebook = ttk.Notebook(right_container)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Tab 1: Real-time Trends
        tab_trends = ttk.Frame(self.notebook, style="TFrame")
        self.notebook.add(tab_trends, text="Real-time Trends")
        self.setup_trends_plot(tab_trends)
        
        # Tab 2: Calibration Results & Reports
        tab_results = ttk.Frame(self.notebook, style="TFrame")
        self.notebook.add(tab_results, text="Wind Tunnel Calibration Report")
        self.setup_results_tab(tab_results)

    def setup_trends_plot(self, parent):
        # Embed matplotlib figure
        self.fig_trends = Figure(figsize=(8, 6), dpi=100, facecolor="#0F172A")
        
        # Subplot 1: Airspeed and Differential Pressures
        self.ax_press = self.fig_trends.add_subplot(211)
        self.ax_press.set_facecolor("#1E293B")
        self.ax_press.set_title("Sensor Pressures & Speed", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_press.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_press.grid(True, color="#334155", linestyle=":")
        
        self.line_speed, = self.ax_press.plot([], [], label="Airspeed (m/s)", color="#3B82F6", lw=2)
        self.line_aoa_p, = self.ax_press.plot([], [], label="AoA DP1 (Pa)", color="#10B981", lw=1.5)
        self.line_aos_p, = self.ax_press.plot([], [], label="AoS DP2 (Pa)", color="#EC4899", lw=1.5)
        self.ax_press.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8)
        
        # Subplot 2: IMU Attitudes
        self.ax_att = self.fig_trends.add_subplot(212)
        self.ax_att.set_facecolor("#1E293B")
        self.ax_att.set_title("IMU Pitch & Yaw Angles", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_att.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_att.grid(True, color="#334155", linestyle=":")
        
        self.line_pitch, = self.ax_att.plot([], [], label="Pitch Angle (°)", color="#F59E0B", lw=2)
        self.line_yaw, = self.ax_att.plot([], [], label="Yaw Angle (°)", color="#10B981", lw=2)
        self.ax_att.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8)
        
        self.fig_trends.tight_layout()
        
        self.canvas_trends = FigureCanvasTkAgg(self.fig_trends, master=parent)
        self.canvas_trends.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_results_tab(self, parent):
        # A container splits into Left (Text report) and Right (Scatter plots)
        split_frame = ttk.Frame(parent, style="TFrame")
        split_frame.pack(fill=tk.BOTH, expand=True, padding=5)
        
        # Left side: Scrollable text box for averages and formulas
        report_frame = ttk.LabelFrame(split_frame, text="AVERAGE & REGRESSION SUMMARY", padding=10)
        report_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        self.txt_report = tk.Text(report_frame, bg="#1E293B", fg="#F8FAFC", font=("Consolas", 10), wrap=tk.WORD, borderwidth=0)
        self.txt_report.pack(fill=tk.BOTH, expand=True)
        self.txt_report.insert(tk.END, "Calibration results will appear here after stopping a recording.\n")
        
        btn_box = ttk.Frame(report_frame, style="TFrame")
        btn_box.pack(fill=tk.X, pady=5)
        
        self.btn_export_csv = ttk.Button(btn_box, text="Export CSV Data", command=self.export_csv, style="Action.TButton")
        self.btn_export_csv.pack(side=tk.LEFT, padx=5)
        self.btn_export_csv.state(["disabled"])
        
        self.btn_save_report = ttk.Button(btn_box, text="Save Report", command=self.save_report_file, style="Action.TButton")
        self.btn_save_report.pack(side=tk.LEFT, padx=5)
        self.btn_save_report.state(["disabled"])
        
        # Right side: Scatter plot for calibration (Pitch vs AoA ratio, Yaw vs AoS ratio)
        self.fig_calib = Figure(figsize=(5, 5), dpi=100, facecolor="#0F172A")
        
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
            
            # Start background reading thread
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
        if self.recording:
            self.stop_recording()

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
            # Format: >key:value
            parts = line[1:].split(":")
            if len(parts) == 2:
                key, val_str = parts[0], parts[1]
                val = float(val_str)
                
                # Assign values
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
                    
                # Append sample to run recording if active
                if self.recording:
                    temp = self.get_temperature()
                    humid = self.get_humidity()
                    p_hpa = self.get_pressure()
                    
                    rho = self.calculate_density(temp, humid, p_hpa)
                    q = 0.5 * rho * (self.current_data["airspeed"] ** 2)
                    
                    self.recorded_samples.append({
                        "timestamp": time.time() - self.record_start_time,
                        "airspeed": self.current_data["airspeed"],
                        "aoa_pa": self.current_data["aoa_pa"],
                        "aos_pa": self.current_data["aos_pa"],
                        "pitch": self.current_data["pitch"],
                        "roll": self.current_data["roll"],
                        "yaw": self.current_data["yaw"],
                        "temp": self.current_data["temp"],
                        "humid": self.current_data["humid"],
                        "density": rho,
                        "q": q
                    })
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
        # Gas constants & Density equation
        tk = temp + 273.15
        p_pa = p_hpa * 100.0
        
        # Tetens equation for saturation vapor pressure
        p_sat = 610.78 * (10 ** (7.5 * temp / (temp + 237.3)))
        
        # Partial vapor pressure
        p_v = (humid / 100.0) * p_sat
        p_d = p_pa - p_v
        
        Rd = 287.058
        Rv = 461.495
        
        rho = (p_d / (Rd * tk)) + (p_v / (Rv * tk))
        return rho

    def start_recording(self):
        if not self.serial_connected:
            messagebox.showwarning("Record Error", "Please connect to a COM port first.")
            return
            
        self.recording = True
        self.recorded_samples = []
        self.record_start_time = time.time()
        
        self.btn_start.state(["disabled"])
        self.btn_stop.state(["!disabled"])
        self.lbl_status.configure(text="Status: RECORDING DATA...", foreground="#3B82F6")

    def stop_recording(self):
        if not self.recording:
            return
            
        self.recording = False
        self.btn_start.state(["!disabled"])
        self.btn_stop.state(["disabled"])
        self.lbl_status.configure(text="Status: ANALYSIS COMPLETE", foreground="#10B981")
        
        # Perform calibration math
        self.analyze_recorded_data()

    def analyze_recorded_data(self):
        if not self.recorded_samples:
            self.txt_report.delete(1.0, tk.END)
            self.txt_report.insert(tk.END, "Error: No samples recorded. Check telemetry data flow.\n")
            return
            
        # Extract lists
        times = [s["timestamp"] for s in self.recorded_samples]
        speeds = [s["airspeed"] for s in self.recorded_samples]
        aoas = [s["aoa_pa"] for s in self.recorded_samples]
        aoss = [s["aos_pa"] for s in self.recorded_samples]
        pitches = [s["pitch"] for s in self.recorded_samples]
        yaws = [s["yaw"] for s in self.recorded_samples]
        temps = [s["temp"] for s in self.recorded_samples]
        humids = [s["humid"] for s in self.recorded_samples]
        densities = [s["density"] for s in self.recorded_samples]
        qs = [s["q"] for s in self.recorded_samples]
        
        num_samples = len(self.recorded_samples)
        
        # Average statistics
        mean_speed = np.mean(speeds)
        std_speed = np.std(speeds)
        
        mean_aoa = np.mean(aoas)
        std_aoa = np.std(aoas)
        
        mean_aos = np.mean(aoss)
        std_aos = np.std(aoss)
        
        mean_pitch = np.mean(pitches)
        std_pitch = np.std(pitches)
        
        mean_yaw = np.mean(yaws)
        std_yaw = np.std(yaws)
        
        mean_density = np.mean(densities)
        
        # Perform Linear Regression for AoA and AoS coefficients
        # Formula: Angle = K * (DP / q) + Offset
        # Filter samples where dynamic pressure q is extremely low to avoid noise division spikes
        x_aoa = []
        y_pitch = []
        x_aos = []
        y_yaw = []
        
        for idx in range(num_samples):
            # If dynamic pressure > 5 Pa (~3 m/s)
            if qs[idx] > 5.0:
                # Ratio: DP / q
                x_aoa.append(aoas[idx] / qs[idx])
                y_pitch.append(pitches[idx])
                
                x_aos.append(aoss[idx] / qs[idx])
                y_yaw.append(yaws[idx])
                
        # Fit lines
        aoa_valid = len(x_aoa) > 5
        aos_valid = len(x_aos) > 5
        
        k_aoa = 0.0
        offset_aoa = 0.0
        r2_aoa = 0.0
        
        if aoa_valid:
            slope, intercept = np.polyfit(x_aoa, y_pitch, 1)
            k_aoa = slope
            offset_aoa = intercept
            # Calculate R^2
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
            # Calculate R^2
            y_pred = [slope * x + intercept for x in x_aos]
            ss_tot = np.sum((y_yaw - np.mean(y_yaw))**2)
            ss_res = np.sum((y_yaw - y_pred)**2)
            r2_aos = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

        # Construct textual report
        report = []
        report.append("=========================================")
        report.append("  TEAM ЯTR - WIND TUNNEL CALIBRATION REPORT")
        report.append("=========================================")
        report.append(f"Recorded at: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"Duration: {times[-1]:.2f} seconds")
        report.append(f"Total Samples: {num_samples}")
        report.append("-----------------------------------------")
        report.append("ENVIRONMENT SETTINGS:")
        report.append(f"  Baro Pressure:   {self.get_pressure():.2f} hPa")
        report.append(f"  Reference Temp:  {self.get_temperature():.1f} °C (Manual: {self.temp_override_var.get()})")
        report.append(f"  Reference Humid: {self.get_humidity():.1f} % (Manual: {self.humid_override_var.get()})")
        report.append(f"  Calculated Density: {mean_density:.4f} kg/m³")
        report.append("-----------------------------------------")
        report.append("AVERAGES & DEVIATIONS:")
        report.append(f"  Airspeed:    {mean_speed:6.2f} ± {std_speed:.2f} m/s")
        report.append(f"  AoA DP (P1): {mean_aoa:6.2f} ± {std_aoa:.2f} Pa")
        report.append(f"  AoS DP (P2): {mean_aos:6.2f} ± {std_aos:.2f} Pa")
        report.append(f"  Pitch (IMU): {mean_pitch:6.2f} ± {std_pitch:.2f} °")
        report.append(f"  Yaw (IMU):   {mean_yaw:6.2f} ± {std_yaw:.2f} °")
        report.append("-----------------------------------------")
        report.append("CALIBRATION COEFFICIENTS (Angle = K * (DP / q) + Offset):")
        
        if aoa_valid:
            report.append(f"  Angle of Attack (AoA / Pitch):")
            report.append(f"    Sensitivity K_AoA:  {k_aoa:.5f} °/(Pa/Pa)")
            report.append(f"    Offset Angle:        {offset_aoa:.3f} °")
            report.append(f"    Fit Quality R²:      {r2_aoa:.4f}")
        else:
            report.append("  AoA Calibration: Insufficient valid airflow samples (Speed too low)")
            
        if aos_valid:
            report.append(f"  Angle of Sideslip (AoS / Yaw):")
            report.append(f"    Sensitivity K_AoS:  {k_aos:.5f} °/(Pa/Pa)")
            report.append(f"    Offset Angle:        {offset_aos:.3f} °")
            report.append(f"    Fit Quality R²:      {r2_aos:.4f}")
        else:
            report.append("  AoS Calibration: Insufficient valid airflow samples (Speed too low)")
            
        report.append("=========================================")
        
        self.txt_report.delete(1.0, tk.END)
        self.txt_report.insert(tk.END, "\n".join(report))
        
        # Save results in local fields for exporting
        self.report_text = "\n".join(report)
        self.btn_export_csv.state(["!disabled"])
        self.btn_save_report.state(["!disabled"])
        
        # Plot Scatter & Fits
        self.draw_calibration_plots(x_aoa, y_pitch, k_aoa, offset_aoa, x_aos, y_yaw, k_aos, offset_aos)

    def draw_calibration_plots(self, x_aoa, y_pitch, k_aoa, offset_aoa, x_aos, y_yaw, k_aos, offset_aos):
        self.ax_calib_aoa.clear()
        self.ax_calib_aoa.set_facecolor("#1E293B")
        self.ax_calib_aoa.set_title("Pitch vs. AoA Differential Ratio", color="#06B6D4", fontsize=9, fontweight="bold")
        self.ax_calib_aoa.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_calib_aoa.grid(True, color="#334155", linestyle=":")
        
        if x_aoa:
            self.ax_calib_aoa.scatter(x_aoa, y_pitch, color="#10B981", s=3, alpha=0.6, label="Measured samples")
            # Fit line
            x_line = np.linspace(min(x_aoa), max(x_aoa), 100)
            y_line = k_aoa * x_line + offset_aoa
            self.ax_calib_aoa.plot(x_line, y_line, color="#EF4444", lw=1.5, label=f"Fit (K={k_aoa:.3f})")
            self.ax_calib_aoa.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=7)
            self.ax_calib_aoa.set_xlabel("Ratio: DP_AoA / q", color="#94A3B8", fontsize=7)
            self.ax_calib_aoa.set_ylabel("Pitch Angle (°)", color="#94A3B8", fontsize=7)
            
        self.ax_calib_aos.clear()
        self.ax_calib_aos.set_facecolor("#1E293B")
        self.ax_calib_aos.set_title("Yaw vs. AoS Differential Ratio", color="#06B6D4", fontsize=9, fontweight="bold")
        self.ax_calib_aos.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_calib_aos.grid(True, color="#334155", linestyle=":")
        
        if x_aos:
            self.ax_calib_aos.scatter(x_aos, y_yaw, color="#EC4899", s=3, alpha=0.6, label="Measured samples")
            # Fit line
            x_line = np.linspace(min(x_aos), max(x_aos), 100)
            y_line = k_aos * x_line + offset_aos
            self.ax_calib_aos.plot(x_line, y_line, color="#3B82F6", lw=1.5, label=f"Fit (K={k_aos:.3f})")
            self.ax_calib_aos.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=7)
            self.ax_calib_aos.set_xlabel("Ratio: DP_AoS / q", color="#94A3B8", fontsize=7)
            self.ax_calib_aos.set_ylabel("Yaw Angle (°)", color="#94A3B8", fontsize=7)
            
        self.fig_calib.tight_layout()
        self.canvas_calib.draw()

    def export_csv(self):
        if not self.recorded_samples:
            return
            
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
            title="Export Recorded Calibration Data"
        )
        
        if not filepath:
            return
            
        try:
            import csv
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                # Header
                writer.writerow([
                    "Timestamp (s)", "Airspeed (m/s)", "AoA DP_P1 (Pa)", "AoS DP_P2 (Pa)",
                    "Pitch (°)", "Roll (°)", "Yaw (°)", "Board Temp (°C)", "Board Humid (%)",
                    "Calculated Air Density (kg/m3)", "Calculated Dynamic Pressure (Pa)"
                ])
                for s in self.recorded_samples:
                    writer.writerow([
                        f"{s['timestamp']:.3f}", f"{s['airspeed']:.2f}", f"{s['aoa_pa']:.2f}", f"{s['aos_pa']:.2f}",
                        f"{s['pitch']:.2f}", f"{s['roll']:.2f}", f"{s['yaw']:.2f}", f"{s['temp']:.2f}", f"{s['humid']:.2f}",
                        f"{s['density']:.4f}", f"{s['q']:.4f}"
                    ])
            messagebox.showinfo("Export Successful", f"Data exported successfully to:\n{filepath}")
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
        # Update labels in left panel
        temp = self.get_temperature()
        humid = self.get_humidity()
        p_hpa = self.get_pressure()
        rho = self.calculate_density(temp, humid, p_hpa)
        
        self.readout_labels["airspeed"][0].configure(text=f"{self.current_data['airspeed']:.2f}{self.readout_labels['airspeed'][1]}")
        self.readout_labels["aoa_pa"][0].configure(text=f"{self.current_data['aoa_pa']:.2f}{self.readout_labels['aoa_pa'][1]}")
        self.readout_labels["aos_pa"][0].configure(text=f"{self.current_data['aos_pa']:.2f}{self.readout_labels['aos_pa'][1]}")
        self.readout_labels["pitch"][0].configure(text=f"{self.current_data['pitch']:.2f}{self.readout_labels['pitch'][1]}")
        self.readout_labels["yaw"][0].configure(text=f"{self.current_data['yaw']:.2f}{self.readout_labels['yaw'][1]}")
        self.readout_labels["temp"][0].configure(text=f"{self.current_data['temp']:.2f}{self.readout_labels['temp'][1]}")
        self.readout_labels["humid"][0].configure(text=f"{self.current_data['humid']:.2f}{self.readout_labels['humid'][1]}")
        self.readout_labels["density"][0].configure(text=f"{rho:.4f}{self.readout_labels['density'][1]}")
        
        # Accumulate history for real-time scrolling plots
        curr_t = time.time() - self.start_app_time
        self.history_time.append(curr_t)
        self.history_airspeed.append(self.current_data["airspeed"])
        self.history_aoa.append(self.current_data["aoa_pa"])
        self.history_aos.append(self.current_data["aos_pa"])
        self.history_pitch.append(self.current_data["pitch"])
        self.history_yaw.append(self.current_data["yaw"])
        
        # Keep arrays capped at history length
        if len(self.history_time) > self.plot_history_len:
            self.history_time.pop(0)
            self.history_airspeed.pop(0)
            self.history_aoa.pop(0)
            self.history_aos.pop(0)
            self.history_pitch.pop(0)
            self.history_yaw.pop(0)
            
        # Refresh real-time trends graph if Tab 1 is visible
        if self.notebook.index(self.notebook.select()) == 0:
            self.redraw_trends_plot()
            
        # Loop every 100ms (10Hz matches ESP32 telemetry speed)
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
