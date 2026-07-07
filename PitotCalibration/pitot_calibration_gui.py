import sys
import os
import time
import datetime
import serial
import serial.tools.list_ports
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
matplotlib.rcParams['font.family'] = 'MS Gothic'
matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import csv

def log_debug(msg):
    try:
        log_dir = "C:/Users/aoiyu/Documents/PlatformIO/Projects/RTR2026/PitotCalibration"
        os.makedirs(log_dir, exist_ok=True)
        with open(os.path.join(log_dir, "debug.log"), "a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
    except:
        pass

class PitotLoggerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Team ЯTR - 自作ピトー管データロガー (UTC同期対応)")
        self.root.geometry("1400x900")
        self.root.configure(bg="#0F172A") # slate-900

        # Serial configuration
        self.ser = None
        self.serial_connected = False
        self.read_thread = None
        
        # Logging configuration
        self.is_logging = False
        self.log_file = None
        self.log_writer = None
        self.log_filepath = ""
        self.logging_thread = None
        
        # Real-time data storage
        self.current_data = {
            "airspeed": 0.0,
            "press_speed": 0.0,
            "aoa_pa": 0.0,
            "aos_pa": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "accx": 0.0,
            "accy": 0.0,
            "accz": 0.0,
            "gyrox": 0.0,
            "gyroy": 0.0,
            "gyroz": 0.0,
            "magx": 0.0,
            "magy": 0.0,
            "magz": 0.0,
            "temp": 0.0,
            "humid": 0.0
        }
        
        # For trend plots
        self.plot_history_len = 100
        self.history_time = []
        self.history_airspeed = []
        self.history_press_speed = []
        self.history_aoa = []
        self.history_aos = []
        self.history_pitch = []
        self.history_roll = []
        self.history_yaw = []
        self.history_accx = []
        self.history_accy = []
        self.history_accz = []
        
        self.start_app_time = time.time()
        
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

        # Entry & Combobox
        style.configure("TCombobox", fieldbackground="#1E293B", foreground="#FFFFFF", background="#334155", arrowcolor="#FFFFFF")
        style.map("TCombobox", fieldbackground=[("readonly", "#1E293B")], foreground=[("readonly", "#FFFFFF")])
        style.configure("TEntry", fieldbackground="#1E293B", foreground="#FFFFFF", insertcolor="#FFFFFF")
        
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
        
        # 1. Connection Card
        conn_frame = ttk.LabelFrame(left_container, text="1. ピトー管基板 接続設定", padding=10)
        conn_frame.pack(fill=tk.X, pady=4, padx=5)
        
        ttk.Label(conn_frame, text="COMポート:").pack(anchor=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, values=self.get_serial_ports(), width=15)
        self.port_combo.pack(fill=tk.X, pady=2)
        if self.port_combo["values"]:
            self.port_combo.current(0)
            
        ttk.Label(conn_frame, text="ボーレート:").pack(anchor=tk.W)
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, values=["9600", "38400", "57600", "115200"], width=15)
        self.baud_combo.pack(fill=tk.X, pady=2)
        
        btn_box = ttk.Frame(conn_frame)
        btn_box.pack(fill=tk.X, pady=4)
        
        self.btn_connect = ttk.Button(btn_box, text="接続する", command=self.toggle_connection, style="Action.TButton")
        self.btn_connect.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        btn_refresh = ttk.Button(btn_box, text="再検出", command=self.refresh_ports, style="Action.TButton", width=8)
        btn_refresh.pack(side=tk.RIGHT, padx=2)
        
        # 2. Logging Card
        log_frame = ttk.LabelFrame(left_container, text="2. CSVデータロギング (UTC同期)", padding=10)
        log_frame.pack(fill=tk.X, pady=4, padx=5)
        
        self.lbl_log_status = ttk.Label(log_frame, text="ステータス: 停止中", font=("Segoe UI", 10, "bold"), foreground="#94A3B8")
        self.lbl_log_status.pack(anchor=tk.W, pady=2)
        
        self.lbl_filepath = ttk.Label(log_frame, text="保存先: 未設定", font=("Segoe UI", 9), wraplength=300, justify=tk.LEFT, foreground="#94A3B8")
        self.lbl_filepath.pack(anchor=tk.W, pady=4)
        
        btn_log_box = ttk.Frame(log_frame)
        btn_log_box.pack(fill=tk.X, pady=4)
        
        self.btn_start_log = ttk.Button(btn_log_box, text="記録開始", command=self.start_logging, style="Start.TButton")
        self.btn_start_log.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        self.btn_stop_log = ttk.Button(btn_log_box, text="記録停止", command=self.stop_logging, style="Stop.TButton")
        self.btn_stop_log.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)
        self.btn_stop_log.state(["disabled"])

        # 3. Real-time Readouts Card
        readout_frame = ttk.LabelFrame(left_container, text="3. リアルタイム数値モニター", padding=10)
        readout_frame.pack(fill=tk.BOTH, expand=True, pady=4, padx=5)
        
        # We will structure readouts using a grid layout
        readout_grid = ttk.Frame(readout_frame)
        readout_grid.pack(fill=tk.BOTH, expand=True)
        
        vars_to_show = [
            ("対気速度 (参考)", "airspeed", " m/s", "#3B82F6", 0, 0),
            ("速度差圧 (SDP32)", "press_speed", " Pa", "#06B6D4", 1, 0),
            ("迎角差圧 (AoA SDP31)", "aoa_pa", " Pa", "#10B981", 2, 0),
            ("横滑り角差圧 (AoS SDP31)", "aos_pa", " Pa", "#EC4899", 3, 0),
            
            ("BNOロール角", "roll", "°", "#F59E0B", 4, 0),
            ("BNOピッチ角", "pitch", "°", "#F59E0B", 5, 0),
            ("BNOヨー角", "yaw", "°", "#F59E0B", 6, 0),
            
            ("SHT41 気温", "temp", " °C", "#10B981", 7, 0),
            ("SHT41 湿度", "humid", " %", "#06B6D4", 8, 0),
            
            ("BNO加速度 X", "accx", " m/s²", "#F8FAFC", 0, 2),
            ("BNO加速度 Y", "accy", " m/s²", "#F8FAFC", 1, 2),
            ("BNO加速度 Z", "accz", " m/s²", "#F8FAFC", 2, 2),
            
            ("BNO角速度 X", "gyrox", " dps", "#F8FAFC", 3, 2),
            ("BNO角速度 Y", "gyroy", " dps", "#F8FAFC", 4, 2),
            ("BNO角速度 Z", "gyroz", " dps", "#F8FAFC", 5, 2),
            
            ("BNO地磁気 X", "magx", " uT", "#F8FAFC", 6, 2),
            ("BNO地磁気 Y", "magy", " uT", "#F8FAFC", 7, 2),
            ("BNO地磁気 Z", "magz", " uT", "#F8FAFC", 8, 2),
        ]
        
        self.readout_labels = {}
        for label, key, unit, color, row, col in vars_to_show:
            ttk.Label(readout_grid, text=label + ":", font=("Segoe UI", 9)).grid(row=row, column=col, sticky=tk.W, pady=2, padx=5)
            lbl_val = ttk.Label(readout_grid, text="0.00" + unit, font=("Segoe UI", 10, "bold"), foreground=color)
            lbl_val.grid(row=row, column=col+1, sticky=tk.E, pady=2, padx=15)
            self.readout_labels[key] = (lbl_val, unit)
            
        # Right Panel
        right_container = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(right_container, weight=3)
        
        # Real-time Trend Plots
        self.setup_plots(right_container)

    def setup_plots(self, parent):
        self.fig = Figure(figsize=(8, 8), dpi=100, facecolor="#0F172A")
        
        # Subplot 1: Speed and SDP Pressures
        self.ax_press = self.fig.add_subplot(211)
        self.ax_press.set_facecolor("#1E293B")
        self.ax_press.set_title("対気速度 (m/s) ＆ 差圧データ (Pa) リアルタイム表示", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_press.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_press.grid(True, color="#334155", linestyle=":")
        
        self.line_speed, = self.ax_press.plot([], [], label="対気速度 (m/s)", color="#3B82F6", lw=2)
        self.line_press_speed, = self.ax_press.plot([], [], label="速度差圧 (Pa)", color="#06B6D4", lw=1.5, linestyle="--")
        self.line_aoa, = self.ax_press.plot([], [], label="AoA差圧 (Pa)", color="#10B981", lw=1.5)
        self.line_aos, = self.ax_press.plot([], [], label="AoS差圧 (Pa)", color="#EC4899", lw=1.5)
        
        self.ax_press.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8, loc="upper left")
        
        # Subplot 2: IMU Euler Angles & Acceleration
        self.ax_imu = self.fig.add_subplot(212)
        self.ax_imu.set_facecolor("#1E293B")
        self.ax_imu.set_title("BNO055 姿勢角度 (°)", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_imu.tick_params(colors="#94A3B8", labelsize=8)
        self.ax_imu.grid(True, color="#334155", linestyle=":")
        
        self.line_roll, = self.ax_imu.plot([], [], label="ロール角 (Roll)", color="#EC4899", lw=1.5)
        self.line_pitch, = self.ax_imu.plot([], [], label="ピッチ角 (Pitch)", color="#F59E0B", lw=2)
        self.line_yaw, = self.ax_imu.plot([], [], label="ヨー角 (Yaw)", color="#10B981", lw=1.5)
        
        self.ax_imu.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8, loc="upper left")
        
        self.fig.tight_layout()
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def refresh_ports(self):
        ports = self.get_serial_ports()
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
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
            messagebox.showerror("接続エラー", "ピトー管基板のCOMポートを選択してください。")
            return
            
        try:
            self.ser = serial.Serial(port, int(baud), timeout=1)
            self.ser.dtr = True
            self.ser.rts = True
            self.serial_connected = True
            self.btn_connect.configure(text="接続を切断", style="Stop.TButton")
            log_debug(f"Connected to {port} at {baud} bps")
            
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
        if self.is_logging:
            self.stop_logging()
        log_debug("Disconnected from serial port")

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
                elif key == "pitot_press_speed":
                    self.current_data["press_speed"] = val
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
                elif key == "pitot_accx":
                    self.current_data["accx"] = val
                elif key == "pitot_accy":
                    self.current_data["accy"] = val
                elif key == "pitot_accz":
                    self.current_data["accz"] = val
                elif key == "pitot_gyrox":
                    self.current_data["gyrox"] = val
                elif key == "pitot_gyroy":
                    self.current_data["gyroy"] = val
                elif key == "pitot_gyroz":
                    self.current_data["gyroz"] = val
                elif key == "pitot_magx":
                    self.current_data["magx"] = val
                elif key == "pitot_magy":
                    self.current_data["magy"] = val
                elif key == "pitot_magz":
                    self.current_data["magz"] = val
                elif key == "pitot_temp":
                    self.current_data["temp"] = val
                elif key == "pitot_humid":
                    self.current_data["humid"] = val
        except Exception as e:
            pass

    def start_logging(self):
        if not self.serial_connected:
            messagebox.showwarning("ロギングエラー", "ピトー管基板に接続してから記録を開始してください。")
            return
            
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSVファイル", "*.csv"), ("すべてのファイル", "*.*")],
            title="ロギングデータの保存先を設定"
        )
        
        if not filepath:
            return
            
        try:
            self.log_file = open(filepath, 'w', newline='', encoding='utf-8')
            self.log_writer = csv.writer(self.log_file)
            
            # Header matching user request:
            # sdp values in Pa (speed, aoa, aos), BNO055 9-axis (acc, gyro, mag) + roll, pitch, yaw
            self.log_writer.writerow([
                "timestamp_utc", "timestamp_epoch",
                "sdp_press_speed_pa", "sdp_press_aoa_pa", "sdp_press_aos_pa",
                "bno_roll_deg", "bno_pitch_deg", "bno_yaw_deg",
                "bno_acc_x_ms2", "bno_acc_y_ms2", "bno_acc_z_ms2",
                "bno_gyro_x_dps", "bno_gyro_y_dps", "bno_gyro_z_dps",
                "bno_mag_x_ut", "bno_mag_y_ut", "bno_mag_z_ut",
                "sht_temp_c", "sht_humid_pct"
            ])
            self.log_file.flush()
            
            self.log_filepath = filepath
            self.is_logging = True
            self.lbl_filepath.configure(text=f"保存先: {os.path.basename(filepath)}", foreground="#10B981")
            self.lbl_log_status.configure(text="ステータス: 記録中 (10Hz)", foreground="#10B981")
            
            self.btn_start_log.state(["disabled"])
            self.btn_stop_log.state(["!disabled"])
            
            log_debug(f"Started logging to {filepath}")
            
            # Launch logging thread
            self.logging_thread = threading.Thread(target=self.logging_loop, daemon=True)
            self.logging_thread.start()
            
        except Exception as e:
            messagebox.showerror("ロギングエラー", f"ロギング開始に失敗しました:\n{str(e)}")

    def stop_logging(self):
        self.is_logging = False
        if self.log_file:
            try:
                self.log_file.close()
            except:
                pass
            self.log_file = None
            
        self.lbl_log_status.configure(text="ステータス: 停止中", foreground="#94A3B8")
        self.btn_start_log.state(["!disabled"])
        self.btn_stop_log.state(["disabled"])
        messagebox.showinfo("記録完了", f"データを保存しました:\n{self.log_filepath}")
        log_debug(f"Stopped logging to {self.log_filepath}")

    def logging_loop(self):
        while self.is_logging:
            if self.log_file and not self.log_file.closed:
                now = datetime.datetime.now(datetime.timezone.utc)
                utc_str = now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] + 'Z'
                epoch = now.timestamp()
                
                try:
                    self.log_writer.writerow([
                        utc_str,
                        f"{epoch:.3f}",
                        f"{self.current_data['press_speed']:.3f}",
                        f"{self.current_data['aoa_pa']:.3f}",
                        f"{self.current_data['aos_pa']:.3f}",
                        f"{self.current_data['roll']:.3f}",
                        f"{self.current_data['pitch']:.3f}",
                        f"{self.current_data['yaw']:.3f}",
                        f"{self.current_data['accx']:.4f}",
                        f"{self.current_data['accy']:.4f}",
                        f"{self.current_data['accz']:.4f}",
                        f"{self.current_data['gyrox']:.4f}",
                        f"{self.current_data['gyroy']:.4f}",
                        f"{self.current_data['gyroz']:.4f}",
                        f"{self.current_data['magx']:.3f}",
                        f"{self.current_data['magy']:.3f}",
                        f"{self.current_data['magz']:.3f}",
                        f"{self.current_data['temp']:.2f}",
                        f"{self.current_data['humid']:.2f}"
                    ])
                    self.log_file.flush()
                except Exception as e:
                    log_debug(f"CSV line write error: {str(e)}")
            time.sleep(0.1) # Log at 10Hz

    def update_gui_loop(self):
        # Update text labels
        for key, (lbl, unit) in self.readout_labels.items():
            val = self.current_data[key]
            if key in ["accx", "accy", "accz", "gyrox", "gyroy", "gyroz"]:
                lbl.configure(text=f"{val:+.3f}{unit}")
            elif key in ["magx", "magy", "magz", "roll", "pitch", "yaw"]:
                lbl.configure(text=f"{val:+.2f}{unit}")
            else:
                lbl.configure(text=f"{val:.2f}{unit}")
                
        # History queue for plots
        curr_t = time.time() - self.start_app_time
        self.history_time.append(curr_t)
        self.history_airspeed.append(self.current_data["airspeed"])
        self.history_press_speed.append(self.current_data["press_speed"])
        self.history_aoa.append(self.current_data["aoa_pa"])
        self.history_aos.append(self.current_data["aos_pa"])
        self.history_pitch.append(self.current_data["pitch"])
        self.history_roll.append(self.current_data["roll"])
        self.history_yaw.append(self.current_data["yaw"])
        
        if len(self.history_time) > self.plot_history_len:
            self.history_time.pop(0)
            self.history_airspeed.pop(0)
            self.history_press_speed.pop(0)
            self.history_aoa.pop(0)
            self.history_aos.pop(0)
            self.history_pitch.pop(0)
            self.history_roll.pop(0)
            self.history_yaw.pop(0)
            
        # Draw trends graph
        self.redraw_plots()
            
        self.root.after(100, self.update_gui_loop)

    def redraw_plots(self):
        if not self.history_time:
            return
            
        # Update upper plot
        self.line_speed.set_data(self.history_time, self.history_airspeed)
        self.line_press_speed.set_data(self.history_time, self.history_press_speed)
        self.line_aoa.set_data(self.history_time, self.history_aoa)
        self.line_aos.set_data(self.history_time, self.history_aos)
        
        self.ax_press.relim()
        self.ax_press.autoscale_view()
        
        # Update lower plot
        self.line_roll.set_data(self.history_time, self.history_roll)
        self.line_pitch.set_data(self.history_time, self.history_pitch)
        self.line_yaw.set_data(self.history_time, self.history_yaw)
        
        self.ax_imu.relim()
        self.ax_imu.autoscale_view()
        
        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = PitotLoggerApp(root)
    root.mainloop()
