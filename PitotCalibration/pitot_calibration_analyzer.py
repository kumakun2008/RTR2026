import sys
import os
import time
import datetime
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
import csv

class PitotAnalyzerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Team ЯTR - ピトー管校正データ解析ツール")
        self.root.geometry("1450x920")
        self.root.configure(bg="#0F172A") # slate-900
        
        # Paths
        self.pitot_path = ""
        self.mano_path = ""
        
        # Raw Data
        self.pitot_data = []
        self.mano_data = []
        
        # Synced Data
        self.synced_data = []
        
        # Calibration results
        self.calib_results = {}
        
        self.setup_styles()
        self.build_ui()

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        
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

        # Entry, Combobox, Treeview
        style.configure("TCombobox", fieldbackground="#1E293B", foreground="#FFFFFF", background="#334155", arrowcolor="#FFFFFF")
        style.map("TCombobox", fieldbackground=[("readonly", "#1E293B")], foreground=[("readonly", "#FFFFFF")])
        style.configure("TEntry", fieldbackground="#1E293B", foreground="#FFFFFF", insertcolor="#FFFFFF")
        
        style.configure("TNotebook", background="#0F172A", borderwidth=0)
        style.configure("TNotebook.Tab", background="#1E293B", foreground="#94A3B8", font=("Segoe UI", 10))
        style.map("TNotebook.Tab", background=[("selected", "#0F172A")], foreground=[("selected", "#06B6D4")])
        
        style.configure("Treeview", background="#1E293B", fieldbackground="#1E293B", foreground="#F8FAFC")
        style.configure("Treeview.Heading", background="#334155", foreground="#06B6D4", font=("Segoe UI", 9, "bold"))
        
        self.root.option_add("*TCombobox*Listbox.background", "#1E293B")
        self.root.option_add("*TCombobox*Listbox.foreground", "#FFFFFF")
        self.root.option_add("*TCombobox*Listbox.selectBackground", "#3B82F6")
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#FFFFFF")

    def build_ui(self):
        # Main split
        main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        
        # Left Panel (File Inputs & Configs)
        left_container = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(left_container, weight=1)
        
        # Card 1: Files
        files_frame = ttk.LabelFrame(left_container, text="1. ログファイル選択", padding=10)
        files_frame.pack(fill=tk.X, pady=4, padx=5)
        
        # Pitot File
        ttk.Label(files_frame, text="自作ピトー管データ (CSV):").pack(anchor=tk.W)
        pitot_row = ttk.Frame(files_frame)
        pitot_row.pack(fill=tk.X, pady=2)
        self.ent_pitot = ttk.Entry(pitot_row)
        self.ent_pitot.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        btn_pitot = ttk.Button(pitot_row, text="選択...", command=self.select_pitot_file, width=8)
        btn_pitot.pack(side=tk.RIGHT, padx=2)
        
        # Mano and Flow File
        ttk.Label(files_frame, text="Mano and Flowデータ (.xls / .csv):").pack(anchor=tk.W, pady=(5,0))
        mano_row = ttk.Frame(files_frame)
        mano_row.pack(fill=tk.X, pady=2)
        self.ent_mano = ttk.Entry(mano_row)
        self.ent_mano.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        btn_mano = ttk.Button(mano_row, text="選択...", command=self.select_mano_file, width=8)
        btn_mano.pack(side=tk.RIGHT, padx=2)
        
        # Card 2: Time Alignment Settings
        time_frame = ttk.LabelFrame(left_container, text="2. 時刻同期・アライメント設定", padding=10)
        time_frame.pack(fill=tk.X, pady=4, padx=5)
        
        # Mano and Flow timezone
        ttk.Label(time_frame, text="Mano and Flow の時刻設定:").grid(row=0, column=0, sticky=tk.W, pady=3)
        self.mano_tz_var = tk.StringVar(value="JST (UTC+9)")
        cb_mano_tz = ttk.Combobox(time_frame, textvariable=self.mano_tz_var, values=["JST (UTC+9)", "UTC (協定世界時)", "指定なし (時間差0)"], state="readonly", width=18)
        cb_mano_tz.grid(row=0, column=1, sticky=tk.W, pady=3, padx=5)
        
        # Fine sync offset
        ttk.Label(time_frame, text="微調整オフセット (秒):").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.ent_offset = ttk.Entry(time_frame, width=10)
        self.ent_offset.insert(0, "0.0")
        self.ent_offset.grid(row=1, column=1, sticky=tk.W, pady=3, padx=5)
        ttk.Label(time_frame, text="※基準計の時間を進める場合は正,遅らせる場合は負").grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=1)

        # Card 3: Calibration Parameters
        param_frame = ttk.LabelFrame(left_container, text="3. 校正計算設定", padding=10)
        param_frame.pack(fill=tk.X, pady=4, padx=5)
        
        # Min dynamic pressure q
        ttk.Label(param_frame, text="有効基準動圧下限 (Pa):").grid(row=0, column=0, sticky=tk.W, pady=3)
        self.ent_min_q = ttk.Entry(param_frame, width=10)
        self.ent_min_q.insert(0, "5.0")
        self.ent_min_q.grid(row=0, column=1, sticky=tk.W, pady=3, padx=5)
        
        # Air density source
        ttk.Label(param_frame, text="空気密度 (ρ) [kg/m³]:").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.ent_rho = ttk.Entry(param_frame, width=10)
        self.ent_rho.insert(0, "1.225")
        self.ent_rho.grid(row=1, column=1, sticky=tk.W, pady=3, padx=5)
        
        # Action Buttons
        self.btn_run = ttk.Button(left_container, text="同期 ＆ 校正計算の実行", command=self.run_analysis, style="Start.TButton")
        self.btn_run.pack(fill=tk.X, pady=10, padx=5)
        
        # Right Panel (Results displays)
        right_container = ttk.Frame(main_pane, style="TFrame")
        main_pane.add(right_container, weight=3)
        
        self.notebook = ttk.Notebook(right_container)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Tab 1: Text Report
        tab_report = ttk.Frame(self.notebook, style="TFrame")
        self.notebook.add(tab_report, text="校正レポート")
        self.build_report_tab(tab_report)
        
        # Tab 2: Regression Plots
        tab_plots = ttk.Frame(self.notebook, style="TFrame")
        self.notebook.add(tab_plots, text="回帰グラフ")
        self.build_plots_tab(tab_plots)
        
        # Tab 3: Synced Data Table
        tab_table = ttk.Frame(self.notebook, style="TFrame")
        self.notebook.add(tab_table, text="同期データプレビュー")
        self.build_table_tab(tab_table)

    def build_report_tab(self, parent):
        report_frame = ttk.Frame(parent, padding=10)
        report_frame.pack(fill=tk.BOTH, expand=True)
        
        self.txt_report = tk.Text(report_frame, bg="#1E293B", fg="#F8FAFC", font=("Consolas", 10), wrap=tk.WORD)
        self.txt_report.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        self.txt_report.insert(tk.END, "ログファイルを選択して『同期 ＆ 校正計算の実行』ボタンを押してください。\n")
        
        btn_box = ttk.Frame(report_frame)
        btn_box.pack(fill=tk.X)
        self.btn_save_report = ttk.Button(btn_box, text="校正レポートを保存...", command=self.save_report_file, style="Action.TButton")
        self.btn_save_report.pack(side=tk.LEFT, padx=5)
        self.btn_save_report.state(["disabled"])
        
        self.btn_export_csv = ttk.Button(btn_box, text="同期・校正済みCSVをエクスポート...", command=self.export_synced_csv, style="Action.TButton")
        self.btn_export_csv.pack(side=tk.LEFT, padx=5)
        self.btn_export_csv.state(["disabled"])

    def build_plots_tab(self, parent):
        self.fig = Figure(figsize=(8, 8), dpi=100, facecolor="#0F172A")
        
        # Plot 1: Pitch vs AoA ratio
        self.ax_aoa = self.fig.add_subplot(221)
        self.ax_aoa.set_facecolor("#1E293B")
        self.ax_aoa.set_title("ピッチ角 vs. AoA差圧比", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_aoa.grid(True, color="#334155", linestyle=":")
        
        # Plot 2: Yaw vs AoS ratio
        self.ax_aos = self.fig.add_subplot(222)
        self.ax_aos.set_facecolor("#1E293B")
        self.ax_aos.set_title("ヨー角 vs. AoS差圧比", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_aos.grid(True, color="#334155", linestyle=":")
        
        # Plot 3: Speed comparison
        self.ax_speed = self.fig.add_subplot(212)
        self.ax_speed.set_facecolor("#1E293B")
        self.ax_speed.set_title("時系列 速度対比 (基準計 vs 自作ピトー管)", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_speed.grid(True, color="#334155", linestyle=":")
        
        self.fig.tight_layout()
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def build_table_tab(self, parent):
        table_frame = ttk.Frame(parent, padding=10)
        table_frame.pack(fill=tk.BOTH, expand=True)
        
        columns = ("time", "ref_press", "ref_speed", "pitot_press", "pitot_speed", "aoa_press", "aos_press", "pitch", "yaw")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings")
        
        self.tree.heading("time", text="同期時刻 (UTC)")
        self.tree.heading("ref_press", text="基準動圧 (Pa)")
        self.tree.heading("ref_speed", text="基準速度 (m/s)")
        self.tree.heading("pitot_press", text="自作動圧 (Pa)")
        self.tree.heading("pitot_speed", text="自作速度 (m/s)")
        self.tree.heading("aoa_press", text="AoA差圧 (Pa)")
        self.tree.heading("aos_press", text="AoS差圧 (Pa)")
        self.tree.heading("pitch", text="Pitch (°)")
        self.tree.heading("yaw", text="Yaw (°)")
        
        for col in columns:
            self.tree.column(col, width=110, anchor=tk.CENTER)
            
        # Add scrollbar
        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    def select_pitot_file(self):
        filepath = filedialog.askopenfilename(
            filetypes=[("CSVファイル", "*.csv"), ("すべてのファイル", "*.*")],
            title="自作ピトー管データを選択"
        )
        if filepath:
            self.pitot_path = filepath
            self.ent_pitot.delete(0, tk.END)
            self.ent_pitot.insert(0, filepath)

    def select_mano_file(self):
        filepath = filedialog.askopenfilename(
            filetypes=[("Mano & Flow / Excel / CSV", "*.xls *.csv"), ("すべてのファイル", "*.*")],
            title="Mano and Flowデータを選択"
        )
        if filepath:
            self.mano_path = filepath
            self.ent_mano.delete(0, tk.END)
            self.ent_mano.insert(0, filepath)

    def load_pitot_csv(self, path):
        data = []
        with open(path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    data.append({
                        "epoch": float(row["timestamp_epoch"]),
                        "utc": row["timestamp_utc"],
                        "press_speed": float(row["sdp_press_speed_pa"]),
                        "aoa_pa": float(row["sdp_press_aoa_pa"]),
                        "aos_pa": float(row["sdp_press_aos_pa"]),
                        "roll": float(row["bno_roll_deg"]),
                        "pitch": float(row["bno_pitch_deg"]),
                        "yaw": float(row["bno_yaw_deg"]),
                        "accx": float(row["bno_acc_x_ms2"]),
                        "accy": float(row["bno_acc_y_ms2"]),
                        "accz": float(row["bno_acc_z_ms2"]),
                        "gyrox": float(row["bno_gyro_x_dps"]),
                        "gyroy": float(row["bno_gyro_y_dps"]),
                        "gyroz": float(row["bno_gyro_z_dps"]),
                        "magx": float(row["bno_mag_x_ut"]),
                        "magy": float(row["bno_mag_y_ut"]),
                        "magz": float(row["bno_mag_z_ut"]),
                        "temp": float(row["sht_temp_c"]),
                        "humid": float(row["sht_humid_pct"])
                    })
                except Exception as e:
                    pass
        return data

    def load_mano_flow_file(self, path):
        data = []
        # Support tab-separated/space-separated format (even if named .xls)
        with open(path, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
            
        header = None
        for line in lines:
            line_str = line.strip()
            if not line_str:
                continue
                
            # Split by whitespace/tab
            parts = re.split(r'\s+', line_str)
            
            # Detect header
            if "PRESS" in parts or "VEL" in parts or "TIME" in parts:
                header = [p.upper() for p in parts]
                continue
                
            if header and len(parts) >= min(len(header), 5):
                try:
                    # Map values based on header indices
                    row_dict = {}
                    for i, h in enumerate(header):
                        if i < len(parts):
                            row_dict[h] = parts[i]
                            
                    # Parse time: 07-04-26/16:06:40
                    time_str = row_dict["TIME"]
                    dt = datetime.datetime.strptime(time_str, "%m-%d-%y/%H:%M:%S")
                    
                    data.append({
                        "datetime": dt,
                        "press": float(row_dict["PRESS"]),
                        "vel": float(row_dict["VEL"]),
                        "temp": float(row_dict["TEMP"]),
                        "flow": float(row_dict["FLOW"])
                    })
                except Exception as e:
                    pass
        return data

    def run_analysis(self):
        self.pitot_path = self.ent_pitot.get().strip()
        self.mano_path = self.ent_mano.get().strip()
        
        if not self.pitot_path or not self.mano_path:
            messagebox.showerror("入力エラー", "両方のログファイルを選択してください。")
            return
            
        try:
            self.pitot_data = self.load_pitot_csv(self.pitot_path)
            self.mano_data = self.load_mano_flow_file(self.mano_path)
        except Exception as e:
            messagebox.showerror("読込エラー", f"ファイルの読み込みに失敗しました:\n{str(e)}")
            return
            
        if not self.pitot_data:
            messagebox.showerror("データエラー", "自作ピトー管の有効なデータが見つかりませんでした。")
            return
        if not self.mano_data:
            messagebox.showerror("データエラー", "Mano and Flow の有効なデータが見つかりませんでした。")
            return
            
        # Parse configurations
        try:
            fine_offset = float(self.ent_offset.get())
        except:
            fine_offset = 0.0
            
        try:
            min_q = float(self.ent_min_q.get())
        except:
            min_q = 5.0
            
        try:
            rho_air = float(self.ent_rho.get())
        except:
            rho_air = 1.225
            
        # Compute epoch timestamp for Mano and Flow data based on timezone settings
        tz_opt = self.mano_tz_var.get()
        if "JST" in tz_opt:
            tz_offset = 9.0  # JST is UTC+9
        else:
            tz_offset = 0.0  # UTC or None
            
        for row in self.mano_data:
            # Shift time to UTC
            utc_dt = row["datetime"] - datetime.timedelta(hours=tz_offset)
            # Add manual offset
            utc_dt = utc_dt + datetime.timedelta(seconds=fine_offset)
            row["epoch_utc"] = utc_dt.timestamp()
            row["utc_str"] = utc_dt.strftime('%Y-%m-%d %H:%M:%S')

        # Synchronize data:
        # For each Mano and Flow data point (1Hz), aggregate pitot data around that timestamp (+- 0.5s)
        self.synced_data = []
        pitot_epochs = np.array([p["epoch"] for p in self.pitot_data])
        
        for m_row in self.mano_data:
            ref_t = m_row["epoch_utc"]
            # Find indices of pitot data within [ref_t - 0.5, ref_t + 0.5]
            idxs = np.where((pitot_epochs >= ref_t - 0.5) & (pitot_epochs <= ref_t + 0.5))[0]
            
            if len(idxs) > 0:
                # Average pitot values in this window
                window_pitot = [self.pitot_data[i] for i in idxs]
                avg_press_speed = np.mean([p["press_speed"] for p in window_pitot])
                avg_aoa = np.mean([p["aoa_pa"] for p in window_pitot])
                avg_aos = np.mean([p["aos_pa"] for p in window_pitot])
                avg_roll = np.mean([p["roll"] for p in window_pitot])
                avg_pitch = np.mean([p["pitch"] for p in window_pitot])
                avg_yaw = np.mean([p["yaw"] for p in window_pitot])
                avg_airspeed = np.mean([p["airspeed"] for p in window_pitot])
                
                # Accel, gyro, mag averages
                avg_accx = np.mean([p["accx"] for p in window_pitot])
                avg_accy = np.mean([p["accy"] for p in window_pitot])
                avg_accz = np.mean([p["accz"] for p in window_pitot])
                avg_gyrox = np.mean([p["gyrox"] for p in window_pitot])
                avg_gyroy = np.mean([p["gyroy"] for p in window_pitot])
                avg_gyroz = np.mean([p["gyroz"] for p in window_pitot])
                avg_magx = np.mean([p["magx"] for p in window_pitot])
                avg_magy = np.mean([p["magy"] for p in window_pitot])
                avg_magz = np.mean([p["magz"] for p in window_pitot])
                avg_temp = np.mean([p["temp"] for p in window_pitot])
                avg_humid = np.mean([p["humid"] for p in window_pitot])

                self.synced_data.append({
                    "utc_str": m_row["utc_str"],
                    "epoch_utc": ref_t,
                    "ref_press": m_row["press"], # Pa
                    "ref_speed": m_row["vel"],   # m/s
                    "pitot_press_speed": avg_press_speed,
                    "pitot_aoa_pa": avg_aoa,
                    "pitot_aos_pa": avg_aos,
                    "pitot_roll": avg_roll,
                    "pitot_pitch": avg_pitch,
                    "pitot_yaw": avg_yaw,
                    "pitot_airspeed": avg_airspeed,
                    "pitot_accx": avg_accx,
                    "pitot_accy": avg_accy,
                    "pitot_accz": avg_accz,
                    "pitot_gyrox": avg_gyrox,
                    "pitot_gyroy": avg_gyroy,
                    "pitot_gyroz": avg_gyroz,
                    "pitot_magx": avg_magx,
                    "pitot_magy": avg_magy,
                    "pitot_magz": avg_magz,
                    "pitot_temp": avg_temp,
                    "pitot_humid": avg_humid
                })

        if not self.synced_data:
            messagebox.showerror("同期エラー", "時刻同期できる共通の時間帯が見つかりませんでした。\nタイムゾーンや微調整オフセットの設定を確認してください。")
            return
            
        # Perform Regression Analysis
        self.calculate_calibration(min_q, rho_air)
        
        # Update Displays
        self.update_report_text(min_q, rho_air)
        self.update_table_view()
        self.draw_plots()
        
        self.btn_save_report.state(["!disabled"])
        self.btn_export_csv.state(["!disabled"])
        
        self.notebook.select(0) # Select report tab
        messagebox.showinfo("解析完了", f"同期完了: {len(self.synced_data)}点のデータを処理しました。")

    def calculate_calibration(self, min_q, rho):
        # Filter data points where reference dynamic pressure is above threshold
        # Mano and Flow's PRESS is used as reference dynamic pressure q. If PRESS is zero, compute q from VEL.
        valid_points = []
        for d in self.synced_data:
            q = d["ref_press"]
            if q <= 0.0:
                q = 0.5 * rho * (d["ref_speed"] ** 2)
            
            if q >= min_q:
                valid_points.append((d, q))
                
        self.calib_results = {
            "num_total": len(self.synced_data),
            "num_valid": len(valid_points),
            "min_q_filter": min_q,
            "rho": rho,
            "cp_values": [],
            "mean_cp": 1.0,
            "aoa_k": 0.0,
            "aoa_offset": 0.0,
            "aoa_r2": 0.0,
            "aos_k": 0.0,
            "aos_offset": 0.0,
            "aos_r2": 0.0,
            "fit_aoa_x": [], "fit_aoa_y": [],
            "fit_aos_x": [], "fit_aos_y": []
        }
        
        if len(valid_points) < 3:
            return
            
        # 1. Airspeed calibration factor Cp (pitot_press / ref_q)
        cps = []
        for d, q in valid_points:
            cps.append(d["pitot_press_speed"] / q)
        self.calib_results["cp_values"] = cps
        self.calib_results["mean_cp"] = np.mean(cps)
        
        # 2. AoA Calibration (Pitch vs. AoA Differential Pressure Ratio)
        x_aoa = []
        y_pitch = []
        for d, q in valid_points:
            x_aoa.append(d["pitot_aoa_pa"] / q)
            y_pitch.append(d["pitot_pitch"])
            
        slope_aoa, intercept_aoa = np.polyfit(x_aoa, y_pitch, 1)
        self.calib_results["aoa_k"] = slope_aoa
        self.calib_results["aoa_offset"] = intercept_aoa
        
        # Calculate R2 for AoA
        y_pred = [slope_aoa * x + intercept_aoa for x in x_aoa]
        ss_tot = np.sum((y_pitch - np.mean(y_pitch))**2)
        ss_res = np.sum((y_pitch - y_pred)**2)
        self.calib_results["aoa_r2"] = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
        self.calib_results["fit_aoa_x"] = x_aoa
        self.calib_results["fit_aoa_y"] = y_pitch
        
        # 3. AoS Calibration (Yaw vs. AoS Differential Pressure Ratio)
        x_aos = []
        y_yaw = []
        for d, q in valid_points:
            x_aos.append(d["pitot_aos_pa"] / q)
            y_yaw.append(d["pitot_yaw"])
            
        slope_aos, intercept_aos = np.polyfit(x_aos, y_yaw, 1)
        self.calib_results["aos_k"] = slope_aos
        self.calib_results["aos_offset"] = intercept_aos
        
        # Calculate R2 for AoS
        y_pred_s = [slope_aos * x + intercept_aos for x in x_aos]
        ss_tot_s = np.sum((y_yaw - np.mean(y_yaw))**2)
        ss_res_s = np.sum((y_yaw - y_pred_s)**2)
        self.calib_results["aos_r2"] = 1.0 - (ss_res_s / ss_tot_s) if ss_tot_s > 0 else 0.0
        self.calib_results["fit_aos_x"] = x_aos
        self.calib_results["fit_aos_y"] = y_yaw

    def update_report_text(self, min_q, rho):
        r = self.calib_results
        
        report = []
        report.append("==================================================")
        report.append("     TEAM ЯTR - ピトー管 校正係数＆迎角・横滑り角解析")
        report.append("==================================================")
        report.append(f"解析実行日時: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"自作ピトー管ログ: {os.path.basename(self.pitot_path)}")
        report.append(f"基準計Mano & Flowログ: {os.path.basename(self.mano_path)}")
        report.append("--------------------------------------------------")
        report.append(f"同期データ点数: {r['num_total']} 点")
        report.append(f"有効校正点数 (q >= {min_q} Pa): {r['num_valid']} 点")
        report.append(f"設定空気密度 (ρ): {rho:.4f} kg/m³")
        report.append("--------------------------------------------------")
        
        if r["num_valid"] < 3:
            report.append("【エラー】有効な校正ポイントが不足しています(最低3点必要)。")
            report.append("基準動圧が小さすぎるか、時刻同期範囲がずれている可能性があります。")
        else:
            # Cp
            report.append("【ピトー管係数 (動圧比 Cp)】")
            report.append(f"  Cp (dP_speed / q_ref) 平均値: {r['mean_cp']:.5f}")
            report.append(f"  (※理想値は1.00。Cp > 1の場合,自作ピトー管の動圧出力が基準より大きく出ています)")
            report.append(f"  速度校正乗数 K_v (V_ref / V_pitot): {1.0 / np.sqrt(r['mean_cp']):.5f}")
            report.append("")
            
            # AoA
            report.append("【迎角 (AoA) 特性パラメータ】")
            report.append(f"  感度係数 K_AoA:           {r['aoa_k']:.5f} ° / (Pa/Pa)")
            report.append(f"  アライメントオフセット:    {r['aoa_offset']:.3f} °")
            report.append(f"  決定係数 (適合度 R²):      {r['aoa_r2']:.4f}")
            report.append(f"  換算式: AoA = {r['aoa_k']:.4f} * (dP_AoA / q) + ({r['aoa_offset']:.3f})")
            report.append("")
            
            # AoS
            report.append("【横滑り角 (AoS) 特性パラメータ】")
            report.append(f"  感度係数 K_AoS:           {r['aos_k']:.5f} ° / (Pa/Pa)")
            report.append(f"  アライメントオフセット:    {r['aos_offset']:.3f} °")
            report.append(f"  決定係数 (適合度 R²):      {r['aos_r2']:.4f}")
            report.append(f"  換算式: AoS = {r['aos_k']:.4f} * (dP_AoS / q) + ({r['aos_offset']:.3f})")
            
        report.append("==================================================")
        
        self.txt_report.delete(1.0, tk.END)
        self.txt_report.insert(tk.END, "\n".join(report))
        self.report_text = "\n".join(report)

    def update_table_view(self):
        # Clear table
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        for d in self.synced_data[:200]: # Limit to first 200 for GUI performance
            self.tree.insert("", "end", values=(
                d["utc_str"],
                f"{d['ref_press']:.2f}",
                f"{d['ref_speed']:.2f}",
                f"{d['pitot_press_speed']:.2f}",
                f"{d['pitot_airspeed']:.2f}",
                f"{d['pitot_aoa_pa']:.2f}",
                f"{d['pitot_aos_pa']:.2f}",
                f"{d['pitot_pitch']:.2f}°",
                f"{d['pitot_yaw']:.2f}°"
            ))

    def draw_plots(self):
        r = self.calib_results
        
        # 1. AoA plot
        self.ax_aoa.clear()
        self.ax_aoa.set_facecolor("#1E293B")
        self.ax_aoa.set_title("ピッチ角 vs. AoA差圧比", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_aoa.tick_params(colors="#94A3B8", labelsize=8)
        
        if "fit_aoa_x" in r and len(r["fit_aoa_x"]) > 0:
            self.ax_aoa.scatter(r["fit_aoa_x"], r["fit_aoa_y"], color="#10B981", s=15, alpha=0.6, label="測定データ")
            x_line = np.linspace(min(r["fit_aoa_x"]), max(r["fit_aoa_x"]), 100)
            y_line = r["aoa_k"] * x_line + r["aoa_offset"]
            self.ax_aoa.plot(x_line, y_line, color="#EF4444", lw=2, label=f"回帰直線 R²={r['aoa_r2']:.4f}")
            self.ax_aoa.set_xlabel("差圧比 (dP_AoA / q)", color="#94A3B8", fontsize=8)
            self.ax_aoa.set_ylabel("ピッチ角 (°)", color="#94A3B8", fontsize=8)
            self.ax_aoa.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8)
            
        # 2. AoS plot
        self.ax_aos.clear()
        self.ax_aos.set_facecolor("#1E293B")
        self.ax_aos.set_title("ヨー角 vs. AoS差圧比", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_aos.tick_params(colors="#94A3B8", labelsize=8)
        
        if "fit_aos_x" in r and len(r["fit_aos_x"]) > 0:
            self.ax_aos.scatter(r["fit_aos_x"], r["fit_aos_y"], color="#EC4899", s=15, alpha=0.6, label="測定データ")
            x_line = np.linspace(min(r["fit_aos_x"]), max(r["fit_aos_x"]), 100)
            y_line = r["aos_k"] * x_line + r["aos_offset"]
            self.ax_aos.plot(x_line, y_line, color="#3B82F6", lw=2, label=f"回帰直線 R²={r['aos_r2']:.4f}")
            self.ax_aos.set_xlabel("差圧比 (dP_AoS / q)", color="#94A3B8", fontsize=8)
            self.ax_aos.set_ylabel("ヨー角 (°)", color="#94A3B8", fontsize=8)
            self.ax_aos.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8)
            
        # 3. Speed Comparison plot (Time-series)
        self.ax_speed.clear()
        self.ax_speed.set_facecolor("#1E293B")
        self.ax_speed.set_title("時系列 速度対比 (基準計 vs 自作ピトー管)", color="#06B6D4", fontsize=10, fontweight="bold")
        self.ax_speed.tick_params(colors="#94A3B8", labelsize=8)
        
        if self.synced_data:
            times = [i for i in range(len(self.synced_data))]
            ref_speeds = [d["ref_speed"] for d in self.synced_data]
            pitot_speeds = [d["pitot_airspeed"] for d in self.synced_data]
            
            self.ax_speed.plot(times, ref_speeds, label="基準計 (Mano & Flow)", color="#06B6D4", lw=2)
            self.ax_speed.plot(times, pitot_speeds, label="自作ピトー管 (未校正)", color="#3B82F6", lw=1.5, linestyle="--")
            self.ax_speed.set_xlabel("データ点番号", color="#94A3B8", fontsize=8)
            self.ax_speed.set_ylabel("対気速度 (m/s)", color="#94A3B8", fontsize=8)
            self.ax_speed.legend(facecolor="#1E293B", edgecolor="#334155", labelcolor="#F8FAFC", fontsize=8)
            
        self.fig.tight_layout()
        self.canvas.draw()

    def save_report_file(self):
        if not hasattr(self, 'report_text') or not self.report_text:
            return
            
        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("テキストファイル", "*.txt"), ("すべてのファイル", "*.*")],
            title="校正レポートを保存する"
        )
        if filepath:
            try:
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write(self.report_text)
                messagebox.showinfo("保存完了", f"校正レポートを保存しました：\n{filepath}")
            except Exception as e:
                messagebox.showerror("保存エラー", f"レポートファイルを保存できませんでした:\n{str(e)}")

    def export_synced_csv(self):
        if not self.synced_data:
            return
            
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSVファイル", "*.csv"), ("すべてのファイル", "*.*")],
            title="同期データのエクスポート"
        )
        
        if filepath:
            try:
                r = self.calib_results
                cp = r.get("mean_cp", 1.0)
                aoa_k = r.get("aoa_k", 0.0)
                aoa_off = r.get("aoa_offset", 0.0)
                aos_k = r.get("aos_k", 0.0)
                aos_off = r.get("aos_offset", 0.0)
                
                with open(filepath, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    
                    # Columns
                    writer.writerow([
                        "timestamp_utc", "epoch_utc",
                        "ref_press_pa", "ref_speed_m_s",
                        "pitot_raw_press_speed_pa", "pitot_raw_press_aoa_pa", "pitot_raw_press_aos_pa",
                        "bno_roll_deg", "bno_pitch_deg", "bno_yaw_deg",
                        "pitot_calibrated_airspeed_m_s",
                        "pitot_calculated_aoa_deg",
                        "pitot_calculated_aos_deg",
                        "bno_acc_x_ms2", "bno_acc_y_ms2", "bno_acc_z_ms2",
                        "bno_gyro_x_dps", "bno_gyro_y_dps", "bno_gyro_z_dps",
                        "bno_mag_x_ut", "bno_mag_y_ut", "bno_mag_z_ut",
                        "sht_temp_c", "sht_humid_pct"
                    ])
                    
                    for d in self.synced_data:
                        # Compute calibrated outputs
                        # Dynamic pressure q (use reference pressure or calculate from speed)
                        q = d["ref_press"]
                        if q <= 0.0:
                            q = 0.5 * r["rho"] * (d["ref_speed"] ** 2)
                        
                        # Calibrated speed = sqrt(2 * pitot_press_corrected / rho)
                        # pitot_press_corrected = pitot_press_speed / Cp
                        cp_corr = cp if cp > 0 else 1.0
                        press_corr = d["pitot_press_speed"] / cp_corr
                        cal_speed = np.sqrt(2 * press_corr / r["rho"]) if press_corr > 0 else 0.0
                        
                        # Calculated Angles
                        cal_aoa = aoa_k * (d["pitot_aoa_pa"] / q) + aoa_off if q > 0 else 0.0
                        cal_aos = aos_k * (d["pitot_aos_pa"] / q) + aos_off if q > 0 else 0.0
                        
                        writer.writerow([
                            d["utc_str"],
                            f"{d['epoch_utc']:.3f}",
                            f"{d['ref_press']:.3f}",
                            f"{d['ref_speed']:.3f}",
                            f"{d['pitot_press_speed']:.3f}",
                            f"{d['pitot_aoa_pa']:.3f}",
                            f"{d['pitot_aos_pa']:.3f}",
                            f"{d['pitot_roll']:.3f}",
                            f"{d['pitot_pitch']:.3f}",
                            f"{d['pitot_yaw']:.3f}",
                            f"{cal_speed:.3f}",
                            f"{cal_aoa:.3f}",
                            f"{cal_aos:.3f}",
                            f"{d['pitot_accx']:.4f}",
                            f"{d['pitot_accy']:.4f}",
                            f"{d['pitot_accz']:.4f}",
                            f"{d['pitot_gyrox']:.4f}",
                            f"{d['pitot_gyroy']:.4f}",
                            f"{d['pitot_gyroz']:.4f}",
                            f"{d['pitot_magx']:.3f}",
                            f"{d['pitot_magy']:.3f}",
                            f"{d['pitot_magz']:.3f}",
                            f"{d['pitot_temp']:.2f}",
                            f"{d['pitot_humid']:.2f}"
                        ])
                        
                messagebox.showinfo("保存完了", f"同期＆校正済みCSVを正常に保存しました：\n{filepath}")
            except Exception as e:
                messagebox.showerror("保存エラー", f"CSVファイルを保存できませんでした:\n{str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = PitotAnalyzerApp(root)
    root.mainloop()
