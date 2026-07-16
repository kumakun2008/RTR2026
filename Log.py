import struct
import csv
import os

def parse_rtr_binary_log(bin_filename, output_dir="extracted_logs"):
    os.makedirs(output_dir, exist_ok=True)
    
    # センサーIDごとのCSVヘッダ、構造体フォーマット、出力ファイル名の定義
    log_configs = {
        0x01: {"name": "main_imu.csv",     "fmt": "<ffffff",   "cols": ["Timestamp_us", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]},
        0x02: {"name": "main_mag.csv",     "fmt": "<fff",      "cols": ["Timestamp_us", "mag_x", "mag_y", "mag_z"]},
        0x03: {"name": "main_baro.csv",    "fmt": "<ff",       "cols": ["Timestamp_us", "pressure", "temperature"]},
        0x04: {"name": "pitot_data.csv",   "fmt": "<ffffff",   "cols": ["Timestamp_us", "diff_press_sdp32", "diff_press_sdp31_1", "diff_press_sdp31_2", "temp_sdp32", "temp_sdp31_1", "temp_sdp31_2"]},
        0x05: {"name": "battery.csv",      "fmt": "<ff",       "cols": ["Timestamp_us", "bus_voltage", "shunt_current"]},
        0x06: {"name": "gps_um982c.csv",   "fmt": "<ddffBBHff", "cols": ["Timestamp_us", "latitude", "longitude", "altitude", "speed", "sat_count", "fix_status", "heading", "utc", "hdop"]},
        0x07: {"name": "altimeter.csv",    "fmt": "<ff",       "cols": ["Timestamp_us", "ultrasonic_dist", "lidar_dist"]},
        0x08: {"name": "rudder.csv",       "fmt": "<ff",       "cols": ["Timestamp_us", "rudder_angle", "yaw_rate"]},
        0xFF: {"name": "event_mark.csv",   "fmt": "<fff",      "cols": ["Timestamp_us", "calib_offset_sdp32", "calib_offset_sdp31_1", "calib_offset_sdp31_2"]}
    }
    
    file_handlers = {}
    csv_writers = {}
    
    with open(bin_filename, "rb") as f:
        while True:
            # 1. ヘッダ読み込み (12バイト)
            header_data = f.read(12)
            if len(header_data) < 12:
                break
                
            sync, timestamp, sensor_id, length = struct.unpack("<HQBB", header_data)
            
            if sync != 0xAA55:
                # 同期ワード不正時は1バイト進めてシーク同期を試みる
                f.seek(-11, 1)
                continue
                
            # 2. ペイロード読み込み
            payload_data = f.read(length)
            if len(payload_data) < length:
                break
                
            # 3. フッター読み込み (2バイト)
            footer_data = f.read(2)
            if len(footer_data) < 2:
                break
                
            # 4. 該当するセンサーIDのCSVへ書き出し
            if sensor_id in log_configs:
                config = log_configs[sensor_id]
                
                if sensor_id not in csv_writers:
                    file_handlers[sensor_id] = open(os.path.join(output_dir, config["name"]), "w", newline="", encoding="utf-8")
                    csv_writers[sensor_id] = csv.writer(file_handlers[sensor_id])
                    csv_writers[sensor_id].writerow(config["cols"])
                
                try:
                    parsed_payload = struct.unpack(config["fmt"], payload_data)
                    csv_writers[sensor_id].writerow([timestamp] + list(parsed_payload))
                except struct.error:
                    print(f"パケット解析エラー: ID {hex(sensor_id)} / データ長 {length} バイト")
                    
    # すべてのファイルハンドラをクローズ
    for fh in file_handlers.values():
        fh.close()
    print("ログの抽出が完了しました。")

import sys

if __name__ == "__main__":
    # 引数があればそれを使用、なければデフォルトで "log_0020.bin" を解析
    target_bin_file = sys.argv[1] if len(sys.argv) > 1 else "log_0020.bin"
    
    if not os.path.exists(target_bin_file):
        print(f"[エラー] ファイル '{target_bin_file}' が見つかりません。")
        print("使い方: python Log.py [バイナリログファイル名.bin]")
        sys.exit(1)
        
    print(f"{target_bin_file} の解析を開始します...")
    parse_rtr_binary_log(target_bin_file)