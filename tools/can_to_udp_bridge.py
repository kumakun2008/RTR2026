#!/usr/bin/env python3
"""
RTR2026 CAN-to-UDP Bridge Gateway.
Receives CAN frames from the CAN bus (via python-can) and forwards them 
over a UDP socket as JSON telemetry packets (either raw or parsed format).

Author: Team ЯTR
Date: 2026-06-25
"""

import sys
import socket
import struct
import json
import time

# Attempt to import python-can, with instructions if missing
try:
    import can
except ImportError:
    print("[WARNING] python-can is not installed.")
    print("Please install it on your PC using: pip install python-can")
    can = None

# Target UDP configurations
UDP_IP = "255.255.255.255"  # Broadcast by default
UDP_PORT = 5005

# CAN ID Mapping definitions
CAN_IDS = {
    0x010: "ATTITUDE",      # Pitch (float) + Roll (float)
    0x011: "AIRSPEED",      # SDP32 press (float) + Airspeed (float)
    0x012: "RUDDER_ANGLE",  # Rudder angle (float)
    0x020: "ALTITUDE",      # Static press (float) / LiDAR (float) + US (float)
    0x021: "GPS_POS",       # Lat (int32 * 1e7) + Lon (int32 * 1e7)
    0x022: "AOA_AOS",       # SDP31_1 press (float) + SDP31_2 press (float)
    0x050: "BATTERY_VOLT",  # Voltage (float)
    0x060: "VOICE_CMD",     # Voice Command Index (uint8)
    0x070: "CALIB_ZERO",    # Calib command
    0x080: "OTA_START",     # OTA command
}

def parse_can_payload(can_id, data):
    """
    Parses binary CAN frame payloads into human-readable telemetry dictionaries
    based on the RTR2026 standard 11-bit CAN protocol.
    """
    if len(data) < 1:
        return {}

    parsed = {"name": CAN_IDS.get(can_id, f"UNKNOWN_{hex(can_id)}")}
    
    try:
        if can_id == 0x010 and len(data) >= 8:
            pitch, roll = struct.unpack("<ff", data[:8])
            parsed.update({"pitch": round(pitch, 2), "roll": round(roll, 2)})
            
        elif can_id == 0x011 and len(data) >= 8:
            press, airspeed = struct.unpack("<ff", data[:8])
            parsed.update({"sdp32_press": round(press, 2), "airspeed": round(airspeed, 2)})
            
        elif can_id == 0x012 and len(data) >= 4:
            angle = struct.unpack("<f", data[:4])[0]
            parsed.update({"rudder_angle": round(angle, 2)})
            
        elif can_id == 0x020 and len(data) >= 8:
            val1, val2 = struct.unpack("<ff", data[:8])
            if val2 > 0.001:  # Altimeter node sending LiDAR + Ultrasonic
                parsed.update({"lidar_alt": round(val1, 2), "ultrasonic_alt": round(val2, 2)})
            else:  # Main board sending static pressure
                parsed.update({"static_pressure": round(val1, 2)})
                
        elif can_id == 0x021 and len(data) >= 8:
            lat_int, lon_int = struct.unpack("<ii", data[:8])
            parsed.update({"latitude": lat_int / 10000000.0, "longitude": lon_int / 10000000.0})
            
        elif can_id == 0x022 and len(data) >= 8:
            p1, p2 = struct.unpack("<ff", data[:8])
            parsed.update({"sdp31_1_press": round(p1, 2), "sdp31_2_press": round(p2, 2)})
            
        elif can_id == 0x050 and len(data) >= 4:
            voltage = struct.unpack("<f", data[:4])[0]
            parsed.update({"battery_voltage": round(voltage, 2)})
            
        elif can_id == 0x060 and len(data) >= 1:
            parsed.update({"voice_alert_code": data[0]})
            
        elif can_id in (0x070, 0x080):
            parsed.update({"command_triggered": True})
            
    except Exception as e:
        parsed.update({"error": f"Parse failed: {str(e)}"})
        
    return parsed

def main():
    print("=== RTR2026 CAN-to-UDP Telemetry Bridge ===")
    
    # Initialize UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"[UDP] Broadcasting JSON packets on port {UDP_PORT}...")

    if not can:
        print("[ERR] Cannot run bridge without python-can installed. Exiting.")
        sys.exit(1)

    # Initialize CAN interface
    # Modify interface and channel below depending on your hardware (e.g. 'socketcan', 'vector', 'pcan', 'serial')
    interface = 'virtual'
    channel = 'vcan0'
    
    print(f"[CAN] Attempting connection on '{interface}' interface, channel '{channel}'...")
    try:
        bus = can.interface.Bus(bustype=interface, channel=channel, bitrate=1000000)
        print("[CAN] Connected successfully! Listening for CAN frames...")
    except Exception as e:
        print(f"[ERR] Failed to connect to CAN bus: {str(e)}")
        print("Falling back to a mock loop (generating virtual CAN frames for UDP output)...")
        bus = None

    try:
        while True:
            if bus:
                # Read CAN Frame
                msg = bus.recv(timeout=1.0)
                if msg is None:
                    continue
                can_id = msg.arbitration_id
                data = msg.data
                dlc = msg.dlc
            else:
                # Mock CAN frame generator if no hardware is present
                time.sleep(0.05) # 20Hz
                can_id = 0x010
                data = struct.pack("<ff", 1.25, -2.40) # simulated pitch, roll
                dlc = 8

            # Parse payload
            parsed_data = parse_can_payload(can_id, data)
            
            # Format UDP JSON message
            packet = {
                "type": "telemetry",
                "timestamp": time.time(),
                "can_id_hex": hex(can_id),
                "can_id_dec": can_id,
                "dlc": dlc,
                "data_hex": data.hex(),
                "parsed": parsed_data
            }
            
            # Serialize to JSON and send via UDP
            json_str = json.dumps(packet)
            sock.sendto(json_str.encode('utf-8'), (UDP_IP, UDP_PORT))
            
            # Print status to local console
            print(f"Forwarded CAN ID {hex(can_id)} -> {json_str}")

    except KeyboardInterrupt:
        print("\nBridge terminated by user. Exiting.")
    finally:
        if bus:
            bus.shutdown()
        sock.close()

if __name__ == "__main__":
    main()
