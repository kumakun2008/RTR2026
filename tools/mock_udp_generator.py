#!/usr/bin/env python3
"""
RTR2026 Virtual Telemetry UDP Generator.
Generates realistic flight data (sinusoidal attitude oscillations, climbing
altitudes, landing sequences, dropping battery, and moving GPS path) 
and broadcasts it as JSON telemetry packets over UDP port 5005.

Useful for testing ground stations and dashboards without hardware.

Author: Team ЯTR
Date: 2026-06-25
"""

import socket
import json
import time
import math
import random
import struct

# Target UDP configurations
UDP_IP = "255.255.255.255"  # Broadcast
UDP_PORT = 5005

def main():
    print("=== RTR2026 Virtual UDP Telemetry Generator ===")
    
    # Initialize UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"[UDP] Broadcasting simulated data on {UDP_IP}:{UDP_PORT}...")
    print("Press Ctrl+C to stop.\n")

    # Simulation state variables
    start_time = time.time()
    lat_start = 35.3126  # Lake Biwa Birdman site approx latitude
    lon_start = 136.2140 # Lake Biwa Birdman site approx longitude
    battery_volt = 8.4   # Starts fully charged (2S LiPo max)
    
    try:
        while True:
            current_time = time.time()
            elapsed = current_time - start_time
            
            # 1. Simulate flight dynamics (using time)
            # Oscillate pitch between -5 and 5 deg, roll between -10 and 10 deg
            pitch = 4.0 * math.sin(elapsed * 0.5) + random.uniform(-0.2, 0.2)
            roll = 8.0 * math.sin(elapsed * 0.3) + random.uniform(-0.3, 0.3)
            
            # Airspeed: accelerate and then fly around 12-15 m/s
            if elapsed < 5.0:
                airspeed = (elapsed / 5.0) * 14.0 # accelerate
            else:
                airspeed = 13.5 + 1.2 * math.sin(elapsed * 0.8) + random.uniform(-0.1, 0.1)
                
            # Pressures
            press_sdp32 = 0.5 * 1.225 * (airspeed ** 2) # Dynamic pressure P = 0.5 * rho * V^2
            press_sdp31_1 = press_sdp32 + random.uniform(-2.0, 2.0)
            press_sdp31_2 = press_sdp32 + random.uniform(-2.0, 2.0)
            
            # Altitude: climb up to 12m, then descend/glide down
            if elapsed < 10.0:
                altitude = (elapsed / 10.0) * 10.0  # initial launch height
            else:
                # slow glide down at 0.15 m/s
                altitude = max(0.0, 10.0 - (elapsed - 10.0) * 0.15 + 0.5 * math.sin(elapsed * 0.2))
                
            # Static pressure decreases with altitude: P = P0 - alt * 12 Pa/m
            static_press = 101325.0 - altitude * 12.0
            
            # GPS position: slow movement north-east
            latitude = lat_start + (elapsed * 0.000002)
            longitude = lon_start + (elapsed * 0.000003)
            
            # Battery voltage: drops slowly over time
            battery_volt = max(6.0, 8.4 - (elapsed * 0.001))
            
            # Rudder angle: sinusoidal movement
            rudder_angle = 15.0 * math.sin(elapsed * 0.7)
            
            # Generate CAN-alike structured packets to broadcast sequentially
            # Message 1: Attitude (0x010)
            p1 = {
                "type": "telemetry",
                "timestamp": current_time,
                "can_id_hex": "0x10",
                "can_id_dec": 16,
                "dlc": 8,
                "data_hex": struct.pack("<ff", pitch, roll).hex(),
                "parsed": {
                    "name": "ATTITUDE",
                    "pitch": round(pitch, 2),
                    "roll": round(roll, 2)
                }
            }
            sock.sendto(json.dumps(p1).encode('utf-8'), (UDP_IP, UDP_PORT))
            
            # Message 2: Airspeed (0x011)
            p2 = {
                "type": "telemetry",
                "timestamp": current_time,
                "can_id_hex": "0x11",
                "can_id_dec": 17,
                "dlc": 8,
                "data_hex": struct.pack("<ff", press_sdp32, airspeed).hex(),
                "parsed": {
                    "name": "AIRSPEED",
                    "sdp32_press": round(press_sdp32, 2),
                    "airspeed": round(airspeed, 2)
                }
            }
            sock.sendto(json.dumps(p2).encode('utf-8'), (UDP_IP, UDP_PORT))
            
            # Message 3: Rudder Angle (0x012)
            p3 = {
                "type": "telemetry",
                "timestamp": current_time,
                "can_id_hex": "0x12",
                "can_id_dec": 18,
                "dlc": 4,
                "data_hex": struct.pack("<f", rudder_angle).hex(),
                "parsed": {
                    "name": "RUDDER_ANGLE",
                    "rudder_angle": round(rudder_angle, 2)
                }
            }
            sock.sendto(json.dumps(p3).encode('utf-8'), (UDP_IP, UDP_PORT))

            # Message 4: Altitude (0x020)
            # Simulating LiDAR + US ranges from altimeter node
            p4 = {
                "type": "telemetry",
                "timestamp": current_time,
                "can_id_hex": "0x20",
                "can_id_dec": 32,
                "dlc": 8,
                "data_hex": struct.pack("<ff", altitude, altitude + random.uniform(-0.05, 0.05)).hex(),
                "parsed": {
                    "name": "ALTITUDE",
                    "lidar_alt": round(altitude, 2),
                    "ultrasonic_alt": round(max(0.0, altitude + random.uniform(-0.05, 0.05)), 2)
                }
            }
            sock.sendto(json.dumps(p4).encode('utf-8'), (UDP_IP, UDP_PORT))
            
            # Message 5: GPS Pos (0x021)
            p5 = {
                "type": "telemetry",
                "timestamp": current_time,
                "can_id_hex": "0x21",
                "can_id_dec": 33,
                "dlc": 8,
                "data_hex": struct.pack("<ii", int(latitude * 1e7), int(longitude * 1e7)).hex(),
                "parsed": {
                    "name": "GPS_POS",
                    "latitude": round(latitude, 6),
                    "longitude": round(longitude, 6)
                }
            }
            sock.sendto(json.dumps(p5).encode('utf-8'), (UDP_IP, UDP_PORT))
            
            # Message 6: Battery Voltage (0x050)
            p6 = {
                "type": "telemetry",
                "timestamp": current_time,
                "can_id_hex": "0x50",
                "can_id_dec": 80,
                "dlc": 4,
                "data_hex": struct.pack("<f", battery_volt).hex(),
                "parsed": {
                    "name": "BATTERY_VOLT",
                    "battery_voltage": round(battery_volt, 2)
                }
            }
            sock.sendto(json.dumps(p6).encode('utf-8'), (UDP_IP, UDP_PORT))

            print(f"[{elapsed:5.1f}s] Sent Telemetry Bundle (Attitude, Airspeed, Rudder, Altitude, GPS, Battery)")
            
            # Sleep 100ms (10Hz transmission cycle)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nGenerator stopped by user. Exiting.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
