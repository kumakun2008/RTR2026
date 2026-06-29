# RTR2026 Telemetry & OTA Android App

This is a premium, high-performance telemetry dashboard application built in Kotlin. It connects to the RTR2026 Glider avionics board via Bluetooth SPP (Classic), receives real-time telemetry, plots an altitude graph, and supports Dynamic Calibration and OTA Firmware updates.

## Features
- **Real-time Bluetooth SPP Telemetry** ($TEL packet parsing: Battery, Static Pressure, LiDAR Altitude, GPS Satellite count)
- **Live Bezier-Curved Graph** with animated gradient fill plotting altitude in real-time
- **Dynamic Zero Calibration Trigger** (`>CMD:CALIB_ZERO`)
- **Over-The-Air (OTA) Mode Activation Trigger** (`>CMD:OTA_START`)
- **Interactive Log Console** displaying raw RX/TX packets and connection details

## How to Import & Build in Android Studio
1. Open **Android Studio**.
2. Select **File > New > Import Project...**
3. Navigate to and select this `AndroidTelemetry` folder.
4. Android Studio will automatically resolve Gradle dependencies and sync the project.
5. Click **Run** to install the app on your Android device.

## OTA Wireless Firmware Upload Workflow
1. Pair your Android phone with the Glider Main board Bluetooth (Device name: `RTR_Main_Avionics`).
2. Open this App, tap **Connect**, and select the paired device.
3. Once connected, telemetry data will begin streaming and plotting.
4. Tap **Start OTA Update** on the app.
5. The Main board will immediately disconnect Bluetooth, start a Wi-Fi Access Point with SSID: `RTR_Glider_OTA_AP` and password: `rtr2026glider`, and broadcast a CAN command telling all other ESP32 nodes to start their OTA receivers.
6. Connect your laptop (running PlatformIO / VSCode) to the `RTR_Glider_OTA_AP` Wi-Fi network.
7. Open PlatformIO and you will see the network ports corresponding to all ESP32 nodes. Select the target node and click **Upload** to flash the new firmware wirelessly!
8. When the upload finishes (or times out after 5 minutes), the nodes will automatically reboot back to low-power flight mode with Wi-Fi disabled to conserve battery.
