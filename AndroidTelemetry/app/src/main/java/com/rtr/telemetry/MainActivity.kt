package com.rtr.telemetry

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.ScrollView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.model.BitmapDescriptorFactory
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.maps.model.Marker
import com.google.android.gms.maps.model.MarkerOptions
import com.rtr.telemetry.databinding.ActivityMainBinding
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.Locale
import java.util.UUID

@android.annotation.SuppressLint("InlinedApi")
class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothSocket: BluetoothSocket? = null
    private var outputStream: OutputStream? = null
    private var inputStream: InputStream? = null
    private var isConnected = false
    private var receiveThread: Thread? = null

    // Google Maps
    private var googleMap: GoogleMap? = null
    private var gliderMarker: Marker? = null

    // CAN node communication timestamps (from Main Board's millis() reference)
    private var nodeMainTs = 0L
    private var nodePitotTs = 0L
    private var nodeRudderTs = 0L
    private var nodeGPSTs = 0L
    private var nodeAltTs = 0L
    private var nodeBridgeTs = 0L

    private val mainHandler = Handler(Looper.getMainLooper())
    private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

    // Dynamic Permission Requests
    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val connectGranted = permissions[Manifest.permission.BLUETOOTH_CONNECT] ?: false
        val scanGranted = permissions[Manifest.permission.BLUETOOTH_SCAN] ?: false
        if (connectGranted && scanGranted) {
            logConsole("Permissions granted. Tap Connect.")
        } else {
            logConsole("Bluetooth permissions denied. Please enable in settings.")
        }
    }

    private val statusRunnable = object : Runnable {
        override fun run() {
            updateNodeStatusIndicators()
            mainHandler.postDelayed(this, 500)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Initialize MapView
        binding.mapView.onCreate(savedInstanceState)
        binding.mapView.getMapAsync { map ->
            googleMap = map
            map.mapType = GoogleMap.MAP_TYPE_HYBRID
            // Set default view to Lake Biwa / Matsubara Beach coordinates
            val defaultLoc = LatLng(35.267, 136.244)
            map.moveCamera(CameraUpdateFactory.newLatLngZoom(defaultLoc, 15f))
        }

        // Get Bluetooth Adapter using modern BluetoothManager to fix deprecation warning
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter

        if (bluetoothAdapter == null) {
            logConsole("Device doesn't support Bluetooth.")
            binding.btnConnect.isEnabled = false
            return
        }

        checkPermissions()

        binding.btnConnect.setOnClickListener {
            if (isConnected) {
                disconnectBluetooth()
            } else {
                showDeviceSelectionDialog()
            }
        }

        binding.btnCalibrate.setOnClickListener {
            sendCommand(">CMD:CALIB_ZERO\n")
        }

        binding.btnStartOTA.setOnClickListener {
            AlertDialog.Builder(this)
                .setTitle("Enter OTA Mode")
                .setMessage("Are you sure you want to trigger OTA? WiFi will start, and Bluetooth will disconnect.")
                .setPositiveButton("Yes") { _, _ ->
                    sendCommand(">CMD:OTA_START\n")
                }
                .setNegativeButton("No", null)
                .show()
        }

        // Start periodic status updates
        mainHandler.post(statusRunnable)
    }

    private fun checkPermissions() {
        val requiredPermissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }

        val missing = requiredPermissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (missing.isNotEmpty()) {
            requestPermissionLauncher.launch(missing.toTypedArray())
        }
    }

    private fun showDeviceSelectionDialog() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) 
            == PackageManager.PERMISSION_DENIED && Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            checkPermissions()
            return
        }

        val pairedDevices: Set<BluetoothDevice>? = bluetoothAdapter?.bondedDevices
        val deviceList = ArrayList<BluetoothDevice>()
        val deviceNames = ArrayList<String>()

        pairedDevices?.forEach { device ->
            deviceList.add(device)
            deviceNames.add("${device.name}\n${device.address}")
        }

        if (deviceNames.isEmpty()) {
            Toast.makeText(this, "No paired devices found. Pair RTR_Main_Avionics first.", Toast.LENGTH_LONG).show()
            return
        }

        AlertDialog.Builder(this)
            .setTitle("Select Bluetooth Device")
            .setItems(deviceNames.toTypedArray()) { _, which ->
                connectToDevice(deviceList[which])
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun connectToDevice(device: BluetoothDevice) {
        logConsole("Connecting to ${device.name}...")
        binding.btnConnect.text = "CONNECTING..."
        binding.btnConnect.isEnabled = false

        Thread {
            try {
                if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) 
                    == PackageManager.PERMISSION_DENIED && Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                    runOnUiThread {
                        logConsole("Error: Permission denied.")
                        resetConnectButton()
                    }
                    return@Thread
                }

                bluetoothSocket = device.createRfcommSocketToServiceRecord(SPP_UUID)
                bluetoothSocket?.connect()
                outputStream = bluetoothSocket?.outputStream
                inputStream = bluetoothSocket?.inputStream
                isConnected = true

                runOnUiThread {
                    logConsole("Connected successfully!")
                    binding.btnConnect.text = "DISCONNECT"
                    binding.btnConnect.isEnabled = true
                    binding.btnConnect.backgroundTintList = ContextCompat.getColorStateList(this, android.R.color.holo_red_dark)
                }

                startReading()

            } catch (e: IOException) {
                Log.e("MainActivity", "Bluetooth connection failed", e)
                runOnUiThread {
                    logConsole("Connection failed: ${e.message}")
                    resetConnectButton()
                }
            }
        }.start()
    }

    private fun resetConnectButton() {
        binding.btnConnect.text = "CONNECT"
        binding.btnConnect.isEnabled = true
        binding.btnConnect.backgroundTintList = ContextCompat.getColorStateList(this, R.color.accent_blue)
    }

    private fun startReading() {
        receiveThread = Thread {
            val buffer = ByteArray(1024)
            var bytesRead: Int
            val builder = StringBuilder()

            while (isConnected) {
                try {
                    bytesRead = inputStream?.read(buffer) ?: -1
                    if (bytesRead > 0) {
                        val str = String(buffer, 0, bytesRead)
                        builder.append(str)

                        var index: Int
                        while (builder.indexOf("\n").also { index = it } >= 0) {
                            val line = builder.substring(0, index).trim()
                            builder.delete(0, index + 1)
                            if (line.isNotEmpty()) {
                                handleReceivedLine(line)
                            }
                        }
                    }
                } catch (e: IOException) {
                    if (isConnected) {
                        runOnUiThread {
                            logConsole("Connection lost.")
                            disconnectBluetooth()
                        }
                    }
                    break
                }
            }
        }
        receiveThread?.start()
    }

    private fun handleReceivedLine(line: String) {
        runOnUiThread {
            logConsole("RX: $line")

            // Format check: $TEL,battery,pressure,altitude,gpsSats,airspeed,pitch,roll,heading,rudder,lat,lon,gpsAlt,lastRxMain,lastRxPitot,lastRxRudder,lastRxGPS,lastRxAlt,lastRxBridge*
            if (line.startsWith("\$TEL,") && line.endsWith("*")) {
                try {
                    val cleanLine = line.substring(5, line.length - 1)
                    val tokens = cleanLine.split(",")
                    if (tokens.size >= 18) {
                        val bat = tokens[0].toFloatOrNull() ?: 0f
                        val press = tokens[1].toFloatOrNull() ?: 0f
                        val alt = tokens[2].toFloatOrNull() ?: 0f
                        val sats = tokens[3].toIntOrNull() ?: 0
                        val airspeed = tokens[4].toFloatOrNull() ?: 0f
                        val pitch = tokens[5].toFloatOrNull() ?: 0f
                        val roll = tokens[6].toFloatOrNull() ?: 0f
                        val heading = tokens[7].toIntOrNull() ?: 0
                        val lat = tokens[9].toDoubleOrNull() ?: 0.0
                        val lon = tokens[10].toDoubleOrNull() ?: 0.0
                        
                        // Node communication timestamps
                        nodeMainTs = tokens[12].toLongOrNull() ?: 0L
                        nodePitotTs = tokens[13].toLongOrNull() ?: 0L
                        nodeRudderTs = tokens[14].toLongOrNull() ?: 0L
                        nodeGPSTs = tokens[15].toLongOrNull() ?: 0L
                        nodeAltTs = tokens[16].toLongOrNull() ?: 0L
                        nodeBridgeTs = tokens[17].toLongOrNull() ?: 0L

                        // Update top telemetry strip using Locale.US to avoid format bugs
                        binding.tvBattery.text = String.format(Locale.US, "BAT: %.2fV", bat)
                        binding.tvPressure.text = String.format(Locale.US, "BARO: %.1fhPa", press)
                        binding.tvAltitude.text = String.format(Locale.US, "LALT: %.2fm", alt)
                        binding.tvGPSSats.text = String.format(Locale.US, "GPS: %d Sat", sats)

                        if (bat < 7.0f) {
                            binding.tvBattery.setTextColor(ContextCompat.getColor(this, android.R.color.holo_red_light))
                        } else {
                            binding.tvBattery.setTextColor(ContextCompat.getColor(this, android.R.color.holo_green_light))
                        }

                        // Update dials
                        binding.asiView.airspeed = airspeed
                        binding.attView.pitch = pitch
                        binding.attView.roll = roll
                        binding.altView.altitude = alt
                        binding.hdgView.heading = heading.toFloat()

                        // Update GPS map
                        if (lat != 0.0 && lon != 0.0) {
                            val pos = LatLng(lat, lon)
                            if (gliderMarker == null) {
                                gliderMarker = googleMap?.addMarker(
                                    MarkerOptions()
                                        .position(pos)
                                        .title("Glider Position")
                                        .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_RED))
                                )
                                googleMap?.moveCamera(CameraUpdateFactory.newLatLngZoom(pos, 16f))
                            } else {
                                gliderMarker?.position = pos
                            }
                        }
                    }
                } catch (e: Exception) {
                    Log.e("MainActivity", "Error parsing telemetry line", e)
                }
            }
        }
    }

    private fun updateNodeStatusIndicators() {
        val refTime = nodeMainTs
        
        fun updateIndicator(tv: android.widget.TextView, ts: Long) {
            // Active if updated within 1.5 seconds
            if (refTime > 0 && refTime - ts < 1500) {
                tv.setBackgroundColor(Color.parseColor("#10B981")) // Green (OK)
            } else {
                tv.setBackgroundColor(Color.parseColor("#EF4444")) // Red (LOST)
            }
        }

        updateIndicator(binding.tvNodeMain, nodeMainTs)
        updateIndicator(binding.tvNodePitot, nodePitotTs)
        updateIndicator(binding.tvNodeRudder, nodeRudderTs)
        updateIndicator(binding.tvNodeGPS, nodeGPSTs)
        updateIndicator(binding.tvNodeAltimeter, nodeAltTs)
        updateIndicator(binding.tvNodeBridge, nodeBridgeTs)
    }

    private fun sendCommand(cmd: String) {
        if (!isConnected) {
            Toast.makeText(this, "Connect to Bluetooth first.", Toast.LENGTH_SHORT).show()
            return
        }
        Thread {
            try {
                outputStream?.write(cmd.toByteArray())
                outputStream?.flush()
                runOnUiThread {
                    logConsole("TX: ${cmd.trim()}")
                }
            } catch (e: IOException) {
                runOnUiThread {
                    logConsole("Failed to send command: ${e.message}")
                }
            }
        }.start()
    }

    private fun disconnectBluetooth() {
        isConnected = false
        try {
            inputStream?.close()
            outputStream?.close()
            bluetoothSocket?.close()
        } catch (e: IOException) {
            e.printStackTrace()
        }
        inputStream = null
        outputStream = null
        bluetoothSocket = null
        
        runOnUiThread {
            logConsole("Disconnected.")
            resetConnectButton()
        }
    }

    private fun logConsole(message: String) {
        binding.tvConsole.append("\n$message")
        binding.scrollConsole.post {
            binding.scrollConsole.fullScroll(ScrollView.FOCUS_DOWN)
        }
    }

    // Google Map Lifecycle Delegation
    override fun onStart() {
        super.onStart()
        binding.mapView.onStart()
    }

    override fun onResume() {
        super.onResume()
        binding.mapView.onResume()
    }

    override fun onPause() {
        super.onPause()
        binding.mapView.onPause()
    }

    override fun onStop() {
        super.onStop()
        binding.mapView.onStop()
    }

    override fun onDestroy() {
        super.onDestroy()
        binding.mapView.onDestroy()
        disconnectBluetooth()
        mainHandler.removeCallbacks(statusRunnable)
    }

    override fun onSaveInstanceState(outState: Bundle) {
        super.onSaveInstanceState(outState)
        binding.mapView.onSaveInstanceState(outState)
    }

    override fun onLowMemory() {
        super.onLowMemory()
        binding.mapView.onLowMemory()
    }
}
