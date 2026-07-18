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
    private var rxMainTs = 0L
    private var rxPitotTs = 0L
    private var rxRudderTs = 0L
    private var rxGPSTs = 0L
    private var rxAltTs = 0L
    private var rxBridgeTs = 0L
    private var rxElevatorTs = 0L

    private var hbMainTs = 0L
    private var hbPitotTs = 0L
    private var hbRudderTs = 0L
    private var hbGPSTs = 0L
    private var hbAltTs = 0L
    private var hbElevatorTs = 0L
    private var hbSpeakerTs = 0L

    private var currentMcuTime = 0L

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

            // Format check: $TEL2,battery,pressure,... or old $TEL,...
            if ((line.startsWith("\$TEL2,") || line.startsWith("\$TEL,")) && line.endsWith("*")) {
                try {
                    val isNewFormat = line.startsWith("\$TEL2,")
                    val prefixLen = if (isNewFormat) 6 else 5
                    val cleanLine = line.substring(prefixLen, line.length - 1)
                    val tokens = cleanLine.split(",")

                    var bat = 0f
                    var press = 0f
                    var alt = 0f
                    var sats = 0
                    var airspeed = 0f
                    var pitch = 0f
                    var roll = 0f
                    var heading = 0
                    var lat = 0.0
                    var lon = 0.0

                    if (isNewFormat && tokens.size >= 31) {
                        bat = tokens[0].toFloatOrNull() ?: 0f
                        press = tokens[1].toFloatOrNull() ?: 0f
                        alt = tokens[2].toFloatOrNull() ?: 0f
                        sats = tokens[3].toIntOrNull() ?: 0
                        airspeed = tokens[4].toFloatOrNull() ?: 0f
                        pitch = tokens[5].toFloatOrNull() ?: 0f
                        roll = tokens[6].toFloatOrNull() ?: 0f
                        heading = tokens[7].toIntOrNull() ?: 0
                        lat = tokens[9].toDoubleOrNull() ?: 0.0
                        lon = tokens[10].toDoubleOrNull() ?: 0.0
                        
                        // Node sensor data rx timestamps
                        rxMainTs = tokens[14].toLongOrNull() ?: 0L
                        rxPitotTs = tokens[15].toLongOrNull() ?: 0L
                        rxRudderTs = tokens[16].toLongOrNull() ?: 0L
                        rxGPSTs = tokens[17].toLongOrNull() ?: 0L
                        rxAltTs = tokens[18].toLongOrNull() ?: 0L
                        rxBridgeTs = tokens[19].toLongOrNull() ?: 0L
                        rxElevatorTs = tokens[21].toLongOrNull() ?: 0L

                        // Node heartbeat rx timestamps
                        hbMainTs = tokens[22].toLongOrNull() ?: 0L
                        hbPitotTs = tokens[23].toLongOrNull() ?: 0L
                        hbRudderTs = tokens[24].toLongOrNull() ?: 0L
                        hbGPSTs = tokens[25].toLongOrNull() ?: 0L
                        hbAltTs = tokens[26].toLongOrNull() ?: 0L
                        hbElevatorTs = tokens[28].toLongOrNull() ?: 0L
                        hbSpeakerTs = tokens[29].toLongOrNull() ?: 0L

                        currentMcuTime = tokens[30].toLongOrNull() ?: 0L
                    } else if (tokens.size >= 18) {
                        // Fallback to old format
                        bat = tokens[0].toFloatOrNull() ?: 0f
                        press = tokens[1].toFloatOrNull() ?: 0f
                        alt = tokens[2].toFloatOrNull() ?: 0f
                        sats = tokens[3].toIntOrNull() ?: 0
                        airspeed = tokens[4].toFloatOrNull() ?: 0f
                        pitch = tokens[5].toFloatOrNull() ?: 0f
                        roll = tokens[6].toFloatOrNull() ?: 0f
                        heading = tokens[7].toIntOrNull() ?: 0
                        lat = tokens[9].toDoubleOrNull() ?: 0.0
                        lon = tokens[10].toDoubleOrNull() ?: 0.0
                        
                        rxMainTs = tokens[12].toLongOrNull() ?: 0L
                        rxPitotTs = tokens[13].toLongOrNull() ?: 0L
                        rxRudderTs = tokens[14].toLongOrNull() ?: 0L
                        rxGPSTs = tokens[15].toLongOrNull() ?: 0L
                        rxAltTs = tokens[16].toLongOrNull() ?: 0L
                        rxBridgeTs = tokens[17].toLongOrNull() ?: 0L
                        
                        // Treat old RX timestamps also as HB timestamps in fallback
                        hbMainTs = rxMainTs
                        hbPitotTs = rxPitotTs
                        hbRudderTs = rxRudderTs
                        hbGPSTs = rxGPSTs
                        hbAltTs = rxAltTs
                        hbSpeakerTs = rxBridgeTs
                        currentMcuTime = rxMainTs
                    }

                    // Update top telemetry strip using Locale.US to avoid format bugs
                    binding.tvBattery.text = String.format(Locale.US, "BAT: %.2fV", bat)
                    binding.tvPressure.text = String.format(Locale.US, "BARO: %.1fhPa", press)
                    binding.tvAltitude.text = String.format(Locale.US, "LALT: %.2fm", alt)
                    binding.tvGPSSats.text = String.format(Locale.US, "GPS: %d Sat", sats)

                    if (bat < 7.0f) {
                        binding.tvBattery.setTextColor(ContextCompat.getColor(this@MainActivity, android.R.color.holo_red_light))
                    } else {
                        binding.tvBattery.setTextColor(ContextCompat.getColor(this@MainActivity, android.R.color.holo_green_light))
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
                } catch (e: Exception) {
                    Log.e("MainActivity", "Error parsing telemetry line", e)
                }
            }
        }
    }

    private fun updateNodeStatusIndicators() {
        val refTime = currentMcuTime
        
        fun updateIndicator(tv: android.widget.TextView, hbTs: Long, rxTs: Long) {
            val hbActive = (refTime > 0 && refTime - hbTs < 3000)
            val rxActive = (refTime > 0 && refTime - rxTs < 3000)

            if (rxActive) {
                tv.setBackgroundColor(Color.parseColor("#10B981")) // Green (OK)
            } else if (hbActive) {
                tv.setBackgroundColor(Color.parseColor("#F59E0B")) // Orange (CONNECTED)
            } else {
                tv.setBackgroundColor(Color.parseColor("#EF4444")) // Red (LOST)
            }
        }

        updateIndicator(binding.tvNodeMain,      hbMainTs,     rxMainTs)
        updateIndicator(binding.tvNodePitot,     hbPitotTs,    rxPitotTs)
        updateIndicator(binding.tvNodeRudder,    hbRudderTs,   rxRudderTs)
        updateIndicator(binding.tvNodeGPS,       hbGPSTs,      rxGPSTs)
        updateIndicator(binding.tvNodeAltimeter, hbAltTs,      rxAltTs)
        updateIndicator(binding.tvNodeBridge,    hbSpeakerTs,  rxBridgeTs)
        updateIndicator(binding.tvNodeElevator,  hbElevatorTs, rxElevatorTs)
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
