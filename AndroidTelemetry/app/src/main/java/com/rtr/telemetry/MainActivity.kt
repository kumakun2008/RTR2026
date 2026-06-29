package com.rtr.telemetry

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.ScrollView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.rtr.telemetry.databinding.ActivityMainBinding
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.UUID

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothSocket: BluetoothSocket? = null
    private var outputStream: OutputStream? = null
    private var inputStream: InputStream? = null
    private var isConnected = false
    private var receiveThread: Thread? = null

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

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()

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
        binding.btnConnect.text = "Connecting..."
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
                    binding.btnConnect.text = "Disconnect"
                    binding.btnConnect.isEnabled = true
                    binding.btnConnect.backgroundTintList = ContextCompat.getColorStateList(this, R.color.accent_red)
                }

                startReading()

            } catch (e: IOException) {
                e.printStackTrace()
                runOnUiThread {
                    logConsole("Connection failed: ${e.message}")
                    resetConnectButton()
                }
            }
        }.start()
    }

    private fun resetConnectButton() {
        binding.btnConnect.text = "Connect"
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

                        // Parse complete lines
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

            // Format check: $TEL,battery,pressure,altitude,gpsSats*
            if (line.startsWith("$TEL,") && line.endsWith("*")) {
                try {
                    val cleanLine = line.substring(5, line.length - 1)
                    val tokens = cleanLine.split(",")
                    if (tokens.size >= 4) {
                        val bat = tokens[0].toFloatOrNull() ?: 0.0f
                        val press = tokens[1].toFloatOrNull() ?: 0.0f
                        val alt = tokens[2].toFloatOrNull() ?: 0.0f
                        val sats = tokens[3].toIntOrNull() ?: 0

                        binding.tvBattery.text = String.format("%.2f V", bat)
                        binding.tvPressure.text = String.format("%.1f hPa", press)
                        binding.tvAltitude.text = String.format("%.2f m", alt)
                        binding.tvGPSSats.text = "$sats Sats"

                        if (bat < 7.0f) {
                            binding.tvBattery.setTextColor(ContextCompat.getColor(this, R.color.accent_red))
                        } else {
                            binding.tvBattery.setTextColor(ContextCompat.getColor(this, R.color.accent_green))
                        }

                        // Plot to chart
                        binding.lineChartView.addDataPoint(alt)
                    }
                } catch (e: Exception) {
                    e.printStackTrace()
                }
            }
        }
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

    override fun onDestroy() {
        super.onDestroy()
        disconnectBluetooth()
    }
}
