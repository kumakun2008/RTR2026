/**
 * @file battery.hpp
 * @brief Battery voltage monitoring utility using ESP32 ADC.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <Arduino.h>

/**
 * @class BatteryMonitor
 * @brief Handles ADC reading and conversion to actual battery bus voltage.
 */
class BatteryMonitor {
public:
    /**
     * @brief Construct a new Battery Monitor object.
     * @param pin ESP32 analog input pin (e.g., GPIO 36 / ADC1_CH0).
     * @param r1 Divider resistor connected to Battery V+ (Ohms, default 100k).
     * @param r2 Divider resistor connected to GND (Ohms, default 10k).
     * @param vref Real ADC reference voltage (V, default 3.3V).
     */
    BatteryMonitor(int pin, float r1 = 100000.0f, float r2 = 10000.0f, float vref = 3.3f)
        : _pin(pin), _vref(vref) {
        // Divider ratio = (R1 + R2) / R2
        _dividerRatio = (r1 + r2) / r2;
    }

    /**
     * @brief Initialize the ADC pin.
     */
    void begin() {
        pinMode(_pin, INPUT);
        analogSetPinAttenuation(_pin, ADC_11db); // Configure for ~3.3V max input voltage
    }

    /**
     * @brief Measure the battery bus voltage.
     * @return Measured bus voltage in Volts.
     */
    float readVoltage() {
        int rawAdc = analogRead(_pin);
        // Convert to voltage at ADC pin (ESP32 12-bit ADC -> 0 to 4095)
        float vPin = ((float)rawAdc / 4095.0f) * _vref;
        // Scale back to battery bus voltage
        return vPin * _dividerRatio;
    }

private:
    int _pin;
    float _dividerRatio;
    float _vref;
};

#endif // BATTERY_HPP
