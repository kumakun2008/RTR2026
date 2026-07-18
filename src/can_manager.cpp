/**
 * @file can_manager.cpp
 * @brief TWAI (CAN) bus driver manager implementation with cross-platform support.
 * @author Team ЯTR
 * @date 2026-07-16
 */

#include "can_manager.hpp"

#ifdef ESP32

CANManager::CANManager() : _initialized(false) {}

CANManager::~CANManager() {
    end();
}

bool CANManager::begin(int txPin, int rxPin, int stbPin) {
    if (_initialized) return true;

    // NOTE: STB pin (MCP2561 / BD41041FJ-CE2) is hardware-grounded on this PCB.
    // No software control needed. The transceiver is always in Normal mode.
    // (stbPin parameter is accepted but ignored when -1)
    (void)stbPin;
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)txPin, 
        (gpio_num_t)rxPin, 
        TWAI_MODE_NORMAL
    );
    // Larger queues to handle bursts from all nodes simultaneously
    g_config.tx_queue_len = 15;
    g_config.rx_queue_len = 30;
    
    // 1Mbps - confirmed working baud rate (ref: message.txt altimeter test code)
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    
    _initialized = true;
    return true;
}

void CANManager::end() {
    if (_initialized) {
        twai_stop();
        twai_driver_uninstall();
        _initialized = false;
    }
}

bool CANManager::transmitRaw(uint32_t id, const uint8_t* data, uint8_t dlc) {
    if (!_initialized) return false;
    handleAutoRecovery();

    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        if (status.state != TWAI_STATE_RUNNING) {
            return false;
        }
    }
    
    twai_message_t msg = {};
    msg.identifier = id;
    msg.extd = 0; 
    msg.rtr = 0;  
    msg.data_length_code = dlc > 8 ? 8 : dlc;
    
    if (dlc > 0 && data != NULL) {
        for (int i = 0; i < msg.data_length_code; i++) {
            msg.data[i] = data[i];
        }
    }
    
    return (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK);
}

bool CANManager::receiveRaw(uint32_t& id, uint8_t* data, uint8_t& dlc, uint32_t timeoutMs) {
    if (!_initialized) return false;
    handleAutoRecovery();

    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        if (status.state != TWAI_STATE_RUNNING) {
            return false;
        }
    }
    
    twai_message_t msg = {};
    if (twai_receive(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK) {
        id = msg.identifier;
        dlc = msg.data_length_code;
        for (int i = 0; i < dlc; i++) {
            data[i] = msg.data[i];
        }
        return true;
    }
    return false;
}

void CANManager::printStatus() {
    if (!_initialized) return;
    twai_status_info_t status_info;
    if (twai_get_status_info(&status_info) == ESP_OK) {
        Serial.printf("[CAN Status] State: %d, TX Err: %u, RX Err: %u, Tx Fail: %u, Rx Miss: %u\n",
                      status_info.state,
                      status_info.tx_error_counter,
                      status_info.rx_error_counter,
                      status_info.tx_failed_count,
                      status_info.rx_missed_count);
        if (status_info.state == TWAI_STATE_BUS_OFF) {
            Serial.println("[CAN Warning] Bus-Off detected! Attempting recovery...");
            twai_initiate_recovery();
        } else if (status_info.state == TWAI_STATE_STOPPED) {
            Serial.println("[CAN Warning] TWAI Stopped! Restarting...");
            twai_start();
        }
        // Error Passive threshold warning (EC > 127 = Error Passive per ISO 11898-1)
        if (status_info.tx_error_counter > 96 || status_info.rx_error_counter > 96) {
            Serial.printf("[CAN Warning] Approaching Error Passive state (TEC=%u, REC=%u). "
                          "Check wiring, termination, and STB pin.\n",
                          status_info.tx_error_counter, status_info.rx_error_counter);
        }
        if (status_info.rx_missed_count > 0) {
            Serial.printf("[CAN Warning] %u RX frames missed - consider increasing rx_queue_len.\n",
                          status_info.rx_missed_count);
        }
    }
}

void CANManager::handleAutoRecovery() {
    if (!_initialized) return;
    uint32_t now = millis();
    if (now - _lastRecoveryAttemptMs < 200) return; // Shortened from 2000ms to 200ms for faster recovery
    _lastRecoveryAttemptMs = now;

    twai_status_info_t status_info;
    if (twai_get_status_info(&status_info) == ESP_OK) {
        if (status_info.state == TWAI_STATE_BUS_OFF) {
            Serial.println("[CAN Warning] Bus-Off detected via auto-recovery! Recovering...");
            twai_initiate_recovery();
        } else if (status_info.state == TWAI_STATE_STOPPED) {
            Serial.println("[CAN Warning] Stopped detected via auto-recovery! Restarting...");
            twai_start();
        }
    }
}

#elif defined(ARDUINO_ARCH_STM32)

// Note: #include <stm32f3xx_hal.h> is now in can_manager.hpp

static CAN_HandleTypeDef hcan1;

CANManager::CANManager() : _initialized(false) {}

CANManager::~CANManager() {
    end();
}

bool CANManager::begin(int txPin, int rxPin, int stbPin) {
    if (_initialized) return true;

    // NOTE: BD41041FJ-CE2 STB is hardware-grounded on this PCB.
    // Transceiver always in Normal mode. stbPin parameter accepted but ignored.
    (void)stbPin;

    // Enable CAN clock
    __HAL_RCC_CAN1_CLK_ENABLE();
    
    // Enable GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure PA11 (CAN_RX) and PA12 (CAN_TX)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN; // AF9 is CAN on STM32F303
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize CAN settings (1 Mbps) - matches ESP32 nodes and confirmed working altimeter code
    // APB1 clock is usually 36 MHz on F303 at 72MHz.
    // For 1Mbps: Prescaler = 2, tq = Prescaler / APB1 = 2 / 36MHz = 1/18 us.
    // TimeSeg1 = 12, TimeSeg2 = 5, SyncJumpWidth = 1.
    // Total tq = 1 + 12 + 5 = 18 tq.
    // Time for 1 bit = 18 tq = 18 * (1/18 us) = 1 us.
    // Bitrate = 1 / 1 us = 1 Mbps.
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 2;  // Changed from 4 (500kbps) to 2 (1Mbps)
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        return false;
    }

    // Configure filter: accept all messages
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        return false;
    }

    // Start CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return false;
    }

    _initialized = true;
    return true;
}

void CANManager::end() {
    if (_initialized) {
        HAL_CAN_Stop(&hcan1);
        _initialized = false;
    }
}

bool CANManager::transmitRaw(uint32_t id, const uint8_t* data, uint8_t dlc) {
    if (!_initialized) return false;

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = id;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = dlc > 8 ? 8 : dlc;
    TxHeader.TransmitGlobalTime = DISABLE;

    uint32_t TxMailbox;
    uint8_t txData[8] = {0};
    if (data && dlc > 0) {
        memcpy(txData, data, TxHeader.DLC);
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &TxMailbox) != HAL_OK) {
        return false;
    }

    // Wait for transmission with timeout
    uint32_t start = millis();
    while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
        if (millis() - start > 10) {
            return false;
        }
    }
    return true;
}

bool CANManager::receiveRaw(uint32_t& id, uint8_t* data, uint8_t& dlc, uint32_t timeoutMs) {
    if (!_initialized) return false;

    uint32_t start = millis();
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0) {
        if (millis() - start > timeoutMs) {
            return false;
        }
    }

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rxData[8] = {0};
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rxData) != HAL_OK) {
        return false;
    }

    id = RxHeader.StdId;
    dlc = RxHeader.DLC;
    if (data) {
        memcpy(data, rxData, dlc);
    }
    return true;
}

void CANManager::printStatus() {
    if (!_initialized) return;
    uint32_t err = HAL_CAN_GetError(&hcan1);
    Serial.printf("[CAN Status STM32] Error Code: 0x%08lX\n", err);
}

#else // Non-ESP32, Non-STM32 Mock implementations

CANManager::CANManager() : _initialized(false) {}
CANManager::~CANManager() {}
bool CANManager::begin(int txPin, int rxPin, int stbPin) { (void)txPin; (void)rxPin; (void)stbPin; _initialized = true; return true; }
void CANManager::end() { _initialized = false; }
bool CANManager::transmitRaw(uint32_t id, const uint8_t* data, uint8_t dlc) { (void)id; (void)data; (void)dlc; return true; }
bool CANManager::receiveRaw(uint32_t& id, uint8_t* data, uint8_t& dlc, uint32_t timeoutMs) { (void)id; (void)data; (void)dlc; (void)timeoutMs; return false; }
void CANManager::printStatus() {}

#endif // ESP32 / ARDUINO_ARCH_STM32

// --- Shared Telemetry Transmission Helpers (Used by all platforms) ---

bool CANManager::transmitScaled(uint32_t id, float value, float scale) {
    int32_t val = (int32_t)(value * scale);
    uint8_t data[4];
    memcpy(data, &val, 4);
    return transmitRaw(id, data, 4);
}

bool CANManager::transmitInt32(uint32_t id, int32_t value) {
    uint8_t data[4];
    memcpy(data, &value, 4);
    return transmitRaw(id, data, 4);
}

bool CANManager::transmitDoubleSplit(uint32_t idUpper, uint32_t idLower, double value) {
    uint64_t binVal;
    memcpy(&binVal, &value, 8);
    uint32_t upper = (uint32_t)(binVal >> 32);
    uint32_t lower = (uint32_t)(binVal & 0xFFFFFFFF);
    
    uint8_t dataUpper[4];
    uint8_t dataLower[4];
    memcpy(dataUpper, &upper, 4);
    memcpy(dataLower, &lower, 4);
    
    bool okUpper = transmitRaw(idUpper, dataUpper, 4);
    bool okLower = transmitRaw(idLower, dataLower, 4);
    return okUpper && okLower;
}

bool CANManager::transmitVoiceCmd(uint8_t alertCode) {
    return transmitRaw(CAN_ID_VOICE_CMD, &alertCode, 1);
}

bool CANManager::transmitCalibZero() {
    return transmitRaw(CAN_ID_CALIB_ZERO, NULL, 0);
}

bool CANManager::transmitOtaStart() {
    return transmitRaw(CAN_ID_OTA_START, NULL, 0);
}
