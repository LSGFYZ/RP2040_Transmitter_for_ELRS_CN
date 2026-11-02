#pragma once
/*
 * This file is part of ESP32C3-TX (Ported to RP2040 Zero)
 *
 * ESP32C3-TX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ESP32C3-TX is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * <http://www.gnu.org/licenses/>.
 */
 
#include <Arduino.h>
#include <stdint.h>
#include "hardware.h"

//#define CRSFDEBUG

// The UART instance is selected in hardware.h. CRSF protocol code should not
// maintain a second, independent serial-port definition.
#ifndef ELRS_PORT
#define ELRS_PORT ELRS_UART
#endif

// Basic setup
#define CRSF_MAX_CHANNEL        16
#define CRSF_FRAME_SIZE_MAX     64
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

#define CRSF_TIME_NEEDED_PER_FRAME_US   1100
#define SERIAL_BAUDRATE                 921600
#define CRSF_TIME_BETWEEN_FRAMES_US     2000   // 500Hz
#define CRSF_PAYLOAD_OFFSET             offsetof(crsfFrameDef_t, type)
#define CRSF_MAX_PACKET_SIZE            64
#define CRSF_QUEUE_SIZE                 8
#define CRSF_MSP_RX_BUF_SIZE            128
#define CRSF_MSP_TX_BUF_SIZE            128
#define CRSF_PAYLOAD_SIZE_MAX           60
#define CRSF_PACKET_LENGTH              22
#define CRSF_PACKET_SIZE                26
#define CRSF_FRAME_LENGTH               24
#define CRSF_CMD_PACKET_SIZE            8
#define CRSF_TLM_PACKET_SIZE            10
#define CRSF_MAX_PARAMS                 32
#define CRSF_MAX_STRING_LEN             32
#define CRSF_MAX_PARAM_DATA_LEN         256
#define CRSF_SUBTYPE_OPENTX_SYNC        0x10

// ELRS (CRSF) Frame types
#define ELRS_CHANNELS                   0x16
#define ELRS_LINK_STATS                 0x14
#define ELRS_DEVICE_PING                0x28
#define ELRS_DEVICE_INFO                0x29
#define ELRS_SETTINGS_RESPONSE          0x2B
#define ELRS_SETTINGS_READ              0x2C
#define ELRS_HEARTBEAT                  0x2D
#define ELRS_STATUS                     0x2E
#define ELRS_RADIO_ID                   0x3A

// Device addresses
#define HANDSET_ADDRESS                 0xEA
#define MODULE_ADDRESS                  0xEE
#define LUA_SCRIPT_ADDRESS              0xEF
#define BROADCAST_ADDRESS               0x00

// ELRS commands
#define ELRS_PKT_RATE_COMMAND           0x01
#define ELRS_TLM_RATIO_COMMAND          0x02
#define ELRS_SWITCH_MODE_COMMAND        0x03
#define ELRS_MODEL_MATCH_COMMAND        0x05
#define ELRS_POWER_COMMAND              0x07
#define ELRS_DYNAMIC_POWER_COMMAND      0x08
#define ELRS_WIFI_COMMAND               0x10
#define ELRS_RXWIFI_COMMAND             0x11
#define ELRS_BLE_JOYSTIC_COMMAND        0x12
#define ELRS_BIND_COMMAND               0x13
#define ELRS_START_COMMAND              0x04
#define ELRS_END_COMMAND                0x05
#define ELRS_SETTINGS_WRITE             0x2D
#define SYNC_BYTE                       0xC8

// ❌ REMOVED: extern HardwareSerial ELRS_PORT;
// RP2040 Arduino Core already defines Serial0/Serial1/Serial2 as global objects.
// Re-declaring them causes linker errors. Just use ELRS_PORT macro directly.

struct crsfPacket {
    uint8_t length;
    uint8_t data[CRSF_MAX_PACKET_SIZE];
};

class crsfPacketQueue {
private:
    crsfPacket _buffer[CRSF_QUEUE_SIZE];
    volatile uint8_t _head;
    volatile uint8_t _tail;
    static const uint8_t _mask = CRSF_QUEUE_SIZE - 1;

public:
    crsfPacketQueue() : _head(0), _tail(0) {}

    bool hasItems() { return _head != _tail; }

    bool push(const uint8_t* rawData, uint8_t len) {
        if (len > CRSF_MAX_PACKET_SIZE) return false;
        uint8_t nextHead = (_head + 1) & _mask;
        if (nextHead == _tail) return false;
        _buffer[_head].length = len;
        memcpy(_buffer[_head].data, rawData, len);
        _head = nextHead;
        return true;
    }

    bool pop(crsfPacket& outPacket) {
        if (_head == _tail) return false;
        outPacket = _buffer[_tail];
        _tail = (_tail + 1) & _mask;
        return true;
    }

    void clear() {
        // On RP2040, __disable_irq/__enable_irq is preferred over noInterrupts/interrupts
        // But in single-core loop context, both work. Keeping original for compatibility.
        noInterrupts();
        _head = 0;
        _tail = 0;
        interrupts();
    }    
};

struct crsfLinkStats {
    int16_t uplinkRssi1;
    int16_t uplinkRssi2;
    uint8_t uplinkLQ;
    int8_t  uplinkSNR;
    uint8_t activeAntenna;
    uint8_t rfMode;
    uint8_t txPower;
    int16_t downlinkRssi;
    uint8_t downlinkLQ;
    int8_t  downlinkSNR;
};

class CRSF {
public:
    crsfLinkStats linkStats;
    bool moduleConnected = false;
    bool telemetryActive = false;
    uint32_t lastValidFrameTime = 0;

    // Receiver battery telemetry (CRSF frame type 0x08).
    // CRSF voltage is reported in 0.1V units; do not apply handset calibration.
    float receiverBatteryVoltage = 0.0f;
    bool receiverBatteryValid = false;
    uint32_t lastReceiverBatteryFrameTime = 0;
    crsfPacketQueue commandQueue;
    crsfPacketQueue telemetryQueue;

    void begin(uint8_t rxPin, uint8_t txPin, bool halfDuplex);
    void crsfSendChannels(int16_t channels[]);
    void crsfSendCommand(uint8_t command, uint8_t value);
    void crsfQueuePacket(uint8_t packet[], uint8_t packetLength);
    void crsfSendQueuedCommand();
    void crsfReadTelemetry();
    uint32_t crsfNextInterval();

private:
    bool isHalfDuplex = false;
    // Incremental RX parser state. A partial frame is retained between loop
    // calls instead of busy-waiting for the rest of the packet.
    uint8_t rxPacketBuffer[CRSF_MAX_PACKET_SIZE] = {};
    uint8_t rxPosition = 0;
    uint8_t rxExpectedLength = 0;
    uint32_t rxLastByteMs = 0;
    volatile uint32_t targetIntervalUs = CRSF_TIME_BETWEEN_FRAMES_US;
    volatile int32_t currentPhaseShift = 0;
    volatile int32_t averagePhaseShift = 0;
    volatile int32_t cumulativePhaseShift = 0;
    int16_t phaseShiftHistory[256] = {};
    uint8_t syncPacketCount = 0;
    volatile bool syncPacketReceived = false;
    uint32_t lastSyncPacketTime = 0;
    uint32_t lastLinkStatsFrameTime = 0;
    uint32_t lastLinkStatRequestTime = 0;
    uint32_t lastSyncPacketDisplay = 0;

    void crsfWritePacket(uint8_t packet[], uint8_t packetLength);
    void crsfParseLinkStatsPacket(uint8_t rxBuffer[]);
    void crsfParseElrsSyncPacket(uint8_t rxBuffer[]);
};

extern CRSF crsfClass;