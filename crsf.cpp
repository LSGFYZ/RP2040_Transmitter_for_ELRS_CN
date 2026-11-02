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
 
#include "crsf.h"

// crc implementation from CRSF protocol document rev7
static uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++){
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

bool checkCrc(uint8_t *packet, uint8_t length) {
    return (crsf_crc8(&packet[2], packet[1] - 1) == packet[length - 1]);
}

// ============================================================================
// Serial begin - RP2040 Port
// ============================================================================
void CRSF::begin(uint8_t rxPin, uint8_t txPin, bool halfDuplex) {
    isHalfDuplex = halfDuplex;

    // The Arduino-Pico core lets the selected UART object map to supported
    // GPIO pairs with setRX/setTX. Keep the actual pin choice in hardware.h
    // and apply the requested RX and TX pins here; do not duplicate pin maps
    // or make protocol code depend on one particular RP2040 board.

    if (isHalfDuplex) {
        // Half-duplex single-wire CRSF: both directions share txPin.
        // The selected UART must support that GPIO for both RX and TX.
#if defined(SERIAL_UART_INSTANCE) && SERIAL_UART_INSTANCE == 0
        // UART instance 0
        ELRS_PORT.setTX(txPin);
        ELRS_PORT.setRX(txPin);
#elif defined(SERIAL_UART_INSTANCE) && SERIAL_UART_INSTANCE == 1
        // UART instance 1
        ELRS_PORT.setTX(txPin);
        ELRS_PORT.setRX(txPin);
#else
        // Fallback: assume default pins for whichever ELRS_PORT is defined
        ELRS_PORT.setTX(txPin);
        ELRS_PORT.setRX(txPin);
#endif
        ELRS_PORT.begin(SERIAL_BAUDRATE);

        // RP2040 does not have OUTPUT_OPEN_DRAIN natively.
        // For single-wire CRSF, the external ELRS module typically has
        // its own pull-up resistor. We just need to ensure the pin is
        // configured correctly via setTX/setRX above.
        // If your hardware requires explicit open-drain emulation,
        // uncomment below (works but adds ~1us overhead per byte):
        // pinMode(txPin, INPUT_PULLUP);
    } else {
        // Full duplex: configure RX and TX pins explicitly
        ELRS_PORT.setRX(rxPin);
        ELRS_PORT.setTX(txPin);
        ELRS_PORT.begin(SERIAL_BAUDRATE);
    }
}

// Send RC Channels
void CRSF::crsfSendChannels(int16_t channels[]) {
    uint8_t packet[CRSF_PACKET_SIZE];

    packet[0] = MODULE_ADDRESS;
    packet[1] = 24;
    packet[2] = ELRS_CHANNELS;
    packet[3]  = (uint8_t)(channels[0] & 0x07FF);
    packet[4]  = (uint8_t)((channels[0] & 0x07FF) >> 8  | (channels[1] & 0x07FF) << 3);
    packet[5]  = (uint8_t)((channels[1] & 0x07FF) >> 5  | (channels[2] & 0x07FF) << 6);
    packet[6]  = (uint8_t)((channels[2] & 0x07FF) >> 2);
    packet[7]  = (uint8_t)((channels[2] & 0x07FF) >> 10 | (channels[3] & 0x07FF) << 1);
    packet[8]  = (uint8_t)((channels[3] & 0x07FF) >> 7  | (channels[4] & 0x07FF) << 4);
    packet[9]  = (uint8_t)((channels[4] & 0x07FF) >> 4  | (channels[5] & 0x07FF) << 7);
    packet[10] = (uint8_t)((channels[5] & 0x07FF) >> 1);
    packet[11] = (uint8_t)((channels[5] & 0x07FF) >> 9  | (channels[6] & 0x07FF) << 2);
    packet[12] = (uint8_t)((channels[6] & 0x07FF) >> 6  | (channels[7] & 0x07FF) << 5);
    packet[13] = (uint8_t)((channels[7] & 0x07FF) >> 3);
    packet[14] = (uint8_t)((channels[8] & 0x07FF));
    packet[15] = (uint8_t)((channels[8] & 0x07FF) >> 8  | (channels[9] & 0x07FF) << 3);
    packet[16] = (uint8_t)((channels[9] & 0x07FF) >> 5  | (channels[10] & 0x07FF) << 6);
    packet[17] = (uint8_t)((channels[10] & 0x07FF) >> 2);
    packet[18] = (uint8_t)((channels[10] & 0x07FF) >> 10 | (channels[11] & 0x07FF) << 1);
    packet[19] = (uint8_t)((channels[11] & 0x07FF) >> 7  | (channels[12] & 0x07FF) << 4);
    packet[20] = (uint8_t)((channels[12] & 0x07FF) >> 4  | (channels[13] & 0x07FF) << 7);
    packet[21] = (uint8_t)((channels[13] & 0x07FF) >> 1);
    packet[22] = (uint8_t)((channels[13] & 0x07FF) >> 9  | (channels[14] & 0x07FF) << 2);
    packet[23] = (uint8_t)((channels[14] & 0x07FF) >> 6  | (channels[15] & 0x07FF) << 5);
    packet[24] = (uint8_t)((channels[15] & 0x07FF) >> 3);
    packet[25] = crsf_crc8(&packet[2], packet[1] - 1);
    
    crsfWritePacket(packet, CRSF_PACKET_SIZE);
}

void CRSF::crsfSendCommand(uint8_t command, uint8_t value) {
    uint8_t packetCmd[CRSF_CMD_PACKET_SIZE];

    packetCmd[0] = MODULE_ADDRESS;
    packetCmd[1] = 6;
    packetCmd[2] = ELRS_SETTINGS_WRITE;
    packetCmd[3] = MODULE_ADDRESS;
    packetCmd[4] = HANDSET_ADDRESS;
    packetCmd[5] = command;
    packetCmd[6] = value;
    packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1] - 1);

    crsfQueuePacket(packetCmd, CRSF_CMD_PACKET_SIZE);
}

void CRSF::crsfSendQueuedCommand() {
    if (commandQueue.hasItems()) {
        crsfPacket pendingPacket;
        if (commandQueue.pop(pendingPacket)) {
            crsfWritePacket(pendingPacket.data, pendingPacket.length);
        }
    }
}

void CRSF::crsfQueuePacket(uint8_t packet[], uint8_t packetLength) {
    packet[packetLength - 1] = crsf_crc8(&packet[2], packetLength - 3); 
    if (!commandQueue.push(packet, packetLength)) {
        // Buffer full - could log or increment error counter
    }
}

void CRSF::crsfWritePacket(uint8_t packet[], uint8_t packetLength) {
    ELRS_PORT.write(packet, packetLength);
    if (isHalfDuplex) {
        // CRITICAL: flush() on RP2040 blocks until all bytes leave the shift register.
        // At 921600 baud, a 26-byte packet takes ~282us to fully transmit.
        ELRS_PORT.flush();

        // Discard TX echo reflected back on the single wire.
        // After flush(), the echo bytes should already be in the RX FIFO.
        // Read them out with a tight timeout to avoid blocking if no echo arrives.
        uint32_t echoStart = micros();
        while (ELRS_PORT.available() > 0) {
            ELRS_PORT.read();
            // Safety: don't spin forever if something goes wrong
            if (micros() - echoStart > 500) break;
        }
    } 
}

void CRSF::crsfParseLinkStatsPacket(uint8_t rxBuffer[]) {
    if (!telemetryActive) {
        telemetryActive = true;
#ifdef CRSFDEBUG
        Serial.println("\n=================================");
        Serial.println("[INFO] -> TELEMETRY RECOVERED <-");
        Serial.println("=================================");
#endif
    }
    lastLinkStatsFrameTime = millis();

    linkStats.uplinkRssi1   = (int8_t)rxBuffer[3]; 
    linkStats.uplinkRssi2   = (int8_t)rxBuffer[4]; 
    linkStats.uplinkLQ      = rxBuffer[5];
    linkStats.uplinkSNR     = (int8_t)rxBuffer[6]; 
    linkStats.activeAntenna = rxBuffer[7];
    linkStats.rfMode        = rxBuffer[8];
    linkStats.txPower       = rxBuffer[9];
    linkStats.downlinkRssi  = (int8_t)rxBuffer[10];
    linkStats.downlinkLQ    = rxBuffer[11];
    linkStats.downlinkSNR   = (int8_t)rxBuffer[12];
}

void CRSF::crsfParseElrsSyncPacket(uint8_t rxBuffer[]) {
    if (rxBuffer[1] >= 11 && rxBuffer[5] == CRSF_SUBTYPE_OPENTX_SYNC) {      
        uint32_t rawInterval = ((uint32_t)rxBuffer[6] << 24) | 
                               ((uint32_t)rxBuffer[7] << 16) | 
                               ((uint32_t)rxBuffer[8] << 8)  | 
                                rxBuffer[9];
        int32_t rawShift = ((int32_t)rxBuffer[10] << 24) | 
                           ((int32_t)rxBuffer[11] << 16) | 
                           ((int32_t)rxBuffer[12] << 8)  | 
                            rxBuffer[13];

        targetIntervalUs   = (uint32_t)((float)rawInterval * 0.1f);
        currentPhaseShift  = (int32_t)((float)rawShift * 0.1f);
        currentPhaseShift = constrain(currentPhaseShift, -(int32_t)targetIntervalUs, (int32_t)targetIntervalUs);

        if (currentPhaseShift > (int32_t)(targetIntervalUs / 2))
            currentPhaseShift -= (int32_t)targetIntervalUs;
        else if (-currentPhaseShift > (int32_t)(targetIntervalUs / 2))
            currentPhaseShift += (int32_t)targetIntervalUs;
        
        syncPacketReceived = true;      
        lastSyncPacketTime = micros();

        phaseShiftHistory[syncPacketCount++] = currentPhaseShift;
        if (syncPacketCount == 0) cumulativePhaseShift = 0;
        else cumulativePhaseShift += currentPhaseShift;

        if (millis() > lastSyncPacketDisplay + 1000) {
            averagePhaseShift = (syncPacketCount == 0) ? 0 : cumulativePhaseShift / (int16_t)syncPacketCount;
#ifdef CRSFDEBUG        
            Serial.println("\n============ [CRSFShot] ============");
            Serial.print("Received "); Serial.print(syncPacketCount); Serial.println(" packets in 1000ms");
            Serial.print("Last Sync Packet. Interval: "); Serial.print(targetIntervalUs);
            Serial.print(" Phase Shift: "); Serial.println(currentPhaseShift);
            Serial.print("Average shift: "); Serial.println(averagePhaseShift);
            Serial.print("Past packets : ");
            for (int i = 0; i < syncPacketCount; i++){
                Serial.print(phaseShiftHistory[i]); Serial.print(" "); 
            } Serial.println();
            Serial.println("====================================");
#endif
            memset(phaseShiftHistory, 0, sizeof(phaseShiftHistory));   
            cumulativePhaseShift = 0;
            syncPacketCount = 0;
            lastSyncPacketDisplay = millis();
        }
    }
}

uint32_t CRSF::crsfNextInterval() {
    uint32_t baseInterval = (targetIntervalUs > 0) ? targetIntervalUs : CRSF_TIME_BETWEEN_FRAMES_US;
    int32_t correction = 0;
    return baseInterval + correction;
}

void CRSF::crsfReadTelemetry() {
    // Expire a partial frame after a generous inter-byte gap (OLED refresh may delay loop calls). Unlike the
    // original blocking parser, this function never waits for bytes to arrive.
    uint32_t now = millis();
    if (rxPosition != 0 && (uint32_t)(now - rxLastByteMs) > 20) {
        rxPosition = 0;
        rxExpectedLength = 0;
    }

    while (ELRS_PORT.available() > 0) {
        const int incoming = ELRS_PORT.read();
        if (incoming < 0) break;
        const uint8_t byteValue = (uint8_t)incoming;
        now = millis();

        if (rxPosition == 0) {
            // Only accept addresses this handset's telemetry parser handles.
            if (byteValue != HANDSET_ADDRESS && byteValue != LUA_SCRIPT_ADDRESS) {
                continue;
            }
            rxPacketBuffer[0] = byteValue;
            rxPosition = 1;
            rxLastByteMs = now;
            continue;
        }

        if (rxPosition == 1) {
            // CRSF length counts type + payload + CRC, excluding address/length.
            if (byteValue < 2 || byteValue > (CRSF_MAX_PACKET_SIZE - 2)) {
                rxPosition = 0;
                rxExpectedLength = 0;
                // If this byte is itself a valid address, retain it as a new
                // frame start instead of throwing away a possible sync point.
                if (byteValue == HANDSET_ADDRESS || byteValue == LUA_SCRIPT_ADDRESS) {
                    rxPacketBuffer[0] = byteValue;
                    rxPosition = 1;
                    rxLastByteMs = now;
                }
                continue;
            }
            rxPacketBuffer[1] = byteValue;
            rxExpectedLength = byteValue + 2;
            rxPosition = 2;
            rxLastByteMs = now;
            continue;
        }

        if (rxPosition >= CRSF_MAX_PACKET_SIZE) {
            rxPosition = 0;
            rxExpectedLength = 0;
            continue;
        }

        rxPacketBuffer[rxPosition++] = byteValue;
        rxLastByteMs = now;

        if (rxPosition >= rxExpectedLength) {
            const uint8_t packetSize = rxExpectedLength;
            if (checkCrc(rxPacketBuffer, packetSize)) {
                moduleConnected = true;
                lastValidFrameTime = now;

                const uint8_t packetType = rxPacketBuffer[2];
                switch (packetType) {
                    case ELRS_LINK_STATS:
                        crsfParseLinkStatsPacket(rxPacketBuffer);
                        break;
                    case ELRS_RADIO_ID:
                        crsfParseElrsSyncPacket(rxPacketBuffer);
                        break;
                    case 0x08: // CRSF_BATTERY_SENSOR
                        // Frame: address, length, type, voltage(2), current(2),
                        // capacity(3), remaining(1), CRC.
                        if (packetSize >= 12) {
                            const uint16_t voltageRaw =
                                ((uint16_t)rxPacketBuffer[3] << 8) |
                                (uint16_t)rxPacketBuffer[4];
                            receiverBatteryVoltage = (float)voltageRaw * 0.1f;
                            lastReceiverBatteryFrameTime = now;
                            receiverBatteryValid = true;
                        }
                        break;
                    default:
                        telemetryQueue.push(rxPacketBuffer, packetSize);
                        break;
                }
            }
            rxPosition = 0;
            rxExpectedLength = 0;
        }
    }

    now = millis();
    // Treat receiver voltage as unavailable if no fresh battery frame arrives.
    if (receiverBatteryValid && (uint32_t)(now - lastReceiverBatteryFrameTime) > 3000) {
        receiverBatteryValid = false;
    }
    if (telemetryActive && (now - lastLinkStatsFrameTime > 1000)) {
        telemetryActive = false;
#ifdef CRSFDEBUG
        Serial.println("\n=================================");
        Serial.println("[ALERT] !!! TELEMETRY LOST !!!");
        Serial.println("=================================");
#endif
    }
    if (moduleConnected && (now - lastValidFrameTime > 2000)) {
        moduleConnected = false;
        syncPacketReceived = false;
        Serial.println("[HANDSHAKE] Telemetry Timeout. Module connection Lost.");
    }   
}