/*
 * RP2040 Zero ELRS transmitter port.
 * Four latency-sensitive stick axes use the RP2040's native 12-bit ADC.
 * ADS1115 is retained only for battery-voltage measurement.
 * RC mapping, calibration behaviour, menu operation and CRSF/ELRS protocol
 * semantics remain based on the original transmitter firmware.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <EEPROM.h>
#include <Joystick.h>  // Arduino-Pico built-in USB HID gamepad (Pico SDK USB Stack)
#include "hardware.h"
#include "config.h"
#include "crsf.h"
#include "elrs.h"
#include "oled.h"

// ADS1115 is used only for low-rate battery measurement. Stick acquisition
// must never wait for an I2C conversion.
Adafruit_ADS1115 ads;
bool adsReady = false;
static uint32_t lastBatterySampleMs = 0;

Handset transmitter;

// oled.cpp's current handleNavigation API still accepts this legacy flag.
// No Bluetooth HID library or Bluetooth gamepad code is used in this sketch.
bool btJoystickModeActive = false;

int16_t stickValues[7];  

struct ADCValues {
    int32_t leftX;
    int32_t leftY;
    int32_t rightX;
    int32_t rightY;
    int32_t voltage;
};
ADCValues rawValues;

struct FilterState {
    int32_t rightX;
    int32_t rightY;
    int32_t leftY;
    int32_t leftX;
};
FilterState filteredRaw;

float batteryVoltage;
bool ARMED = false;
uint8_t navigate = 0;

int16_t rcChannels[CRSF_MAX_CHANNEL];
CRSF crsfClass;
ELRS elrsClass(crsfClass);

#define CALIB_CENT_TMO 5000UL
#define CALIB_MOVE_TMO 15000UL
bool calibrationRequested = false;
uint32_t calibrationTimerStart = 0;
int cal_reset = 0;

struct CalibValues {
    int32_t rightXMin = STICK_ADC_MIN, rightXMax = STICK_ADC_MAX, rightXMid = STICK_ADC_MID;
    int32_t rightYMin = STICK_ADC_MIN, rightYMax = STICK_ADC_MAX, rightYMid = STICK_ADC_MID;
    int32_t leftXMin  = STICK_ADC_MIN, leftXMax  = STICK_ADC_MAX, leftXMid  = STICK_ADC_MID;
    int32_t leftYMin  = STICK_ADC_MIN, leftYMax  = STICK_ADC_MAX, leftYMid  = STICK_ADC_MID;
};
CalibValues calValues;

void calibrationLoad();
void calibrationSave();
void calibrationChirp(uint8_t times);
bool calibrationRun();
void getAnalogInputs();
void getDigitalInputs();
void stickMenuNavigation();
void checkGestureSetting();
bool checkThrottleIdleTimeout();
void playTone(uint8_t timesPerSecond);
void blinkLED(int ledPin, uint16_t blinkRate);
void statusDisplay();
void logData();
void logAnalogDebug();
void updateUsbGamepad();
void mapChannels();
int16_t calibrate(int16_t raw, int16_t minVal, int16_t centerVal, int16_t maxVal);
int32_t applyCyclicExpo(int32_t stickDiff, uint8_t expoFactor);
int32_t applyThrottleExpo(int32_t adcVal, uint8_t expoFactor);
uint16_t processStick(int32_t adcValue, uint8_t chIndex);

inline int32_t lowLatencyFilter(int32_t currentRaw, int32_t *previousFiltered) {
    if (transmitter.globalSettings.adcFilter && *previousFiltered > 0) {
        *previousFiltered += ((currentRaw - *previousFiltered) >> FILTER_SHIFT);
    } else {
        *previousFiltered = currentRaw;
    }
    return *previousFiltered;
}

static bool validAxisCalibration(int32_t mn, int32_t mid, int32_t mx) {
    return mn >= STICK_ADC_MIN && mn < mid && mid < mx && mx <= STICK_ADC_MAX;
}

void calibrationSave() {
    EEPROM.begin(512);
    EEPROM.put(0, calValues);
    EEPROM.commit();
}

void calibrationLoad() {
    EEPROM.begin(512);
    CalibValues saved;
    EEPROM.get(0, saved);
    if (validAxisCalibration(saved.rightXMin, saved.rightXMid, saved.rightXMax) &&
        validAxisCalibration(saved.rightYMin, saved.rightYMid, saved.rightYMax) &&
        validAxisCalibration(saved.leftXMin, saved.leftXMid, saved.leftXMax) &&
        validAxisCalibration(saved.leftYMin, saved.leftYMid, saved.leftYMax)) {
        calValues = saved;
    } else {
        calValues = CalibValues{};
    }
}

void calibrationChirp(uint8_t times) {
    digitalWrite(DIGITAL_PIN_LED, HIGH);
    delay(250);
    digitalWrite(DIGITAL_PIN_LED, LOW);
    delay(250);
    for (uint8_t i = 0; i < times; ++i) {
        digitalWrite(DIGITAL_PIN_LED, HIGH);
        delay(100);
        digitalWrite(DIGITAL_PIN_LED, LOW);
        delay(100);
    }
}

bool calibrationRun() {
    if (ARMED) {
        Serial.println("Cannot calibrate while arm switch is active");
        cal_reset = 0;
        calibrationRequested = false;
        delay(1000);
        return false;
    }
    if (cal_reset < 1) {
        Serial.println("Starting Calibration");
        drawCalibrationScreen(1, 0);
        calibrationChirp(2);
        calibrationTimerStart = millis();
        calValues.rightXMin = calValues.rightYMin = calValues.leftXMin = calValues.leftYMin = STICK_ADC_MAX;
        calValues.rightXMax = calValues.rightYMax = calValues.leftXMax = calValues.leftYMax = STICK_ADC_MIN;
        calValues.rightXMid = calValues.rightYMid = calValues.leftXMid = calValues.leftYMid = 0;
        cal_reset = 1;
    }

    const uint32_t elapsed = millis() - calibrationTimerStart;
    if (elapsed < CALIB_CENT_TMO) {
        calValues.rightXMid = analogRead(STICK_ADC_RIGHT_X_PIN);
        calValues.rightYMid = analogRead(STICK_ADC_RIGHT_Y_PIN);
        calValues.leftYMid = analogRead(STICK_ADC_LEFT_Y_PIN);
        calValues.leftXMid = analogRead(STICK_ADC_LEFT_X_PIN);
        drawCalibrationScreen(2, ((CALIB_CENT_TMO - elapsed) / 1000) + 1);
        blinkLED(DIGITAL_PIN_LED, 1000);
    } else if (elapsed < CALIB_MOVE_TMO) {
        int32_t val = analogRead(STICK_ADC_RIGHT_X_PIN);
        if (val < calValues.rightXMin) calValues.rightXMin = val;
        if (val > calValues.rightXMax) calValues.rightXMax = val;
        val = analogRead(STICK_ADC_RIGHT_Y_PIN);
        if (val < calValues.rightYMin) calValues.rightYMin = val;
        if (val > calValues.rightYMax) calValues.rightYMax = val;
        val = analogRead(STICK_ADC_LEFT_Y_PIN);
        if (val < calValues.leftYMin) calValues.leftYMin = val;
        if (val > calValues.leftYMax) calValues.leftYMax = val;
        val = analogRead(STICK_ADC_LEFT_X_PIN);
        if (val < calValues.leftXMin) calValues.leftXMin = val;
        if (val > calValues.leftXMax) calValues.leftXMax = val;
        drawCalibrationScreen(3, ((CALIB_MOVE_TMO - elapsed) / 1000) + 1);
    } else {
        const bool valid = validAxisCalibration(calValues.rightXMin, calValues.rightXMid, calValues.rightXMax) &&
                           validAxisCalibration(calValues.rightYMin, calValues.rightYMid, calValues.rightYMax) &&
                           validAxisCalibration(calValues.leftXMin, calValues.leftXMid, calValues.leftXMax) &&
                           validAxisCalibration(calValues.leftYMin, calValues.leftYMid, calValues.leftYMax);
        if (valid) {
            calibrationSave();
            Serial.println("Calibration Done (RP2040 native ADC)");
        } else {
            calValues = CalibValues{};
            Serial.println("Calibration failed; using default RP2040 ADC calibration");
        }
        drawCalibrationScreen(4, 0);
        cal_reset = 0;
        calibrationRequested = false;
        calibrationChirp(3);
        lastScreen = CALIBRATION;
    }
    return true;
}

void getAnalogInputs() {
    // Read the four sticks directly from the RP2040 ADC. These calls are local
    // and fast; they do not wait for an I2C/ADS1115 conversion.
    rawValues.rightX = lowLatencyFilter(analogRead(STICK_ADC_RIGHT_X_PIN), &filteredRaw.rightX);
    rawValues.rightY = lowLatencyFilter(analogRead(STICK_ADC_RIGHT_Y_PIN), &filteredRaw.rightY);
    rawValues.leftY = lowLatencyFilter(analogRead(STICK_ADC_LEFT_Y_PIN), &filteredRaw.leftY);
    rawValues.leftX = lowLatencyFilter(analogRead(STICK_ADC_LEFT_X_PIN), &filteredRaw.leftX);

    // Battery voltage is low bandwidth, so sample it separately at 4 Hz.
    // The ADS1115 is not allowed to sit in the stick-sampling path.
    const uint32_t now = millis();
    if (adsReady && (lastBatterySampleMs == 0 || now - lastBatterySampleMs >= 250)) {
        lastBatterySampleMs = now;
        rawValues.voltage = ads.getLastConversionResults();
        const float adcVolts = ads.computeVolts((int16_t)rawValues.voltage);
        const float dividerRatio = (BAT_R1 + BAT_R2) / BAT_R2;
        const float calibrationRatio = (transmitter.globalSettings.voltageScale > 0)
            ? ((float)VOLTAGE_SCALE / (float)transmitter.globalSettings.voltageScale)
            : 1.0f;
        batteryVoltage = adcVolts * dividerRatio * calibrationRatio;
    }


    stickValues[RIGHTX] = calibrate(rawValues.rightX, calValues.rightXMin, calValues.rightXMid, calValues.rightXMax);
    stickValues[RIGHTY] = calibrate(rawValues.rightY, calValues.rightYMin, calValues.rightYMid, calValues.rightYMax);
    stickValues[LEFTY] = calibrate(rawValues.leftY, calValues.leftYMin, calValues.leftYMid, calValues.leftYMax);
    stickValues[LEFTX] = calibrate(rawValues.leftX, calValues.leftXMin, calValues.leftXMid, calValues.leftXMax);
#ifdef RIGHTX_REVERSE
    stickValues[RIGHTX] = ADC_MAX - stickValues[RIGHTX];
#endif
#ifdef RIGHTY_REVERSE
    stickValues[RIGHTY] = ADC_MAX - stickValues[RIGHTY];
#endif
#ifdef LEFTX_REVERSE
    stickValues[LEFTX] = ADC_MAX - stickValues[LEFTX];
#endif
#ifdef LEFTY_REVERSE
    stickValues[LEFTY] = ADC_MAX - stickValues[LEFTY];
#endif
    stickMenuNavigation();
}

void stickMenuNavigation() {
    static uint32_t centerStartTime = 0, deflectionStartTime = 0;
    static bool isCentered = true, hasNavigated = false;
    static uint8_t pendingDirection = 0;
    const uint32_t now = millis();
    navigate = 0;
    const bool centered = stickValues[RIGHTY] > RC_MIN_COMMAND && stickValues[RIGHTY] < RC_MAX_COMMAND &&
                          stickValues[RIGHTX] > RC_MIN_COMMAND && stickValues[RIGHTX] < RC_MAX_COMMAND;
    if (centered) {
        deflectionStartTime = 0;
        if (centerStartTime == 0) centerStartTime = now;
        else if (now - centerStartTime >= BTN_SHORT_PRESS) { isCentered = true; hasNavigated = false; }
    } else {
        centerStartTime = 0;
        uint8_t direction = 0;
        if (stickValues[RIGHTY] >= RC_MAX_COMMAND) direction = 1;
        else if (stickValues[RIGHTY] <= RC_MIN_COMMAND) direction = 2;
        else if (stickValues[RIGHTX] <= RC_MIN_COMMAND) direction = 3;
        else if (stickValues[RIGHTX] >= RC_MAX_COMMAND) direction = 4;
        if (direction != pendingDirection) { deflectionStartTime = now; pendingDirection = direction; }
        else if (now - deflectionStartTime >= BTN_SHORT_PRESS && isCentered && !hasNavigated) {
            navigate = pendingDirection;
            hasNavigated = true;
            isCentered = false;
        }
    }
}

int16_t calibrate(int16_t raw, int16_t minVal, int16_t centerVal, int16_t maxVal) {
    if (maxVal <= minVal || centerVal <= minVal || centerVal >= maxVal) return ADC_MID;
    raw = constrain(raw, minVal, maxVal);
    if (raw <= centerVal) return map(raw, minVal, centerVal, ADC_MIN, ADC_MID);
    return map(raw, centerVal, maxVal, ADC_MID + 1, ADC_MAX);
}

// These formulas intentionally match the original ESP32-C3 firmware.
int32_t applyCyclicExpo(int32_t stickDiff, uint8_t expoFactor) {
    if (expoFactor == 0) return stickDiff;
    const int32_t stickCubed = (stickDiff * stickDiff * stickDiff) >> 20;
    return ((100 - expoFactor) * stickDiff + expoFactor * stickCubed) / 100;
}

int32_t applyThrottleExpo(int32_t adcVal, uint8_t expoFactor) {
    if (expoFactor == 0) return adcVal;
    const int32_t downscaled = adcVal >> 1;
    const int32_t cubed = (downscaled * downscaled * downscaled) >> 20;
    const int32_t throttleCubed = cubed << 1;
    return ((100 - expoFactor) * adcVal + expoFactor * throttleCubed) / 100;
}

uint16_t processStick(int32_t adcValue, uint8_t chIndex) {
    adcValue = constrain(adcValue, ADC_MIN, ADC_MAX);
    int32_t crsfOutput = ((CRSF_DIGITAL_CHANNEL_MAX - CRSF_DIGITAL_CHANNEL_MIN) >> 1);
    const uint8_t expo = transmitter.globalSettings.expo[chIndex];
    const uint8_t limit = transmitter.globalSettings.limit[chIndex];

    if (chIndex == THR && !transmitter.globalSettings.throttleCentered) {
        adcValue = applyThrottleExpo(adcValue, expo);
        adcValue = (adcValue * limit) / 100;
        if (transmitter.globalSettings.reverse[chIndex]) adcValue = ADC_MAX - adcValue;
        crsfOutput = ((adcValue * 820) >> 10) + 172;
    } else {
        int32_t stickCentered = adcValue - ADC_MID;
        if (transmitter.globalSettings.deadband > 0) {
            if (abs(stickCentered) <= transmitter.globalSettings.deadband) {
                stickCentered = 0;
            } else if (stickCentered > 0) {
                stickCentered = ((stickCentered - transmitter.globalSettings.deadband) * 1024) / (1024 - transmitter.globalSettings.deadband);
            } else {
                stickCentered = ((stickCentered + transmitter.globalSettings.deadband) * 1024) / (1024 - transmitter.globalSettings.deadband);
            }
        }
        stickCentered = applyCyclicExpo(stickCentered, expo);
        stickCentered = (stickCentered * limit) / 100;
        if (transmitter.globalSettings.reverse[chIndex]) stickCentered = -stickCentered;
        crsfOutput = ((stickCentered * 820) >> 10) + 992;
    }
    crsfOutput = constrain(crsfOutput, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    return (uint16_t)crsfOutput;
}

void mapChannels() {
    int mode = transmitter.globalSettings.channelMode;
    if (mode < 1 || mode > 4) mode = 2; 

    switch (mode) {
        case 1:
            rcChannels[AIL] = processStick(stickValues[RIGHTX], 0); 
            rcChannels[ELE] = processStick(stickValues[LEFTY],  1);
            rcChannels[THR] = processStick(stickValues[RIGHTY], 2);
            rcChannels[RUD] = processStick(stickValues[LEFTX],  3); 
            break;
        case 2: // 标准美国手
            rcChannels[AIL] = processStick(stickValues[RIGHTX], 0); 
            rcChannels[ELE] = processStick(stickValues[RIGHTY], 1);
            rcChannels[THR] = processStick(stickValues[LEFTY],  2);
            rcChannels[RUD] = processStick(stickValues[LEFTX],  3); 
            break;
        case 3:
            rcChannels[AIL] = processStick(stickValues[LEFTX],  0); 
            rcChannels[ELE] = processStick(stickValues[LEFTY],  1);
            rcChannels[THR] = processStick(stickValues[RIGHTY], 2);
            rcChannels[RUD] = processStick(stickValues[RIGHTX], 3); 
            break;
        case 4:
            rcChannels[AIL] = processStick(stickValues[LEFTX],  0); 
            rcChannels[ELE] = processStick(stickValues[RIGHTY], 1);
            rcChannels[THR] = processStick(stickValues[LEFTY],  2);
            rcChannels[RUD] = processStick(stickValues[RIGHTX], 3); 
            break;
    }

    rcChannels[AUX1] = (stickValues[SW1] == 1) ? CRSF_DIGITAL_CHANNEL_MAX : CRSF_DIGITAL_CHANNEL_MIN;
    rcChannels[AUX2] = (stickValues[SW2] == 1) ? CRSF_DIGITAL_CHANNEL_MAX : CRSF_DIGITAL_CHANNEL_MIN;
    rcChannels[AUX3] = (stickValues[SW3] == 1) ? CRSF_DIGITAL_CHANNEL_MAX : CRSF_DIGITAL_CHANNEL_MIN;    
}

void getDigitalInputs(){
    static uint32_t buttonPressedTime = 0;
    if (digitalRead(DIGITAL_PIN_BUTTON) == LOW) {
        if (buttonPressedTime == 0) buttonPressedTime = millis(); 
    } else {
        if (buttonPressedTime > 0) {  
            unsigned long pressDuration = millis() - buttonPressedTime;
            buttonPressedTime = 0;
            if (pressDuration >= BTN_LONG_PRESS) { 
                navigate = 7;
            } else if (pressDuration >= BTN_SHORT_PRESS) {
                navigate = 6; 
            }
        }
    }

    static uint32_t aux1PressedTime = 0;
    if (digitalRead(DIGITAL_PIN_SWITCH_ARM) == LOW) {
        if (aux1PressedTime == 0) aux1PressedTime = millis(); 
    } else {
        if (aux1PressedTime > 0) {  
            unsigned long pressDuration = millis() - aux1PressedTime;
            aux1PressedTime = 0;
            if (pressDuration < transmitter.globalSettings.auxLongPress && pressDuration >= BTN_SHORT_PRESS) {
                stickValues[SW1] = !stickValues[SW1];
                ARMED = stickValues[SW1];
            }
        }
    }

    static uint32_t aux2PressedTime = 0;
    if (digitalRead(DIGITAL_PIN_SWITCH_AUX2) == LOW) {
        if (aux2PressedTime == 0) aux2PressedTime = millis(); 
    } else {
        if (aux2PressedTime > 0) {  
            unsigned long pressDuration = millis() - aux2PressedTime;
            aux2PressedTime = 0;
            if (pressDuration >= transmitter.globalSettings.auxLongPress) {
                stickValues[SW3] = !stickValues[SW3]; 
            } else if (pressDuration >= BTN_SHORT_PRESS) {
                stickValues[SW2] = !stickValues[SW2]; 
                navigate = 5; 
            }
        }
    }
}

void checkGestureSetting() {
    const bool currentArmState = stickValues[SW1];
    static bool lastArmState = false;
    static uint32_t armTimer = 0;
    static bool gestureTriggerReady = false;
    if (currentArmState && !lastArmState) {
        armTimer = millis();
        gestureTriggerReady = true;
    }
    if (!currentArmState && lastArmState && gestureTriggerReady && millis() - armTimer >= 1000) {
        gestureTriggerReady = false;
    }
    lastArmState = currentArmState;
}

bool checkThrottleIdleTimeout() {
    static int previousThrottle = 191;
    static uint32_t stickMovedMillis = 0;
    if (abs(previousThrottle - rcChannels[THR]) >= 30) {
        previousThrottle = rcChannels[THR];
        stickMovedMillis = millis();
    }
    return (millis() - stickMovedMillis > STICK_ALARM_TIME);
}

// RP2040 Zero version has no buzzer output; retain the original call sites.
void playTone(uint8_t timesPerSecond) { (void)timesPerSecond; }

void blinkLED(int ledPin, uint16_t blinkRate) {
    static uint8_t ledState = LOW;
    static uint32_t previousLedMillis = 0;
    const uint32_t now = millis();
    if (now - previousLedMillis >= blinkRate) {
        previousLedMillis = now;
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
    }
}

void statusDisplay() {
    static int avg = -120;
    int raw = -120;
    int strength = 0;

    if (currentScreen == MODEL_FINDER) {
        raw = crsfClass.telemetryActive ? crsfClass.linkStats.uplinkRssi1 : -120;
        avg = (int)(0.8f * avg + 0.2f * raw);
        strength = constrain((avg + 110) * 100 / 70, 0, 100);
        const uint16_t period = constrain(1200 - strength * 10, 100, 1200);
        blinkLED(DIGITAL_PIN_LED, period);
    } else if (batteryVoltage < WARNING_VOLTAGE && batteryVoltage >= BEEPING_VOLTAGE) {
        blinkLED(DIGITAL_PIN_LED, 500);
    } else if (batteryVoltage < BEEPING_VOLTAGE && batteryVoltage >= ON_USB) {
        blinkLED(DIGITAL_PIN_LED, 250);
        playTone(2);
    }
    if (checkThrottleIdleTimeout()) {
        blinkLED(DIGITAL_PIN_LED, 100);
        playTone(5);
    }

    if (navigate > 0) {
        handleNavigation(navigate, transmitter, elrsClass, btJoystickModeActive, calibrationRequested);
        navigate = 0;
    }

    static uint32_t lastScreenUpdateTime = 0;
    const uint32_t now = millis();
    
    //const uint16_t refreshMs = ARMED ? 500 : 100;
    const uint16_t refreshMs = ARMED ? 50 : 50;

    if (now - lastScreenUpdateTime < refreshMs) return;
    lastScreenUpdateTime = now;

    switch (currentScreen) {
        case MAIN_PAGE:
            drawHomeScreen(batteryVoltage,
                           crsfClass.receiverBatteryVoltage,
                           crsfClass.receiverBatteryValid,
                           crsfClass.telemetryActive ? 1 : 0,
                           crsfClass.linkStats.uplinkLQ, stickValues, ADC_MIN, ADC_MAX);
            break;
        case ELRS_STATS:
            drawElrsStatsScreen(elrsClass);
            break;
        case ELRS_MENU:
            drawMenuScreen(elrsClass.txModule.params);
            break;
        case CHANNEL_OUTPUTS:
            drawChannelOutputsScreen(rcChannels, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
            break;
        case CHANNEL_MENU:
            drawMenuScreen(transmitter.handsetSettingsMenu);
            break;
        case MODEL_FINDER:
            drawModelFinderScreen(raw, avg, strength);
            break;
        case CALIBRATION:
            drawCalibrationScreen(4, 0);
            break;
        case BT_JOYSTICK:
            // Bluetooth is removed; skip the old Bluetooth-only page.
            currentScreen = MODEL_FINDER;
            drawModelFinderScreen(raw, avg, strength);
            break;
    }
}

void logData() {
    static uint32_t lastLog = 0;
    if (millis() - lastLog < 250) return;
    lastLog = millis();
    Serial.print("Bat:"); Serial.print(batteryVoltage, 2);
    Serial.print("V AETR: ");
    Serial.print(rcChannels[AIL]); Serial.print(' ');
    Serial.print(rcChannels[ELE]); Serial.print(' ');
    Serial.print(rcChannels[THR]); Serial.print(' ');
    Serial.print(rcChannels[RUD]);
    Serial.print(" AUX: "); Serial.print(stickValues[SW1]);
    Serial.print(stickValues[SW2]); Serial.println(stickValues[SW3]);
}

// Diagnostic only: reports RP2040 native ADC readings and calibrated logical values.
// Does not alter stick processing, calibration, or menu navigation.
void logAnalogDebug() {
    static uint32_t lastAnalogDebug = 0;
    if (millis() - lastAnalogDebug < 500) return;
    lastAnalogDebug = millis();
    Serial.print("ADC raw RX,RY,LX,LY=");
    Serial.print(rawValues.rightX); Serial.print(',');
    Serial.print(rawValues.rightY); Serial.print(',');
    Serial.print(rawValues.leftX); Serial.print(',');
    Serial.print(rawValues.leftY);
    Serial.print(" | stick RX,RY,LX,LY=");
    Serial.print(stickValues[RIGHTX]); Serial.print(',');
    Serial.print(stickValues[RIGHTY]); Serial.print(',');
    Serial.print(stickValues[LEFTX]); Serial.print(',');
    Serial.print(stickValues[LEFTY]);
    Serial.print(" | channel A,E,T,R=");
    Serial.print(rcChannels[AIL]); Serial.print(',');
    Serial.print(rcChannels[ELE]); Serial.print(',');
    Serial.print(rcChannels[THR]); Serial.print(',');
    Serial.println(rcChannels[RUD]);
}

// USB HID gamepad update. Reuse the already-calibrated stickValues[] so HID
// reporting does not perform extra ADC reads or modify the CRSF channel path.
void updateUsbGamepad() {
    static uint32_t lastHidUpdateMs = 0;
    const uint32_t now = millis();
    if (now - lastHidUpdateMs < 5) return;  // Limit reports to at most 200 Hz.
    lastHidUpdateMs = now;

    // Arduino-Pico Joystick defaults to 10-bit axes (0..1023).
    const auto axis10 = [](int16_t value) -> int {
        if (value < ADC_MIN) value = ADC_MIN;
        if (value > ADC_MAX) value = ADC_MAX;
        return ((int32_t)value * 1023) / ADC_MAX;
    };

    Joystick.X(axis10(stickValues[LEFTX]));
    Joystick.Y(axis10(stickValues[LEFTY]));
    Joystick.Z(axis10(stickValues[RIGHTX]));
    Joystick.Zrotate(axis10(stickValues[RIGHTY]));

    // Three logical switch states already used by the transmitter firmware.
    Joystick.button(1, stickValues[SW1] != 0);  // ARM / GP7
    Joystick.button(2, stickValues[SW2] != 0);  // AUX2 short action / GP8
    Joystick.button(3, stickValues[SW3] != 0);  // AUX2 long action / GP8

    Joystick.send_now();
}

void setup() {
    Serial.begin(115200);
    Joystick.begin();
    Joystick.use10bit();
    Joystick.useManualSend(true);
    delay(200);

    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; ++i) {
        rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;
    }

    pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX2, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_BUTTON, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_LED, OUTPUT);
    // RP2040 Arduino-Pico defaults vary by API/configuration; explicitly use
    // the full native 12-bit ADC range expected by calibration and mapping.
    analogReadResolution(12);
    pinMode(STICK_ADC_RIGHT_X_PIN, INPUT);
    pinMode(STICK_ADC_RIGHT_Y_PIN, INPUT);
    pinMode(STICK_ADC_LEFT_X_PIN, INPUT);
    pinMode(STICK_ADC_LEFT_Y_PIN, INPUT);
    digitalWrite(DIGITAL_PIN_LED, LOW);

    calibrationLoad();

Serial.println("===== CALIBRATION DEBUG =====");

Serial.print("ADC_MIN=");
Serial.print(ADC_MIN);
Serial.print(" ADC_MID=");
Serial.print(ADC_MID);
Serial.print(" ADC_MAX=");
Serial.println(ADC_MAX);

Serial.print("RX min/mid/max: ");
Serial.print(calValues.rightXMin);
Serial.print("/");
Serial.print(calValues.rightXMid);
Serial.print("/");
Serial.println(calValues.rightXMax);

Serial.print("RY min/mid/max: ");
Serial.print(calValues.rightYMin);
Serial.print("/");
Serial.print(calValues.rightYMid);
Serial.print("/");
Serial.println(calValues.rightYMax);

Serial.print("LX min/mid/max: ");
Serial.print(calValues.leftXMin);
Serial.print("/");
Serial.print(calValues.leftXMid);
Serial.print("/");
Serial.println(calValues.leftXMax);

Serial.print("LY min/mid/max: ");
Serial.print(calValues.leftYMin);
Serial.print("/");
Serial.print(calValues.leftYMid);
Serial.print("/");
Serial.println(calValues.leftYMax);

Serial.println("=============================");

    transmitter.loadGlobalSettings(true);

    // Configure and start the shared I2C bus before U8g2 or ADS1115 uses it.
#if defined(ARDUINO_ARCH_RP2040)
    Wire.setSDA(I2C_SDA_OLED);
    Wire.setSCL(I2C_SCL_OLED);
#endif
    Wire.begin();
    Wire.setClock(400000);
    initDisplay();

    adsReady = ads.begin(ADS1115_ADDR);
    if (adsReady) {
        ads.setGain(ADS1115_GAIN);
        ads.setDataRate(RATE_ADS1115_860SPS);
        // Keep A0 in continuous-conversion mode. Reading the last result is
        // non-blocking and avoids waiting ~1 conversion period in the RC loop.
        ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);
        Serial.println("ADS1115 OK (battery channel, continuous mode)");
    } else {
        Serial.println("WARNING: ADS1115 not detected; battery-voltage display unavailable, native-ADC sticks remain active");
    }

    drawBootLogo();
    delay(3000);

#ifdef ELRS_HALF_DUPLEX
    crsfClass.begin(ELRS_RX, ELRS_TX, true);
#else
    crsfClass.begin(ELRS_RX, ELRS_TX, false);
#endif
    currentScreen = MAIN_PAGE;
}

void loop() {
    static uint32_t crsfTime = 0;

    if (calibrationRequested) {
        calibrationRun();
    } else {
        getAnalogInputs();
        getDigitalInputs();
        checkGestureSetting();
        mapChannels();
        logAnalogDebug();

#ifdef DEBUG
        logData();
#endif

        crsfClass.crsfReadTelemetry();
        elrsClass.update(currentScreen == ELRS_MENU);
        statusDisplay();

        const uint32_t currentMicros = micros();
        if ((int32_t)(currentMicros - crsfTime) >= 0) {
            if (crsfClass.commandQueue.hasItems()) {
                crsfClass.crsfSendQueuedCommand();
            } else {
                crsfClass.crsfSendChannels(rcChannels);
            }
            crsfTime = currentMicros + crsfClass.crsfNextInterval();
        }
    }

    // Keep HID reports alive during normal operation and calibration.
    updateUsbGamepad();
}