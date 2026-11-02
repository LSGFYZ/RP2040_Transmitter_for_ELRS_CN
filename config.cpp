/*
 * This file is part of ESP32C3-TX (Ported to RP2040 Zero with ADS1115)
 *
 * ESP32C3-TX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <vector>
#include "config.h"

// ---------------------------------------------------------------------------
// RP2040 settings storage.
// Calibration uses EEPROM bytes 0..sizeof(CalibValues)-1; reserve 128..255
// for handset settings so the two records never overlap.
// ---------------------------------------------------------------------------
static constexpr int SETTINGS_EEPROM_OFFSET = 128;
static constexpr size_t SETTINGS_EEPROM_SIZE = 512;
static constexpr uint32_t SETTINGS_MAGIC = 0x48535458UL; // "HSTX"
static const uint8_t SETTINGS_VERSION = 2;

struct PersistentSettings {
    uint32_t magic;
    uint8_t  version;
    uint8_t  channelMode;       // 摇杆模式 (1-4)
    uint8_t  adcFilter;         // 0=关闭, 1=开启
    uint16_t deadband;          // 死区大小
    uint16_t auxLongPress;      // 长按时间 (ms)
    uint16_t voltageScale;      // 电压缩放比例
    bool     throttleCentered;  // 油门是否居中
    bool     reverse[4];        // 通道反向 (Ail, Ele, Thr, Rud)
    uint8_t  limit[4];          // 通道行程限制 (10-100 %)
    uint8_t  expo[4];           // 通道指数曲线 (0-100 %)
};

static bool ensureSettingsStorage() {
    // calibrationLoad() normally initializes EEPROM first, but keep this
    // module safe if its initialization order changes later.
    EEPROM.begin(SETTINGS_EEPROM_SIZE);
    return true;
}

static PersistentSettings defaultPersistentSettings() {
    PersistentSettings s{};
    s.magic             = SETTINGS_MAGIC;
    s.version           = SETTINGS_VERSION;
    s.channelMode       = 2;
    s.adcFilter         = 0;
    s.deadband          = 0;
    s.auxLongPress      = AUX_LONG_PRESS;
    s.voltageScale      = VOLTAGE_SCALE;
    s.throttleCentered  = false;
    for (int i = 0; i < 4; i++) {
        s.reverse[i] = false;
        s.limit[i]   = 100;
        s.expo[i]    = 0;
    }
    return s;
}

static bool validPersistentSettings(const PersistentSettings& s) {
    if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION ||
        s.channelMode < 1 || s.channelMode > 4 || s.adcFilter > 1 ||
        s.deadband > 200 || s.auxLongPress < 300 || s.auxLongPress > 5000 ||
        s.voltageScale < 100 || s.voltageScale > 2000) return false;
    for (int i = 0; i < 4; i++) {
        if (s.limit[i] < 10 || s.limit[i] > 100 || s.expo[i] > 100) return false;
    }
    return true;
}

static PersistentSettings loadPersistentSettings() {
    ensureSettingsStorage();
    PersistentSettings s{};
    EEPROM.get(SETTINGS_EEPROM_OFFSET, s);
    if (!validPersistentSettings(s)) {
        Serial.println("[Handset] No valid saved settings; using defaults.");
        return defaultPersistentSettings();
    }
    return s;
}

static bool savePersistentSettings(const PersistentSettings& input) {
    ensureSettingsStorage();
    PersistentSettings s = input;
    s.magic = SETTINGS_MAGIC;
    s.version = SETTINGS_VERSION;
    EEPROM.put(SETTINGS_EEPROM_OFFSET, s);
    EEPROM.commit();

    PersistentSettings verify{};
    EEPROM.get(SETTINGS_EEPROM_OFFSET, verify);
    const bool ok = validPersistentSettings(verify) &&
                    memcmp(&s, &verify, sizeof(s)) == 0;
    Serial.println(ok ? "[Handset] Settings saved to EEPROM." :
                        "[Handset] ERROR: EEPROM settings verification failed.");
    return ok;
}

static PersistentSettings buildPersistentFromGlobal(const HandsetSettings& gs) {
    PersistentSettings ps{};
    ps.version          = SETTINGS_VERSION;
    ps.channelMode      = gs.channelMode;
    ps.adcFilter        = gs.adcFilter;
    ps.deadband         = gs.deadband;
    ps.auxLongPress     = gs.auxLongPress;
    ps.voltageScale     = gs.voltageScale;
    ps.throttleCentered = gs.throttleCentered;
    for (int i = 0; i < 4; i++) {
        ps.reverse[i] = gs.reverse[i];
        ps.limit[i]   = gs.limit[i];
        ps.expo[i]    = gs.expo[i];
    }
    return ps;
}

// ===========================================================================
// Handset 类实现
// ===========================================================================

void Handset::loadGlobalSettings(bool initialize) {
    if (initialize) { handsetSettingsMenu.clearParams(); }
    int idx = 0;

    addInfo(idx, idx, "Radio Settings", "");
    idx++;

    PersistentSettings ps = loadPersistentSettings();

    globalSettings.channelMode      = ps.channelMode;
    globalSettings.adcFilter        = ps.adcFilter;
    globalSettings.deadband         = ps.deadband;
    globalSettings.auxLongPress     = ps.auxLongPress;
    globalSettings.voltageScale     = ps.voltageScale;
    globalSettings.throttleCentered = ps.throttleCentered;
    for (int i = 0; i < 4; i++) {
        globalSettings.reverse[i] = ps.reverse[i];
        globalSettings.limit[i]   = ps.limit[i];
        globalSettings.expo[i]    = ps.expo[i];
    }

    addChoiceParam(idx, 0, "Stick Mode", {"","Mode 1","Mode 2","Mode 3","Mode 4"}, globalSettings.channelMode, 1, 4);
    idx++;

    addChoiceParam(idx, 0, "ADC Filter", {"Off","On"}, globalSettings.adcFilter, 0, 1);
    idx++;

    // Keep the original deadband scale/range; ADC raw values are normalized
    // to the original 0..2047 logical range before this setting is applied.
    addNumericParam(idx, 0, CRSF_UINT16, "Deadband", globalSettings.deadband, 0, 200, 10, " / 2048");
    idx++;

    addNumericParam(idx, 0, CRSF_UINT16, "Long Press", globalSettings.auxLongPress, 300, 5000, 100, "ms");
    idx++;

    addNumericParam(idx, 0, CRSF_UINT16, "Volt Scale", globalSettings.voltageScale, 100, 2000, 5, "");
    idx++;

    const char* folderLabels[4] = {"Ail/Roll", "Ele/Pitch", "Throttle", "Rud/Yaw"};
    const char* shortLabels[4]  = {"Ail", "Ele", "Thr", "Yaw"};
    char keyBuf[15];

    for (uint8_t ch = 0; ch < 4; ch++) {
        int currentFolderID = idx;
        addFolder(idx, 0, folderLabels[ch]);
        idx++;

        if (ch == THR) {
            addChoiceParam(idx, currentFolderID, "Center Throt", {"No","Yes"}, globalSettings.throttleCentered, 0, 1);
            idx++;
        }

        snprintf(keyBuf, sizeof(keyBuf), "%s Reverse", shortLabels[ch]);
        addChoiceParam(idx, currentFolderID, keyBuf, {"Normal","Reverse"}, globalSettings.reverse[ch], 0, 1);
        idx++;

        snprintf(keyBuf, sizeof(keyBuf), "%s Limit", shortLabels[ch]);
        addNumericParam(idx, currentFolderID, CRSF_UINT8, keyBuf, globalSettings.limit[ch], 10, 100, 10, "%");
        idx++;

        snprintf(keyBuf, sizeof(keyBuf), "%s Expo", shortLabels[ch]);
        addNumericParam(idx, currentFolderID, CRSF_UINT8, keyBuf, globalSettings.expo[ch], 0, 100, 5, "%");
        idx++;
    }
}

void Handset::saveGlobalSettings() {
    const crsfParameter& param = handsetSettingsMenu.getCurrentParam();
    switch (param.type) {
        case CRSF_UINT8:
        case CRSF_INT8:
        case CRSF_TEXT_SELECTION:
        case CRSF_UINT16:
        case CRSF_INT16:
            break;
        default:
            return; 
    }

    const char* lbl = param.label;
    int32_t val = handsetSettingsMenu.editValue;

    if      (strcmp(lbl, "Stick Mode")  == 0) globalSettings.channelMode  = val;
    else if (strcmp(lbl, "ADC Filter")  == 0) globalSettings.adcFilter    = val;
    else if (strcmp(lbl, "Deadband")    == 0) globalSettings.deadband     = val;
    else if (strcmp(lbl, "Long Press")  == 0) globalSettings.auxLongPress = val;
    else if (strcmp(lbl, "Volt Scale")  == 0) globalSettings.voltageScale = val;
    else if (strcmp(lbl, "Center Throt")== 0) globalSettings.throttleCentered = val;
    else {
        for (int ch = 0; ch < 4; ch++) {
            const char* prefixes[] = {"Ail", "Ele", "Thr", "Yaw"};
            char revKey[15], limKey[15], expKey[15];
            snprintf(revKey, sizeof(revKey), "%s Reverse", prefixes[ch]);
            snprintf(limKey, sizeof(limKey), "%s Limit",   prefixes[ch]);
            snprintf(expKey, sizeof(expKey), "%s Expo",    prefixes[ch]);

            if      (strcmp(lbl, revKey) == 0) { globalSettings.reverse[ch] = val; break; }
            else if (strcmp(lbl, limKey) == 0) { globalSettings.limit[ch]   = val; break; }
            else if (strcmp(lbl, expKey) == 0) { globalSettings.expo[ch]    = val; break; }
        }
    }

    PersistentSettings ps = buildPersistentFromGlobal(globalSettings);
    if (!savePersistentSettings(ps)) {
        Serial.println("[Handset] Settings changed in RAM but could not be verified in EEPROM.");
        return;
    }

    // Refresh displayed current values from the just-saved settings.
    loadGlobalSettings(false);
}

void Handset::executeSettingsCommand() {}
void Handset::cancelSettingsCommand() {}

bool Handset::addFolder(int idx, int parent, const char* label) {
    if (handsetSettingsMenu.getParamSlot(idx) != -1) {
        handsetSettingsMenu[idx].parentFolder = parent;
        handsetSettingsMenu[idx].type = CRSF_FOLDER;
        strcpy(handsetSettingsMenu[idx].label, label);
        return true;
    }
    return false;
}

bool Handset::addInfo(int idx, int parent, const char* label, const char* valueString) {
    if (handsetSettingsMenu.getParamSlot(idx) != -1) {
        handsetSettingsMenu[idx].parentFolder = parent;
        handsetSettingsMenu[idx].type = CRSF_INFO;
        strcpy(handsetSettingsMenu[idx].label, label);
        strcpy(handsetSettingsMenu[idx].valueString, valueString);
        return true;
    }
    return false;
}

bool Handset::addChoiceParam(int idx, int parent, const char* label,
                             const std::vector<const char*>& choices,
                             int currVal, int min, int max) {
    if (handsetSettingsMenu.getParamSlot(idx) != -1) {
        handsetSettingsMenu[idx].parentFolder = parent;
        handsetSettingsMenu[idx].type = CRSF_TEXT_SELECTION;
        strcpy(handsetSettingsMenu[idx].label, label);
        for (size_t i = 0; i < choices.size(); i++) {
            strcpy(handsetSettingsMenu[idx].choices[i].text, choices[i]);
        }
        handsetSettingsMenu[idx].minVal = min;
        handsetSettingsMenu[idx].maxVal = max;
        handsetSettingsMenu[idx].choicesCount = (max - min) + 1;
        handsetSettingsMenu[idx].currentVal = currVal;
        return true;
    }
    return false;
}

bool Handset::addNumericParam(int idx, int parent, crsfValueType type,
                              const char* label, int currVal,
                              int min, int max, int step, const char* units) {
    if (handsetSettingsMenu.getParamSlot(idx) != -1) {
        handsetSettingsMenu[idx].parentFolder = parent;
        handsetSettingsMenu[idx].type = type;
        strcpy(handsetSettingsMenu[idx].label, label);
        handsetSettingsMenu[idx].minVal = min;
        handsetSettingsMenu[idx].maxVal = max;
        handsetSettingsMenu[idx].step = step;
        strcpy(handsetSettingsMenu[idx].units, units);
        handsetSettingsMenu[idx].currentVal = currVal;
        return true;
    }
    return false;
}