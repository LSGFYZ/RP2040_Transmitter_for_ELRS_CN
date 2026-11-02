#pragma once
#include <vector>
#include "params.h"

//==========================================
//   摇杆方向配置 (RP2040 native ADC)
// ==========================================
// 说明：
// 1. 默认是注释状态 (//)，代表【不反转/正常方向】。
// 2. 如果你发现某个轴方向反了，请把前面的 // 删掉，变成 #define ... 即可开启反转。
// ==========================================

#define RIGHTX_REVERSE  // 右摇杆 X 轴反转
//#define RIGHTY_REVERSE  // 右摇杆 Y 轴反转
#define LEFTX_REVERSE   // 左摇杆 X 轴反转
//#define LEFTY_REVERSE   // 左摇杆 Y 轴反转

#define VOLTAGE_SCALE 454
#define WARNING_VOLTAGE 3.7 
#define BEEPING_VOLTAGE 3.5 
#define ON_USB          1.0 

#define AUX_LONG_PRESS  1000  
#define BTN_LONG_PRESS  1500  
#define BTN_SHORT_PRESS 100   

#define RC_MIN_COMMAND 511
#define RC_MAX_COMMAND 1535

#define STICK_ALARM_TIME 300000 

// Keep the original transmitter logical range (0..2047).
// Raw RP2040 ADC samples (0..4095) are calibrated into this range.
#define ADC_MIN 0
#define ADC_MID 1023
#define ADC_MAX 2047

#define FILTER_SHIFT 3

enum ChannelOrder {
    AIL,
    ELE,
    THR,
    RUD,
    AUX1, // (CH5)  ARM switch for Expresslrs
    AUX2, // (CH6)  angel / airmode change
    AUX3, // (CH7)  flip after crash
    AUX4, // (CH8)
    AUX5, // (CH9)
    AUX6, // (CH10)
    AUX7, // (CH11)
    AUX8, // (CH12)
};

enum stickInputs {
    LEFTX,
    LEFTY,
    RIGHTX,
    RIGHTY,
    SW1,
    SW2,
    SW3,
};

struct HandsetSettings {
    uint8_t channelMode;
    bool adcFilter;
    uint16_t deadband;
    uint16_t auxLongPress;
    uint16_t voltageScale;
    bool throttleCentered;

    bool reverse[4];  
    uint8_t limit[4]; 
    uint8_t expo[4];  
};

class Handset {
public:
    HandsetSettings globalSettings;
    ParamCollection handsetSettingsMenu;

    void loadGlobalSettings(bool initialize); 
    void saveGlobalSettings();                
    void executeSettingsCommand();            
    void cancelSettingsCommand();             

private:
    bool addFolder(int idx, int parent, const char* label);
    bool addInfo(int idx, int parent, const char* label, const char* valueString);
    bool addChoiceParam(int idx, int parent, const char* label, const std::vector<const char*>& choices, int currVal, int min, int max);
    bool addNumericParam(int idx, int parent, crsfValueType type, const char* label, int currVal, int min, int max, int step, const char* units);
};