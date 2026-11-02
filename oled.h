#pragma once
#ifndef OLED_H
#define OLED_H
//#define TIMINGDEBUG
/*
 * This file is part of ESP32C3-TX / RP2040 Port
 * Modified for RP2040 (I2C SDA: GPIO 4, SCL: GPIO 5)
 */
#include <Wire.h>
#include "elrs.h"
#include "config.h"
#include "params.h"
#include <U8g2lib.h>

// Two options for the OLED definition. 
//#define OLED_OPTION_1
#define OLED_OPTION_2

#ifdef OLED_OPTION_1
extern U8G2_SSD1306_72X40_ER_F_HW_I2C display;
 //----- OLED Screen setup  -------------------
const int OLED_X_OFFSET = 0; // Standard alignment for 0.42" 72x40 on 128px space
const int OLED_Y_OFFSET = 0; // Adjusted to shift the 40px matrix vertically if needed
#endif
#ifdef OLED_OPTION_2
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
 //----- OLED Screen setup  -------------------
const int OLED_X_OFFSET = 30; // Visible 72x40 area starts at X=30 in the 128x64 controller buffer
const int OLED_Y_OFFSET = 12; // Visible 72x40 area starts at Y=12 in the 128x64 controller buffer
#endif
const int OLED_WIDTH = 72;
const int OLED_HEIGHT = 40;

enum displayState {MAIN_PAGE, ELRS_STATS, ELRS_MENU, CHANNEL_OUTPUTS, CHANNEL_MENU, BT_JOYSTICK, MODEL_FINDER, CALIBRATION};
extern displayState currentScreen; 
extern displayState lastScreen;
enum menuMode { MENU_BROWSE, MENU_EDIT };
extern menuMode menuState;

void initDisplay();
void drawBootLogo();
void clearScreen();
void nextScreen();

static void updateLogicalDisplayArea(uint8_t logicalTileX, uint8_t logicalTileY, uint8_t logicalTileW, uint8_t logicalTileH);
void drawHomeScreen(float voltage, float receiverVoltage, bool receiverVoltageValid,
                    int telemetryState, uint8_t uplinkLQ, int16_t channels[],
                    int16_t minChannelValue, int16_t maxChannelValue);
void drawElrsStatsScreen(const ELRS& elrs);
void drawChannelOutputsScreen(int16_t channels[], int16_t min, int16_t max);
void drawBtJoystickScreen(bool btJoystickActive);

void drawCalibrationScreen(int stage, int secondsLeft);

void drawMenuScreen(const ParamCollection& menu);
void drawScrollingMenuValue(const char* text, int textX, int textY, int displayAreaWidth);
void drawMenuItem(const crsfParameter& menuItem, int offsetX, int offsetY);
void animateMenu(const ParamCollection& menu);
void drawEditValue(const crsfParameter& menuItem, int32_t editValue, int offsetX, int offsetY);
void animateEditValue(const ParamCollection& menu);
void drawScrollingPopupText(const char* text, int textX, int textY, int displayAreaWidth);
void drawPopup(const ParamCollection& menu);
int  navigateMenu(ParamCollection &menu, uint8_t direction);
void handleNavigation(int navigate, Handset& handset, ELRS& elrs, bool& toggleBtJoystick, bool& startCalibration );
void drawModelFinderScreen(int raw, int avg, int strength);

void drawMessage(char title[], char line1[], char line2[]);


// NanoTX Logo
// Dimensions: 72 x 40 pixels
const uint8_t nanotx_logo_72x40[] U8X8_PROGMEM = {
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00001100, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000010, 0b00001000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00001010, 0b00010000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00100101, 0b00100100, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b10010100, 0b00101001, 
  0b00000000, 0b11111100, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11010100, 0b00101001,
  0b10000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11000100, 0b00101000, 
  0b11000000, 0b00000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01100000, 0b00100100, 
  0b11100000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00110000, 0b00010000, 
  0b01100000, 0b00111000, 0b00000110, 0b00000000, 0b00000000, 0b11000000, 0b10011111, 0b00010001, 0b00000000,
  0b00110000, 0b01111000, 0b00000110, 0b00000000, 0b00000000, 0b00000000, 0b10000111, 0b00001011, 0b00100000, 
  0b00110000, 0b01111000, 0b11000110, 0b11001111, 0b10001111, 0b00001111, 0b00000111, 0b00001111, 0b01100000, 
  0b00110000, 0b10011000, 0b01100110, 0b11001100, 0b11011100, 0b00011000, 0b00000111, 0b00001110, 0b01100000, 
  0b00110000, 0b10011000, 0b01100111, 0b11001100, 0b11011000, 0b00011000, 0b00000111, 0b00011111, 0b01100000,
  0b00110000, 0b00011000, 0b01100111, 0b11001100, 0b11011000, 0b00011000, 0b10000111, 0b00011001, 0b00110000, 
  0b01100000, 0b00011000, 0b11000110, 0b11001111, 0b10011000, 0b00001111, 0b11000111, 0b00110000, 0b00110000, 
  0b11100000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00011000, 
  0b11000000, 0b00000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00001110,
  0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b00000111, 
  0b00000000, 0b11111100, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
};

const uint8_t bt_logo_8x10[] = { 0b00001000, 
                                 0b00011000, 
                                 0b00101010, 
                                 0b00011100, 
                                 0b00001000, 
                                 0b00011100, 
                                 0b00101010, 
                                 0b00011000, 
                                 0b00001000,
                                 0b00000000 };
#endif