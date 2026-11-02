#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

// ==========================================
// 1. UART (UART0 on GP0/GP1)
// ==========================================
#define ELRS_UART       Serial1
#define ELRS_TX         0
#define ELRS_RX         1

// ==========================================
// 2. Digital Inputs (开发板原生引脚保持纯数字)
// ==========================================
const uint8_t DIGITAL_PIN_SWITCH_ARM  = 7;
const uint8_t DIGITAL_PIN_SWITCH_AUX2 = 8;
const uint8_t DIGITAL_PIN_BUTTON      = 6;

// ==========================================
// 3. Board LED: WS2812 on GP16
// ==========================================
const uint8_t DIGITAL_PIN_LED = 16;
const bool PIN_LED_LOW_ILLUM  = false;

// ==========================================
// 4. OLED (SSD1306 I2C) - 与 ADS1115 共用总线
// ==========================================
const uint8_t I2C_SDA_OLED = 4;
const uint8_t I2C_SCL_OLED = 5;

// ==========================================
// 5. RP2040 native ADC for the four stick axes
// Change these pin assignments when the final board wiring is decided.
// RP2040 ADC inputs are normally GPIO26..GPIO29.
// ==========================================
#define STICK_ADC_RIGHT_X_PIN 26
#define STICK_ADC_RIGHT_Y_PIN 27
#define STICK_ADC_LEFT_X_PIN  28
#define STICK_ADC_LEFT_Y_PIN  29

#define STICK_ADC_MIN 0
#define STICK_ADC_MAX 4095
#define STICK_ADC_MID 2048

// ==========================================
// 6. ADS1115 is retained for battery-voltage measurement only.
// This keeps the four latency-sensitive stick axes on the RP2040 ADC.
// ==========================================
#define ADS1115_ADDR        0x48
#define ADS1115_GAIN        GAIN_TWOTHIRDS
#define ADS1115_BATTERY_CHANNEL 0  // ADS1115 A0

#define BAT_R1              33000.0f
#define BAT_R2              10000.0f

#endif