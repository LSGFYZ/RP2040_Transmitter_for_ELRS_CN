/*
 * This file is part of ESP32C3-TX / RP2040 Port
 * Modified for RP2040. The main sketch configures the shared I2C bus pins.
 */
 //添加了回传电压显示，和电池电压循环显示，每次3秒，v在上为回传电压，v在下为电池电压.


#include "oled.h"
#include "locale_zh_cn.h"

displayState currentScreen = MAIN_PAGE;
displayState lastScreen;
menuMode menuState = MENU_BROWSE;

// Full GB2312 Chinese font is stored in flash by U8g2; it is not copied to RAM.
static inline void setChineseFont() { display.setFont(u8g2_font_wqy12_t_gb2312); }
static inline void drawLocalizedUtf8(int x, int y, const char* text) {
    setChineseFont();
    display.drawUTF8(x, y, translateTextZh(text));
}

// ⚠️ 关键修复：移除构造函数中的 clock/data 引脚参数
// 让 U8g2 直接使用主 .ino 中已初始化的默认 Wire 实例
// 传入引脚参数会导致 U8g2 内部重新配置 I2C，与主程序冲突导致屏幕不亮
#ifdef OLED_OPTION_1
U8G2_SSD1306_72X40_ER_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

// Convert the project's logical 72x40 tile area to the matching area
// inside this display's 128x64 controller buffer.
static void updateLogicalDisplayArea(uint8_t logicalTileX,
                                     uint8_t logicalTileY,
                                     uint8_t logicalTileW,
                                     uint8_t logicalTileH) {
    if (logicalTileW == 0 || logicalTileH == 0) return;

    const int xStart = OLED_X_OFFSET + logicalTileX * 8;
    const int yStart = OLED_Y_OFFSET + logicalTileY * 8;
    const int xEnd = OLED_X_OFFSET + (logicalTileX + logicalTileW) * 8 - 1;
    const int yEnd = OLED_Y_OFFSET + (logicalTileY + logicalTileH) * 8 - 1;

    const uint8_t physicalTileX = xStart / 8;
    const uint8_t physicalTileY = yStart / 8;
    const uint8_t physicalTileW = (xEnd / 8) - physicalTileX + 1;
    const uint8_t physicalTileH = (yEnd / 8) - physicalTileY + 1;

    display.updateDisplayArea(physicalTileX, physicalTileY,
                              physicalTileW, physicalTileH);
}

void initDisplay() {
    // ⚠️ 关键修复：删除所有 Wire 初始化代码
    // The main sketch configures and starts Wire before calling initDisplay().
    // Do not reinitialize the shared bus here; ADS1115 uses the same Wire bus.
    
    // Start U8g2
    display.begin();
    display.setContrast(255);
    display.setBusClock(400000); // Stable Fast-mode I2C shared by OLED and ADS1115
}

void drawBootLogo() {
    display.clearBuffer();          // Clear the display internal ram memory
    
    // Draw NanoTX logo  开机显示
    display.drawXBM(OLED_X_OFFSET, OLED_Y_OFFSET, 72, 40, nanotx_logo_72x40);

    display.setFont(u8g2_font_tiny5duo_tf); 
    display.setCursor(OLED_X_OFFSET + 2, OLED_Y_OFFSET + 36);
    display.print("ELRS+USB");
    
    display.setCursor(OLED_X_OFFSET + 50, OLED_Y_OFFSET + 36);
    display.print("v3.0");
    
    display.sendBuffer();     
}

void clearScreen() {
    display.clearBuffer();          // Clear the display internal ram memory
    display.sendBuffer(); 
}

void nextScreen() {
    switch (currentScreen) {
        case MAIN_PAGE:       currentScreen = ELRS_STATS;      break;
        case ELRS_STATS:      currentScreen = CHANNEL_OUTPUTS; break;
        case CHANNEL_OUTPUTS: currentScreen = BT_JOYSTICK;     break;
        case BT_JOYSTICK:     currentScreen = MODEL_FINDER;    break;
        case MODEL_FINDER:    currentScreen = MAIN_PAGE;       break; // Wrap around
    }
}

void drawHomeScreen(float voltage, float receiverVoltage, bool receiverVoltageValid,
                    int telemetryState, uint8_t uplinkLQ, int16_t channels[],
                    int16_t minChannelValue, int16_t maxChannelValue) {
    // C3 convention: local battery "v" is lower; receiver-returned "v" is raised.
    // Alternate between the two sources every three seconds.
    static uint32_t lastVoltagePageSwitchMs = 0;
    static bool showReceiverVoltage = false;
    const uint32_t voltageNow = millis();
    if (lastVoltagePageSwitchMs == 0) lastVoltagePageSwitchMs = voltageNow;
    if ((uint32_t)(voltageNow - lastVoltagePageSwitchMs) >= 3000) {
        showReceiverVoltage = !showReceiverVoltage;
        lastVoltagePageSwitchMs = voltageNow;
    }
    const bool displayingReceiverVoltage = showReceiverVoltage;
    const bool displayedVoltageValid = !displayingReceiverVoltage || receiverVoltageValid;
    const float displayedVoltage = displayingReceiverVoltage ? receiverVoltage : voltage;

    static int lastVoltageX10 = -99;
    static int lastTelemetryState = -1;
    static int lastBars = 0;
    static int16_t lastAux1 = -1;
    static int16_t lastAux2 = -1;
    static int16_t lastAux3 = -1;

#ifdef TIMINGDEBUG
    uint32_t processTime;
    uint32_t transmitTime;
    uint32_t startTime = micros();
#endif
    // Clear only the top half (this is all that will be sent)
    display.setDrawColor(0);
    display.drawBox(OLED_X_OFFSET, OLED_Y_OFFSET, 72, 17);

    // Ensure draw color is set to white/on
    display.setDrawColor(1);

    // ==========================================
    // 1. TOP LEFT QUADRANT: LARGE VOLTAGE TEXT
    // ==========================================
    display.setFont(u8g2_font_logisoso16_tf);
    display.setCursor(6 + OLED_X_OFFSET, 16 + OLED_Y_OFFSET);
    if (displayedVoltageValid) {
        display.print(displayedVoltage, 1);
    } else {
        display.print("-.-");
    }

    // C3 original fraction-style voltage marker:
    // the horizontal divider is always visible; upper "v" = receiver voltage,
    // lower "v" = handset battery voltage. The divider never moves.
    const int voltageMarkerX = display.getCursorX() + 2;
    display.drawHLine(voltageMarkerX, 8 + OLED_Y_OFFSET, 5);//V字分界线

    display.setFont(u8g2_font_tiny5duo_tf);
    display.setCursor(voltageMarkerX,
                      (displayingReceiverVoltage ? 6 : 15) + OLED_Y_OFFSET);
    display.print("v");

    // ==========================================
    // 2. TOP RIGHT QUADRANT: LARGE SIGNAL BARS
    // ==========================================
    uint8_t bars = 0;
    if (telemetryState == 1) {
        if (uplinkLQ > 75) bars = 4;
        else if (uplinkLQ > 50) bars = 3;
        else if (uplinkLQ > 25) bars = 2;
        else if (uplinkLQ > 0)  bars = 1;
        
        if (bars >= 1) display.drawBox(45 + OLED_X_OFFSET, 11 + OLED_Y_OFFSET, 3, 3);   
        if (bars >= 2) display.drawBox(52 + OLED_X_OFFSET, 8 + OLED_Y_OFFSET,  3, 6);   
        if (bars >= 3) display.drawBox(59 + OLED_X_OFFSET, 4 + OLED_Y_OFFSET,  3, 10);  
        if (bars >= 4) display.drawBox(66 + OLED_X_OFFSET, 0 + OLED_Y_OFFSET,  3, 14);  
    } else if (telemetryState == 0) {
        display.drawBox(47 + OLED_X_OFFSET, 6 + OLED_Y_OFFSET, 8, 2); 
        display.drawBox(59 + OLED_X_OFFSET, 6 + OLED_Y_OFFSET, 8, 2);
    } else if (telemetryState == 4) { 
        display.setFont(u8g2_font_open_iconic_embedded_2x_t); 
        display.drawStr(OLED_X_OFFSET + 47, OLED_Y_OFFSET + 16, "J");
    }
    if (telemetryState == 2 || telemetryState == 3) { 
        display.drawXBM(45 + OLED_X_OFFSET, 0 + OLED_Y_OFFSET, 8, 10, bt_logo_8x10);
    }
    uint8_t tileX = 0;
    uint8_t tileY = 0;
    uint8_t tileW = 9;
    uint8_t tileH = 2;

    const int displayedVoltageX10 = displayedVoltageValid ? (int)(displayedVoltage * 10.0f) : -999;
    if (displayedVoltageX10 == lastVoltageX10 && currentScreen == lastScreen) {
        tileX = 4;
        tileW = 5;
    }
    if (lastBars == bars && telemetryState == lastTelemetryState && currentScreen == lastScreen){
        tileW -= 5;
    } 
#ifdef TIMINGDEBUG
    processTime = micros() - startTime;
#endif

display.sendBuffer();
//#ifdef OLED_OPTION_1
//    if (tileW > 0) display.updateDisplayArea(tileX,tileY,tileW,tileH);
//#else
 //   if (tileW > 0) updateLogicalDisplayArea(tileX,tileY,tileW,tileH);
//#endif

#ifdef TIMINGDEBUG
    transmitTime = micros() - (startTime + processTime);
    Serial.print("Top Half Process Time: "); Serial.print(processTime); Serial.print("uS. Send time: "); Serial.print(transmitTime); Serial.println("uS.");
#endif
    lastVoltageX10 = displayedVoltageX10;
    lastBars = bars;
    lastTelemetryState = telemetryState;

    // ==========================================
    // 2. CENTER ROW: GIMBAL STICK CROSSHAIR BOXES
    // ==========================================
    const uint8_t boxSize = 22;
    const uint8_t leftBoxX = 12;
    const uint8_t rightBoxX = 40;
    const uint8_t boxesY = 18;

    display.setDrawColor(0);
    display.drawBox(0+OLED_X_OFFSET, 17+OLED_Y_OFFSET, 72, 22); 
    display.setDrawColor(1);

    display.drawFrame(leftBoxX + OLED_X_OFFSET, boxesY + OLED_Y_OFFSET, boxSize, boxSize);
    display.drawFrame(rightBoxX + OLED_X_OFFSET, boxesY + OLED_Y_OFFSET, boxSize, boxSize);
    
    uint8_t leftX = map(channels[0], minChannelValue, maxChannelValue, leftBoxX + 1, leftBoxX + boxSize - 2); 
    uint8_t leftY = map(channels[1], minChannelValue, maxChannelValue, boxesY + boxSize - 2, boxesY + 1);      
    
    uint8_t rightX = map(channels[2], minChannelValue, maxChannelValue, rightBoxX + 1, rightBoxX + boxSize - 2); 
    uint8_t rightY = map(channels[3], minChannelValue, maxChannelValue, boxesY + boxSize - 2, boxesY + 1);       
    
    display.drawBox(leftX - 1 + OLED_X_OFFSET, leftY - 1 + OLED_Y_OFFSET, 2, 2);
    display.drawBox(rightX - 1 + OLED_X_OFFSET, rightY - 1 + OLED_Y_OFFSET, 2, 2);

    // ==========================================
    // 3. FLANKS: AUX SLIDERS
    // ==========================================
    display.drawLine(4 + OLED_X_OFFSET, 18 + OLED_Y_OFFSET, 4 + OLED_X_OFFSET, 38 + OLED_Y_OFFSET);
    uint8_t aux1Y = (channels[4]==0) ? 36 : 16; 
    display.drawBox(2 + OLED_X_OFFSET, aux1Y + OLED_Y_OFFSET, 5, 3);

    display.drawLine(68 + OLED_X_OFFSET, 18 + OLED_Y_OFFSET, 68 + OLED_X_OFFSET, 26 + OLED_Y_OFFSET);
    uint8_t aux2Y = (channels[5]==0) ? 24 : 16; 
    display.drawBox(66 + OLED_X_OFFSET, aux2Y + OLED_Y_OFFSET, 5, 3);

    display.drawLine(68 + OLED_X_OFFSET, 30 + OLED_Y_OFFSET, 68 + OLED_X_OFFSET, 38 + OLED_Y_OFFSET);
    uint8_t aux3Y =(channels[6]==0) ? 36 : 28; 
    display.drawBox(66 + OLED_X_OFFSET, aux3Y + OLED_Y_OFFSET, 5, 3);

    tileX = 0;
    tileY = 2;
    tileW = 9;
    tileH = 3;
    if (channels[4]==lastAux1 && currentScreen == lastScreen) { 
        tileX += 1;
        tileW -= 1;
    }
    if (channels[5]==lastAux2 && channels[6]==lastAux3 && currentScreen == lastScreen) tileW -= 1; 

#ifdef TIMINGDEBUG
    processTime = micros() - startTime;
#endif
#ifdef OLED_OPTION_1
    display.updateDisplayArea(tileX,tileY,tileW,tileH);
#else
    updateLogicalDisplayArea(tileX,tileY,tileW,tileH);
#endif
#ifdef TIMINGDEBUG
    transmitTime = micros() - (startTime + processTime);
    Serial.print("Stick Pos Process Time: "); Serial.print(processTime); Serial.print("uS. Send time: "); Serial.print(transmitTime); Serial.println("uS.");
#endif

    lastAux1 = channels[4];
    lastAux2 = channels[5];
    lastAux3 = channels[6];
    lastScreen = currentScreen;
}

void drawElrsStatsScreen(const ELRS& elrs) {
    display.clearBuffer();

    display.setFont(u8g2_font_squeezed_b7_tr); 
    display.setCursor(OLED_X_OFFSET + 2, OLED_Y_OFFSET + 8);
    if (strlen(elrs.txModule.name) > 0) {
        display.print(elrs.txModule.name);
        setChineseFont();
        display.setCursor(OLED_X_OFFSET + 2, OLED_Y_OFFSET + 22);

        if (elrs.ready()) {
            int index = elrs.txModule.params.findParamByLabel(LABEL_PACKET_RATE);
            if (index >= 0) {
                const crsfParameter &param = elrs.txModule.params.getParam(index);
                display.drawUTF8(display.getCursorX(), display.getCursorY(), translateTextZh(param.choices[param.currentVal].text)); 
            } else {
                display.drawUTF8(display.getCursorX(), display.getCursorY(), "分包速率未知"); 
            }
            display.setCursor(OLED_X_OFFSET + 2, OLED_Y_OFFSET + 36);
            index = elrs.txModule.params.findParamByLabel(LABEL_TX_POWER_STRING);
            if (index >= 0) {
                const crsfParameter &param = elrs.txModule.params.getParam(index);
                display.drawUTF8(display.getCursorX(), display.getCursorY(), translateTextZh(param.label)); 
            } else {
                display.drawUTF8(display.getCursorX(), display.getCursorY(), "发射功率未知"); 
            }

        } else {
            display.drawUTF8(display.getCursorX(), display.getCursorY(), "正在加载参数");
        }

    } else {
        setChineseFont();
        display.drawUTF8(display.getCursorX(), display.getCursorY(), "未检测到模块");
        display.setFont(u8g2_font_streamline_interface_essential_circle_triangle_t);
        display.setCursor(OLED_X_OFFSET + 25, OLED_Y_OFFSET + 38);   
        display.print("3");      
    }

    display.sendBuffer();
    lastScreen = currentScreen;
}


void drawChannelOutputsScreen(int16_t channels[], int16_t min, int16_t max) {
    display.clearBuffer();
    setChineseFont();
    display.drawUTF8(OLED_X_OFFSET + 0, OLED_Y_OFFSET + 10, "通道输出");

    const char* labels[] = {"A", "E", "T", "R"};
    const int16_t startX = OLED_X_OFFSET + 10;                     
    const int16_t endX = OLED_X_OFFSET + 70;                       
    const int16_t midX = startX + (endX - startX) / 2; 
    const int16_t barHeight = 3;                   
    
    for (uint8_t i = 0; i < 4; i++) {
        int16_t y = OLED_Y_OFFSET + 14 + (i * 8); 
        
        display.drawStr(OLED_X_OFFSET + 0, y + 3, labels[i]);
        
        display.drawHLine(startX, y, endX - startX + 1);
        display.drawVLine(startX, y - 2, 5);
        display.drawVLine(midX, y - 2, 5);
        display.drawVLine(endX, y - 2, 5);
        
        int16_t currentVal = channels[i];
        if (currentVal < min) currentVal = min;
        if (currentVal > max) currentVal = max;
        
        int16_t currentX = startX + (int32_t)(currentVal - min) * (endX - startX) / (max - min);
        
        if (currentX > midX) {
            display.drawBox(midX, y - 1, currentX - midX + 1, barHeight);
        } else if (currentX < midX) {
            display.drawBox(currentX, y - 1, midX - currentX + 1, barHeight);
        }
    }
    display.sendBuffer();
    lastScreen = currentScreen;
}

void drawBtJoystickScreen(bool btJoystickActive) {
    display.clearBuffer();
    display.setFont(u8g2_font_open_iconic_embedded_2x_t); 
    display.drawStr(OLED_X_OFFSET + (OLED_WIDTH/2) - 16, OLED_Y_OFFSET + (OLED_HEIGHT/2) + 12, "J");

    setChineseFont();
    display.drawUTF8(OLED_X_OFFSET + 3, OLED_Y_OFFSET + 10, "蓝牙摇杆");
    if (btJoystickActive) {
        display.drawUTF8(OLED_X_OFFSET + (OLED_WIDTH/2) + 8, OLED_Y_OFFSET + (OLED_HEIGHT/2) + 8, "开启");
    } else {
        display.drawUTF8(OLED_X_OFFSET + (OLED_WIDTH/2) + 8, OLED_Y_OFFSET + (OLED_HEIGHT/2) + 8, "关闭");
    }

    display.sendBuffer();
    lastScreen = currentScreen;
}

const unsigned long STATE_START_PAD  = 1000; 
const unsigned long STATE_SCROLL     = 1000;  
const unsigned long STATE_END_PAD    = 500;  
const unsigned long TOTAL_CYCLE      = STATE_START_PAD + STATE_SCROLL + STATE_END_PAD;
const int MAX_TEXT_LENGTH            = 64; 

void drawScrollingMenuValue(const char* text, int textX, int textY, int displayAreaWidth) {
    const char* shownText = translateTextZh(text);
    setChineseFont();
    int textWidth = display.getUTF8Width(shownText);

    unsigned long currentMillis = millis();
    static unsigned long lastResetTime = 0;
    static char lastText[MAX_TEXT_LENGTH] = "";
    enum ScrollState {START,SCROLLING,FINISH};
    static ScrollState currentState = START;

    if (strcmp(text, lastText) != 0) {
        strncpy(lastText, text, sizeof(lastText) - 1);
        lastText[sizeof(lastText) - 1] = '\0'; 
        currentState = START;
        lastResetTime = currentMillis;
    }
    
    if (textWidth <= displayAreaWidth) {
        int centerX = textX + (displayAreaWidth - textWidth) / 2;
        display.drawUTF8(centerX, textY, shownText);
        return;
    }

    if (currentMillis - lastResetTime >= TOTAL_CYCLE) {
        lastResetTime = currentMillis;
    }
    
    unsigned long progress = currentMillis - lastResetTime;
    int maxScrollDistance = textWidth - displayAreaWidth;
    int currentXOffset = 0;

    if (progress < STATE_START_PAD) {
        currentXOffset = 0;
    } 
    else if (progress < (STATE_START_PAD + STATE_SCROLL)) {
        unsigned long scrollProgress = progress - STATE_START_PAD;
        currentXOffset = (maxScrollDistance * scrollProgress) / STATE_SCROLL;
    } 
    else {
        currentXOffset = maxScrollDistance;
    }

    display.drawUTF8(textX - currentXOffset, textY, shownText);
}

void drawMenuItem(const crsfParameter& menuItem, int offsetX, int offsetY) {
    int textWidth;
    int centerX;
    float floatVal;
    static const int tenToThePowerOf[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
    char buffer[32];
    display.setFont(u8g2_font_squeezed_b7_tr); 
    switch (menuItem.type) {
        case CRSF_FOLDER: 
            display.setClipWindow(OLED_X_OFFSET+5, OLED_Y_OFFSET+5, OLED_X_OFFSET+OLED_WIDTH-7, OLED_Y_OFFSET+OLED_HEIGHT-5);
            drawScrollingMenuValue(menuItem.label, OLED_X_OFFSET + 6 + offsetX, OLED_Y_OFFSET + 23 + offsetY, 62);
            break;
        case CRSF_COMMAND:
        case CRSF_INFO:
            display.setClipWindow(OLED_X_OFFSET+5, OLED_Y_OFFSET+5, OLED_X_OFFSET+OLED_WIDTH, OLED_Y_OFFSET+OLED_HEIGHT-5);
            drawScrollingMenuValue(menuItem.label, OLED_X_OFFSET + 6 + offsetX, OLED_Y_OFFSET + 23 + offsetY, 67);
            break;
        case CRSF_UINT8:
        case CRSF_INT8:
        case CRSF_UINT16:
        case CRSF_INT16:
        case CRSF_UINT32:
        case CRSF_INT32:
        case CRSF_UINT64:
        case CRSF_INT64:
            display.setClipWindow(OLED_X_OFFSET+5, OLED_Y_OFFSET+5, OLED_X_OFFSET+OLED_WIDTH, OLED_Y_OFFSET+OLED_HEIGHT-5);
            setChineseFont();
            textWidth = display.getUTF8Width(translateTextZh(menuItem.label));
            centerX = (OLED_WIDTH - textWidth) / 2;
            display.drawUTF8(OLED_X_OFFSET + centerX + offsetX, OLED_Y_OFFSET + 15 + offsetY, translateTextZh(menuItem.label));
            setChineseFont();
            snprintf(buffer, sizeof(buffer), "%d%s", (int32_t)menuItem.currentVal, menuItem.units);
            drawScrollingMenuValue(buffer, OLED_X_OFFSET + 6 + offsetX, OLED_Y_OFFSET + 30 + offsetY, 62);
            break;            
        case CRSF_FLOAT:
            display.setClipWindow(OLED_X_OFFSET+5, OLED_Y_OFFSET+5, OLED_X_OFFSET+OLED_WIDTH, OLED_Y_OFFSET+OLED_HEIGHT-5);
            setChineseFont();
            textWidth = display.getUTF8Width(translateTextZh(menuItem.label));
            centerX = (OLED_WIDTH - textWidth) / 2;
            display.drawUTF8(OLED_X_OFFSET + centerX + offsetX, OLED_Y_OFFSET + 15 + offsetY, translateTextZh(menuItem.label));
            setChineseFont();                
            floatVal = (float) menuItem.currentVal / tenToThePowerOf[menuItem.precision];
            snprintf(buffer, sizeof(buffer), "%.*f%s", menuItem.precision, floatVal, menuItem.units);
            drawScrollingMenuValue(buffer, OLED_X_OFFSET + 6 + offsetX, OLED_Y_OFFSET + 30 + offsetY, 62);
            break;
        case CRSF_TEXT_SELECTION:
            display.setClipWindow(OLED_X_OFFSET+5, OLED_Y_OFFSET+5, OLED_X_OFFSET+OLED_WIDTH, OLED_Y_OFFSET+OLED_HEIGHT-5);
            setChineseFont();
            textWidth = display.getUTF8Width(translateTextZh(menuItem.label));
            centerX = (OLED_WIDTH - textWidth) / 2;
            display.drawUTF8(OLED_X_OFFSET + centerX + offsetX, OLED_Y_OFFSET + 15 + offsetY, translateTextZh(menuItem.label));
            setChineseFont();                
            drawScrollingMenuValue(menuItem.choices[menuItem.currentVal].text, OLED_X_OFFSET + 6 + offsetX, OLED_Y_OFFSET + 30 + offsetY, 62);
            break;   
    }
    display.setMaxClipWindow(); 
}

static uint8_t activeMenuAnimation = 0;   
static unsigned long menuAnimStartTime = 0;
const unsigned long ANIM_DURATION = 500; 
static int prevMenuItemIndex = 0;   
static int currentMenuItemIndex = 0;

void animateMenu(const ParamCollection& menu) {
    if (activeMenuAnimation == 0) {
        drawMenuItem(menu.getCurrentParam(), 0, 0);
    } 
    else {
        unsigned long elapsed = millis() - menuAnimStartTime;
        
        if (elapsed >= ANIM_DURATION) {
            activeMenuAnimation = 0;
            drawMenuItem(menu.getCurrentParam(), 0, 0);
        } 
        else {
            long progress = (elapsed * 1000) / ANIM_DURATION; 
            int offsetX_Prev = 0, offsetY_Prev = 0;
            int offsetX_Curr = 0, offsetY_Curr = 0;

            switch (activeMenuAnimation) {
                case 1: 
                    offsetY_Prev = - (OLED_HEIGHT * progress) / 1000;
                    offsetY_Curr = OLED_HEIGHT - (OLED_HEIGHT * progress) / 1000;
                    break;
                case 2: 
                    offsetY_Prev = (OLED_HEIGHT * progress) / 1000;
                    offsetY_Curr = -OLED_HEIGHT + (OLED_HEIGHT * progress) / 1000;
                    break;
                case 3: 
                    offsetX_Prev = - (OLED_WIDTH * progress) / 1000;
                    offsetX_Curr = OLED_WIDTH - (OLED_WIDTH * progress) / 1000;
                    break;
                case 4: 
                    offsetX_Prev = (OLED_WIDTH * progress) / 1000;
                    offsetX_Curr = -OLED_WIDTH + (OLED_WIDTH * progress) / 1000;
                    break;
            }

            drawMenuItem(menu.getParam(prevMenuItemIndex), offsetX_Prev, offsetY_Prev);
            drawMenuItem(menu.getParam(currentMenuItemIndex), offsetX_Curr, offsetY_Curr);
        }
    }    
}

void drawEditValue(const crsfParameter& menuItem, int32_t editValue, int offsetX, int offsetY) {
    int textWidth;
    int centerX;
    float floatVal;
    static const int tenToThePowerOf[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
    char buffer[32];
    setChineseFont(); 
    switch (menuItem.type) {
        case CRSF_UINT8:
        case CRSF_INT8:
        case CRSF_UINT16:
        case CRSF_INT16:
        case CRSF_UINT32:
        case CRSF_INT32:
        case CRSF_UINT64:
        case CRSF_INT64:
        case CRSF_FLOAT:
            display.setClipWindow(OLED_X_OFFSET+1, OLED_Y_OFFSET+22, OLED_X_OFFSET+OLED_WIDTH-1, OLED_Y_OFFSET+OLED_HEIGHT-7);

            if (menuItem.precision > 0) {
                floatVal = (float) editValue / tenToThePowerOf[menuItem.precision];
                snprintf(buffer, sizeof(buffer), "%.*f%s", menuItem.precision, floatVal, menuItem.units);
            } 
            else {
                snprintf(buffer, sizeof(buffer), "%d%s", (int32_t)editValue, menuItem.units);
            }
            textWidth = display.getUTF8Width(buffer);
            centerX = (OLED_WIDTH - textWidth) / 2;
            display.drawUTF8(OLED_X_OFFSET + centerX + offsetX, OLED_Y_OFFSET + 31 + offsetY, buffer);
            break;            

        case CRSF_TEXT_SELECTION:
            display.setClipWindow(OLED_X_OFFSET+10, OLED_Y_OFFSET+15, OLED_X_OFFSET+OLED_WIDTH-10, OLED_Y_OFFSET+OLED_HEIGHT-1);
            snprintf(buffer, sizeof(buffer), "%s%s", translateTextZh(menuItem.choices[editValue].text), menuItem.units);
            textWidth = display.getUTF8Width(buffer);
            centerX = (OLED_WIDTH - textWidth) / 2;
            display.drawUTF8(OLED_X_OFFSET + centerX + offsetX, OLED_Y_OFFSET + 31 + offsetY, buffer);
            break;   
    }
    display.setMaxClipWindow(); 
}

static uint8_t activeEditAnimation = 0;   
static unsigned long editAnimStartTime = 0;
static int prevEditValue = 0;   
static int currentEditValue = 0;

void animateEditValue(const ParamCollection& menu) {
    if (activeEditAnimation == 0) {
        drawEditValue(menu.getCurrentParam(), menu.editValue, 0, 0);
    } 
    else {
        unsigned long elapsed = millis() - editAnimStartTime;
        
        if (elapsed >= ANIM_DURATION) {
            activeEditAnimation = 0;
            drawEditValue(menu.getCurrentParam(), menu.editValue, 0, 0);
        } 
        else {
            long progress = (elapsed * 1000) / ANIM_DURATION; 
            int offsetX_Prev = 0, offsetY_Prev = 0;
            int offsetX_Curr = 0, offsetY_Curr = 0;

            switch (activeEditAnimation) {
                case 1: 
                    offsetY_Prev = - (OLED_HEIGHT * progress) / 1000;
                    offsetY_Curr = OLED_HEIGHT - (OLED_HEIGHT * progress) / 1000;
                    break;
                case 2: 
                    offsetY_Prev = (OLED_HEIGHT * progress) / 1000;
                    offsetY_Curr = -OLED_HEIGHT + (OLED_HEIGHT * progress) / 1000;
                    break;
                case 3: 
                    offsetX_Prev = - (OLED_WIDTH * progress) / 1000;
                    offsetX_Curr = OLED_WIDTH - (OLED_WIDTH * progress) / 1000;
                    break;
                case 4: 
                    offsetX_Prev = (OLED_WIDTH * progress) / 1000;
                    offsetX_Curr = -OLED_WIDTH + (OLED_WIDTH * progress) / 1000;
                    break;
            }

            drawEditValue(menu.getCurrentParam(), prevEditValue, offsetX_Prev, offsetY_Prev);
            drawEditValue(menu.getCurrentParam(), currentEditValue, offsetX_Curr, offsetY_Curr);
        }
    }    
}

void drawScrollingPopupText(const char* text, int textX, int textY, int displayAreaWidth) {
    const char* shownText = translateTextZh(text);
    setChineseFont();
    int textWidth = display.getUTF8Width(shownText);

    unsigned long currentMillis = millis();
    static unsigned long lastResetTime = 0;
    static char lastText[MAX_TEXT_LENGTH] = "";
    enum ScrollState {START,SCROLLING,FINISH};
    static ScrollState currentState = START;

    if (strcmp(text, lastText) != 0) {
        strncpy(lastText, text, sizeof(lastText) - 1);
        lastText[sizeof(lastText) - 1] = '\0'; 
        currentState = START;
        lastResetTime = currentMillis;
    }
    
    if (textWidth <= displayAreaWidth) {
        int centerX = textX + (displayAreaWidth - textWidth) / 2;
        display.drawUTF8(centerX, textY, shownText);
        return;
    }

    if (currentMillis - lastResetTime >= TOTAL_CYCLE) {
        lastResetTime = currentMillis;
    }
    
    unsigned long progress = currentMillis - lastResetTime;
    int maxScrollDistance = textWidth - displayAreaWidth;
    int currentXOffset = 0;

    if (progress < STATE_START_PAD) {
        currentXOffset = 0;
    } 
    else if (progress < (STATE_START_PAD + STATE_SCROLL)) {
        unsigned long scrollProgress = progress - STATE_START_PAD;
        currentXOffset = (maxScrollDistance * scrollProgress) / STATE_SCROLL;
    } 
    else {
        currentXOffset = maxScrollDistance;
    }

    display.drawUTF8(textX - currentXOffset, textY, shownText);
}

void drawPopup(const ParamCollection& menu) {
    if (menu.popupActive()) {
        display.setDrawColor(0);
        display.drawBox(3+OLED_X_OFFSET, 3+OLED_Y_OFFSET, OLED_WIDTH-6, OLED_HEIGHT-6); 
        display.setDrawColor(1);
        display.drawFrame(3+OLED_X_OFFSET, 3+OLED_Y_OFFSET, OLED_WIDTH-6, OLED_HEIGHT-6);

        display.setClipWindow(5+OLED_X_OFFSET, 5+OLED_Y_OFFSET, OLED_X_OFFSET+OLED_WIDTH-10, OLED_Y_OFFSET+OLED_HEIGHT-10);
        drawScrollingPopupText(menu.getCurrentParam().valueString, OLED_X_OFFSET + 5, OLED_Y_OFFSET + 17, OLED_WIDTH-10);
        
        switch (menu.getCurrentParam().step) {
            case ELRS_COMMAND_EXECUTING:
                drawLocalizedUtf8(7+OLED_X_OFFSET, OLED_Y_OFFSET+OLED_HEIGHT-12, "[Running...]");
                break;
            case ELRS_COMMAND_ASKCONFIRM:
                drawLocalizedUtf8(10+OLED_X_OFFSET, OLED_Y_OFFSET+OLED_HEIGHT-12, "[Confirm?]");
                break;
        }
    }
}

int navigateMenu(ParamCollection &menu, uint8_t direction) {
    uint8_t nextMenuAnimation;
    uint8_t nextEditAnimation;
    int returnVal = 0;
    if (direction == 0) return 0;
    if (menu.popupActive()) {
        switch (direction) {
            case 5:
                return 2;  
                break;   
            case 6:
                return -1; 
                break;   
        }
    }

    switch (menuState) {
        case MENU_BROWSE:
            nextMenuAnimation = 0;
            switch (direction) {
                case 1:
                    nextMenuAnimation = menu.prevInFolder();
                    break;
                case 2:
                    nextMenuAnimation = menu.nextInFolder();
                    break;
                case 3:
                    nextMenuAnimation = menu.exitFolder();
                    break;
                case 4:
                    nextMenuAnimation = menu.enterFolder();
                    break;
                case 5:
                    switch (menu.getCurrentParam().type) {
                        case CRSF_COMMAND:
                            returnVal = 2;
                            break;
                        case CRSF_UINT8:
                        case CRSF_INT8:
                        case CRSF_UINT16:
                        case CRSF_INT16:
                        case CRSF_UINT32:
                        case CRSF_INT32:
                        case CRSF_UINT64:
                        case CRSF_INT64:
                        case CRSF_FLOAT:
                        case CRSF_TEXT_SELECTION:
                        case CRSF_STRING:
                            menu.editParam();
                            menuState = MENU_EDIT;  
                            break;
                        default:
                            break;   
                    }
                    break;
                case 6:
                    menu.selectedParam = 0;
                    returnVal = -1;  
                    break;
            }
            if (nextMenuAnimation != 0) {
                activeMenuAnimation = nextMenuAnimation;
                menuAnimStartTime = millis(); 
                prevMenuItemIndex = currentMenuItemIndex;
                currentMenuItemIndex = menu.selectedParam;
            }            
            break;

        case MENU_EDIT:
            nextEditAnimation = 0;
            switch (direction) {
                case 1:
                    if (menu.getCurrentParam().type != CRSF_TEXT_SELECTION) nextEditAnimation = menu.editParamInc();
                    break;
                case 2:
                    if (menu.getCurrentParam().type != CRSF_TEXT_SELECTION) nextEditAnimation = menu.editParamDec();
                    break;
                case 3:
                    if (menu.getCurrentParam().type == CRSF_TEXT_SELECTION) nextEditAnimation = menu.editParamDec();
                    break;
                case 4:
                    if (menu.getCurrentParam().type == CRSF_TEXT_SELECTION) nextEditAnimation = menu.editParamInc();
                    break;
                case 5:
                    returnVal = 1;  
                    menuState = MENU_BROWSE;
                    break;
                case 6:
                    menu.editParam(); 
                    menuState = MENU_BROWSE;  
                    break;
            }
            if (nextEditAnimation != 0) {
                activeEditAnimation = nextEditAnimation;
                editAnimStartTime = millis(); 
                prevEditValue = currentEditValue;
                currentEditValue = menu.editValue;
            }       
            break;
    }

    return returnVal;
}

void handleNavigation(int navigate, Handset& handset, ELRS& elrs, bool& toggleBtJoystick, bool& startCalibration ) {
    if (navigate == 0) return;

    if (navigate == 7) {
        switch (currentScreen) {
            case MAIN_PAGE:
                startCalibration = true;
                break;
            case ELRS_STATS:
                currentScreen = ELRS_MENU;
                break;
            case CHANNEL_OUTPUTS: 
                currentScreen = CHANNEL_MENU;
                break;
            case BT_JOYSTICK: 
                toggleBtJoystick = !toggleBtJoystick;
                break;
        }
    } else {
        if (currentScreen == ELRS_MENU) {
            int result = navigateMenu(elrs.txModule.params, navigate);
            switch (result) {
                case -1: 
                    if (elrs.popup()) {
                        elrs.cancelCommand();
                    } 
                    else {
                        currentScreen = ELRS_STATS;  
                    }
                    break;
                case 1: 
                    elrs.editParamSave();
                    break;
                case 2:  
                    elrs.executeCommand();
                    break;
                default: 
                    break;
            }
        } else if (currentScreen == CHANNEL_MENU) {
            int result = navigateMenu(handset.handsetSettingsMenu, navigate);
            switch (result) {
                case -1:
                    currentScreen = CHANNEL_OUTPUTS;  
                    break;
                case 1:
                    handset.saveGlobalSettings();
                    break;
                case 2:  
                    handset.executeSettingsCommand();
                    break;                        
                default: 
                    break;
            }
        } else {
            if (navigate == 6) {
                nextScreen();  
            }
        }
    }   
}

void drawMenuScreen(const ParamCollection& menu) {
    static int menuIndex;
    static const int tenToThePowerOf[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
    float floatVal;
    const crsfParameter &param = menu.getCurrentParam();

    display.clearBuffer();
    display.setFont(u8g2_font_NokiaSmallPlain_tr);    
    
    switch (menuState) {
        case MENU_EDIT:
            setChineseFont();
            display.drawUTF8(OLED_X_OFFSET + 3, OLED_Y_OFFSET + 12, translateTextZh(param.label));
            
            display.drawFrame(OLED_X_OFFSET, OLED_Y_OFFSET + 14, 72, 26);

            if (menu.editValue > param.minVal) {   
                if (param.type == CRSF_TEXT_SELECTION) {  
                    display.setDrawColor(0);  
                    display.drawBox(OLED_X_OFFSET, OLED_Y_OFFSET + 21, 6, 11);
                    display.setDrawColor(1);
                    display.drawTriangle(OLED_X_OFFSET + 5, OLED_Y_OFFSET + 22,
                                         OLED_X_OFFSET + 5, OLED_Y_OFFSET + 30,
                                         OLED_X_OFFSET    , OLED_Y_OFFSET + 26);
                } else {                                 
                    display.setDrawColor(0);  
                    display.drawBox(OLED_X_OFFSET + 30, OLED_Y_OFFSET + 35, 11, 6);
                    display.setDrawColor(1);
                    display.drawTriangle(OLED_X_OFFSET + 31, OLED_Y_OFFSET + 34,
                                         OLED_X_OFFSET + 39, OLED_Y_OFFSET + 34,
                                         OLED_X_OFFSET + 35, OLED_Y_OFFSET + 39);
                }
            }
            if (menu.editValue < param.maxVal) { 
                if (param.type == CRSF_TEXT_SELECTION) {  
                    display.setDrawColor(0);  
                    display.drawBox(OLED_X_OFFSET + 66, OLED_Y_OFFSET + 21, 6, 11);
                    display.setDrawColor(1);                    
                    display.drawTriangle(OLED_X_OFFSET + 67, OLED_Y_OFFSET + 22,
                                         OLED_X_OFFSET + 67, OLED_Y_OFFSET + 30,
                                         OLED_X_OFFSET + 72, OLED_Y_OFFSET + 26);
                } else {                                  
                    display.setDrawColor(0);  
                    display.drawBox(OLED_X_OFFSET + 30, OLED_Y_OFFSET + 14, 11, 6);
                    display.setDrawColor(1);
                    display.drawTriangle(OLED_X_OFFSET + 31, OLED_Y_OFFSET + 19,
                                         OLED_X_OFFSET + 39, OLED_Y_OFFSET + 19,
                                         OLED_X_OFFSET + 35, OLED_Y_OFFSET + 14);
                }
            }
            animateEditValue(menu);
            break;

        default: 
            if (param.id > 0) {                          
                display.setDrawColor(0);
                display.drawBox(OLED_X_OFFSET + 30, OLED_Y_OFFSET + 0, 11, 6);
                display.setDrawColor(1);
                display.drawTriangle(OLED_X_OFFSET + 31, OLED_Y_OFFSET + 5,
                                    OLED_X_OFFSET + 39, OLED_Y_OFFSET + 5,
                                    OLED_X_OFFSET + 35, OLED_Y_OFFSET + 0);
            }
            if (param.id < menu.paramCount) {   
                display.setDrawColor(0);
                display.drawBox(OLED_X_OFFSET + 30, OLED_Y_OFFSET + 35, 11, 6);
                display.setDrawColor(1);
                display.drawTriangle(OLED_X_OFFSET + 31, OLED_Y_OFFSET + 36,
                                    OLED_X_OFFSET + 39, OLED_Y_OFFSET + 36,
                                    OLED_X_OFFSET + 35, OLED_Y_OFFSET + 40);
            }    
            if (param.parentFolder > 0) {               
                display.setDrawColor(0);
                display.drawBox(OLED_X_OFFSET + 0, OLED_Y_OFFSET + 14, 5, 11);
                display.setDrawColor(1);
                display.drawTriangle(OLED_X_OFFSET + 4, OLED_Y_OFFSET + 15,
                                    OLED_X_OFFSET + 4, OLED_Y_OFFSET + 23,
                                    OLED_X_OFFSET + 0, OLED_Y_OFFSET + 19);
            }    
            if (param.type == CRSF_FOLDER) {             
                display.setDrawColor(0);
                display.drawBox(OLED_X_OFFSET + 66, OLED_Y_OFFSET + 14, 5, 11);
                display.setDrawColor(1);
                display.drawTriangle(OLED_X_OFFSET + 67, OLED_Y_OFFSET + 15,
                                    OLED_X_OFFSET + 67, OLED_Y_OFFSET + 23,
                                    OLED_X_OFFSET + 71, OLED_Y_OFFSET + 19);
            }

            animateMenu(menu);
            break;
    }
    drawPopup(menu);
    display.sendBuffer();
}

void drawModelFinderScreen(int raw, int avg, int strength) {
    display.clearBuffer();
    display.setDrawColor(1);
    display.drawBox(0 + OLED_X_OFFSET, 15 + OLED_Y_OFFSET, 6, 3);
    display.setFont(u8g2_font_logisoso28_tn); 
    display.setCursor(8 + OLED_X_OFFSET, 28 + OLED_Y_OFFSET);
    display.print(-raw);

    display.setFont(u8g2_font_squeezed_b6_tr); 
    display.setCursor(display.getCursorX()+2, 28 + OLED_Y_OFFSET);
    display.print("dBm");
    
    display.drawBox(0 + OLED_X_OFFSET, 30 + OLED_Y_OFFSET, (strength * OLED_WIDTH / 100), 10);
    display.sendBuffer();
}

void drawMessage(char title[], char line1[], char line2[]) {
    display.clearBuffer();
    display.setDrawColor(1);
    setChineseFont();
    const char* shownTitle = translateTextZh(title);
    int titleWidth = display.getUTF8Width(shownTitle);
    int titleX = OLED_X_OFFSET + (OLED_WIDTH - titleWidth) / 2;
    display.drawUTF8(titleX, 10 + OLED_Y_OFFSET, shownTitle);
    if (strlen(line2) > 0) {
        display.drawUTF8(2 + OLED_X_OFFSET, 24 + OLED_Y_OFFSET, translateTextZh(line1));
        display.drawUTF8(2 + OLED_X_OFFSET, 37 + OLED_Y_OFFSET, translateTextZh(line2));
    } else {
        display.drawUTF8(2 + OLED_X_OFFSET, 30 + OLED_Y_OFFSET, translateTextZh(line1));
    }
}

void drawCalibrationScreen(int stage, int secondsLeft) {
    display.clearBuffer();
    display.setDrawColor(1);

    setChineseFont();
    display.setCursor(2 + OLED_X_OFFSET, 10 + OLED_Y_OFFSET);
    display.drawUTF8(display.getCursorX(), display.getCursorY(), "摇杆校准");
    display.setCursor(2 + OLED_X_OFFSET, 23 + OLED_Y_OFFSET); 
    switch (stage) {
        case 1:
            display.drawUTF8(display.getCursorX(), display.getCursorY(), "正在开始");
            break;
        case 2:
            display.drawUTF8(display.getCursorX(), display.getCursorY(), "摇杆回中");
            display.setCursor(2 + OLED_X_OFFSET + display.getUTF8Width("摇杆回中"), 23 + OLED_Y_OFFSET);
            display.print(secondsLeft);
            break;
        case 3:
            display.drawUTF8(display.getCursorX(), display.getCursorY(), "转动摇杆");
            display.setCursor(2 + OLED_X_OFFSET + display.getUTF8Width("转动摇杆"), 23 + OLED_Y_OFFSET);
            display.print(secondsLeft);
            break;
        case 4:
            display.drawUTF8(display.getCursorX(), display.getCursorY(), "校准完成");
            break;
    }

    const uint8_t boxSize = 22;
    const uint8_t leftBoxX = 12;
    const uint8_t rightBoxX = 40;
    const uint8_t boxesY = 18;
    
    display.drawFrame(leftBoxX + OLED_X_OFFSET, boxesY + OLED_Y_OFFSET, boxSize, boxSize);
    display.drawFrame(rightBoxX + OLED_X_OFFSET, boxesY + OLED_Y_OFFSET, boxSize, boxSize);
    
    static bool direction = true;
    static float currXPos = 2;
    static float currYPos = 2;
    switch (stage) {
        case 1:
        case 2:
        case 4:
            display.drawBox(leftBoxX + ((boxSize-2)/2) + OLED_X_OFFSET, boxesY + ((boxSize-2)/2) + OLED_Y_OFFSET, 2, 2);
            display.drawBox(rightBoxX + ((boxSize-2)/2) + OLED_X_OFFSET, boxesY + ((boxSize-2)/2) + OLED_Y_OFFSET, 2, 2);
            break;
        case 3:
            display.drawBox(leftBoxX + (int)currXPos + OLED_X_OFFSET, boxesY + (int)currYPos + OLED_Y_OFFSET, 2, 2);
            display.drawBox(rightBoxX + boxSize + OLED_X_OFFSET - (int)currXPos, boxesY + (int)currYPos + OLED_Y_OFFSET, 2, 2);
            if (direction) {
                currXPos+=0.3;
                currYPos+=0.3;
                if (currXPos > boxSize - 2) {
                    currXPos-=0.6;
                    currYPos-=0.6;
                    direction = false;
                }
            } else {
                currXPos-=0.3;
                currYPos-=0.3;
                if (currXPos < 2) {
                    currXPos+=0.6;
                    currYPos+=0.6;
                    direction = true;
                }
            }
            break;
    }
    display.sendBuffer();
}