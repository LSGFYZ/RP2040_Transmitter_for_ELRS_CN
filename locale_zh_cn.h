#pragma once
#ifndef LOCALE_ZH_CN_H
#define LOCALE_ZH_CN_H

#include <Arduino.h>

// Display-only Simplified Chinese dictionary. Internal CRSF parameter labels
// remain English so existing setting persistence and protocol lookups stay stable.
static inline const char* translateTextZh(const char* source) {
    if (!source) return "";
    struct Entry { const char* en; const char* zh; };
    static const Entry dictionary[] = {
        {"Radio Settings", "遥控器设置"}, {"Stick Mode", "摇杆模式"},
        {"Mode 1", "模式1"}, {"Mode 2", "模式2"}, {"Mode 3", "模式3"}, {"Mode 4", "模式4"},
        {"ADC Filter", "摇杆滤波"}, {"Off", "关闭"}, {"On", "开启"},
        {"Deadband", "中位死区"}, {"Long Press", "长按时间"}, {"Volt Scale", "电压校准"},
        {"Ail/Roll", "副翼/横滚"}, {"Ele/Pitch", "升降/俯仰"}, {"Throttle", "油门"}, {"Rud/Yaw", "方向/偏航"},
        {"Center Throt", "油门居中"}, {"No", "否"}, {"Yes", "是"},
        {"Normal", "正常"}, {"Reverse", "反向"},
        {"Ail Reverse", "副翼反向"}, {"Ail Limit", "副翼行程"}, {"Ail Expo", "副翼指数"},
        {"Ele Reverse", "升降反向"}, {"Ele Limit", "升降行程"}, {"Ele Expo", "升降指数"},
        {"Thr Reverse", "油门反向"}, {"Thr Limit", "油门行程"}, {"Thr Expo", "油门指数"},
        {"Yaw Reverse", "方向反向"}, {"Yaw Limit", "方向行程"}, {"Yaw Expo", "方向指数"},
        {"Packet rate ???", "分包速率未知"}, {"TX power ???", "发射功率未知"},
        {"Loading Params...", "正在加载参数"}, {"No TX Module", "未检测到发射模块"},
        {"Channel Output", "通道输出"}, {"BLE Joystick", "蓝牙摇杆"},
        {"ON", "开启"}, {"OFF", "关闭"}, {"[Running...]", "[执行中...]"}, {"[Confirm?]", "[确认?]"},
        {"CALIBRATION", "摇杆校准"}, {"Starting...", "正在开始"}, {"Center Sticks ", "摇杆回中 "},
        {"Move Sticks ", "转动摇杆 "}, {"Complete", "校准完成"},
        {"ELRS Config", "ELRS配置"}, {"Packet Rate", "分包速率"}, {"TX Power", "发射功率"},
        {"Telemetry Ratio", "回传比例"}, {"Switch Mode", "开关模式"}, {"Model Match", "模型匹配"},
        {"Dynamic Power", "动态功率"}, {"Antenna", "天线"}, {"Fan", "风扇"}, {"WiFi", "无线网络"},
        {"Enabled", "已启用"}, {"Disabled", "已禁用"}, {"Enable", "启用"}, {"Disable", "禁用"},
        {"Auto", "自动"}, {"Manual", "手动"}, {"Low", "低"}, {"Medium", "中"}, {"High", "高"},
        {"Pitmode", "维修模式"}, {"Armed", "已解锁"}, {"Disarmed", "已锁定"},
        {"Back", "返回"}, {"Save", "保存"}, {"Cancel", "取消"}, {"Confirm", "确认"},
        {"Yes/No", "是/否"}, {"Loading...", "加载中"}, {"Not connected", "未连接"},
        {"Disconnected", "已断开"}, {"Connected", "已连接"}, {"Searching...", "搜索中"},
        {"Current", "当前"}, {"Default", "默认"}, {"Factory", "出厂设置"},
        {"Channel", "通道"}, {"Output", "输出"}, {"Link Statistics", "链路统计"},
        {"Uplink", "上行"}, {"Downlink", "下行"}, {"Signal", "信号"}, {"Power", "功率"},
        {"Frequency", "频率"}, {"Temperature", "温度"}, {"Voltage", "电压"},
        {"Current Limit", "电流限制"}, {"Max Power", "最大功率"}, {"Min", "最小"}, {"Max", "最大"},
        {"Fan Threshold", "风扇启动温度"}, {"Fan Runtime", "风扇运行时间"},
        {"WiFi Connectivity", "无线网络连接"}, {"Bind", "对频"}, {"Binding", "正在对频"},
        {"VTX Administrator", "图传管理"}, {"VTX Band", "图传频段"}, {"VTX Channel", "图传频道"},
        {"VTX Power", "图传功率"}, {"VTX Pit Mode", "图传维修模式"}, {"VTX Frequency", "图传频率"},
        {"RF Frequency", "射频频率"}, {"Regulatory Domain", "法规区域"}, {"Binding Phrase", "绑定短语"},
        {"Model ID", "模型编号"}, {"UART Inverted", "UART反相"}, {"Serial Protocol", "串口协议"},
        {"Baud Rate", "波特率"}, {"GPS", "GPS"}, {"Current Sensor", "电流传感器"},
        {"Voltage Sensor", "电压传感器"}, {"AUX", "辅助通道"}, {"Model", "模型"},
        {"Rate", "速率"}, {"Ratio", "比例"}, {"Power Level", "功率等级"},
        {"Boost Channel", "增益通道"}, {"Dynamic Power Enable", "启用动态功率"},
        {"Fan Enable", "启用风扇"}, {"Unlock", "解锁"}, {"Lock", "锁定"},
        {"Restart", "重启"}, {"Reboot", "重新启动"}, {"Reset", "重置"},
        {"Update", "更新"}, {"Firmware Update", "固件更新"}, {"Version", "版本"},
        {"Serial Number", "序列号"}, {"UID", "唯一标识"}, {"Device", "设备"},
        {"Device Name", "设备名称"}, {"WiFi Update", "无线更新"}, {"ExpressLRS", "ExpressLRS"}
    };
    for (const Entry& entry : dictionary) {
        if (strcmp(source, entry.en) == 0) return entry.zh;
    }
    return source; // Unknown/module-specific labels safely fall back to source text.
}

#endif // LOCALE_ZH_CN_H
