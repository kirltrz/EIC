/**
 * 本库用于便捷地将浮点数据发送至VOFA+上位机软件便于可视化调试
 */
#pragma once

#include <Arduino.h>

/**
 * 发送浮点数据到上位机 (基本函数)
 * @param data 浮点数数组指针
 * @param count 数组长度
 * @param serial 串口对象指针，默认为Serial
 */
void sendDebugData(const float* data, uint8_t count, HardwareSerial* serial = &Serial);

/**
 * 可变参数版本的发送函数，类似于printf
 * 使用示例: sendDebugValues(x, y, yaw, v1, v2, ...);
 */
template<typename... Args>
void sendDebugValues(Args... args) {
    // 计算参数数量
    constexpr size_t count = sizeof...(args);
    
    // 创建临时数组
    float data[count] = {static_cast<float>(args)...};
    
    // 调用基本函数
    sendDebugData(data, count);
}

/**
 * 可变参数版本的发送函数，支持指定串口
 * 使用示例: sendDebugValues(&Serial2, x, y, yaw, v1, v2, ...);
 */
template<typename... Args>
void sendDebugValues(HardwareSerial* serial, Args... args) {
    // 计算参数数量
    constexpr size_t count = sizeof...(args);
    
    // 创建临时数组
    float data[count] = {static_cast<float>(args)...};
    
    // 调用基本函数
    sendDebugData(data, count, serial);
}
