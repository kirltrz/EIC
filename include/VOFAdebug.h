/**
 * 本库用于便捷地将浮点数据发送至VOFA+上位机软件便于可视化调试
 */
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// UDP配置结构体
struct UDPConfig {
    const char* targetIP;
    uint16_t targetPort;
    bool enabled;
};

void initVOFAdebug();
/**
 * 发送浮点数据到上位机 (基本函数)
 * @param data 浮点数数组指针
 * @param count 数组长度
 * @param serial 串口对象指针，默认为Serial
 */
void sendDebugData(const float* data, uint8_t count, HardwareSerial* serial = &Serial);

/**
 * 通过UDP发送浮点数据到上位机
 * @param data 浮点数数组指针
 * @param count 数组长度
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口
 */
void sendDebugDataUDP(const float* data, uint8_t count, const char* targetIP, uint16_t targetPort);

/**
 * 初始化WiFi连接
 * @param ssid WiFi名称
 * @param password WiFi密码
 * @param timeout 连接超时时间(ms)，默认10秒
 * @return 连接成功返回true，失败返回false
 */
bool initWiFi(const char* ssid, const char* password, unsigned long timeout = 10000);

/**
 * 设置UDP配置
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口
 * @param enabled 是否启用UDP发送
 */
void setUDPConfig(const char* targetIP, uint16_t targetPort, bool enabled = true);

/**
 * 获取当前WiFi连接状态
 * @return WiFi连接状态
 */
bool isWiFiConnected();

/**
 * 获取本机IP地址
 * @return IP地址字符串
 */
String getLocalIP();

/**
 * 检查UDP配置是否启用且有效
 * @return 配置有效返回true
 */
bool isUDPConfigValid();

/**
 * 获取UDP配置信息
 * @param targetIP 输出参数：目标IP
 * @param targetPort 输出参数：目标端口
 * @return 配置有效返回true
 */
bool getUDPConfig(const char*& targetIP, uint16_t& targetPort);

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

/**
 * 可变参数版本的UDP发送函数
 * 使用示例: sendDebugValuesUDP("192.168.1.100", 1234, x, y, yaw, v1, v2, ...);
 */
template<typename... Args>
void sendDebugValuesUDP(const char* targetIP, uint16_t targetPort, Args... args) {
    // 计算参数数量
    constexpr size_t count = sizeof...(args);
    
    // 创建临时数组
    float data[count] = {static_cast<float>(args)...};
    
    // 调用基本函数
    sendDebugDataUDP(data, count, targetIP, targetPort);
}

/**
 * 使用预设配置的UDP发送函数
 * 使用示例: sendDebugValuesUDP(x, y, yaw, v1, v2, ...);
 * 注意：需要先调用setUDPConfig设置目标地址和端口
 */
template<typename... Args>
void sendDebugValuesUDP(Args... args) {
    // 计算参数数量
    constexpr size_t count = sizeof...(args);
    
    // 创建临时数组
    float data[count] = {static_cast<float>(args)...};
    
    // 直接调用基本UDP发送函数，避免依赖cpp文件中的函数
    const char* targetIP;
    uint16_t targetPort;
    if (getUDPConfig(targetIP, targetPort)) {
        sendDebugDataUDP(data, count, targetIP, targetPort);
    }
}

/**
 * 通过UDP发送字符串到指定地址
 * @param message 要发送的字符串
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口
 */
void sendStringUDP(const char* message, const char* targetIP, uint16_t targetPort);

/**
 * 使用预设配置通过UDP发送字符串
 * @param message 要发送的字符串
 * 注意：需要先调用setUDPConfig设置目标地址和端口
 */
void sendStringUDP(const char* message);

/**
 * 类似printf的UDP发送函数，支持格式化字符串
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口
 * @param format 格式化字符串
 * @param ... 可变参数
 * 使用示例: udpPrintf("192.168.1.100", 1234, "Position: x=%.2f, y=%.2f\n", x, y);
 */
void udpPrintf(const char* targetIP, uint16_t targetPort, const char* format, ...);

/**
 * 使用预设配置的类似printf的UDP发送函数
 * @param format 格式化字符串
 * @param ... 可变参数
 * 使用示例: udpPrintf("Position: x=%.2f, y=%.2f\n", x, y);
 * 注意：需要先调用setUDPConfig设置目标地址和端口
 */
void udpPrintf(const char* format, ...);

// 便捷宏定义，类似DEBUG_LOG
#define UDP_LOG(...) udpPrintf(__VA_ARGS__)

/**
 * 设置UDP日志的目标地址（便捷函数）
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口，默认1234
 */
void setUDPLogTarget(const char* targetIP, uint16_t targetPort = 1234);
