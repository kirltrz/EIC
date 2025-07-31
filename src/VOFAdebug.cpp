#include "VOFAdebug.h"
#include <stdarg.h>

// 固定的帧尾标识符
const unsigned char DEBUG_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7f};

// UDP字符串发送缓冲区大小
#define UDP_STRING_BUFFER_SIZE 1024

// UDP相关全局变量
static WiFiUDP udp;
static UDPConfig udpConfig = {nullptr, 0, false};
static bool wifiInitialized = false;

void initVOFAdebug(){
    initWiFi("kirl", "TRZtrz20060906");
    setUDPConfig("192.168.1.98", 1347, true);
}
/**
 * 发送浮点数据到上位机
 * @param data 浮点数数组指针
 * @param count 数组长度
 * @param serial 串口对象指针，默认为Serial
 */
void sendDebugData(const float* data, uint8_t count, HardwareSerial* serial) {
    if (data == nullptr || serial == nullptr || count == 0) {
        return;
    }
    
    // 发送浮点数据
    serial->write((const uint8_t*)data, sizeof(float) * count);
    
    // 发送帧尾标识符
    serial->write(DEBUG_FRAME_TAIL, 4);
}

/**
 * 通过UDP发送浮点数据到上位机
 * @param data 浮点数数组指针
 * @param count 数组长度
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口
 */
void sendDebugDataUDP(const float* data, uint8_t count, const char* targetIP, uint16_t targetPort) {
    if (data == nullptr || targetIP == nullptr || count == 0 || !isWiFiConnected()) {
        return;
    }
    
    // 开始UDP数据包
    if (udp.beginPacket(targetIP, targetPort)) {
        // 发送浮点数据
        udp.write((const uint8_t*)data, sizeof(float) * count);
        
        // 发送帧尾标识符
        udp.write(DEBUG_FRAME_TAIL, 4);
        
        // 结束并发送数据包
        udp.endPacket();
    }
}

/**
 * 初始化WiFi连接
 * @param ssid WiFi名称
 * @param password WiFi密码
 * @param timeout 连接超时时间(ms)，默认10秒
 * @return 连接成功返回true，失败返回false
 */
bool initWiFi(const char* ssid, const char* password, unsigned long timeout) {
    if (ssid == nullptr || password == nullptr) {
        return false;
    }
    
    // 开始WiFi连接
    WiFi.begin(ssid, password);
    
    unsigned long startTime = millis();
    
    // 等待连接，直到超时
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < timeout) {
        delay(500);
    }
    
    wifiInitialized = (WiFi.status() == WL_CONNECTED);
    
    if (wifiInitialized) {
        // 初始化UDP
        udp.begin(1346); // 使用任意本地端口
    }
    
    return wifiInitialized;
}

/**
 * 设置UDP配置
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口
 * @param enabled 是否启用UDP发送
 */
void setUDPConfig(const char* targetIP, uint16_t targetPort, bool enabled) {
    udpConfig.targetIP = targetIP;
    udpConfig.targetPort = targetPort;
    udpConfig.enabled = enabled;
}

/**
 * 获取当前WiFi连接状态
 * @return WiFi连接状态
 */
bool isWiFiConnected() {
    return wifiInitialized && (WiFi.status() == WL_CONNECTED);
}

/**
 * 获取本机IP地址
 * @return IP地址字符串
 */
String getLocalIP() {
    if (isWiFiConnected()) {
        return WiFi.localIP().toString();
    }
    return "Not Connected";
}

/**
 * 检查UDP配置是否启用且有效
 * @return 配置有效返回true
 */
bool isUDPConfigValid() {
    return udpConfig.enabled && udpConfig.targetIP != nullptr;
}

/**
 * 获取UDP配置信息
 * @param targetIP 输出参数：目标IP
 * @param targetPort 输出参数：目标端口
 * @return 配置有效返回true
 */
bool getUDPConfig(const char*& targetIP, uint16_t& targetPort) {
    if (isUDPConfigValid()) {
        targetIP = udpConfig.targetIP;
        targetPort = udpConfig.targetPort;
        return true;
    }
    return false;
}

/**
 * 通过UDP发送字符串到指定地址
 * @param message 要发送的字符串
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口
 */
void sendStringUDP(const char* message, const char* targetIP, uint16_t targetPort) {
    if (message == nullptr || targetIP == nullptr || !isWiFiConnected()) {
        return;
    }
    
    // 开始UDP数据包
    if (udp.beginPacket(targetIP, targetPort)) {
        // 发送字符串数据
        udp.print(message);
        
        // 结束并发送数据包
        udp.endPacket();
    }
}

/**
 * 使用预设配置通过UDP发送字符串
 * @param message 要发送的字符串
 */
void sendStringUDP(const char* message) {
    const char* targetIP;
    uint16_t targetPort;
    if (getUDPConfig(targetIP, targetPort)) {
        sendStringUDP(message, targetIP, targetPort);
    }
}

/**
 * 类似printf的UDP发送函数，支持格式化字符串
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void udpPrintf(const char* targetIP, uint16_t targetPort, const char* format, ...) {
    if (format == nullptr || targetIP == nullptr || !isWiFiConnected()) {
        return;
    }
    
    // 创建格式化缓冲区
    static char buffer[UDP_STRING_BUFFER_SIZE];
    
    // 处理可变参数
    va_list args;
    va_start(args, format);
    
    // 格式化字符串
    int len = vsnprintf(buffer, UDP_STRING_BUFFER_SIZE, format, args);
    
    va_end(args);
    
    // 检查格式化是否成功
    if (len > 0 && len < UDP_STRING_BUFFER_SIZE) {
        sendStringUDP(buffer, targetIP, targetPort);
    }
}

/**
 * 使用预设配置的类似printf的UDP发送函数
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void udpPrintf(const char* format, ...) {
    if (format == nullptr || !isWiFiConnected()) {
        return;
    }
    
    const char* targetIP;
    uint16_t targetPort;
    if (!getUDPConfig(targetIP, targetPort)) {
        return;
    }
    
    // 创建格式化缓冲区
    static char buffer[UDP_STRING_BUFFER_SIZE];
    
    // 处理可变参数
    va_list args;
    va_start(args, format);
    
    // 格式化字符串
    int len = vsnprintf(buffer, UDP_STRING_BUFFER_SIZE, format, args);
    
    va_end(args);
    
    // 检查格式化是否成功
    if (len > 0 && len < UDP_STRING_BUFFER_SIZE) {
        sendStringUDP(buffer, targetIP, targetPort);
    }
}

/**
 * 设置UDP日志的目标地址（便捷函数）
 * @param targetIP 目标IP地址
 * @param targetPort 目标端口，默认1234
 */
void setUDPLogTarget(const char* targetIP, uint16_t targetPort) {
    setUDPConfig(targetIP, targetPort, true);
} 