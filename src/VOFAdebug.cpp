#include "VOFAdebug.h"

// 固定的帧尾标识符
const unsigned char DEBUG_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7f};

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