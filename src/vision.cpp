#include "vision.h"
#include "HardwareSerial.h"
#include "taskManager.h"
#include "config.h"

typedef uint8_t byte;

// 全局变量定义
SemaphoreHandle_t visionDataMutex = NULL;        // 视觉数据互斥锁
vision_packet_t visionDataCache={-1,-1,-1,-1};                 // 视觉数据缓存

void visionInit(void)
{
    /*初始化串口通信*/
    // 如果视觉串口和调试串口不同，或者调试未启用，则初始化视觉串口
    #if (DEBUG_SERIAL != VISION_SERIAL) || (DEBUG_ENABLE == 0)
        VISION_SERIAL.begin(SERIAL_BAUDRATE);
    #endif
    
    // 创建互斥锁
    if (visionDataMutex == NULL)
    {
        visionDataMutex = xSemaphoreCreateMutex();
    }
}

/*
@brief 视觉监听任务，持续监听串口数据
*/
void visionListenerTask(void *pvParameters)
{
    // 任务启动时先清空串口缓冲区中可能存在的旧数据
    while (VISION_SERIAL.available() > 0)
    {
        VISION_SERIAL.read();
    }
    
    // 短暂延迟确保清空完成
    vTaskDelay(pdMS_TO_TICKS(50));
    
    while (true)
    {
        receiveDataContinuous();
        vTaskDelay(pdMS_TO_TICKS(1)); // 短暂延迟，让出CPU时间
    }
}

/*
@brief 持续监听并接收数据，验证格式后写入缓存
*/
void receiveDataContinuous(void)
{
    // 检查是否有数据可读
    if (VISION_SERIAL.available() == 0)
    {
        return;
    }
    
    byte buffer[9];
    
    // 查找帧头0x7B
    if (VISION_SERIAL.read() != 0x7B)
    {
        return;
    }
    
    buffer[0] = 0x7B;
    
    // 读取剩余8个字节，如果数据不足则返回
    if (VISION_SERIAL.readBytes(buffer + 1, 8) != 8)
    {
        return;
    }
    
    // 验证帧尾
    if (buffer[8] != 0x7D)
    {
        return;
    }
    
    // 验证校验位
    byte checksum = 0;
    for (int i = 0; i < 7; i++)
    {
        checksum ^= buffer[i];
    }
    if (buffer[7] != checksum)
    {
        return;
    }
    
    // 数据格式正确，解析并写入缓存
    vision_packet_t new_data;
    new_data.mode = buffer[1];
    new_data.data1 = (((int16_t)buffer[2]) << 8) | ((int16_t)buffer[3]);
    new_data.data2 = (((int16_t)buffer[4]) << 8) | ((int16_t)buffer[5]);
    new_data.color = buffer[6] == 4 ? buffer[6] - 1 : buffer[6]; // 如果是蓝色0x04则减一
    
    // 获取互斥锁并写入缓存
    if (xSemaphoreTake(visionDataMutex, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        visionDataCache = new_data;
        xSemaphoreGive(visionDataMutex);
    }
}

bool readVisionCache(vision_packet_t *data)
{
    if (xSemaphoreTake(visionDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        *data = visionDataCache;
        xSemaphoreGive(visionDataMutex);
        return true;
    }
    return false;
}

void sendCommand(int mode)
{
    /*发送命令*/
    byte frame[9] = {0x7B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D};

    frame[1] = mode & 0xFF;
    for (int i = 0; i < 7; i++)
    {
        frame[7] ^= frame[i];
    }
    VISION_SERIAL.write(frame, 9);
}

void visionToIDLE(void)
{
    sendCommand(CMD_IDLE);
}

bool visionScanQRcode(int taskcode1[3], int taskcode2[3])
{
    vision_packet_t packet;
    unsigned long startTime = millis();
    
    sendCommand(CMD_QRCODE);

    while (millis() - startTime < VISION_TIMEOUT)
    {
        if (readVisionCache(&packet))
        {
            /*判断是否为扫码模式*/
            if (packet.mode == CMD_QRCODE)
            {
                if (packet.data1 != 0 && packet.data2 != 0)
                {
                    /*将任务码分解*/
                    taskcode1[0] = packet.data1 / 100;
                    taskcode1[1] = (packet.data1 / 10) % 10;
                    taskcode1[2] = packet.data1 % 10;
                    taskcode2[0] = packet.data2 / 100;
                    taskcode2[1] = (packet.data2 / 10) % 10;
                    taskcode2[2] = packet.data2 % 10;
                    /*判断任务码是否有效*/
                    if (taskcode1[0] && taskcode1[1] && taskcode1[2] &&
                        taskcode2[0] && taskcode2[1] && taskcode2[2])
                    {
                        return true;
                    }
                }
            }
            else
            {/*如果不是扫码模式，重新发送扫码指令*/
                sendCommand(CMD_QRCODE);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;/*超时*/
}

bool visionGetCircle(int *x, int *y)
{
    vision_packet_t packet;
    unsigned long startTime = millis();
    
    sendCommand(CMD_CIRCLE);
    
    while (millis() - startTime < VISION_TIMEOUT)
    {
        if (readVisionCache(&packet))
        {
            if (packet.mode == CMD_CIRCLE)
            {
                *x = packet.data1;
                *y = packet.data2;
                return true;
            }
            else
            {
                // 如果不是色环检测模式，重新发送色环检测指令
                sendCommand(CMD_CIRCLE);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    return false;
}

bool visionGetMaterial(int color, int *x, int *y)
{
    vision_packet_t packet;
    unsigned long startTime = millis();
    
    sendCommand(CMD_MATERIAL);
    
    while (millis() - startTime < VISION_TIMEOUT)
    {
        if (readVisionCache(&packet))
        {
            if (packet.mode == CMD_MATERIAL && packet.color == color)
            {
                *x = packet.data1;
                *y = packet.data2;
                return true;
            }
            else if (packet.mode != CMD_MATERIAL)
            {
                // 如果不是物料检测模式，重新发送物料检测指令
                sendCommand(CMD_MATERIAL);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    return false;
}