#include "vision.h"
#include "HardwareSerial.h"
#include "taskManager.h"

#define VISION_TIMEOUT 1000
typedef uint8_t byte;

void initVision(void)
{
    /*初始化串口通信*/
    Serial.begin(115200);
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
    Serial.write(frame, 9);
}

bool receiveData(vision_packet_t *data)
{
    /*接收数据*/
    // 定义一个长度为9的字节数组，用于存储从串口读取的数据
    byte buffer[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // 等待串口有数据可读，如果没有数据则延迟10ms
    unsigned long startTime = millis();
    while (Serial.available() == 0)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        // 添加超时检测，防止死循环
        if (millis() - startTime > VISION_TIMEOUT)
        {
            return false; // 超时返回失败
        }
    }

    // 读取第一个字节，检查是否为帧头0x7B
    int count = 0;
    do
    {
        count++;
        if (count > 10)
        {
            return false;
        }
        Serial.read(buffer, 1);
    } while (buffer[0] != 0x7B);

    // 读取接下来的8个字节，存储到buffer中
    Serial.readBytes((byte *)buffer + 1, 8);

    // 检查最后一个字节是否为帧尾0x7D
    if (buffer[8] != 0x7D)
    {
        return false; // 如果不是帧尾，直接返回
    }
    // 检查校验位是否正确，校验位为buffer[7]，应为buffer[0]到buffer[6]的异或结果
    if (buffer[7] != (buffer[0] ^ buffer[1] ^ buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5] ^ buffer[6]))
    {
        return false; // 如果校验失败，直接返回
    }
    // 将读取到的数据解析并存储到data结构体中
    data->mode = buffer[1];
    data->data1 = (((int16_t)buffer[2]) << 8) | ((int16_t)buffer[3]);
    data->data2 = (((int16_t)buffer[4]) << 8) | ((int16_t)buffer[5]);
    data->color = buffer[6] == 4 ? buffer[6] - 1 : buffer[6]; // 如果是蓝色0x04则减一，方便后续使用
    return true;
}

/*
@brief 视觉返回待机状态
@return 是否已返回待机状态
*/
bool visionToIDLE(void)
{
    /*视觉返回待机状态*/
    sendCommand(CMD_IDLE);
    vision_packet_t packet;
    int startTime = millis();
    while (!receiveData(&packet))
    {
        wait(100);
        if (millis() - startTime > VISION_TIMEOUT)
        {
            return false;
        }
    }
    if (packet.mode == CMD_IDLE)
    {
        return true;
    }
    return false;
}
/*
@brief 视觉扫描二维码
@param taskcode1 任务码1
@param taskcode2 任务码2
@return 是否扫描成功
*/
bool visionScanQRcode(int taskcode1[3], int taskcode2[3])
{
    /*视觉扫描二维码*/
    sendCommand(CMD_QRCODE);
    vision_packet_t packet;
    int startTime = millis();
    while (!receiveData(&packet))
    {
        wait(100);
        if (millis() - startTime > VISION_TIMEOUT)
        {
            return false;
        }
    }
    if (packet.mode == CMD_QRCODE)
    {
        // 将data1分解为三个数字存入taskcode1
        taskcode1[0] = packet.data1 / 100;       // 百位
        taskcode1[1] = (packet.data1 / 10) % 10; // 十位
        taskcode1[2] = packet.data1 % 10;        // 个位

        // 将data2分解为三个数字存入taskcode2
        taskcode2[0] = packet.data2 / 100;       // 百位
        taskcode2[1] = (packet.data2 / 10) % 10; // 十位
        taskcode2[2] = packet.data2 % 10;        // 个位
        return true;
    }
    return false;
}

bool visionGetCircle(int *x, int *y)
{
    /*视觉扫描色环*/
    sendCommand(CMD_CIRCLE);
    vision_packet_t packet;
    int startTime = millis();
    while (!receiveData(&packet))
    {
        wait(100);
        if (millis() - startTime > VISION_TIMEOUT)
        {
            return false;
        }
    }
    if (packet.mode == CMD_CIRCLE)
    {
        *x = packet.data1;
        *y = packet.data2;
        return true;
    }
    return false;
}

/*
@brief 视觉扫描物料
@param x 存储获取到的物料x坐标的指针
@param y 存储获取到的物料y坐标的指针
@param color 要识别的物料颜色
@return 是否扫描成功
*/
bool visionGetMaterial(int color, int *x, int *y)
{
    /*视觉扫描物料*/
    sendCommand(CMD_MATERIAL);
    vision_packet_t packet;
    int startTime = millis();
    while(!receiveData(&packet)){
        wait(100);
        if(millis()-startTime>VISION_TIMEOUT){
            return false;   
        }
    }
    if (packet.mode == CMD_MATERIAL || packet.color == color)
    {
        *x = packet.data1;
        *y = packet.data2;
        return true;
    }
    return false;
}