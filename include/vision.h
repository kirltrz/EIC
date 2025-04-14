/*用于通过串口与视觉模块进行通信*/
#include "stdint.h"

/*命令类型*/
#define CMD_IDLE 0
#define CMD_QRCODE 1
#define CMD_CIRCLE 2
#define CMD_MATERIAL 3

struct vision_packet_t
{
    int mode;
    int16_t data1;
    int16_t data2;
    int8_t color;
};

/*
@brief 初始化串口通信
*/
void initVision(void);
/*
@brief 发送命令
@param mode 命令模式 0：待命模式 1：二维码识别 2：色环识别 3：物料识别
*/
void sendCommand(int mode);
/*
@brief 接收数据
@param data 数据结构体
@return 是否接收成功
*/
bool receiveData(vision_packet_t *data);


