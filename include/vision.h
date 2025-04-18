/*用于通过串口与视觉模块进行通信*/
#include "stdint.h"

/*命令类型*/
#define CMD_IDLE 0
#define CMD_QRCODE 1
#define CMD_CIRCLE 2
#define CMD_MATERIAL 3

/*
@brief 视觉接收数据包结构体
@param mode 命令模式
@param data1 数据1
@param data2 数据2
@param color 颜色
*/
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
/*
@brief 视觉返回待机状态
@return 是否已返回待机状态
*/
bool visionToIDLE(void);

/*
@brief 视觉扫描二维码
@param taskcode1 任务码1
@param taskcode2 任务码2
@return 是否扫描成功
*/
bool visionScanQRcode(int taskcode1[3], int taskcode2[3]);

/*
@brief 视觉扫描色环
@param x 存储获取到的色环x坐标的指针
@param y 存储获取到的色环y坐标的指针
@return 是否扫描成功
*/
bool visionGetCircle(int *x, int *y);

/*
@brief 视觉扫描物料
@param color 要识别的物料颜色
@param x 存储获取到的物料x坐标的指针
@param y 存储获取到的物料y坐标的指针
@return 是否扫描成功
*/
bool visionGetMaterial(int color, int *x, int *y);
