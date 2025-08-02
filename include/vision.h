/*用于通过串口与视觉模块进行通信*/
#pragma once
#include "stdint.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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
    bool fresh;
};

// 全局变量声明
extern SemaphoreHandle_t visionDataMutex;        // 视觉数据互斥锁
extern vision_packet_t visionDataCache;          // 视觉数据缓存

/*
@brief 初始化串口通信
*/
void visionInit(void);

/*
@brief 视觉监听任务，持续监听串口数据
*/
void visionListenerTask(void *pvParameters);

/*
@brief 发送命令
@param mode 命令模式 0：待命模式 1：二维码识别 2：物料识别 3：色环识别 4：转盘识别 5：获取圆形颜色模式
*/
void sendCommand(int mode, int color = 0);

/*
@brief 持续监听并接收数据，验证格式后写入缓存
*/
void receiveDataContinuous(void);

/*
@brief 读取缓存中的视觉数据（线程安全）
@param data 输出数据结构体
@return 是否成功读取
*/
bool readVisionCache(vision_packet_t *data);

/*
@brief 视觉返回待机状态
*/
void visionToIDLE(void);

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

/*
@brief 视觉扫描转盘
@param x 存储获取到的转盘x坐标的指针
@param y 存储获取到的转盘y坐标的指针
@return 是否扫描成功
*/
bool visionGetTurntable(int *x, int *y);

/*
@brief 视觉获取圆形区域颜色
@param color 存储获取到的颜色信息的指针
@return 是否检测成功
*/
bool visionGetCircleColor(int *color);