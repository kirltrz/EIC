#pragma once
#include "ZDTX42V2.h"
#include "config.h"
#include <Arduino.h>
#include "taskManager.h"

struct POS{
    double x;
    double y;
    float yaw;
};
// 外部声明电机驱动实例，使其全局可用
extern ZDTX42V2 *motor;
extern systemState currentState;
void initMotor(void);
void moveTo(POS pos, float speed=DEFAULT_SPEED, int acc=DEFAULT_ACC, int dec=DEFAULT_ACC);
void moveTask(void*pvParameters);
bool arrived(void);//返回true表示到达目标点
void stopMotion(void);//停止运动并释放电机控制
void forceStopAllMotors(void);//强制停止所有电机，确保速度为0