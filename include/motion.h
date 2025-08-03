#pragma once
#include "ZDT_MOTOR_EMM_V5.h"
#include "config.h"
#include <Arduino.h>
#include "taskManager.h"

struct POS{
    double x;
    double y;
    float yaw;
};

// 外部声明电机驱动实例，使其全局可用
extern ZDT_MOTOR_EMM_V5 *motor;

void initMotor(void);
void moveTo(POS pos,bool slower=false);
void resetPIDparam(void);
void moveTask(void*pvParameters);
/*
** @param: pos_tolerance 位置容差，单位mm，放宽到30mm
** @param: yaw_tolerance 偏航角容差，单位为度，放宽到5度
** @param: anti_oscillation 是否开启防抖动
*/
bool arrived(float pos_tolerance, float yaw_tolerance, bool anti_oscillation);//返回true表示到达目标点
void waitArrived(void);
void waitNear(void);
void waitCompeletelyArrived(void);
void stopMotion(void);//停止运动并释放电机控制
void forceStopAllMotors(void);//强制停止所有电机，确保速度为0