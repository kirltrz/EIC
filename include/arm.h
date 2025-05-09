#pragma once
#include "FashionStar_UartServo.h"
#include "config.h"
#include "taskManager.h"
#include <Arduino.h>

extern FSUS_Servo servo0;
extern FSUS_Servo servo1;
extern FSUS_Servo servo2;
extern FSUS_Servo servo3;
extern FSUS_Servo servo4;

/*
@brief 机械臂运动函数
*/
void initArm(void);
void stopArm(bool stop);
void setOriginPoint(void);
void arm_ScanQRcode();
void arm_catchFromTurntable(int taskcode[3]);
void arm_putToGround(int taskcode[3], int circleOffset[3][2]/*传出当前色环偏移量*/);
void arm_catchFromGround(int taskcode[3], const int circleOffset[3][2]/*传入当前色环偏移量*/);
void arm_putToMaterial(int taskcode[3]);
void armControl_xyz(float x, float y, float z);
void armSet_position(float first_arm_degree, float second_arm_degree, float theta0);
void armCalculate_inverse(float x, float y, float z, float *out_arm_degree);
void armSet_position(float first_arm_degree, float second_arm_degree, float theta0);
void armTestTask(void *pvParameters);
void startArmTest(void);