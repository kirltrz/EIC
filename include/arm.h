#pragma once
#include "FashionStar_UartServo.h"
#include "config.h"
#include "taskManager.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern FSUS_Servo servo0;
extern FSUS_Servo servo1;
extern FSUS_Servo servo2;
extern FSUS_Servo servo3;
extern FSUS_Servo servo4;

// 夹爪位置监测相关变量
extern bool gripperHolding;           // 夹持状态标志
extern float currentGripperAngle;     // 当前夹爪角度，单位度
extern SemaphoreHandle_t gripperMutex; // 夹爪状态互斥锁

/*
@brief 机械臂运动函数
*/
void initArm(void);
void stopArm(uint16_t power = 500);
void setOriginPoint(void);
void arm_ScanQRcode();
void arm_setClaw(bool open);
void arm_catchFromTurntable(int taskcode[3]);
void arm_putToGround(int taskcode[3]);
void arm_catchFromGround(int taskcode[3]);
void arm_putToMaterial(int taskcode[3]);
void armControl_xyz(float x, float y, float z, uint16_t interval, uint16_t acc, uint16_t dec, bool needDelay=true);
void armSet_position(float theta0, float first_arm_degree, float second_arm_degree, float third_arm_degree, uint16_t interval, uint16_t acc, uint16_t dec);
bool armCalculate_inverse(float x, float y, float z, float *out_arm_degree);
void armCalculate_forward(float theta0, float first_arm_degree, float second_arm_degree, float *out_arm_location_xyz);
void waitArm(void);

void localToGlobalCoordinates(float local_x, float local_y, float servo0_angle, float *global_x, float *global_y);

// 机械臂控制MainSequence的推进与暂停相关函数
void armPauseMainSequence(void);     // 暂停主流程
void armResumeMainSequence(void);    // 恢复/推进主流程