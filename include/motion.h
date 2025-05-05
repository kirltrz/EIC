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

enum systemState
{
  IDLE,               // 空闲状态
  COARSE_POSITIONING, // 粗定位状态
  FINE_POSITIONING,   // 精定位状态
  COMPLETED           // 定位完成
};

// 外部声明电机驱动实例，使其全局可用
extern ZDTX42V2 *motor;
extern systemState currentState;
float calculateMinAngleDifference(float angle1, float angle2);
void initMotor(void);
void moveTo(POS _targetPos);
void stopMotion(void);
void emergencyStopMotor(bool is_enable);
void globalToLocalVelocity(float global_vx, float global_vy, float yaw_rad, float &local_vx, float &local_vy);
void calculateWheelVelocities(float vx, float vy, float omega, float wheelVelocities[4]);
void calculateWheelPositions(float wheelPositions[4]);
float calculateDistance(float x1, float y1, float x2, float y2);
void moveTask(void*pvParameters);
bool coarsePositioning(void);
bool finePositioning(void);
void updateCurrentPosition();