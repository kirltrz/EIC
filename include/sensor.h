#pragma once
#include "Wire.h"
#include "HWT101.h"
#include "PAW3395.h"
#include "vision.h"

struct global_position_t{
    float x;
    float y;
    float rawYaw;//原始偏航角(度)
    float continuousYaw;//连续偏航角(度)
};
/// @brief 传感器初始化(包括paw3395、hwt101、视觉模块)
void initSensor(void);

void resetSensor(void);

void calculateGlobalPosition(void *pvParameters);

void getGlobalPosition(global_position_t *position);

void setGlobalPosition(float x, float y);

bool isSensorResetInProgress(void);
