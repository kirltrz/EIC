#pragma once
#include "ZDTX42V2.h"
#include "config.h"
#include <Arduino.h>
struct POS{
    float x;
    float y;
    float yaw;
};
void initMotor(void);
void globalToLocalVelocity(float global_vx, float global_vy, float yaw_rad, float &local_vx, float &local_vy);
void calculateWheelVelocities(float vx, float vy, float omega, float wheelVelocities[4]);
void calculateWheelPositions(float wheelPositions[4]);
float calculateDistance(float x1, float y1, float x2, float y2);
void moveTask(void*pvParameters);
bool coarsePositioning(void);
bool finePositioning(void);
void updateCurrentPosition();