#pragma once
#include "ZDTX42V2.h"
#include "config.h"

struct POS{
    float x;
    float y;
    float yaw;
};
void initMotor(void);
void moveTo(POS pos);
void moveTask(void*pvParameters);
bool arrived(void);//返回true表示到达目标点
void stopMotion(void);
