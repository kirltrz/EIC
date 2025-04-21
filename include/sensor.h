#include "Wire.h"
#include "HWT101.h"
#include "PAW3395.h"
#include "vision.h"

struct global_position_t{
    volatile float x;
    volatile float y;
    volatile float rawYaw;
    volatile float continuousYaw;
};
/// @brief 传感器初始化(包括paw3395、hwt101、视觉模块)
void initSensor(void);

void calculateGlobalPosition(void *pvParameters);
void getGlobalPosition(global_position_t *position);

bool checkPaw3395(void);

bool checkHWT101(void);

bool checkVision(void);
