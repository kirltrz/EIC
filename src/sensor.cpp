#include "sensor.h"
#include "taskManager.h"
#include "config.h"

SemaphoreHandle_t positionMutex = NULL; // 全局坐标互斥锁
global_position_t currentPosition; // 用于积分运算

void initSensor(void)
{
    /*初始化传感器*/
    positionMutex = xSemaphoreCreateMutex(); // 创建全局坐标互斥锁
    visionInit();
    Wire.begin(PIN_HWT101_SDA, PIN_HWT101_SCL); // 初始化IIC(HWT101)
    paw3395Init(PAW3395_DPI, PIN_PAW3395_NRESET, PIN_PAW3395_NCS, PIN_PAW3395_SCLK, PIN_PAW3395_MISO, PIN_PAW3395_MOSI);
    delay(100);
    resetSensor();
}

void resetSensor(void)
{
    visionToIDLE();
    HWT101.toZero();
    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE)
    {
        // 在获取到互斥锁后，清零全局位置数据
        currentPosition.x = 0;
        currentPosition.y = 0;
        currentPosition.rawYaw = 0;
        currentPosition.continuousYaw = 0;

        // 释放互斥锁
        xSemaphoreGive(positionMutex);
    }
}
bool checkPaw3395(void)
{
    /*检查paw3395是否正常工作*/
    return paw3395_check();
}

bool checkHWT101(void)
{
    /*检查hwt101是否正常工作*/
    Wire.beginTransmission(80); // HWT101地址0x50对应十进制为80
    delay(5);
    return Wire.endTransmission() == 0;
}

bool checkVision(void)
{
    /*检查与视觉模块串口通信是否正常*/
    sendCommand(CMD_IDLE);
    vision_packet_t data;
    return receiveData(&data);
}

void calculateGlobalPosition(void *pvParameters)
{
    /*计算全局位置*/
    float prevYaw = HWT101.getZ();  // 上一次的yaw角度
    int rotationCount = 0;          // 旋转圈数，正值表示顺时针旋转的圈数，负值表示逆时针

    int16_t dx = 0;
    int16_t dy = 0;
    const float scaleFactor = 0.0009769f; // 比例因子，将传感器读数转换为实际位移
    
    // 未能更新的位移增量
    float pendingDx = 0.0f;
    float pendingDy = 0.0f;

    // 传感器数据有效性检查阈值
    const int16_t MAX_VALID_MOTION = 1000;  // 位移传感器单次读数最大有效值
    const float MAX_YAW_CHANGE = 20.0f;     // 单次最大有效角度变化(度)
    
    // 防抖动处理参数
    const float MOTION_DEADZONE = 2.0f;     // 位移传感器死区阈值
    const float LOW_PASS_ALPHA = 0.7f;      // 低通滤波系数(0-1)，越大滤波越强
    
    // 滤波后的数据
    float filteredDx = 0.0f;
    float filteredDy = 0.0f;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        // 获取传感器数据
        Motion_Burst(&dx, &dy);
        float rawYaw = HWT101.getZ(); // 获取原始角度值（-180到180度）
        
        // 传感器数据有效性检查
        bool validMotion = true;
        bool validYaw = true;
        
        // 检查位移传感器数据是否在合理范围内
        if (abs(dx) > MAX_VALID_MOTION || abs(dy) > MAX_VALID_MOTION) {
            validMotion = false;
            // 异常数据，重置为0
            dx = 0;
            dy = 0;
        }
        
        // 检查角度传感器数据是否在合理范围内(排除±180°跳变的情况)
        float yawDiff = rawYaw - prevYaw;
        if (abs(yawDiff) > MAX_YAW_CHANGE && 
            abs(abs(yawDiff) - 360.0f) > MAX_YAW_CHANGE) {
            validYaw = false;
            // 异常数据，使用上次的角度值
            rawYaw = prevYaw;
        }

        // 检测是否发生了跨越±180°的跳变
        if (prevYaw > 150.0f && rawYaw < -150.0f) // 从正半圈跳到负半圈（顺时针转过了180°）
            rotationCount++;                      // 顺时针转过了一圈
        else if (prevYaw < -150.0f && rawYaw > 150.0f) // 从负半圈跳到正半圈（逆时针转过了180°）
            rotationCount--;                           // 逆时针转过了一圈

        // 将角度转换为弧度
        float yawRad = rawYaw * DEG_TO_RAD;
        float sinYaw, cosYaw;
        sincosf(yawRad, &sinYaw, &cosYaw); // 使用sincosf同时计算sin和cos，提高效率

        // 计算实际位移（考虑传感器方向与车身方向的关系）
        float dxActual = -dy * scaleFactor;
        float dyActual = dx * scaleFactor;
        
        // 防抖动处理 - 应用死区
        if (abs(dxActual) < MOTION_DEADZONE) {
            dxActual = 0.0f;
        }
        if (abs(dyActual) < MOTION_DEADZONE) {
            dyActual = 0.0f;
        }
        
        // 低通滤波处理
        filteredDx = LOW_PASS_ALPHA * filteredDx + (1.0f - LOW_PASS_ALPHA) * dxActual;
        filteredDy = LOW_PASS_ALPHA * filteredDy + (1.0f - LOW_PASS_ALPHA) * dyActual;

        // 使用旋转矩阵将局部坐标系的变化转换到全局坐标系
        float dxGlobal = filteredDx * cosYaw - filteredDy * sinYaw;
        float dyGlobal = filteredDx * sinYaw + filteredDy * cosYaw;

        // 计算连续角度 = 原始角度 + 旋转圈数 * 360°
        float continuousYaw = rawYaw + rotationCount * 360.0f;

        // 只有在数据有效的情况下才累加位移
        if (validMotion && validYaw) {
            // 累加未能更新的位移
            pendingDx += dxGlobal;
            pendingDy += dyGlobal;
        }

        // 更新全局坐标
        if (xSemaphoreTake(positionMutex, 10) == pdTRUE)
        {
            // 更新位置，包括之前未能更新的增量
            currentPosition.x += pendingDx;
            currentPosition.y += pendingDy;
            currentPosition.rawYaw = rawYaw;
            currentPosition.continuousYaw = continuousYaw;
            
            // 清零累积的位移增量
            pendingDx = 0.0f;
            pendingDy = 0.0f;
            
            xSemaphoreGive(positionMutex);
        }

        // 更新前一次的角度值
        prevYaw = rawYaw;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

void getGlobalPosition(global_position_t *position)
{
    /*获取全局位置*/
    // 使用互斥锁保护全局位置数据的读取
    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE)
    {
        // 在获取到互斥锁后，复制全局位置数据
        position->x = currentPosition.x;
        position->y = currentPosition.y;
        position->rawYaw = currentPosition.rawYaw;
        position->continuousYaw = currentPosition.continuousYaw;

        // 释放互斥锁
        xSemaphoreGive(positionMutex);
    }
}
