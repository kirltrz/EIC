#include "sensor.h"
#include "taskManager.h"
#include "config.h"
#include "VOFAdebug.h"

SemaphoreHandle_t positionMutex = NULL; // 全局坐标互斥锁
global_position_t currentPosition; // 用于积分运算

// 内部旋转圈数跟踪（不对外暴露）
static int rotationCount = 0;          // 旋转圈数，正值表示顺时针旋转的圈数，负值表示逆时针

// 传感器重置控制标志
volatile bool sensorResetInProgress = false;

void initSensor(void)
{
    /*初始化传感器*/
    positionMutex = xSemaphoreCreateMutex(); // 创建全局坐标互斥锁
    
    // 初始化全局位置数据
    currentPosition.x = 0;
    currentPosition.y = 0;
    currentPosition.rawYaw = 0;
    currentPosition.continuousYaw = 0;
    rotationCount = 0; // 初始化旋转圈数
    
    visionInit();
    Wire.begin(PIN_HWT101_SDA, PIN_HWT101_SCL); // 初始化IIC(HWT101)
    paw3395Init(PAW3395_DPI, PIN_PAW3395_NRESET, PIN_PAW3395_NCS, PIN_PAW3395_SCLK, PIN_PAW3395_MISO, PIN_PAW3395_MOSI);
    delay(100);
    
    // 移除resetSensor()调用，避免在启动时等待视觉模块响应导致延迟
    // resetSensor()会在主流程开始时调用
}

void resetSensor(void)
{
    const int RESET_CHECK_COUNT = 8;     // 检查次数
    const int RESET_CHECK_DELAY = 15;    // 每次检查间隔(ms)
    const float RESET_THRESHOLD = 0.15f; // 判断为0的阈值
    const int RESET_FAIL_COUNT = 6;      // 超过多少次为0则认为reset成功

    DEBUG_LOG("开始重置传感器...");
    
    // 设置重置标志，通知其他任务
    sensorResetInProgress = true;
    
    // 等待位置计算任务感知到重置标志
    delay(20);
    
    // 重置传感器硬件
    visionToIDLE();
    HWT101.toZero();

    // 等待传感器稳定
    delay(80);

    // 强制重置全局位置数据（使用更短的超时时间）
    bool mutexAcquired = false;
    int retryCount = 0;
    
    while (retryCount < 5 && !mutexAcquired) {
        if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // 清零全局位置数据
            currentPosition.x = 0;
            currentPosition.y = 0;
            currentPosition.rawYaw = 0;
            currentPosition.continuousYaw = 0;
            rotationCount = 0; // 清零旋转圈数
            
            xSemaphoreGive(positionMutex);
            DEBUG_LOG("全局位置数据已清零");
            mutexAcquired = true;
        } else {
            DEBUG_LOG("获取位置互斥锁超时，重试 %d/5", retryCount + 1);
            retryCount++;
            delay(20);
        }
    }
    
    if (!mutexAcquired) {
        DEBUG_LOG("无法获取位置互斥锁，强制重置");
        // 即使无法获取互斥锁，也要强制重置
        currentPosition.x = 0;
        currentPosition.y = 0;
        currentPosition.rawYaw = 0;
        currentPosition.continuousYaw = 0;
        rotationCount = 0;
    }

    // 延迟一段时间，等待传感器稳定和位置计算任务更新
    delay(150);

    // 检查reset是否成功
    int zeroCount = 0;
    
    for (int i = 0; i < RESET_CHECK_COUNT; i++) {
        if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            float x = currentPosition.x;
            float y = currentPosition.y;
            float rawYaw = currentPosition.rawYaw;
            xSemaphoreGive(positionMutex);

            if (fabs(x) < RESET_THRESHOLD && fabs(y) < RESET_THRESHOLD && fabs(rawYaw) < RESET_THRESHOLD) {
                zeroCount++;
            }
            
            DEBUG_LOG("检查 %d: x=%.3f, y=%.3f, yaw=%.3f", i+1, x, y, rawYaw);
        } else {
            DEBUG_LOG("检查 %d: 获取互斥锁失败", i+1);
        }
        delay(RESET_CHECK_DELAY);
    }

    if (zeroCount >= RESET_FAIL_COUNT) {
        DEBUG_LOG("传感器重置成功 (零值计数: %d/%d)", zeroCount, RESET_CHECK_COUNT);
    } else {
        DEBUG_LOG("传感器重置可能不完整 (零值计数: %d/%d)，但继续执行", zeroCount, RESET_CHECK_COUNT);
    }
    
    // 等待一段时间确保所有任务都感知到重置完成
    delay(20);
    
    // 清除重置标志
    sensorResetInProgress = false;
    
    // 再次等待确保标志清除生效
    delay(20);
    
    DEBUG_LOG("传感器重置流程结束");
}

void calculateGlobalPosition(void *pvParameters)
{
    /*计算全局位置*/
    float prevYaw = HWT101.getZ();  // 上一次的yaw角度

    int16_t dx = 0;
    int16_t dy = 0;
    const float scaleFactor = 0.0009769f; // 26000CPI下的理论比例因子，将传感器读数转换为实际位移
    const float scaleFactorX = scaleFactor * (300.0f /*实际位移*/ / 285.0f /*传感器读数*/);
    const float scaleFactorY = scaleFactor * (300.0f /*实际位移*/ / 295.0f /*传感器读数*/);

    // 未能更新的位移增量
    float pendingDx = 0.0f;
    float pendingDy = 0.0f;

    // 传感器数据有效性检查阈值
    const int16_t MAX_VALID_MOTION = INT16_MAX;  // 位移传感器单次读数最大有效值
    const float MAX_YAW_CHANGE = 20.0f;     // 单次最大有效角度变化(度)
    
    // 防抖动处理参数
    const float MOTION_DEADZONE = 5.0f;     // 位移传感器死区阈值
    const float LOW_PASS_ALPHA = 0.5f;      // 低通滤波系数(0-1)，越大滤波越强
    
    // 滤波后的数据
    float filteredDx = 0.0f;
    float filteredDy = 0.0f;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        // 检查是否正在进行传感器重置
        if (sensorResetInProgress) {
            // 重置期间暂停位置计算，避免数据冲突
            // 同时重置内部状态，确保重置后从干净状态开始
            prevYaw = HWT101.getZ(); // 重新获取当前角度作为起始点
            pendingDx = 0.0f;
            pendingDy = 0.0f;
            filteredDx = 0.0f;
            filteredDy = 0.0f;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // 获取传感器数据
        Motion_Burst(&dx, &dy);
        //sendDebugValuesUDP(sqrt(dx*dx+dy*dy));
        //sendDebugValues(dx,dy);
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

        // 将角度转换为弧度
        float yawRad = rawYaw * DEG_TO_RAD;
        float sinYaw, cosYaw;
        sincosf(yawRad, &sinYaw, &cosYaw); // 使用sincosf同时计算sin和cos，提高效率
        
        // 应用传感器方向配置、比例因子
        float dxActual, dyActual;
        if (PAW3395_SWAP_XY) {
            dxActual = dy * PAW3395_DIRECTION_X_SCALE *scaleFactorX;
            dyActual = dx * PAW3395_DIRECTION_Y_SCALE *scaleFactorY;
        } else {
            dxActual = dx * PAW3395_DIRECTION_X_SCALE *scaleFactorX;
            dyActual = dy * PAW3395_DIRECTION_Y_SCALE *scaleFactorY;
        }
        
        // 低通滤波处理
        filteredDx = LOW_PASS_ALPHA * filteredDx + (1.0f - LOW_PASS_ALPHA) * dxActual;
        filteredDy = LOW_PASS_ALPHA * filteredDy + (1.0f - LOW_PASS_ALPHA) * dyActual;

        // 使用旋转矩阵将局部坐标系的变化转换到全局坐标系
        float dxGlobal = filteredDx * cosYaw - filteredDy * sinYaw;
        float dyGlobal = filteredDx * sinYaw + filteredDy * cosYaw;

        // 只有在数据有效的情况下才累加位移
        if (validMotion && validYaw) {
            // 累加未能更新的位移
            pendingDx += dxGlobal;
            pendingDy += dyGlobal;
        }
        
        // 更新全局坐标（使用更短的超时时间，避免长时间阻塞）
        if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
            // 检测是否发生了跨越±180°的跳变
            if (prevYaw > 150.0f && rawYaw < -150.0f) // 从正半圈跳到负半圈（顺时针转过了180°）
                rotationCount++;                      // 顺时针转过了一圈
            else if (prevYaw < -150.0f && rawYaw > 150.0f) // 从负半圈跳到正半圈（逆时针转过了180°）
                rotationCount--;                           // 逆时针转过了一圈

            // 计算连续角度 = 原始角度 + 旋转圈数 * 360°
            float continuousYaw = rawYaw + rotationCount * 360.0f;
            
            // 使用原始积分位置
            currentPosition.x += pendingDx;
            currentPosition.y += pendingDy;

            currentPosition.rawYaw = rawYaw;
            currentPosition.continuousYaw = continuousYaw;
            
            // 清零累积的位移增量
            pendingDx = 0.0f;
            pendingDy = 0.0f;
            
            xSemaphoreGive(positionMutex);
        } else {
            // 如果获取互斥锁失败，保留pendingDx和pendingDy，下次再尝试更新
            // 这样可以避免数据丢失，同时不会阻塞任务
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
    if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // 在获取到互斥锁后，复制全局位置数据
        position->x = currentPosition.x;
        position->y = currentPosition.y;
        position->rawYaw = currentPosition.rawYaw;
        position->continuousYaw = currentPosition.continuousYaw;

        // 释放互斥锁
        xSemaphoreGive(positionMutex);
    }
}
void setGlobalPosition(float x, float y){
    sensorResetInProgress = true;
    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
        currentPosition.x = x;
        currentPosition.y = y;
        xSemaphoreGive(positionMutex);
        sensorResetInProgress = false;
    }
}

bool isSensorResetInProgress(void)
{
    /*检查传感器是否正在重置*/
    return sensorResetInProgress;
}
