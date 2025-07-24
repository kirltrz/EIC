#include "sensor.h"
#include "taskManager.h"
#include "config.h"
#include "motion.h"
#include "VOFAdebug.h"

SemaphoreHandle_t positionMutex = NULL; // 全局坐标互斥锁
global_position_t currentPosition;      // 用于积分运算

// 内部旋转圈数跟踪（不对外暴露）
static int rotationCount = 0; // 旋转圈数，正值表示顺时针旋转的圈数，负值表示逆时针

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

    while (retryCount < 5 && !mutexAcquired)
    {
        if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            // 清零全局位置数据
            currentPosition.x = 0;
            currentPosition.y = 0;
            currentPosition.rawYaw = 0;
            currentPosition.continuousYaw = 0;
            rotationCount = 0; // 清零旋转圈数

            xSemaphoreGive(positionMutex);
            DEBUG_LOG("全局位置数据已清零");
            mutexAcquired = true;
        }
        else
        {
            DEBUG_LOG("获取位置互斥锁超时，重试 %d/5", retryCount + 1);
            retryCount++;
            delay(20);
        }
    }

    if (!mutexAcquired)
    {
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

    for (int i = 0; i < RESET_CHECK_COUNT; i++)
    {
        if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            float x = currentPosition.x;
            float y = currentPosition.y;
            float rawYaw = currentPosition.rawYaw;
            xSemaphoreGive(positionMutex);

            if (fabs(x) < RESET_THRESHOLD && fabs(y) < RESET_THRESHOLD && fabs(rawYaw) < RESET_THRESHOLD)
            {
                zeroCount++;
            }

            DEBUG_LOG("检查 %d: x=%.3f, y=%.3f, yaw=%.3f", i + 1, x, y, rawYaw);
        }
        else
        {
            DEBUG_LOG("检查 %d: 获取互斥锁失败", i + 1);
        }
        delay(RESET_CHECK_DELAY);
    }

    if (zeroCount >= RESET_FAIL_COUNT)
    {
        DEBUG_LOG("传感器重置成功 (零值计数: %d/%d)", zeroCount, RESET_CHECK_COUNT);
    }
    else
    {
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

void calculateGlobalPosition(/*void *pvParameters*/)
{
    /*计算全局位置*/
    static float prevYaw = HWT101.getZ(); // 上一次的yaw角度
    static uint16_t lastTime = millis();
    float dx = 0, dy = 0;
    double vx_global = 0, vy_global = 0;
    uint16_t dt = 0;
    int motorVelocity[4] = {0};
    double linerVelocity[4] = {0};

    float rawYaw = HWT101.getZ(); // 获取原始角度值（-180到180度）
    for (int i = 1; i <= 4; i++)
    {
        motorVelocity[i - 1] = motor->getVelocity(i);
        linerVelocity[i - 1] = ((double)motorVelocity[i - 1] / 60.0) * 2.0 * PI * WHEEL_RADIUS;
        //delay(2);
    }
    // 计算底盘坐标系下的速度
    double vx_chassis = (linerVelocity[1] - linerVelocity[3]) / 2.0; // x方向速度
    double vy_chassis = (linerVelocity[0] - linerVelocity[2]) / 2.0; // y方向速度

    if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(15)) == pdTRUE)
    {
        // 检测是否发生了跨越±180°的跳变
        if (prevYaw > 150.0f && rawYaw < -150.0f)      // 从正半圈跳到负半圈（顺时针转过了180°）
            rotationCount++;                           // 顺时针转过了一圈
        else if (prevYaw < -150.0f && rawYaw > 150.0f) // 从负半圈跳到正半圈（逆时针转过了180°）
            rotationCount--;                           // 逆时针转过了一圈
        // 计算连续角度 = 原始角度 + 旋转圈数 * 360°
        float continuousYaw = rawYaw + rotationCount * 360.0f;

        vx_global = vx_chassis * cos(continuousYaw*DEG_TO_RAD) - vy_chassis * sin(continuousYaw*DEG_TO_RAD);
        vy_global = vx_chassis * sin(continuousYaw*DEG_TO_RAD) + vy_chassis * cos(continuousYaw*DEG_TO_RAD);
        dt = millis() - lastTime;
        dx = vx_global * dt * 0.001;
        dy = vy_global * dt * 0.001;
        lastTime = millis();

        // 使用原始积分位置
        currentPosition.x += dx;
        currentPosition.y += dy;

        currentPosition.rawYaw = rawYaw;
        currentPosition.continuousYaw = continuousYaw;

        /*sendDebugValues(
            currentPosition.x,
            currentPosition.y,
            currentPosition.rawYaw,
            currentPosition.continuousYaw,
            dx, dy,
            vx_global, vy_global,
            vx_chassis, vy_chassis,
            dt,
            motorVelocity[0], motorVelocity[1], motorVelocity[2], motorVelocity[3]
        );*/
        
        xSemaphoreGive(positionMutex);
    }
    // 更新前一次的角度值
    prevYaw = rawYaw;

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

bool isSensorResetInProgress(void)
{
    /*检查传感器是否正在重置*/
    return sensorResetInProgress;
}
