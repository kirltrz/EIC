#include "sensor.h"
#include "taskManager.h"
/*paw3395参数及针脚设置*/
#define DPI 26000
#define NRESET 2
#define NCS 42
#define SCLK 39
#define MISO 41
#define MOSI 40

SemaphoreHandle_t positionMutex = NULL; // 全局坐标互斥锁

void initSensor(void)
{
    /*初始化传感器*/
    positionMutex = xSemaphoreCreateMutex(); // 创建互斥锁
    initVision();
    Wire.begin();
    paw3395Init(DPI, NRESET, NCS, SCLK, MISO, MOSI);
}

bool checkPaw3395(void)
{
    /*检查paw3395是否正常工作*/
    return paw3395_check();
}

bool checkHWT101(void)
{
    /*检查hwt101是否正常工作*/
    Wire.beginTransmission(80);
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

global_position_t currentPosition;//用于积分运算
void calculateGlobalPosition()
{
    /*计算全局位置*/
    float prevYaw = 0.0f;       // 上一次的yaw角度
    float yawDiff = 0.0f;       // 角度变化量
    float continuousYaw = 0.0f; // 连续的角度（不限于±180°）
    bool isFirstReading = true; // 是否是第一次读取

    volatile int16_t dx = 0;
    volatile int16_t dy = 0;
    volatile float globalX = 0.0f;
    volatile float globalY = 0.0f;

    const float scaleFactor = 0.0009769f; // 比例因子，将传感器读数转换为实际位移

    while (true)
    {
        Motion_Burst(&dx, &dy);
        float rawYaw = HWT101.getZ(); // 获取原始角度值（-180到180度）

        // 第一次读取时初始化
        if (isFirstReading)
        {
            prevYaw = rawYaw;
            continuousYaw = rawYaw;
            isFirstReading = false;
        }
        else
        {
            // 处理角度跳变，计算最短角度差
            yawDiff = rawYaw - prevYaw;

            // 处理±180°附近的跳变
            if (yawDiff > 180.0f)
            {
                yawDiff -= 360.0f; // 例如: 从-179°到+179°，差值应为-2°而不是358°
            }
            else if (yawDiff < -180.0f)
            {
                yawDiff += 360.0f; // 例如: 从+179°到-179°，差值应为+2°而不是-358°
            }

            // 累加变化量得到连续角度
            continuousYaw += yawDiff;
            prevYaw = rawYaw;
        }

        // 将角度转换为弧度（使用原始角度即可，因为sin/cos函数对于±180°是连续的）
        float yawRad = continuousYaw * DEG_TO_RAD;

        // 计算实际位移（考虑传感器方向与车身方向的关系）
        float dxActual = dx; // dx可能需要取反，取决于传感器安装方向
        float dyActual = dy; // dy可能需要取反，取决于传感器安装方向

        // 使用旋转矩阵将局部坐标系的变化转换到全局坐标系
        float dxGlobal = dxActual * cos(yawRad) - dyActual * sin(yawRad);
        float dyGlobal = dxActual * sin(yawRad) + dyActual * cos(yawRad);

        // 更新全局坐标（乘以比例因子，将像素变化转换为实际距离变化）
        if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE)
        {
            currentPosition.x += dxGlobal * scaleFactor;
            currentPosition.y += dyGlobal * scaleFactor;
            currentPosition.rawYaw = rawYaw;
            currentPosition.continuousYaw = continuousYaw;
            xSemaphoreGive(positionMutex); // 释放互斥锁
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);
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
