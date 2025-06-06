#include "sensor.h"
#include "taskManager.h"
#include "config.h"

SemaphoreHandle_t positionMutex = NULL; // 全局坐标互斥锁
global_position_t currentPosition; // 用于积分运算

// 卡尔曼滤波器结构体 - 用于位置滤波
typedef struct {
    // 状态向量 [x, y, vx, vy]
    float state[4];
    // 状态协方差矩阵 P[4][4]
    float P[4][4];
    // 过程噪声协方差 Q[4][4]
    float Q[4][4];
    // 测量噪声协方差 R[2][2]
    float R[2][2];
    // 卡尔曼增益 K[4][2]
    float K[4][2];
    // 采样时间间隔(秒)
    float dt;
} KalmanFilter_t;

// 全局卡尔曼滤波器实例
KalmanFilter_t positionKalman;

// 初始化卡尔曼滤波器
void initKalmanFilter(KalmanFilter_t *kf, float dt, float processNoise, float measurementNoise)
{
    // 初始化状态向量
    for (int i = 0; i < 4; i++) {
        kf->state[i] = 0.0f;
    }
    
    // 初始化协方差矩阵P (初始不确定性)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] = (i == j) ? 1000.0f : 0.0f; // 对角线设置较大值表示初始不确定性高
        }
    }
    
    // 初始化过程噪声协方差矩阵Q
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->Q[i][j] = 0.0f;
        }
    }
    // 位置过程噪声
    kf->Q[0][0] = kf->Q[1][1] = 0.01f * processNoise;
    // 速度过程噪声
    kf->Q[2][2] = kf->Q[3][3] = 0.1f * processNoise;
    
    // 初始化测量噪声协方差矩阵R
    kf->R[0][0] = kf->R[1][1] = measurementNoise;
    kf->R[0][1] = kf->R[1][0] = 0.0f;
    
    // 设置采样时间
    kf->dt = dt;
}

// 卡尔曼滤波预测步骤
void kalmanPredict(KalmanFilter_t *kf)
{
    // 临时变量
    float F[4][4] = {0}; // 状态转移矩阵
    float FP[4][4] = {0}; // 中间计算结果
    
    // 构建状态转移矩阵F
    // [1, 0, dt, 0]
    // [0, 1, 0, dt]
    // [0, 0, 1, 0]
    // [0, 0, 0, 1]
    F[0][0] = F[1][1] = F[2][2] = F[3][3] = 1.0f;
    F[0][2] = F[1][3] = kf->dt;
    
    // 状态预测: x = F*x
    float predicted_x = kf->state[0] + kf->state[2] * kf->dt;
    float predicted_y = kf->state[1] + kf->state[3] * kf->dt;
    kf->state[0] = predicted_x;
    kf->state[1] = predicted_y;
    // 速度保持不变
    
    // 协方差预测: P = F*P*F' + Q
    // 计算F*P
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            FP[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                FP[i][j] += F[i][k] * kf->P[k][j];
            }
        }
    }
    
    // 计算F*P*F' + Q
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] = kf->Q[i][j];
            for (int k = 0; k < 4; k++) {
                kf->P[i][j] += FP[i][k] * F[j][k]; // 注意这里F是转置的
            }
        }
    }
}

// 卡尔曼滤波更新步骤
void kalmanUpdate(KalmanFilter_t *kf, float measurement[2])
{
    // 测量矩阵H [2x4]
    // [1, 0, 0, 0]
    // [0, 1, 0, 0]
    
    // 计算测量残差: y = z - H*x
    float y[2];
    y[0] = measurement[0] - kf->state[0];
    y[1] = measurement[1] - kf->state[1];
    
    // 计算残差协方差: S = H*P*H' + R
    float S[2][2];
    S[0][0] = kf->P[0][0] + kf->R[0][0];
    S[0][1] = kf->P[0][1] + kf->R[0][1];
    S[1][0] = kf->P[1][0] + kf->R[1][0];
    S[1][1] = kf->P[1][1] + kf->R[1][1];
    
    // 计算S的行列式
    float detS = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    
    // 安全检查，确保S可逆
    if (fabsf(detS) < 1e-10f) {
        return; // 如果S接近奇异，放弃本次更新
    }
    
    // 计算S的逆
    float invS[2][2];
    invS[0][0] = S[1][1] / detS;
    invS[0][1] = -S[0][1] / detS;
    invS[1][0] = -S[1][0] / detS;
    invS[1][1] = S[0][0] / detS;
    
    // 计算卡尔曼增益: K = P*H'*inv(S)
    for (int i = 0; i < 4; i++) {
        kf->K[i][0] = kf->P[i][0] * invS[0][0] + kf->P[i][1] * invS[1][0];
        kf->K[i][1] = kf->P[i][0] * invS[0][1] + kf->P[i][1] * invS[1][1];
    }
    
    // 更新状态: x = x + K*y
    for (int i = 0; i < 4; i++) {
        kf->state[i] += kf->K[i][0] * y[0] + kf->K[i][1] * y[1];
    }
    
    // 更新协方差: P = (I - K*H)*P
    // 由于H的特殊结构，可以简化计算
    float IKH[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        IKH[i][i] = 1.0f;
    }
    IKH[0][0] -= kf->K[0][0];
    IKH[0][1] -= kf->K[0][1];
    IKH[1][0] -= kf->K[1][0];
    IKH[1][1] -= kf->K[1][1];
    IKH[2][0] -= kf->K[2][0];
    IKH[2][1] -= kf->K[2][1];
    IKH[3][0] -= kf->K[3][0];
    IKH[3][1] -= kf->K[3][1];
    
    // 计算(I - K*H)*P
    float newP[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                newP[i][j] += IKH[i][k] * kf->P[k][j];
            }
        }
    }
    
    // 更新P
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] = newP[i][j];
        }
    }
}

void initSensor(void)
{
    /*初始化传感器*/
    positionMutex = xSemaphoreCreateMutex(); // 创建全局坐标互斥锁
    visionInit();
    Wire.begin(PIN_HWT101_SDA, PIN_HWT101_SCL); // 初始化IIC(HWT101)
    paw3395Init(PAW3395_DPI, PIN_PAW3395_NRESET, PIN_PAW3395_NCS, PIN_PAW3395_SCLK, PIN_PAW3395_MISO, PIN_PAW3395_MOSI);
    delay(100);
    
    // 初始化卡尔曼滤波器 (5ms采样周期，适当的过程噪声和测量噪声值)
    initKalmanFilter(&positionKalman, 0.005f, 8.0f, 0.05f);
    
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
        
        // 重置卡尔曼滤波器状态
        for (int i = 0; i < 4; i++) {
            positionKalman.state[i] = 0.0f;
        }
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
    const int16_t MAX_VALID_MOTION = INT16_MAX;  // 位移传感器单次读数最大有效值
    const float MAX_YAW_CHANGE = 20.0f;     // 单次最大有效角度变化(度)
    
    // 防抖动处理参数
    const float MOTION_DEADZONE = 5.0f;     // 位移传感器死区阈值
    const float LOW_PASS_ALPHA = 0.8f;      // 低通滤波系数(0-1)，越大滤波越强
    
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
        
        /*
        // 防抖动处理 - 应用死区
        if (abs(dx) < MOTION_DEADZONE) {
            dx = 0;
        }
        if (abs(dy) < MOTION_DEADZONE) {
            dy = 0;
        }
        */
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

        // 卡尔曼滤波器预测步骤
        //kalmanPredict(&positionKalman);
        
        // 更新全局坐标
        if (xSemaphoreTake(positionMutex, 10) == pdTRUE)
        {
            // 原始积分位置(作为卡尔曼滤波的测量值)
            float rawPositionX = currentPosition.x + pendingDx;
            float rawPositionY = currentPosition.y + pendingDy;
            
            // 卡尔曼滤波器更新步骤
            //float measurement[2] = {rawPositionX, rawPositionY};
            //kalmanUpdate(&positionKalman, measurement);
            
            // 使用卡尔曼滤波后的位置作为最终结果
            /*currentPosition.x = positionKalman.state[0];
            currentPosition.y = positionKalman.state[1];
*/
            currentPosition.x += pendingDx;
            currentPosition.y += pendingDy;

            currentPosition.v = sqrtf(pendingDx * pendingDx + pendingDy * pendingDy);

            currentPosition.rawYaw = rawYaw;
            currentPosition.continuousYaw = continuousYaw;

            /*
            float debugData[5] = {currentPosition.x, currentPosition.y, pendingDx, pendingDy, rawYaw};
            Serial.write((char*)debugData, sizeof(debugData));
            char tail[4] = {0x00, 0x00, 0x80, 0x7f};
            Serial.write(tail, sizeof(tail));
*/
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
