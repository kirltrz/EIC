#include "devMonitor.h"
#include "Wire.h"
#include "HWT101.h"
#include "vision.h"
#include "motion.h"
#include "arm.h"
#include "PAW3395.h"
#include "sensor.h"
#include "config.h"
#include <Arduino.h>
#include <ESP.h>
#include <math.h>

deviceStatus_t devStatus;
SemaphoreHandle_t devStatusMutex = NULL;

taskProgress_t taskProgress = TASK_IDLE;
SemaphoreHandle_t taskProgressMutex = NULL;

// 添加设备检测任务句柄和标志
TaskHandle_t deviceCheckTasks[5] = {NULL};
volatile bool deviceCheckInProgress = false;

// 位置数据监测相关变量
static unsigned long lastPositionUpdateTime = 0;
static global_position_t lastPosition = {0, 0, 0, 0};
static const unsigned long POSITION_DEADLOCK_TIMEOUT = 5000; // 10秒超时
static const float POSITION_CHANGE_THRESHOLD = 0.1f; // 位置变化阈值（毫米）
static bool positionMonitorEnabled = false;

bool checkPaw3395(void)
{
    /*检查paw3395是否正常工作*/
    return paw3395_check();
}

bool checkHWT101(void)
{
    /*检查hwt101是否正常工作*/
    Wire.beginTransmission(HWT101.devAddr);
    delay(2); // 减少延时从5ms到2ms
    return Wire.endTransmission() == 0;
}

bool checkVision(void)
{
    /*检查与视觉模块串口通信是否正常，设备监测本身有重试机制故该函数不需重试*/
    
    // 清空串口缓冲区中可能存在的旧数据，确保获取最新响应
    while (VISION_SERIAL.available() > 0)
    {
        VISION_SERIAL.read();
    }
    
    sendCommand(CMD_IDLE);
    delay(100);
    vision_packet_t data;
    readVisionCache(&data);
    return data.mode == CMD_IDLE;
}

bool checkMotor(void)
{
    /*检查电机是否正常工作*/
    return motor->getVoltage(1) > 1000 && motor->getVoltage(2) > 1000 && motor->getVoltage(3) > 1000 && motor->getVoltage(4) > 1000;
}

bool checkServo(void)
{
    /*检查舵机是否正常工作*/
    return servo0.ping() && servo1.ping() && servo2.ping() && servo3.ping() && servo4.ping();
}

bool checkPositionData(void)
{
    /*检查位置数据是否锁死*/
    
    // 如果位置监测未启用，则始终返回true
    if (!positionMonitorEnabled) {
        return true;
    }
    
    // 如果传感器正在重置，暂停监测
    if (isSensorResetInProgress()) {
        lastPositionUpdateTime = millis(); // 重置时间戳
        return true;
    }
    
    global_position_t currentPosition;
    getGlobalPosition(&currentPosition);
    
    unsigned long currentTime = millis();
    
    // 计算位置变化量
    float deltaX = fabs(currentPosition.x - lastPosition.x);
    float deltaY = fabs(currentPosition.y - lastPosition.y);
    float deltaYaw = fabs(currentPosition.rawYaw - lastPosition.rawYaw);
    
    // 检查是否有显著的位置变化
    bool positionChanged = (deltaX > POSITION_CHANGE_THRESHOLD) || 
                          (deltaY > POSITION_CHANGE_THRESHOLD) || 
                          (deltaYaw > 0.5f); // 角度变化阈值0.5度
    
    if (positionChanged) {
        // 位置有变化，更新时间戳和位置记录
        lastPositionUpdateTime = currentTime;
        lastPosition = currentPosition;
        return true;
    }
    
    // 检查是否超时
    if (currentTime - lastPositionUpdateTime > POSITION_DEADLOCK_TIMEOUT) {
        DEBUG_LOG("位置数据锁死检测：超过%lums未更新，当前位置(%.3f, %.3f, %.3f)", 
                  POSITION_DEADLOCK_TIMEOUT, 
                  currentPosition.x, currentPosition.y, currentPosition.rawYaw);
        
        // 检查是否真的是全零状态（可能的锁死情况）
        if (fabs(currentPosition.x) < 0.01f && 
            fabs(currentPosition.y) < 0.01f && 
            fabs(currentPosition.rawYaw) < 0.01f) {
            DEBUG_LOG("检测到位置数据全零锁死，准备重启系统");
            delay(100); // 确保日志输出
            ESP.restart(); // 立即重启
        }
        
        // 即使不是全零，超时也要重置监测时间，避免反复触发
        lastPositionUpdateTime = currentTime;
        return false;
    }
    
    return true;
}

void getDeviceStatus(deviceStatus_t *status) 
{
    if (status == NULL || devStatusMutex == NULL) {
        return;
    }
    
    if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(100))) {
        *status = devStatus;
        xSemaphoreGive(devStatusMutex);
    }
}

void updateTaskProgress(taskProgress_t progress)
{
    if (taskProgressMutex != NULL) {
        if (xSemaphoreTake(taskProgressMutex, pdMS_TO_TICKS(100))) {
            taskProgress = progress;
            xSemaphoreGive(taskProgressMutex);
        }
    }
}

uint8_t getTaskProgressPercent(void)
{
    uint8_t percent = 0;
    
    if (taskProgressMutex != NULL) {
        if (xSemaphoreTake(taskProgressMutex, pdMS_TO_TICKS(50))) {
            // 根据任务进度计算百分比
            switch (taskProgress) {
                case TASK_IDLE:
                    percent = 0;
                    break;
                case TASK_RESET_SENSOR:
                    percent = 5;
                    break;
                case TASK_SCAN_QRCODE:
                    percent = 15;
                    break;
                case TASK_FIRST_TURNTABLE:
                    percent = 25;
                    break;
                case TASK_FIRST_ROUGH:
                    percent = 35;
                    break;
                case TASK_FIRST_STORAGE:
                    percent = 50;
                    break;
                case TASK_SECOND_TURNTABLE:
                    percent = 60;
                    break;
                case TASK_SECOND_ROUGH:
                    percent = 75;
                    break;
                case TASK_SECOND_STORAGE:
                    percent = 85;
                    break;
                case TASK_RETURN_HOME:
                    percent = 95;
                    break;
                case TASK_COMPLETED:
                    percent = 100;
                    break;
                default:
                    percent = 0;
                    break;
            }
            xSemaphoreGive(taskProgressMutex);
        }
    }
    
    return percent;
}

// 并行设备检测任务函数
void deviceCheckTask(void *pvParameters)
{
    int deviceType = (int)pvParameters;
    bool result = false;
    
    // 添加延时避免任务冲突
    vTaskDelay(pdMS_TO_TICKS(10 * deviceType));
    
    switch (deviceType) {
        case 0: // HWT101
            result = checkHWT101();
            if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(100))) {
                devStatus.hwt101_status = result;
                xSemaphoreGive(devStatusMutex);
            }
            break;
        case 1: // PAW3395
            result = checkPaw3395();
            if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(100))) {
                devStatus.paw3395_status = result;
                xSemaphoreGive(devStatusMutex);
            }
            break;
        case 2: // Vision
            result = checkVision();
            if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(100))) {
                devStatus.vision_status = result;
                xSemaphoreGive(devStatusMutex);
            }
            break;
        case 3: // Motor
            // 延迟200ms再检测电机，让电机系统稳定
            vTaskDelay(pdMS_TO_TICKS(200));
            result = checkMotor();
            if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(100))) {
                devStatus.motor_status = result;
                xSemaphoreGive(devStatusMutex);
            }
            break;
        case 4: // Servo
            result = checkServo();
            if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(100))) {
                devStatus.servo_status = result;
                xSemaphoreGive(devStatusMutex);
            }
            break;
    }
    
    deviceCheckTasks[deviceType] = NULL; // 清除任务句柄
    vTaskDelete(NULL);
}

bool checkDevice(void){
    if (devStatusMutex == NULL) return false;
    
    // 检查位置数据状态
    bool positionOk = checkPositionData();
    if (!positionOk) {
        DEBUG_LOG("位置数据检测失败");
    }
    
    // 如果正在进行设备检测，直接返回当前状态
    if (deviceCheckInProgress) {
        if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(10))) {
            bool allOk = devStatus.hwt101_status && devStatus.paw3395_status && 
                        devStatus.vision_status && devStatus.motor_status && devStatus.servo_status;
            xSemaphoreGive(devStatusMutex);
            return allOk && positionOk;
        }
        return false;
    }
    
    if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(100))) {
        // 检查哪些设备需要重新检测
        bool needCheckHWT101 = !devStatus.hwt101_status;
        bool needCheckPAW3395 = !devStatus.paw3395_status;
        bool needCheckVision = !devStatus.vision_status;
        bool needCheckMotor = !devStatus.motor_status;
        bool needCheckServo = !devStatus.servo_status;
        
        xSemaphoreGive(devStatusMutex);
        
        // 如果有设备需要检测，启动并行检测任务
        if (needCheckHWT101 || needCheckPAW3395 || needCheckVision || needCheckMotor || needCheckServo) {
            deviceCheckInProgress = true;
            
            // 创建并行检测任务，增加堆栈大小避免溢出
            if (needCheckHWT101 && deviceCheckTasks[0] == NULL) {
                if (xTaskCreate(deviceCheckTask, "CheckHWT101", 2048, (void*)0, 2, &deviceCheckTasks[0]) != pdPASS) {
                    deviceCheckTasks[0] = NULL; // 创建失败时清空句柄
                }
            }
            if (needCheckPAW3395 && deviceCheckTasks[1] == NULL) {
                if (xTaskCreate(deviceCheckTask, "CheckPAW3395", 2048, (void*)1, 2, &deviceCheckTasks[1]) != pdPASS) {
                    deviceCheckTasks[1] = NULL;
                }
            }
            if (needCheckVision && deviceCheckTasks[2] == NULL) {
                if (xTaskCreate(deviceCheckTask, "CheckVision", 2048, (void*)2, 2, &deviceCheckTasks[2]) != pdPASS) {
                    deviceCheckTasks[2] = NULL;
                }
            }
            if (needCheckMotor && deviceCheckTasks[3] == NULL) {
                if (xTaskCreate(deviceCheckTask, "CheckMotor", 2048, (void*)3, 2, &deviceCheckTasks[3]) != pdPASS) {
                    deviceCheckTasks[3] = NULL;
                }
            }
            if (needCheckServo && deviceCheckTasks[4] == NULL) {
                if (xTaskCreate(deviceCheckTask, "CheckServo", 3072, (void*)4, 2, &deviceCheckTasks[4]) != pdPASS) { // 舵机检测需要更大堆栈
                    deviceCheckTasks[4] = NULL;
                }
            }
            
            // 等待所有检测任务完成（最多等待3秒）
            int waitCount = 0;
            while (waitCount < 30) { // 30 * 100ms = 3秒
                bool allTasksDone = true;
                for (int i = 0; i < 5; i++) {
                    if (deviceCheckTasks[i] != NULL) {
                        allTasksDone = false;
                        break;
                    }
                }
                if (allTasksDone) break;
                vTaskDelay(pdMS_TO_TICKS(100));
                waitCount++;
            }
            
            deviceCheckInProgress = false;
        }
        
        // 检查最终状态
        if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(100))) {
            bool allOk = devStatus.hwt101_status && devStatus.paw3395_status && 
                        devStatus.vision_status && devStatus.motor_status && devStatus.servo_status;
            xSemaphoreGive(devStatusMutex);
            return allOk && positionOk;
        }
    }
    return false;
}

void initDevMonitor(void){
    // 创建互斥锁
    devStatusMutex = xSemaphoreCreateMutex();
    taskProgressMutex = xSemaphoreCreateMutex();
    
    // 初始化设备状态为false
    devStatus.hwt101_status = false;
    devStatus.paw3395_status = false;
    devStatus.vision_status = false;
    devStatus.motor_status = false;
    devStatus.servo_status = false;
    
    // 初始化位置监测
    lastPositionUpdateTime = millis();
    getGlobalPosition(&lastPosition);
    positionMonitorEnabled = true;
    DEBUG_LOG("位置数据监测已启用，超时阈值: %lums", POSITION_DEADLOCK_TIMEOUT);
    
    xTaskCreate(
        [](void *pvParameters){
            // 减少初始等待时间，从100ms减少到50ms
            vTaskDelay(pdMS_TO_TICKS(50));
            while(1){
                if(checkDevice()){
                    break;
                }else{
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
            vTaskDelete(NULL);
        },
        "DevMonitor",
        3072,
        NULL,
        1,
        NULL
    );
}