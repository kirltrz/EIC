#include "devMonitor.h"
#include "Wire.h"
#include "HWT101.h"
#include "vision.h"
#include "motion.h"
#include "arm.h"
#include "PAW3395.h"

deviceStatus_t devStatus;
SemaphoreHandle_t devStatusMutex = NULL;

taskProgress_t taskProgress = TASK_IDLE;
SemaphoreHandle_t taskProgressMutex = NULL;

// 添加设备检测任务句柄和标志
TaskHandle_t deviceCheckTasks[5] = {NULL};
volatile bool deviceCheckInProgress = false;

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
    
    // 如果正在进行设备检测，直接返回当前状态
    if (deviceCheckInProgress) {
        if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(10))) {
            bool allOk = devStatus.hwt101_status && devStatus.paw3395_status && 
                        devStatus.vision_status && devStatus.motor_status && devStatus.servo_status;
            xSemaphoreGive(devStatusMutex);
            return allOk;
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
            return allOk;
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