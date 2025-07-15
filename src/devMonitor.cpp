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

bool checkPaw3395(void)
{
    /*检查paw3395是否正常工作*/
    return paw3395_check();
}

bool checkHWT101(void)
{
    /*检查hwt101是否正常工作*/
    Wire.beginTransmission(HWT101.devAddr);
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

bool checkDevice(void){
    if (devStatusMutex != NULL) {
        if (xSemaphoreTake(devStatusMutex, pdMS_TO_TICKS(100))) {
            if(devStatus.hwt101_status == false){
                devStatus.hwt101_status = checkHWT101();
            }
            if(devStatus.paw3395_status == false){
                devStatus.paw3395_status = checkPaw3395();
            }
            if(devStatus.vision_status == false){
                devStatus.vision_status = checkVision();
            }
            if(devStatus.motor_status == false){
                devStatus.motor_status = checkMotor();
            }
            if(devStatus.servo_status == false){
                devStatus.servo_status = checkServo();
            }
            
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
    
    xTaskCreate(
        [](void *pvParameters){
            // 等待电机通信稳定，避免在release模式下因缺少DEBUG_LOG延时而导致检测失败
            vTaskDelay(pdMS_TO_TICKS(100));
            while(1){
                if(checkDevice()){
                    vTaskDelete(NULL);
                }else{
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
        },
        "DevMonitor",
        2048,
        NULL,
        1,
        NULL
    );
}