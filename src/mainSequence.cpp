#include "mainSequence.h"
#include "motion.h"
#include "arm.h"
#include "vision.h"
#include "errorHandler.h"

#define TASK_TIMEOUT 3000
POS pos[] = {
    {-178, 586, 0},   // 0扫二维码
    {-10, 1411, 0},   // 1转盘抓取
    {-164, 986, 90},  // 2离开转盘
    {-1900, 986, 90}, // 3粗加工区
    {-1824, 1827, 0}, // 4离开粗加工区
    {-991, 1924, 0},  // 5暂存区
    {-125, 1846, 0},  // 6离开暂存区（准备第二轮到转盘）
    {-991, 183, 0},   // 7离开暂存区（准备回到启停区）
    {-50, 183, 0},    // 8准备进入启停区
    {50, -50, 0},     // 9回到启停区
};
int taskcode[2][3] = {0};
int circleOffset[3][2] = {0};
int startTime = 0;
void startMainSequence(void)
{
    /*启动主流程*/
    if (xSemaphoreTake(xSemaphoreMainsequence, portMAX_DELAY) == pdTRUE)
    {
        // 释放信号量，表示主流程可以开始
        xSemaphoreGive(xSemaphoreMainsequence);
    }
}

void mainSequenceTask(void *pvParameters)
{
    if (xSemaphoreTake(xSemaphoreMainsequence, portMAX_DELAY) == pdTRUE) // 等待信号量，表示主流程可以开始
    {
        // 主流程的代码
        moveTo(pos[0]);   // 前往二维码前方
        arm_ScanQRcode(); // 机械臂运动至扫描二维码状态
        delay(1000);
        startTime = millis();
        while (!visionScanQRcode(taskcode[0], taskcode[1]))
        {
            delay(100);
            if (millis() - startTime > TASK_TIMEOUT)
            {
                // 如果超过1秒，则认为二维码识别失败
                errorHandle(ERROR_QRCODE_RECOGNITION_FAILED);
                break;
            }
        }
        moveTo(pos[1]); // 1 前往转盘
        delay(1000);
        arm_catchFromTurntable(taskcode[0]);
        moveTo(pos[2]); // 1 前往离开转盘状态
        delay(1000);
        moveTo(pos[3]); // 1 前往粗加工区
        delay(2000);
        arm_putToGround(taskcode[0], circleOffset);
        arm_catchFromGround(taskcode[0], circleOffset);
        moveTo(pos[4]); // 1 离开粗加工区
        delay(1000);
        moveTo(pos[5]); // 1 前往暂存区
        delay(1000);
        arm_putToGround(taskcode[0], circleOffset);
        moveTo(pos[6]); // 1 离开暂存区
        delay(1000);
        moveTo(pos[1]); // 2 前往转盘
        delay(1000);
        arm_catchFromTurntable(taskcode[1]);
        moveTo(pos[2]); // 2 前往离开转盘状态
        delay(1000);
        moveTo(pos[3]); // 2 前往粗加工区
        delay(2000);
        arm_putToGround(taskcode[1], circleOffset);
        arm_catchFromGround(taskcode[1], circleOffset);
        moveTo(pos[4]); // 2 离开粗加工区
        delay(1000);
        moveTo(pos[5]); // 2 前往暂存区
        delay(1000);
        arm_putToGround(taskcode[1], circleOffset);
        moveTo(pos[7]); // 2 离开暂存区
        delay(2000);
        moveTo(pos[8]); // 前往启停区
        delay(1000);
        moveTo(pos[9]); // 回到启停区
        delay(1000);

        xSemaphoreGive(xSemaphoreMainsequence);
    }
    vTaskDelete(NULL); // 删除当前任务
}