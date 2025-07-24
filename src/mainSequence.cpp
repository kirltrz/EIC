#include "mainSequence.h"
#include "motion.h"
#include "arm.h"
#include "vision.h"
#include "errorHandler.h"
#include "config.h"
#include "sensor.h"

#if !DEBUG_ENABLE
#include "ui.h"
#endif

#define TASK_TIMEOUT 5000
POS pos[] = {
    {556.0  , 138.0  , 0.0f }, // 0扫二维码
    {1378.0 , -20.0  , 0.0f }, // 1转盘抓取
    {975.0  , 100.0  , 0.0f }, // 2离开转盘
    {975.0  , 1900.0 , 0.0f }, // 3粗加工区
    {1800.0 , 1800.0 , 0.0f }, // 4离开粗加工区
    {1900.0 , 985.0  , 0.0f }, // 5暂存区
    {1800.0 , 80.0   , 0.0f }, // 6离开暂存区（准备第二轮到转盘）
    {150.0  , 1000.0 , 0.0f }, // 7离开暂存区（准备回到启停区）
    {100.0  , 100.0  , 0.0f }, // 8准备进入启停区
    {-40.0  , -40.0  , 0.0f }, //{50.0    , -50.0  , 0.0f  }, // 9回到启停区
};
int taskcode[2][3] = {0};
int circleOffset[3][2] = {0};
int startTime = 0;

// 检查并等待主流程暂停信号解除
void waitIfPaused(void) {
    if(xSemaphoreTake(xSemaphoreMainsequencePause, 0) == pdTRUE) {
        // 如果成功获取信号量，则说明未暂停，立即释放并继续
        xSemaphoreGive(xSemaphoreMainsequencePause);
    } else {
        // 如果未能获取信号量，说明暂停中，需要等待
        DEBUG_LOG("主流程暂停中，等待恢复...");
        // 等待暂停信号量被释放
        xSemaphoreTake(xSemaphoreMainsequencePause, portMAX_DELAY);
        xSemaphoreGive(xSemaphoreMainsequencePause);
        DEBUG_LOG("主流程已恢复");
    }
}

void startMainSequence(void)
{
    /*启动主流程*/
    // 直接释放信号量，通知mainSequenceTask可以开始运行
    xSemaphoreGive(xSemaphoreMainsequence);
}

void updateTaskcodeDisplay(void)
{
    // 格式化任务码为"123+456"的形式
    char taskcodeStr[10];
    snprintf(taskcodeStr, sizeof(taskcodeStr), "%d%d%d+%d%d%d", 
             taskcode[0][0], taskcode[0][1], taskcode[0][2],
             taskcode[1][0], taskcode[1][1], taskcode[1][2]);
    
#if DEBUG_ENABLE
    // Debug模式：输出到串口
    DEBUG_LOG("任务码: %s\n", taskcodeStr);
#else
    // Release模式：更新UI显示
    if (ui_taskcodeText != NULL) {
        lv_label_set_text(ui_taskcodeText, taskcodeStr);
    }
#endif
}

void mainSequenceTask(void *pvParameters)
{
    // 等待信号量，表示主流程可以开始
    if (xSemaphoreTake(xSemaphoreMainsequence, portMAX_DELAY) == pdTRUE)
    {
        // 主流程的代码
        resetSensor();    // 重置定位系统到 0, 0, 0
        DEBUG_LOG("定位系统已重置到 0, 0, 0");
        P(TASK_RESET_SENSOR);

        motor->enControl(MOTOR_BROADCAST, true);//使能电机，准备运动
        delay(500);

        moveTo(pos[0]);   // 前往二维码前方
        arm_ScanQRcode(); // 机械臂运动至扫描二维码状态
        waitNear();

        // QR码扫描逻辑
        if (!visionScanQRcode(taskcode[0], taskcode[1])) {
            errorHandle(ERROR_QRCODE_RECOGNITION_FAILED);
        }
        
        // 获取到任务码后更新UI显示
        updateTaskcodeDisplay();
        P(TASK_SCAN_QRCODE);

        moveTo(pos[1]); // 1 前往转盘
        waitArrived();
        arm_catchFromTurntable(taskcode[0]);
        P(TASK_FIRST_TURNTABLE);
        
        // 在关键点检查暂停状态
        waitIfPaused();
        
        moveTo(pos[2]); // 1 前往离开转盘状态
        waitNear();
        
        moveTo(pos[3]); // 1 前往粗加工区
        waitArrived();
        arm_putToGround(taskcode[0]);
        arm_catchFromGround(taskcode[0]);
        P(TASK_FIRST_ROUGH);
        
        // 在关键点检查暂停状态
        waitIfPaused();
        
        moveTo(pos[4]); // 1 离开粗加工区
        waitNear();
        
        moveTo(pos[5]); // 1 前往暂存区
        waitArrived();
        arm_putToGround(taskcode[0]);
        P(TASK_FIRST_STORAGE);
        
        // 在关键点检查暂停状态
        waitIfPaused();
        
        moveTo(pos[6]); // 1 离开暂存区
        waitNear();
        
        moveTo(pos[1]); // 2 前往转盘
        waitArrived();
        arm_catchFromTurntable(taskcode[1]);
        P(TASK_SECOND_TURNTABLE);
        
        // 在关键点检查暂停状态
        waitIfPaused();
        
        moveTo(pos[2]); // 2 前往离开转盘状态
        waitNear();
        
        moveTo(pos[3]); // 2 前往粗加工区
        waitArrived();
        arm_putToGround(taskcode[1]);
        arm_catchFromGround(taskcode[1]);
        P(TASK_SECOND_ROUGH);
        
        // 在关键点检查暂停状态
        waitIfPaused();
        
        moveTo(pos[4]); // 2 离开粗加工区
        waitNear();
        
        moveTo(pos[5]); // 2 前往暂存区
        waitArrived();
        arm_putToGround(taskcode[1]);
        P(TASK_SECOND_STORAGE);
        
        // 在关键点检查暂停状态
        waitIfPaused();
        
        moveTo(pos[7]); // 2 离开暂存区
        waitNear();
        
        moveTo(pos[8]); // 前往启停区
        P(TASK_RETURN_HOME);
        waitNear();
        moveTo(pos[9]); // 回到启停区
        waitArrived();
        
        P(TASK_COMPLETED);
        // 释放信号量，表示主流程已完成
        xSemaphoreGive(xSemaphoreMainsequence);
    }
    
    vTaskDelete(NULL); // 删除当前任务
}