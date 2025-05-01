#include "taskManager.h"
#include "displayInterface.h"
#include "mainSequence.h"
#include "motion.h"
#include "sensor.h"
SemaphoreHandle_t xSemaphoreMainsequence = NULL; // 创建一个信号量句柄

void initTaskManager(void)
{
    xSemaphoreMainsequence = xSemaphoreCreateBinary(); // 创建一个指示主流程开始的二进制信号量
    /*初始化任务管理器*/
    xTaskCreate(
        lvglTask,    // 任务函数
        "LVGL Task", // 任务名称
        8192,        // 堆栈大小（1 字 = 4 字节）
        NULL,        // 任务参数
        1,           // 任务优先级
        NULL         // 任务句柄
    );
    xTaskCreate(mainSequenceTask, "Main Sequence Task", 4096, NULL, 1, NULL);
    xTaskCreate(moveTask, "Move Task", 4096, NULL, 1, NULL);
    xTaskCreate(calculateGlobalPosition, "Calculate Global Position", 2048, NULL, 1, NULL);
}
