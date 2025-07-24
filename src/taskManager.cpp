#include "taskManager.h"
#include "displayInterface.h"
#include "mainSequence.h"
#include "motion.h"
#include "arm.h"
#include "sensor.h"
#include "ota.h"
#include "vision.h"

SemaphoreHandle_t xSemaphoreMainsequence = NULL; // 创建一个信号量句柄
SemaphoreHandle_t xSemaphoreOTA = NULL;
SemaphoreHandle_t xSemaphoreMainsequencePause = NULL; // 用于暂停主流程的信号量

void initTaskManager(void)
{
    xSemaphoreMainsequence = xSemaphoreCreateBinary(); // 创建一个指示主流程开始的二进制信号量
    xSemaphoreOTA = xSemaphoreCreateBinary();
    xSemaphoreMainsequencePause = xSemaphoreCreateBinary(); // 创建暂停主流程的信号量
    xSemaphoreGive(xSemaphoreMainsequencePause); // 初始化为可用状态（未暂停）
    
    /*初始化任务管理器*/
    xTaskCreate(
        lvglTask,    // 任务函数
        "LVGL Task", // 任务名称
        8192,        // 堆栈大小（1 字 = 4 字节）
        NULL,        // 任务参数
        2,           // 任务优先级
        NULL         // 任务句柄
    );
    xTaskCreate(mainSequenceTask, "Main Sequence Task", 4096, NULL, 1, NULL);
    xTaskCreate(moveTask, "Move Task", 8192, NULL, 1, NULL);
    //xTaskCreate(calculateGlobalPosition, "Calculate Global Position", 4096, NULL, 1, NULL); // 增加堆栈大小到4096
    
    // Vision监听任务
    xTaskCreate(visionListenerTask, "Vision Listener Task", 2048, NULL, 2, NULL);
    
    // OTA任务
    //xTaskCreate(otaTask, "OTA Task", 4096, NULL, 1, NULL);
}
