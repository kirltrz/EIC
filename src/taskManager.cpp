#include "taskManager.h"
#include "displayInterface.h"

void initTaskManager(void)
{
    /*初始化任务管理器*/
    xTaskCreate(
        lvglTask,    // 任务函数
        "LVGL Task", // 任务名称
        8192,        // 堆栈大小（1 字 = 4 字节）
        NULL,        // 任务参数
        1,           // 任务优先级
        NULL         // 任务句柄
    );
    
}
void lvglTask(void *pvParameters)
{
    while (true)
    {
        lv_timer_handler();
        vTaskDelay(5);
    }
}