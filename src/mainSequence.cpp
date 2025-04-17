#include"mainSequence.h"

void startMainSequence(void){
    /*启动主流程*/
    if (xSemaphoreTake(xSemaphoreMainsequence, portMAX_DELAY) == pdTRUE)
    {
        // 释放信号量，表示主流程可以开始
        xSemaphoreGive(xSemaphoreMainsequence);
    }
}
void mainSequenceTask(void *pvParameters)
{
    if(xSemaphoreTake(xSemaphoreMainsequence, portMAX_DELAY) == pdTRUE)
    {
        // 等待信号量，表示主流程可以开始
        // 主流程的代码
        moveTo();
        xSemaphoreGive(xSemaphoreMainsequence);
    }
    vTaskDelete(NULL); // 删除当前任务
}