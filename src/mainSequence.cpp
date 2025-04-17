#include "mainSequence.h"
#include "motion.h"

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
void startMainSequence(void)
{
    /*启动主流程*/
    if (xSemaphoreTake(xSemaphoreMainsequence, portMAX_DELAY) == pdTRUE)
    {
        // 释放信号量，表示主流程可以开始
        xSemaphoreGive(xSemaphoreMainsequence);
    }
}
void wait(int ms){vTaskDelay(ms/portTICK_PERIOD_MS);}

void mainSequenceTask(void *pvParameters)
{
    if (xSemaphoreTake(xSemaphoreMainsequence, portMAX_DELAY) == pdTRUE) // 等待信号量，表示主流程可以开始
    {
        // 主流程的代码
        moveTo(pos[0]);
        xSemaphoreGive(xSemaphoreMainsequence);
    }
    vTaskDelete(NULL); // 删除当前任务
}