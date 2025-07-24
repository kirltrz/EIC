#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern SemaphoreHandle_t xSemaphoreMainsequence;
extern SemaphoreHandle_t xSemaphoreArmTest;
extern SemaphoreHandle_t xSemaphoreOTA;
extern SemaphoreHandle_t xSemaphoreMainsequencePause; // 用于暂停主流程的信号量

void initTaskManager(void);
