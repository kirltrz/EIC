#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define wait(ms) delay(ms)//才知道delay其实就是vTaskDelay，其他地方就不改了，就这样了

extern SemaphoreHandle_t xSemaphoreMainsequence;
extern SemaphoreHandle_t xSemaphoreArmTest;
void initTaskManager(void);
