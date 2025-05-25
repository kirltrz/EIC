#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern SemaphoreHandle_t xSemaphoreMainsequence;
extern SemaphoreHandle_t xSemaphoreArmTest;
extern SemaphoreHandle_t xSemaphoreOTA;

void initTaskManager(void);
