#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern SemaphoreHandle_t xSemaphoreMainsequence;
void initTaskManager(void);

void lvglTask(void *pvParameters);