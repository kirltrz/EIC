#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern SemaphoreHandle_t xSemaphoreMainsequence;
void wait(int ms);
void initTaskManager(void);
