#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void initTaskManager(void);

void lvglTask(void *pvParameters);