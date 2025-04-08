#include <Arduino.h>
#include "displayInterface.h"
#include "taskManager.h"
#include "comm.h"

void setup()
{
    initComm();
    initGUI();
    initTaskManager();
}

void loop()
{
}
