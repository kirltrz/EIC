#include <Arduino.h>
#include "displayInterface.h"
#include "taskManager.h"
#include "sensor.h"

void setup()
{
    initSensor();
    initGUI();
    initTaskManager();
}

void loop()
{
}
