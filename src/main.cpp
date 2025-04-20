#include <Arduino.h>
#include "displayInterface.h"
#include "taskManager.h"
#include "sensor.h"
#include "LED.h"

void setup()
{
    initLED();
    initSensor();
    initGUI();
    initTaskManager();
}

void loop()
{
}
