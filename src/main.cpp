#include <Arduino.h>
#include "displayInterface.h"
#include "taskManager.h"
#include "sensor.h"
#include "LED.h"
#include "motion.h"
void setup()
{
    initMotor();
    initLED();
    initSensor();
    initGUI();
    initTaskManager();
}

void loop()
{
}
