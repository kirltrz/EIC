#include <Arduino.h>
#include "config.h"
#include "displayInterface.h"
#include "taskManager.h"
#include "sensor.h"
#include "LED.h"
#include "motion.h"
#include "vision.h"
#include "arm.h"

void setup()
{
    initMotor();
    initArm();
    initLED();
    initSensor();
    initGUI();
    initVision();
    initTaskManager();
}

void loop()
{
}
